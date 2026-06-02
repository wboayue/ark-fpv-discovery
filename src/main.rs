#![no_std]
#![no_main]

use panic_halt as _;

mod baro;
mod config;
mod fusion;
mod i2c_regs;
mod imu;
mod mag;

use heapless::String;
use stm32h7xx_hal as hal;

use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use hal::{
    gpio::{Edge, ErasedPin, ExtiPin, Input, Output, PinState, PushPull},
    pac,
    prelude::*,
    rcc::rec::UsbClkSel,
    usb_hs::{UsbBus, USB2},
};

use usb_device::{
    bus::UsbBusAllocator,
    prelude::*,
};

use usbd_serial::SerialPort;

use rtic_monotonics::systick::prelude::*;

systick_monotonic!(Mono, 1_000);

// --- Reboot-to-DFU support ----------------------------------------------------
// Sending 'r' over the USB serial link reboots the board into the STM32 ROM
// bootloader, so it can be reflashed with dfu-util without touching BOOT0/RESET.

// --- Runtime diagnostic mode --------------------------------------------------
// Toggled by the 'd' byte over USB serial (alongside 'r' for reboot-to-DFU). When on, sensor
// tasks emit verbose register-level dumps instead of concise readings. Lock-free so any task can
// read it without an RTIC resource lock.
static DIAG: AtomicBool = AtomicBool::new(false);

fn diag_enabled() -> bool {
    DIAG.load(Ordering::Relaxed)
}

/// Flip the diagnostic flag; returns the new state.
fn toggle_diag() -> bool {
    !DIAG.fetch_xor(true, Ordering::Relaxed)
}

/// Written to `BOOT_FLAG` to request a ROM-bootloader jump on the next boot.
const BOOTLOADER_MAGIC: u32 = 0xB007_0DF1;
/// STM32H743 system-memory (ROM) bootloader entry vector — see ST AN2606.
const SYSTEM_BOOTLOADER: *const u32 = 0x1FF0_9800 as *const u32;

/// Lives in `.uninit`, which cortex-m-rt does NOT zero, so it survives the soft
/// reset that carries the request from the running app into `pre_init`.
#[unsafe(link_section = ".uninit.BOOT_FLAG")]
static mut BOOT_FLAG: core::mem::MaybeUninit<u32> = core::mem::MaybeUninit::uninit();

#[inline(always)]
fn boot_flag() -> *mut u32 {
    core::ptr::addr_of_mut!(BOOT_FLAG).cast()
}

/// Flag a bootloader jump and reset. The jump itself happens at the top of `init`,
/// after the reset, while clocks are still at their reset defaults.
fn reboot_to_bootloader() -> ! {
    unsafe { boot_flag().write_volatile(BOOTLOADER_MAGIC) };
    cortex_m::peripheral::SCB::sys_reset();
}

/// If a reboot-to-DFU was requested before the last reset, enter the ROM bootloader.
/// Call this as the very first thing in `init`, before touching any peripheral/clock.
#[inline(always)]
fn maybe_enter_bootloader() {
    unsafe {
        if boot_flag().read_volatile() == BOOTLOADER_MAGIC {
            boot_flag().write_volatile(0);
            cortex_m::asm::bootload(SYSTEM_BOOTLOADER);
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::pac, peripherals = true, dispatchers = [FDCAN1_IT0])]
mod app {
    use super::*;
    use crate::baro::{Baro, CHIP_ID_BMP388, CHIP_ID_BMP390};
    use crate::fusion::{FusedState, Fusion, SensorState};
    use crate::imu::{Imu, ImuSample};
    use crate::mag::Mag;

    /// Lock the shared USB serial port and write `msg`. Generic over the RTIC resource
    /// proxy so every task shares one code path. Best-effort: write errors are dropped.
    fn write_serial(serial: &mut impl rtic::Mutex<T = SerialPort<'static, UsbBus<USB2>>>, msg: &str) {
        serial.lock(|serial| {
            let _ = serial.write(msg.as_bytes());
        });
    }

    /// Longest formatted log line we emit (the verbose mag diagnostic dump); sizes the buffer.
    const LOG_LINE_CAP: usize = 192;

    /// Format and write one line to the shared serial port. Collapses the repeated
    /// `String::new()` / `write!` / `write_serial` boilerplate, owning the one buffer size so call
    /// sites carry no magic number. Best-effort: a line longer than `LOG_LINE_CAP` is truncated and
    /// write errors are dropped. Pass the message with `format_args!`, e.g.
    /// `log_fmt(&mut cx.shared.serial, format_args!("tick {}", n))`.
    fn log_fmt(
        serial: &mut impl rtic::Mutex<T = SerialPort<'static, UsbBus<USB2>>>,
        args: core::fmt::Arguments,
    ) {
        let mut msg: String<LOG_LINE_CAP> = String::new();
        let _ = msg.write_fmt(args);
        write_serial(serial, &msg);
    }

    // Status LED indices into `Local::leds` — ARK FPV board pins PE3/PE4/PE5.
    // See docs/ark-fpv-board.md. log_tick cycles through them red→green→blue.
    const LED_RED: usize = 0;
    const LED_GREEN: usize = 1;
    const LED_BLUE: usize = 2;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBus<USB2>>,
        serial: SerialPort<'static, UsbBus<USB2>>,
        // Latest raw sensor inputs: producers write, the fusion task drains. imu_drdy (prio 2)
        // also touches this, so its ceiling is prio 2 — keep every critical section tiny.
        latest: SensorState,
        // Latest fused estimate, for a future control loop to read.
        fused: FusedState,
    }

    #[local]
    struct Local {
        counter: u32,
        leds: [ErasedPin<Output<PushPull>>; 3],
        imu: Imu,
        imu_drdy: ErasedPin<Input>,
        imu_id: u8,
        baro: Baro,
        mag: Mag,
        fusion: Fusion,
    }

    #[init(local = [
        ep_mem: [u32; 1024] = [0; 1024],
        usb_bus: Option<UsbBusAllocator<UsbBus<USB2>>> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Honor a pending reboot-to-DFU request before configuring anything.
        maybe_enter_bootloader();

        let dp: pac::Peripherals = cx.device;
        Mono::start(cx.core.SYST, 400_000_000);

        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();

        let rcc = dp.RCC.constrain();

        // pll1_q_ck must be enabled: it's SPI1's kernel clock, which the HAL .expect()s when
        // building the SPI (otherwise `init` panics → no CDC). Independent of the HSI48 USB path.
        let mut ccdr = rcc
            .sys_ck(400.MHz())
            .pll1_q_ck(80.MHz())
            .freeze(vos, &dp.SYSCFG);

        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
        let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
        let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI);

        // Status LEDs: red=PE3, green=PE4, blue=PE5. These are active-low on the
        // ARK FPV (pin LOW = lit), so start all three HIGH (off). log_tick blinks green.
        let leds = [
            gpioe.pe3.into_push_pull_output_in_state(PinState::High).erase(),
            gpioe.pe4.into_push_pull_output_in_state(PinState::High).erase(),
            gpioe.pe5.into_push_pull_output_in_state(PinState::High).erase(),
        ];

        // IIM-42653 IMU on SPI1: SCK=PA5, MISO=PG9, MOSI=PB5 (all AF5), soft CS=PI9.
        // MODE_3 (CPOL=1, CPHA=1); ~8 MHz, well under the 24 MHz max.
        let spi = dp.SPI1.spi(
            (
                gpioa.pa5.into_alternate::<5>(),
                gpiog.pg9.into_alternate::<5>(),
                gpiob.pb5.into_alternate::<5>(),
            ),
            hal::spi::MODE_3,
            8.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );
        let mut imu = Imu::new(spi, gpioi.pi9.into_push_pull_output().erase());
        // Control-mode bring-up: reset, brief settle (~2 ms @ 400 MHz), then configure ODR,
        // filtering, and DRDY-on-INT1. The gyro takes ~50 ms to start — DRDY simply won't fire
        // until then, so no explicit wait is needed here.
        imu.soft_reset();
        cortex_m::asm::delay(800_000);
        let imu_id = imu.who_am_i();
        imu.configure_control_mode(config::IMU_ODR);

        // IMU data-ready (INT1) → PF2 → EXTI line 2. Rising edge (INT1 is active-high push-pull).
        let mut syscfg = dp.SYSCFG;
        let mut exti = dp.EXTI;
        let mut imu_drdy = gpiof.pf2.into_floating_input().erase();
        imu_drdy.make_interrupt_source(&mut syscfg);
        imu_drdy.trigger_on_edge(&mut exti, Edge::Rising);
        imu_drdy.enable_interrupt(&mut exti);

        // BMP388/BMP390 barometer on I2C2: SCL=PF1, SDA=PF0 (AF4, open-drain). No kernel-clock
        // setup needed — I2C123 runs off PCLK1, which is always live after freeze().
        let i2c = dp.I2C2.i2c(
            (
                gpiof.pf1.into_alternate_open_drain::<4>(),
                gpiof.pf0.into_alternate_open_drain::<4>(),
            ),
            400.kHz(),
            ccdr.peripheral.I2C2,
            &ccdr.clocks,
        );
        let baro = Baro::new(i2c);

        // IIS2MDC/LIS2MDL magnetometer on I2C4: SCL=PF14, SDA=PF15 (AF4, open-drain). Like I2C2,
        // no kernel-clock setup needed — I2C4 runs off pclk4 (APB4/D3), always live after freeze().
        let i2c4 = dp.I2C4.i2c(
            (
                gpiof.pf14.into_alternate_open_drain::<4>(),
                gpiof.pf15.into_alternate_open_drain::<4>(),
            ),
            400.kHz(),
            ccdr.peripheral.I2C4,
            &ccdr.clocks,
        );
        let mag = Mag::new(i2c4);

        // Sensor fusion (attitude + altitude). Owned by the fusion_step task; fed by the latest
        // IMU/mag/baro samples. 9-DOF vs 6-DOF is config::FUSION_USE_MAG.
        let fusion = Fusion::new(config::FUSION_USE_MAG);

        // PA11 = USB DM, PA12 = USB DP
        let usb_dm = gpioa.pa11.into_alternate();
        let usb_dp = gpioa.pa12.into_alternate();

        let usb = USB2::new(
            dp.OTG2_HS_GLOBAL,
            dp.OTG2_HS_DEVICE,
            dp.OTG2_HS_PWRCLK,
            usb_dm,
            usb_dp,
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        let bus = UsbBus::new(usb, cx.local.ep_mem);
        cx.local.usb_bus.replace(bus);

        let serial = SerialPort::new(cx.local.usb_bus.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(
            cx.local.usb_bus.as_ref().unwrap(),
            UsbVidPid(0x1209, 0x0001),
        )
        .strings(&[usb_device::device::StringDescriptors::default()
            .manufacturer("Wil")
            .product("RTIC STM32H743 USB Serial")
            .serial_number("001")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        log_tick::spawn().ok();
        baro_sample::spawn().ok();
        mag_sample::spawn().ok();
        fusion_step::spawn().ok();
        // The IMU loop is driven by the DRDY interrupt (EXTI2), not spawned here.

        (
            Shared { usb_dev, serial, latest: SensorState::default(), fused: FusedState::default() },
            Local { counter: 0, leds, imu, imu_drdy, imu_id, baro, mag, fusion },
        )
    }

    #[task(binds = OTG_FS, shared = [usb_dev, serial])]
    fn usb_irq(mut cx: usb_irq::Context) {
        cx.shared.usb_dev.lock(|usb_dev| {
            cx.shared.serial.lock(|serial| {
                if usb_dev.poll(&mut [serial]) {
                    let mut buf = [0u8; 32];
                    if let Ok(n) = serial.read(&mut buf) {
                        // 'r' reboots into the ROM bootloader for dfu-util flashing (never returns).
                        if buf[..n].contains(&b'r') {
                            reboot_to_bootloader();
                        }
                        // 'd' toggles verbose sensor diagnostics at runtime.
                        if buf[..n].contains(&b'd') {
                            let on = toggle_diag();
                            let _ = serial.write(if on { b"diag on\r\n" } else { b"diag off\r\n" });
                        }
                    }
                }
            });
        });
    }

    #[task(shared = [serial], local = [counter, leds])]
    async fn log_tick(mut cx: log_tick::Context) {
        // Cycle order: red → green → blue, one lit per tick.
        const SEQUENCE: [usize; 3] = [LED_RED, LED_GREEN, LED_BLUE];

        loop {
            // Light only the LED for this step (active-low: LOW = lit), others off.
            let step = (*cx.local.counter as usize) % SEQUENCE.len();
            for (i, led) in cx.local.leds.iter_mut().enumerate() {
                led.set_state(PinState::from(SEQUENCE[step] != i));
            }

            log_fmt(
                &mut cx.shared.serial,
                format_args!("hello from RTIC on STM32H743, tick {}\r\n", *cx.local.counter),
            );

            *cx.local.counter = cx.local.counter.wrapping_add(1);

            Mono::delay(1_000.millis()).await;
        }
    }

    // Gyro-synchronous control-loop tick: fires on the IMU's data-ready interrupt (INT1 → PF2 →
    // EXTI line 2) at the configured ODR. High priority so it preempts logging/baro/tick. The
    // future control law runs here; for now it reads the sample and hands every Nth one to the
    // low-priority logger so USB never gates the fast path. `n` counts DRDY events (lets us
    // confirm the real loop rate from the logged value).
    #[task(binds = EXTI2, priority = 2, shared = [latest], local = [imu, imu_drdy, imu_id, n: u32 = 0])]
    fn imu_drdy(mut cx: imu_drdy::Context) {
        cx.local.imu_drdy.clear_interrupt_pending_bit();
        let s = cx.local.imu.read();
        cx.local.imu.clear_interrupt(); // read INT_STATUS → drop the latched INT1 line
        // Hand the sample to the fusion path: accumulate into the running sum (pure adds, no math)
        // for delta-angle downsampling. This is the only fast-loop work added — keep it this short.
        cx.shared.latest.lock(|st| st.accumulate(&s));
        // (control step goes here)
        *cx.local.n = cx.local.n.wrapping_add(1);
        if *cx.local.n % config::IMU_LOG_DIV == 0 {
            imu_log::spawn(*cx.local.n, *cx.local.imu_id, s).ok();
        }
    }

    // Low-priority: format an IMU sample and write it to USB serial. Decoupled from the fast loop.
    // `n` is the DRDY count (confirms the real loop rate); `id` is the WHO_AM_I read at startup.
    #[task(shared = [serial])]
    async fn imu_log(mut cx: imu_log::Context, n: u32, id: u8, s: ImuSample) {
        log_fmt(
            &mut cx.shared.serial,
            format_args!(
                "imu[{}] id=0x{:02x}(exp {:02x}) accel[g]={:.2},{:.2},{:.2} gyro[dps]={:.1},{:.1},{:.1} temp={:.1}C\r\n",
                n,
                id,
                crate::imu::EXPECTED_WHO_AM_I,
                s.accel_g[0],
                s.accel_g[1],
                s.accel_g[2],
                s.gyro_dps[0],
                s.gyro_dps[1],
                s.gyro_dps[2],
                s.temp_c
            ),
        );
    }

    // Bring up the BMP388/BMP390 barometer (polled) and stream pressure/temp. Sample rate and
    // log rate come from `config` and are decoupled (sampled fast, logged every Nth).
    #[task(shared = [serial, latest], local = [baro])]
    async fn baro_sample(mut cx: baro_sample::Context) {
        let baro = cx.local.baro;

        let id = baro.chip_id();
        let part = match id {
            CHIP_ID_BMP388 => "BMP388",
            CHIP_ID_BMP390 => "BMP390",
            _ => "unknown",
        };
        log_fmt(
            &mut cx.shared.serial,
            format_args!("baro CHIP_ID=0x{:02x} ({})\r\n", id, part),
        );

        // Reset, load calibration, and start normal-mode sampling (driver owns the timing).
        if baro
            .bring_up(config::BARO_ODR, config::BARO_OSR_P, config::BARO_OSR_T, |ms| {
                Mono::delay(ms.millis())
            })
            .await
            .is_err()
        {
            log_fmt(
                &mut cx.shared.serial,
                format_args!("baro: invalid ODR/OSR (too fast for oversampling)\r\n"),
            );
            return;
        }

        let period = (1_000 / config::BARO_SAMPLE_HZ).millis();
        let mut n: u32 = 0;
        loop {
            let s = baro.read();
            n = n.wrapping_add(1);

            // Stash raw pressure for fusion; the pressure→altitude conversion stays in fusion.rs.
            cx.shared.latest.lock(|st| st.set_pressure(s.pressure_hpa));

            if n % config::BARO_LOG_DIV == 0 {
                log_fmt(
                    &mut cx.shared.serial,
                    format_args!("baro press={:.2}hPa temp={:.2}C\r\n", s.pressure_hpa, s.temp_c),
                );
            }

            Mono::delay(period).await;
        }
    }

    // Bring up the IIS2MDC/LIS2MDL magnetometer (polled) and stream the field/temp. Sample and log
    // rates come from `config` and are decoupled (sampled fast, logged every Nth), like the baro.
    #[task(shared = [serial, latest], local = [mag])]
    async fn mag_sample(mut cx: mag_sample::Context) {
        let mag = cx.local.mag;

        let id = mag.who_am_i();
        let ok = if id == crate::mag::EXPECTED_WHO_AM_I { "ok" } else { "MISMATCH" };
        log_fmt(
            &mut cx.shared.serial,
            format_args!(
                "mag WHO_AM_I=0x{:02x}(exp {:02x}) {}\r\n",
                id,
                crate::mag::EXPECTED_WHO_AM_I,
                ok
            ),
        );

        // Reset and enter continuous mode (driver owns the reset-wait + continuous-latch retries).
        if !mag.bring_up(config::MAG_ODR, |ms| Mono::delay(ms.millis())).await {
            log_fmt(
                &mut cx.shared.serial,
                format_args!("mag: failed to enter continuous mode\r\n"),
            );
        }

        let period = (1_000 / config::MAG_SAMPLE_HZ).millis();
        let mut n: u32 = 0;
        loop {
            let st = mag.status();
            let s = mag.read();
            n = n.wrapping_add(1);

            // Stash the latest field for fusion (MagSample is Copy, so `s` is still usable below).
            cx.shared.latest.lock(|state| state.set_mag(s));

            if n % config::MAG_LOG_DIV == 0 {
                if diag_enabled() {
                    // Verbose, read-only: chip ID, the three config registers, and STATUS.
                    let id = mag.who_am_i();
                    let (ca, cb, cc) = mag.read_cfg();
                    log_fmt(
                        &mut cx.shared.serial,
                        format_args!(
                            "mag[diag] id=0x{:02x} cfgA=0x{:02x} B=0x{:02x} C=0x{:02x} field[uT]={:.1},{:.1},{:.1} temp={:.1}C status=0x{:02x}\r\n",
                            id, ca, cb, cc, s.field_ut[0], s.field_ut[1], s.field_ut[2], s.temp_c, st
                        ),
                    );
                } else {
                    log_fmt(
                        &mut cx.shared.serial,
                        format_args!(
                            "mag field[uT]={:.1},{:.1},{:.1} temp={:.1}C\r\n",
                            s.field_ut[0], s.field_ut[1], s.field_ut[2], s.temp_c
                        ),
                    );
                }
            }

            Mono::delay(period).await;
        }
    }

    // Sensor fusion loop. Decoupled at FUSION_RATE_HZ: drains the IMU samples imu_drdy accumulated
    // (delta-angle downsampling → full 1 kHz gyro fidelity at a 250 Hz estimator), folds in the
    // latest mag/baro, stores the result for a future control loop, and logs it throttled. All the
    // nalgebra/libm math lives here at priority 1 — never in the imu_drdy ISR. `n` counts fused
    // states (for the log throttle). See CLAUDE.md "Sensors — Fusion".
    #[task(shared = [serial, latest, fused],
           local = [fusion, last_ms: Option<u32> = None, n: u32 = 0])]
    async fn fusion_step(mut cx: fusion_step::Context) {
        let period = (1_000 / config::FUSION_RATE_HZ).millis();
        loop {
            // One short critical section: copy out the IMU mean + latest mag/pressure, clear them.
            let (imu, mag, pressure) = cx
                .shared
                .latest
                .lock(|st| (st.drain_imu(), st.take_mag(), st.take_pressure()));

            // Measured dt (Mono is 1 kHz → ticks are ms). First iteration has no prior; use the
            // nominal period. Clamp to guard scheduling gaps from corrupting the velocity integral.
            let now_ms = Mono::now().ticks();
            let dt = match *cx.local.last_ms {
                Some(prev_ms) => (now_ms.wrapping_sub(prev_ms) as f32 / 1_000.0)
                    .clamp(config::FUSION_DT_MIN_S, config::FUSION_DT_MAX_S),
                None => 1.0 / config::FUSION_RATE_HZ as f32,
            };
            *cx.local.last_ms = Some(now_ms);

            // Gate on the gyro having started (no DRDY edges yet → nothing accumulated).
            if let Some(mean) = imu {
                let state = cx.local.fusion.update(&mean, mag.as_ref(), pressure, dt);
                cx.shared.fused.lock(|f| *f = state);

                *cx.local.n = cx.local.n.wrapping_add(1);
                if *cx.local.n % config::FUSION_LOG_DIV == 0 {
                    log_fmt(
                        &mut cx.shared.serial,
                        format_args!(
                            "fus roll={:.1} pitch={:.1} yaw={:.1}deg alt={:.2}m vz={:.2}m/s\r\n",
                            state.roll_deg,
                            state.pitch_deg,
                            state.yaw_deg,
                            state.altitude_m,
                            state.vertical_velocity
                        ),
                    );
                }
            }

            Mono::delay(period).await;
        }
    }
}