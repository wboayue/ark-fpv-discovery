#![no_std]
#![no_main]

use panic_halt as _;

mod baro;
mod imu;

use heapless::String;
use stm32h7xx_hal as hal;

use core::fmt::Write;

use hal::{
    gpio::{ErasedPin, Output, PinState, PushPull},
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

/// Written to `BOOT_FLAG` to request a ROM-bootloader jump on the next boot.
const BOOTLOADER_MAGIC: u32 = 0xB007_0DF1;
/// STM32H743 system-memory (ROM) bootloader entry vector — see ST AN2606.
const SYSTEM_BOOTLOADER: *const u32 = 0x1FF0_9800 as *const u32;

/// Lives in `.uninit`, which cortex-m-rt does NOT zero, so it survives the soft
/// reset that carries the request from the running app into `pre_init`.
#[link_section = ".uninit.BOOT_FLAG"]
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
    use crate::imu::Imu;

    /// Lock the shared USB serial port and write `msg`. Generic over the RTIC resource
    /// proxy so every task shares one code path. Best-effort: write errors are dropped.
    fn write_serial(serial: &mut impl rtic::Mutex<T = SerialPort<'static, UsbBus<USB2>>>, msg: &str) {
        serial.lock(|serial| {
            let _ = serial.write(msg.as_bytes());
        });
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
    }

    #[local]
    struct Local {
        counter: u32,
        leds: [ErasedPin<Output<PushPull>>; 3],
        imu: Imu,
        baro: Baro,
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
        let imu = Imu::new(spi, gpioi.pi9.into_push_pull_output().erase());

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
        imu_sample::spawn().ok();
        baro_sample::spawn().ok();

        (
            Shared { usb_dev, serial },
            Local { counter: 0, leds, imu, baro },
        )
    }

    #[task(binds = OTG_FS, shared = [usb_dev, serial])]
    fn usb_irq(mut cx: usb_irq::Context) {
        cx.shared.usb_dev.lock(|usb_dev| {
            cx.shared.serial.lock(|serial| {
                if usb_dev.poll(&mut [serial]) {
                    // 'r' reboots into the ROM bootloader for dfu-util flashing.
                    let mut buf = [0u8; 32];
                    if let Ok(n) = serial.read(&mut buf) {
                        if buf[..n].contains(&b'r') {
                            reboot_to_bootloader();
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

            let mut msg: String<64> = String::new();

            write!(
                &mut msg,
                "hello from RTIC on STM32H743, tick {}\r\n",
                *cx.local.counter
            )
            .ok();

            write_serial(&mut cx.shared.serial, &msg);

            *cx.local.counter = cx.local.counter.wrapping_add(1);

            Mono::delay(1_000.millis()).await;
        }
    }

    // Bring up the IIM-42653 and stream scaled accel/gyro/temp at 10 Hz.
    #[task(shared = [serial], local = [imu])]
    async fn imu_sample(mut cx: imu_sample::Context) {
        let imu = cx.local.imu;

        let id = imu.who_am_i();
        let mut msg: String<128> = String::new();
        write!(
            &mut msg,
            "imu WHO_AM_I=0x{:02x} (expect {:02x})\r\n",
            id,
            crate::imu::EXPECTED_WHO_AM_I
        )
        .ok();
        write_serial(&mut cx.shared.serial, &msg);

        // Reset to a known state, then configure and let the gyro start.
        imu.soft_reset();
        Mono::delay(2.millis()).await;
        imu.configure();
        Mono::delay(50.millis()).await;

        loop {
            let s = imu.read();

            let mut msg: String<128> = String::new();
            write!(
                &mut msg,
                "imu accel[g]={:.2},{:.2},{:.2} gyro[dps]={:.1},{:.1},{:.1} temp={:.1}C\r\n",
                s.accel_g[0],
                s.accel_g[1],
                s.accel_g[2],
                s.gyro_dps[0],
                s.gyro_dps[1],
                s.gyro_dps[2],
                s.temp_c
            )
            .ok();

            write_serial(&mut cx.shared.serial, &msg);

            Mono::delay(100.millis()).await;
        }
    }

    // Bring up the BMP388/BMP390 barometer and stream pressure/temp at ~2 Hz.
    #[task(shared = [serial], local = [baro])]
    async fn baro_sample(mut cx: baro_sample::Context) {
        let baro = cx.local.baro;

        let id = baro.chip_id();
        let part = match id {
            CHIP_ID_BMP388 => "BMP388",
            CHIP_ID_BMP390 => "BMP390",
            _ => "unknown",
        };
        let mut msg: String<128> = String::new();
        write!(&mut msg, "baro CHIP_ID=0x{:02x} ({})\r\n", id, part).ok();
        write_serial(&mut cx.shared.serial, &msg);

        // Reset, read factory calibration, then start normal-mode sampling.
        baro.soft_reset();
        Mono::delay(5.millis()).await;
        baro.read_calibration();
        baro.configure();
        Mono::delay(50.millis()).await;

        loop {
            let s = baro.read();

            let mut msg: String<128> = String::new();
            write!(
                &mut msg,
                "baro press={:.2}hPa temp={:.2}C\r\n",
                s.pressure_hpa, s.temp_c
            )
            .ok();

            write_serial(&mut cx.shared.serial, &msg);

            Mono::delay(500.millis()).await;
        }
    }
}