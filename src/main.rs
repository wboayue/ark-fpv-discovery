#![no_std]
#![no_main]

use panic_halt as _;

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

    // Status LED indices into `Local::leds` — ARK FPV board pins PE3/PE4/PE5.
    // See docs/ark-fpv-board.md. Red/blue are wired but not yet driven.
    #[allow(dead_code)]
    const LED_RED: usize = 0;
    const LED_GREEN: usize = 1;
    #[allow(dead_code)]
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

        let mut ccdr = rcc
            .sys_ck(400.MHz())
            .freeze(vos, &dp.SYSCFG);

        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        // Status LEDs: red=PE3, green=PE4, blue=PE5. These are active-low on the
        // ARK FPV (pin LOW = lit), so start all three HIGH (off). log_tick blinks green.
        let leds = [
            gpioe.pe3.into_push_pull_output_in_state(PinState::High).erase(),
            gpioe.pe4.into_push_pull_output_in_state(PinState::High).erase(),
            gpioe.pe5.into_push_pull_output_in_state(PinState::High).erase(),
        ];

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

        (
            Shared { usb_dev, serial },
            Local { counter: 0, leds },
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
        loop {
            // Heartbeat: blink the green LED once per tick.
            cx.local.leds[LED_GREEN].toggle();

            let mut msg: String<64> = String::new();

            write!(
                &mut msg,
                "hello from RTIC on STM32H743, tick {}\r\n",
                *cx.local.counter
            )
            .ok();

            cx.shared.serial.lock(|serial| {
                let _ = serial.write(msg.as_bytes());
            });

            *cx.local.counter = cx.local.counter.wrapping_add(1);

            Mono::delay(1_000.millis()).await;
        }
    }
}