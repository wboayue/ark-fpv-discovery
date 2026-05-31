#![no_std]
#![no_main]

use panic_halt as _;

use heapless::String;
use stm32h7xx_hal as hal;

use core::fmt::Write;

use hal::{
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

#[rtic::app(device = stm32h7xx_hal::pac, peripherals = true, dispatchers = [FDCAN1_IT0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBus<USB2>>,
        serial: SerialPort<'static, UsbBus<USB2>>,
    }

    #[local]
    struct Local {
        counter: u32,
    }

    #[init(local = [
        ep_mem: [u32; 1024] = [0; 1024],
        usb_bus: Option<UsbBusAllocator<UsbBus<USB2>>> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
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
            Local { counter: 0 },
        )
    }

    #[task(binds = OTG_FS, shared = [usb_dev, serial])]
    fn usb_irq(mut cx: usb_irq::Context) {
        cx.shared.usb_dev.lock(|usb_dev| {
            cx.shared.serial.lock(|serial| {
                usb_dev.poll(&mut [serial]);
            });
        });
    }

    #[task(shared = [serial], local = [counter])]
    async fn log_tick(mut cx: log_tick::Context) {
        loop {
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