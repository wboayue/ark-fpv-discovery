# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`ark-discovery` — bare-metal (`#![no_std]`) firmware targeting the **ARK FPV** board (STM32H743, Cortex-M7), built on the [RTIC 2](https://rtic.rs) async framework. Current functionality: enumerates as a USB CDC serial device and emits a `tick` counter line once per second.

Target board: [ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller. The `stm32h743v` HAL feature and the `memory.x` layout below are chosen to match its MCU.

## Build & flash

Target and linker args are fixed in `.cargo/config.toml` (`thumbv7em-none-eabihf`, `-Tlink.x`), so plain cargo works:

```bash
cargo build              # debug
cargo build --release    # release (use this for flashing — smaller, faster)
```

Produce a raw binary and flash over USB DFU (board in DFU/bootloader mode):

```bash
cargo objcopy --release -- -O binary firmware.bin   # needs cargo-binutils + llvm-tools
dfu-util -l                                          # confirm device is visible
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin    # flash to FLASH origin, then run
```

`firmware.bin` is a checked-in build output but is git-ignored (`*.bin`); regenerate rather than trust the committed copy.

There are no tests — `#![no_std]` firmware has no host test harness. "Does it build" + on-device serial output is the verification loop.

## Memory layout & clocks (the parts that bite)

- `memory.x` defines the regions the linker uses: `FLASH @ 0x08000000` (2048K), `RAM @ 0x20000000` (128K). The DFU flash address above must match `FLASH ORIGIN`.
- The HAL feature `stm32h743v` (in `Cargo.toml`) selects the chip variant; changing the MCU means changing this feature.
- USB clock: the SYSCFG/RCC setup in `init` deliberately routes USB off **HSI48** (`UsbClkSel::Hsi48`, with `hsi48_ck().expect(...)`). SYSCLK is 400 MHz. USB enumeration silently fails if this clock path is disturbed.

## RTIC structure (`src/main.rs`)

Everything lives in one `#[rtic::app]` module — there is no `main()`. Key conventions when editing:

- **`#[shared]` resources** (`usb_dev`, `serial`) are accessed only inside `.lock(|r| ...)` closures; RTIC enforces this for data-race freedom across priorities.
- **`#[local]` resources** belong to exactly one task (e.g. `counter` in `log_tick`).
- **`#[init]` local statics** (`ep_mem`, `usb_bus`) give `'static` backing storage for the USB allocator — the `usb_bus: Option<...> = None` + `.replace()` dance exists because the `SerialPort`/`UsbDevice` borrow from a bus that must outlive `init`.
- **Tasks**: `usb_irq` is hardware-bound (`binds = OTG_FS`) and just pumps `usb_dev.poll`. `log_tick` is an `async` software task that loops with `Mono::delay(...).await`. The `dispatchers = [FDCAN1_IT0]` list donates an unused interrupt vector for RTIC to run software tasks — add more dispatchers if you add more software-task priority levels.
- Timebase is SysTick via `systick_monotonic!(Mono, 1_000)` (1 kHz tick), started in `init` with the 400 MHz core clock.

## Naming note

The USB product string still reads `"RTIC STM32H743 USB Serial"` and old `h753-usb-serial` naming may appear in stray references — the canonical binary/crate name is `ark-discovery`. Treat the H753 references as stale.
