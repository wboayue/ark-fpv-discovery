# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`ark-discovery` — bare-metal (`#![no_std]`) firmware targeting the **ARK FPV** board (STM32H743, Cortex-M7), built on the [RTIC 2](https://rtic.rs) async framework. Current functionality: enumerates as a USB CDC serial device and emits a `tick` counter line once per second.

Target board: [ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller. The `stm32h743v` HAL feature and the `memory.x` layout below are chosen to match its MCU. Full pin map (sensors, LEDs, UARTs, motor outputs, ADC) is in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Build & flash

Target and linker args are fixed in `.cargo/config.toml` (`thumbv7em-none-eabihf`, `-Tlink.x`), so plain cargo works:

```bash
cargo build              # debug
cargo build --release    # release (use this for flashing — smaller, faster)
```

Produce a raw binary and flash over USB DFU:

```bash
cargo objcopy --release -- -O binary firmware.bin   # needs cargo-binutils + llvm-tools
printf 'r' > /dev/cu.usbmodem*                       # reboot running firmware into DFU (~2s)
dfu-util -l                                          # confirm 0483:df11 is visible (2 "Found DFU")
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin    # flash to FLASH origin, then leave
```

Getting **into** DFU — two ways:
- **`r` over the serial port** (preferred): the firmware reboots itself into the ROM bootloader. See "Reboot to DFU" below.
- **BOOT0 + RESET** (manual): always works, even when firmware is hung. Use this for the first flash of any DFU-capable build, or to recover a bricked boot.

Gotchas, all learned the hard way in this codebase:
- **`:leave` auto-run is unreliable on this H7 ROM bootloader.** It often does a bare jump-to-app instead of a clean reset, so the app starts on the bootloader's leftover state and hangs in `init` (no CDC, no LED). Fix: press **NRST** manually — it always boots cleanly. Not a firmware bug. For truly buttonless flash+run, use **SWD via `probe-rs`/`cargo flash`** instead of DFU.
- `dfu-util: Error during download get_status` on the leave request (exit 74) is **benign** — the chip resets before dfu-util reads final status. `File downloaded successfully` means the write worked.
- `dfu-util: Warning: Invalid DFU suffix signature` is **benign** — `firmware.bin` is a raw `objcopy` image with no optional 16-byte DFU suffix (CRC + VID/PID), so the only thing skipped is a device-match check we don't need (we target `-a 0 -s 0x08000000` explicitly). The bytes flashed are exact. A future dfu-util may *require* the suffix; if so, append one with `dfu-suffix -v 0x0483 -p 0xdf11 -a firmware.bin`.
- macOS re-enumerates the CDC port slowly after a reset (up to ~15s); poll, don't assume failure.

`firmware.bin` is a checked-in build output but is git-ignored (`*.bin`); regenerate rather than trust the committed copy.

There are no tests — `#![no_std]` firmware has no host test harness. The verification loop is: build, flash, then read the serial port (`stty -f <tty> 115200 raw -echo` then read it; pyserial is **not** installed) and confirm `hello from RTIC ... tick N` streams once/sec.

## Memory layout & clocks (the parts that bite)

- `memory.x` defines the regions the linker uses: `FLASH @ 0x08000000` (2048K), `RAM @ 0x20000000` (128K). The DFU flash address above must match `FLASH ORIGIN`.
- The HAL feature `stm32h743v` (in `Cargo.toml`) selects the chip variant; changing the MCU means changing this feature.
- USB clock: the SYSCFG/RCC setup in `init` deliberately routes USB off **HSI48** (`UsbClkSel::Hsi48`, with `hsi48_ck().expect(...)`). SYSCLK is 400 MHz. USB enumeration silently fails if this clock path is disturbed.

## RTIC structure (`src/main.rs`)

Everything lives in one `#[rtic::app]` module — there is no `main()`. Key conventions when editing:

- **`#[shared]` resources** (`usb_dev`, `serial`) are accessed only inside `.lock(|r| ...)` closures; RTIC enforces this for data-race freedom across priorities.
- **`#[local]` resources** belong to exactly one task (e.g. `counter` in `log_tick`).
- **`#[init]` local statics** (`ep_mem`, `usb_bus`) give `'static` backing storage for the USB allocator — the `usb_bus: Option<...> = None` + `.replace()` dance exists because the `SerialPort`/`UsbDevice` borrow from a bus that must outlive `init`.
- **Tasks**: `usb_irq` is hardware-bound (`binds = OTG_FS`) — it pumps `usb_dev.poll` and reads the CDC RX, where a `b'r'` byte triggers the reboot-to-DFU. `log_tick` is an `async` software task that loops with `Mono::delay(...).await` and toggles the green LED each tick. The `dispatchers = [FDCAN1_IT0]` list donates an unused interrupt vector for RTIC to run software tasks — add more dispatchers if you add more software-task priority levels.
- Timebase is SysTick via `systick_monotonic!(Mono, 1_000)` (1 kHz tick), started in `init` with the 400 MHz core clock.

## Status LEDs

Red `PE3` / green `PE4` / blue `PE5` (GPIOE), stored as an erased-pin array in `Local`. **Active-low** (confirmed on hardware): pin LOW = lit, HIGH = off. Configure with `into_push_pull_output_in_state(PinState::High)` to start off. `log_tick` blinks green as a heartbeat. Full pin map in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Reboot to DFU (`r` command)

Sending `r` over the serial link reboots into the ROM bootloader so the board can be reflashed without touching BOOT0/RESET. Mechanism:

1. `reboot_to_bootloader()` writes `BOOTLOADER_MAGIC` to `BOOT_FLAG` (a `MaybeUninit<u32>` in the `.uninit` linker section, which cortex-m-rt does **not** zero, so it survives a soft reset), then `SCB::sys_reset()`.
2. After reset, `maybe_enter_bootloader()` runs as the **first line of `init`** — before any clock/peripheral setup, while clocks are at reset defaults — and if the magic is set, clears it and `cortex_m::asm::bootload(0x1FF0_9800)` into the H743 system-memory bootloader.

**Do NOT move this jump into `#[cortex_m_rt::pre_init]`.** That ran before RAM init, was unsound, and bricked the boot (no CDC, no LED, not even DFU — only BOOT0 recovered it). Checking the flag at the top of `init` is the working approach.

## Naming note

The USB product string still reads `"RTIC STM32H743 USB Serial"` and old `h753-usb-serial` naming may appear in stray references — the canonical binary/crate name is `ark-discovery`. Treat the H753 references as stale.
