# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`ark-discovery` — bare-metal (`#![no_std]`) firmware targeting the **ARK FPV** board (STM32H743, Cortex-M7), built on the [RTIC 2](https://rtic.rs) async framework. Current functionality: enumerates as a USB CDC serial device, reads the **IIM-42653 IMU over SPI1** (`src/imu.rs`) and streams scaled accel/gyro/temp at 10 Hz, and emits a `tick` counter line once per second.

Target board: [ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller. The `stm32h743v` HAL feature and the `memory.x` layout below are chosen to match its MCU. Full pin map (sensors, LEDs, UARTs, motor outputs, ADC) is in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Build & flash

Common flows are wrapped in a `justfile` — `just dfu flash` (reboot to DFU, then flash), `just monitor` (read serial), `just reboot`, `just --list` for all. The raw commands below explain what each step does and remain the source of truth for the gotchas.

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
- **Tasks**: `usb_irq` is hardware-bound (`binds = OTG_FS`) — it pumps `usb_dev.poll` and reads the CDC RX, where a `b'r'` byte triggers the reboot-to-DFU. `log_tick` and `imu_sample` are `async` software tasks that loop with `Mono::delay(...).await`; `log_tick` cycles the LEDs and emits the tick line, `imu_sample` reads the IMU at 10 Hz. Both share `serial`. The `dispatchers = [FDCAN1_IT0]` list donates an unused interrupt vector for RTIC to run software tasks (one dispatcher is enough while they all sit at the same priority) — add more dispatchers if you add more software-task priority levels.
- Timebase is SysTick via `systick_monotonic!(Mono, 1_000)` (1 kHz tick), started in `init` with the 400 MHz core clock.

## Status LEDs

Red `PE3` / green `PE4` / blue `PE5` (GPIOE), stored as an erased-pin array in `Local`. **Active-low** (confirmed on hardware): pin LOW = lit, HIGH = off. Configure with `into_push_pull_output_in_state(PinState::High)` to start off. `log_tick` blinks green as a heartbeat. Full pin map in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Sensors — IIM-42653 IMU (`src/imu.rs`)

First sensor brought up. Polled register-level driver, no external crate. Things that bite:

- **SPI1 needs `pll1_q_ck` enabled in the rcc chain** (`.sys_ck(400.MHz()).pll1_q_ck(80.MHz())`). PLL1_Q is SPI1's kernel clock and the HAL `.expect()`s it when building the SPI — without it `init` **panics** (which looks exactly like a hung boot: no CDC). Independent of the HSI48 USB path.
- Pins: SCK `PA5` / MISO `PG9` / MOSI `PB5`, all **AF5** (`into_alternate::<5>()`); soft CS `PI9` driven as GPIO (active-low). **MODE_3** (CPOL=1, CPHA=1), ~8 MHz (24 MHz max). Reads use `reg | 0x80`.
- **WHO_AM_I (`0x75`) = `0x56`** for the IIM-42653 — *not* the ICM-42688's `0x47`. `EXPECTED_WHO_AM_I` in `imu.rs`.
- **PWR_MGMT0 (`0x4E`) = `0x0F`**: GYRO_MODE bits[3:2] + ACCEL_MODE bits[1:0], both `0b11` = Low-Noise.
- **The IIM-42653 is the wide-range part (±32g / ±4000 dps), so its FS_SEL table is shifted up one step vs the ICM-42688**: `FS_SEL=000` = max range here. We select `FS_SEL=001` → ±16g / ±2000 dps, giving the standard 2048 LSB/g and 16.384 LSB/dps. Scaling constants depend on the selected range — change the range, change the constants.
- Data is big-endian; burst-read `0x1D..=0x2A` (TEMP, ACCEL XYZ, GYRO XYZ) in one transaction. Soft-reset (DEVICE_CONFIG `0x11` = `0x01`) on startup gives a known state across our frequent reboots; wait ~2 ms after, then ~50 ms after `configure()` for the gyro to start.
- DRDY (`PF2`, EXTI) is **not** used — `imu_sample` just polls at 10 Hz. Wiring it up is a future step.
- Sanity check on hardware: accel vector magnitude ≈ 1 g at rest, gyro ≈ 0 dps.

## Reboot to DFU (`r` command)

Sending `r` over the serial link reboots into the ROM bootloader so the board can be reflashed without touching BOOT0/RESET. Mechanism:

1. `reboot_to_bootloader()` writes `BOOTLOADER_MAGIC` to `BOOT_FLAG` (a `MaybeUninit<u32>` in the `.uninit` linker section, which cortex-m-rt does **not** zero, so it survives a soft reset), then `SCB::sys_reset()`.
2. After reset, `maybe_enter_bootloader()` runs as the **first line of `init`** — before any clock/peripheral setup, while clocks are at reset defaults — and if the magic is set, clears it and `cortex_m::asm::bootload(0x1FF0_9800)` into the H743 system-memory bootloader.

**Do NOT move this jump into `#[cortex_m_rt::pre_init]`.** That ran before RAM init, was unsound, and bricked the boot (no CDC, no LED, not even DFU — only BOOT0 recovered it). Checking the flag at the top of `init` is the working approach.

## Naming note

The USB product string still reads `"RTIC STM32H743 USB Serial"` and old `h753-usb-serial` naming may appear in stray references — the canonical binary/crate name is `ark-discovery`. Treat the H753 references as stale.
