# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`ark-discovery` — bare-metal (`#![no_std]`) firmware targeting the **ARK FPV** board (STM32H743, Cortex-M7), built on the [RTIC 2](https://rtic.rs) async framework. Current functionality: enumerates as a USB CDC serial device, runs a **gyro-synchronous control loop off the IIM-42653 IMU's data-ready interrupt** (`src/imu.rs`, SPI1, default 1 kHz) and polls the **BMP388/BMP390 barometer** (`src/baro.rs`, I2C2, 25 Hz), streams their readings (logging throttled, decoupled from the loop), and emits a `tick` counter line once per second. Sensor/loop rates are configured in `src/config.rs`.

Target board: [ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller. The `stm32h743v` HAL feature and the `memory.x` layout below are chosen to match its MCU. Full pin map (sensors, LEDs, UARTs, motor outputs, ADC) is in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Always document sources

This is a hardware-bring-up project: nearly every magic number is a register address, bit field, or coefficient from a datasheet or reference driver — and they are easy to get subtly wrong (we've been bitten by hallucinated/transposed values more than once). So **always cite where a value came from**:

- **In code**, put a comment next to any non-obvious constant naming its source (datasheet section, or a reputable driver — e.g. PX4 `InvenSense_ICM42688P_registers.hpp`, Bosch `BMP3_SensorAPI`, Betaflight). See `src/imu.rs` / `src/baro.rs` for the style.
- **In the README `## References` section**, list the authoritative source per sensor/subsystem, with a link.
- **Vendor the datasheet** into [`docs/datasheets/`](docs/datasheets/) when the PDF is freely downloadable; link it when it's gated.
- **Prefer primary sources** (datasheet, vendor reference driver) over forum posts or a model's recollection, and when sources disagree, note which you trusted and why. Verify a flagged value before flashing.

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
- **Tasks**: `usb_irq` (`binds = OTG_FS`) pumps `usb_dev.poll` and reads the CDC RX, where a `b'r'` byte triggers reboot-to-DFU. **`imu_drdy` (`binds = EXTI2`, `priority = 2`)** is the gyro-synchronous control loop — it fires on the IMU data-ready interrupt, reads a sample, acks it, and hands every Nth to `imu_log`. `log_tick`, `baro_sample`, and `imu_log` are `async` software tasks at the default priority (1); the high-priority `imu_drdy` never touches `serial`, so the fast loop is never blocked by USB. The `dispatchers = [FDCAN1_IT0]` list donates one interrupt vector for the priority-1 software tasks (enough while they share a priority) — add more for new priority levels.
- Timebase is SysTick via `systick_monotonic!(Mono, 1_000)` (1 kHz tick), started in `init` with the 400 MHz core clock.

## Status LEDs

Red `PE3` / green `PE4` / blue `PE5` (GPIOE), stored as an erased-pin array in `Local`. **Active-low** (confirmed on hardware): pin LOW = lit, HIGH = off. Configure with `into_push_pull_output_in_state(PinState::High)` to start off. `log_tick` blinks green as a heartbeat. Full pin map in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Sensors — IIM-42653 IMU (`src/imu.rs`)

First sensor brought up. Register-level driver, no external crate; interrupt-driven (see the control-loop subsection below). Things that bite:

- **SPI1 needs `pll1_q_ck` enabled in the rcc chain** (`.sys_ck(400.MHz()).pll1_q_ck(80.MHz())`). PLL1_Q is SPI1's kernel clock and the HAL `.expect()`s it when building the SPI — without it `init` **panics** (which looks exactly like a hung boot: no CDC). Independent of the HSI48 USB path.
- Pins: SCK `PA5` / MISO `PG9` / MOSI `PB5`, all **AF5** (`into_alternate::<5>()`); soft CS `PI9` driven as GPIO (active-low). **MODE_3** (CPOL=1, CPHA=1), ~8 MHz (24 MHz max). Reads use `reg | 0x80`.
- **WHO_AM_I (`0x75`) = `0x56`** for the IIM-42653 — *not* the ICM-42688's `0x47`. `EXPECTED_WHO_AM_I` in `imu.rs`.
- **PWR_MGMT0 (`0x4E`) = `0x0F`**: GYRO_MODE bits[3:2] + ACCEL_MODE bits[1:0], both `0b11` = Low-Noise.
- **The IIM-42653 is the wide-range part (±32g / ±4000 dps), so its FS_SEL table is shifted up one step vs the ICM-42688**: `FS_SEL=000` = max range here. We select `FS_SEL=001` → ±16g / ±2000 dps, giving the standard 2048 LSB/g and 16.384 LSB/dps. Scaling constants depend on the selected range — change the range, change the constants.
- Data is big-endian; burst-read `0x1D..=0x2A` (TEMP, ACCEL XYZ, GYRO XYZ) in one transaction. Soft-reset (DEVICE_CONFIG `0x11` = `0x01`) on startup gives a known state across our frequent reboots; wait ~2 ms after (a `cortex_m::asm::delay` busy-wait in `init`), then ~50 ms for the gyro to start (the DRDY just won't fire until it has).
- Sanity check on hardware: accel vector magnitude ≈ 1 g at rest, gyro ≈ 0 dps.

### Control-loop data path (DRDY interrupt) — the part that bit hardest

The IMU drives a gyro-synchronous loop off its data-ready interrupt (INT1 → `PF2` → EXTI line 2 → RTIC `binds = EXTI2`). `configure_control_mode(odr)` (bank 0) sets FS+ODR, the UI filter bandwidth (`GYRO_ACCEL_CONFIG0` ≈ ODR/4; the AAF stays at its enabled default), routes UI-DRDY to INT1, then powers on. Verified register values (against PX4 `InvenSense_ICM42688P_registers.hpp`):

- **INT_SOURCE0 (`0x65`) = `0x08`** routes UI data-ready to INT1 (bit3). **INT_CONFIG1 (`0x64`) = `0x00`** clears the default `INT_ASYNC_RESET` (bit4) — datasheet-mandated.
- **INT_CONFIG (`0x14`) = `0x07` — LATCHED, push-pull, active-high.** This is the hard-won bit: pulsed mode (`0x03`) ran clean ~1 kHz *most* of the time but intermittently glitched into multi-second **interrupt storms** (the brief edges rang/coupled, and once edges outpaced the ~37 µs ISR it self-sustained at ISR rate ≈ 27 kHz, starving the logger). **Latched** mode holds INT1 until `INT_STATUS` (`0x2D`) is read, giving exactly one clean edge per sample. So the `imu_drdy` ISR **must** read INT_STATUS each time (`imu.clear_interrupt()`) or INT1 stays asserted and no further edge fires.
- **REG_BANK_SEL bank 1/2 = `0x01`/`0x02`** (not `0x10`/`0x20` — a common mis-statement). We don't bank-switch yet (AAF left at default).
- EXTI on the STM32 side: `ExtiPin` trait (`make_interrupt_source`/`trigger_on_edge(Rising)`/`enable_interrupt`/`clear_interrupt_pending_bit`); `dp.SYSCFG`/`dp.EXTI` stay owned after `freeze()`. PF2 is `into_floating_input().erase()` → `ErasedPin<Input>`.
- **Rates live in [`src/config.rs`](src/config.rs)**: IMU ODR (= the gyro-sync loop rate; native 200/500/1000 Hz) + log throttle; baro ODR/OSR/sample/log. Controls sizing — quad/VTOL rate loop ≥400 Hz, gyro ≥1 kHz anti-aliased; baro ~25 Hz. Logging is decoupled (every Nth sample → `imu_log`) so the loop rate isn't capped by USB.
- Verify on hardware: log the DRDY counter `n` — every logged line should advance by exactly `IMU_LOG_DIV` (no big jumps = no storms); net Δn/sec ≈ ODR.
- Known minor: a brief interrupt burst can occur at startup before the gyro stabilizes (EXTI is enabled in `init` before the ~50 ms gyro start). Steady state is clean; gating the EXTI enable on gyro-ready is a future hardening step.

## Sensors — BMP388/BMP390 barometer (`src/baro.rs`)

Second sensor. Inline I2C2 driver, no external crate. Things that bite:

- **I2C2 has NO kernel-clock trap** (unlike SPI1's PLL1_Q): I2C123 runs off `pclk1` (APB1), always live after `freeze()`. No rcc change needed.
- Pins SCL `PF1` / SDA `PF0` must be **AF4 open-drain** — use `into_alternate_open_drain::<4>()`, **not** `into_alternate::<4>()` (push-pull won't satisfy the `Pins<I2C2>` bound; it's a compile error, so at least it fails loud).
- Address `0x76`. CHIP_ID (`0x00`) = `0x50` (BMP388) or `0x60` (BMP390) — accept either.
- **PWR_CTRL (`0x1B`): `mode` is bits[5:4]** (normal = `0b11`), press_en bit0, temp_en bit1 → normal+both = **`0x33`**. NOT `0x0F` — that puts mode=`00`=sleep, and the bug is silent: in sleep the data registers return their reset default `0x800000`, which *compensates to a believable ~23 °C / ~817 hPa that never changes*. If baro readings are plausible but frozen, suspect the mode bits. (Verified the layout against Bosch `bmp3_defs.h`: `BMP3_OP_MODE_MSK 0x30`, pos 4.)
- Calibration: burst-read the 21-byte NVM block at `0x31..=0x45`, parse with the datasheet signedness (P5/P6 are u16; most other P's are i8/i16), and scale each coefficient by its power-of-two divisor (Bosch `parse_calib_data`). Compensation uses **f64** (matches the Bosch double API; only integer powers, so no `libm`).
- Data: 6 bytes from `0x04`, little-endian (XLSB/LSB/MSB), pressure then temperature.
- Sanity check on hardware: pressure ≈ 950–1030 hPa; temperature reads the sensor's *local* board temp (runs well above ambient near the H7 — ~50 °C observed), and breathing on the board swings it noticeably (toward breath temp).

## Reboot to DFU (`r` command)

Sending `r` over the serial link reboots into the ROM bootloader so the board can be reflashed without touching BOOT0/RESET. Mechanism:

1. `reboot_to_bootloader()` writes `BOOTLOADER_MAGIC` to `BOOT_FLAG` (a `MaybeUninit<u32>` in the `.uninit` linker section, which cortex-m-rt does **not** zero, so it survives a soft reset), then `SCB::sys_reset()`.
2. After reset, `maybe_enter_bootloader()` runs as the **first line of `init`** — before any clock/peripheral setup, while clocks are at reset defaults — and if the magic is set, clears it and `cortex_m::asm::bootload(0x1FF0_9800)` into the H743 system-memory bootloader.

**Do NOT move this jump into `#[cortex_m_rt::pre_init]`.** That ran before RAM init, was unsound, and bricked the boot (no CDC, no LED, not even DFU — only BOOT0 recovered it). Checking the flag at the top of `init` is the working approach.

## Naming note

The USB product string still reads `"RTIC STM32H743 USB Serial"` and old `h753-usb-serial` naming may appear in stray references — the canonical binary/crate name is `ark-discovery`. Treat the H753 references as stale.
