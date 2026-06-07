# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`ark-discovery` — bare-metal (`#![no_std]`) firmware targeting the **ARK FPV** board (STM32H743, Cortex-M7), built on the [RTIC 2](https://rtic.rs) async framework. Current functionality: enumerates as a USB CDC serial device, runs a **gyro-synchronous control loop off the IIM-42653 IMU's data-ready interrupt** (`src/sensors/imu.rs`, SPI1, default 1 kHz), polls the **BMP388/BMP390 barometer** (`src/sensors/baro.rs`, I2C2, 25 Hz) and the **IIS2MDC/LIS2MDL magnetometer** (`src/sensors/mag.rs`, I2C4, 50 Hz), streams their readings (logging throttled, decoupled from the loop), and emits a `tick` counter line once per second. Sensor/loop rates are configured in `src/config.rs`.

Target board: [ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller. The `stm32h743v` HAL feature and the `memory.x` layout below are chosen to match its MCU. Full pin map (sensors, LEDs, UARTs, motor outputs, ADC) is in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

**Scope (firm non-goals).** This is the firmware crate of the **discovery-\*** series — a platform for exploring *sensing, estimation, and host visualization*. It is deliberately **not a full flight stack**: **no control loops** (PID/rate/attitude), **no actuation** (motor mixing, ESC/servo outputs), **no ground-control protocol**. The fused attitude/altitude estimate is the endpoint, not a step toward actuation; don't add a control law here. Sibling crates: [`discovery-telemetry`](https://github.com/wboayue/discovery-telemetry) (the telemetry wire format firmware↔host) and `discovery-scope` (planned host viewer). NB: "gyro-synchronous control loop" below names the IMU-DRDY *sampling/processing* loop (data acquisition + fusion), not a control law.

## Branching

Do feature work on a new branch off `main` — never commit directly to `main`. Branch first (`git switch -c <name>`), then commit; land via PR.

## Keep the docs current

`README.md` (what the project does, for a reader) and this `CLAUDE.md` (how to work in it, for the next agent) are part of the deliverable, not an afterthought. **Update both in the same change as the code whenever the change is user- or contributor-visible** — a new sensor/driver/module, a new task or command, a config knob, a build/flash step, a dependency or toolchain bump, or a hard-won gotcha discovered on hardware. Don't leave them stale or defer to a follow-up.

- **`README.md`** — keep the sensor table, "Current state" bullets, roadmap, `## Layout`, and `## References` in sync. New sensor/library → add a row, a state bullet, and a References entry; finished roadmap item → move it from roadmap to current state.
- **`CLAUDE.md`** — add/extend the relevant `## Sensors — …` or subsystem section with the register/protocol facts and **what bit us** (the "plausible but frozen", interrupt-storm, axis-mismatch class of notes is the highest-value content here). Correct any statement a hardware result proves wrong, rather than layering a caveat on top.
- **`CHANGELOG.md`** — follow [Keep a Changelog](https://keepachangelog.com); versions follow [SemVer](https://semver.org). Every user- or contributor-visible change adds a bullet under an `## [Unreleased]` heading at the top, in the right group (`Added`/`Changed`/`Fixed`/`Removed`). On release, rename `[Unreleased]` to the version with the date and start a fresh empty `[Unreleased]` (see `## Releasing`).

A quick heuristic: if a reviewer reading only the diff would be surprised the docs weren't touched, touch them.

## Code style

Keep the code clean as it grows:

- **No duplication (DRY)** — factor repeated logic/constants into one shared place; don't copy-paste. The sensor drivers share register-read/write patterns — extract a helper rather than re-inlining.
- **Composable** — prefer small functions with clear inputs/outputs that combine, over large monolithic ones. Drivers expose narrow methods (`read_reg`, `configure`, `sample`) the tasks compose.
- **Single responsibility (SRP)** — each module/struct/function does one thing. Keep sensor logic in its `src/sensors/<role>.rs` driver; keep RTIC tasks thin (orchestrate, don't embed driver internals).

Concrete examples of these in the tree (reuse them; don't re-inline their patterns):
- **[`src/i2c_regs.rs`](src/i2c_regs.rs)** — generic `I2cRegs<I2C>` (bus + 7-bit address) with `read_reg`/`read_regs`/`write_reg`. `baro` and `mag` each *compose* one instead of duplicating identical I2C access code. New I2C sensors should too. (The SPI `imu` has its own access helpers — different bus, CS toggling — and stays standalone.)
- **`log_fmt(serial, format_args!(…))`** in `main.rs` — the one place that formats a line into a stack buffer and writes it to USB serial. It owns the single buffer size (`LOG_LINE_CAP`), so call sites carry no magic number. Use it (with `format_args!`) for all serial logging; don't hand-roll `String::new()`/`write!`/`write_serial`. Prefer this plain function over a wrapper macro — a sugar-only macro isn't worth the indirection.
- **Driver `bring_up(…)`** (`Baro::bring_up`, `Mag::bring_up`) — own the device's reset/settle/retry *protocol* (timing, retry counts, what "ready/latched" means), taking an injected async delay closure (`|ms| Mono::delay(ms.millis())`) so the RTIC monotonic stays in `main` while the chip quirks live in the driver. This is how tasks stay thin.
- **[`src/sensors.rs`](src/sensors.rs) is the sensor *role layer*** — the contract every sensor of a role produces, kept separate from the concrete chip. It holds (a) the **contract types**: sample structs (`ImuSample`/`BaroSample`/`MagSample`) and the *logical* config enums (`ImuOdr`/`BaroOdr`/`Oversampling`/`MagOdr`, `ConfigError`); and (b) the **role traits** `Imu`/`Baro`/`Mag` plus horizontal `Identify`/`SoftReset`. Each driver lives in a **role-named module** (`imu`/`baro`/`mag` — one impl per role per firmware) whose **struct names the part currently filling it** (`imu::Iim42653`, `baro::Bmp3xx`, `mag::Lis2mdl`); the struct `impl`s its role trait + `Identify`/`SoftReset`, and `main` calls those trait methods on the owned concrete type (so the traits must be `use`d in scope). Module `imu` and trait `Imu` don't clash — different namespaces and case — which is why the struct (not the module) carries the part name. **A logical config enum names a rate/quantity, not a register value** — the chip-specific encoding stays *private to the driver* as `odr_reg`/`osr_reg` (datasheet-cited there), so a second IMU can satisfy the same `ImuOdr` with a different encoding. The enums carry `#[allow(dead_code)]` because they enumerate the full hardware mode set while `config` selects a subset. **The swap seam is the per-role alias** `ImuDriver`/`BaroDriver`/`MagDriver` in `sensors.rs` (every consumer references the alias, never the part struct) — swapping a chip for an existing role = change the alias + add its driver module (struct `impl <Role>` + `*_reg` mapping + the identity constants the tasks read by module path, `EXPECTED_WHO_AM_I` / baro `CHIP_ID_*`). Tasks can't be generic over the role traits and don't need to be (the chip logic already lives in the trait methods) — see the comment above those aliases in `sensors.rs` for the full reasoning. `fusion` consumes the sample contract types (not drivers), so it needs no change. Genuinely new role → add a contract type + role trait here. Don't put register codes on the lifted enums.

## Always document sources

This is a hardware-bring-up project: nearly every magic number is a register address, bit field, or coefficient from a datasheet or reference driver — and they are easy to get subtly wrong (we've been bitten by hallucinated/transposed values more than once). So **always cite where a value came from**:

- **In code**, put a comment next to any non-obvious constant naming its source (datasheet section, or a reputable driver — e.g. PX4 `InvenSense_ICM42688P_registers.hpp`, Bosch `BMP3_SensorAPI`, Betaflight). See `src/sensors/imu.rs` / `src/sensors/baro.rs` for the style.
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

## Workspace & host tools (`tools/`)

The repo is a **Cargo workspace**: the firmware crate is the root package, host-side (`std`) tools live under `tools/`. The whole point is isolation — **the firmware build must never compile a host tool** (different std-ness, different target). How that's enforced, and the traps:

- **`default-members = ["."]` in the root `[workspace]`** is what keeps it clean: a bare `cargo build`, `cargo objcopy`, and the entire flash/release flow act on the firmware crate alone, for the `thumbv7em-none-eabihf` target pinned in `.cargo/config.toml`. A host tool is never in that set, so it's never cross-compiled to the MCU.
- **Never run `cargo build --workspace` here** — it ignores `default-members` and tries to build the std host tools for `thumbv7em`, which fails (no std for the target). Same for `cargo test --workspace`. Build/run a tool **explicitly and for the host triple**: `cargo run -p telem --target "$(rustc -vV | sed -n 's/^host: //p')"` (wrapped as `just telem`). The `--target <host>` is **mandatory** — without it, `.cargo/config.toml`'s `build.target` makes cargo try to build the std tool for the MCU.
- **`tools/telem`** — the binary-telemetry decoder. Opens the CDC serial port (`serialport` crate), sends `b` to switch the firmware to binary, then runs `discovery_telemetry::codec::Decoder` (streaming COBS + postcard) over the byte stream and pretty-prints each `Frame`. It depends on the **same `discovery-telemetry` crate the firmware encodes with**, so the decoder can't drift from the encoder — it doubles as the reference decoder for the wire format. The expected **1 dropped frame right after `b`** is the stale-text flush (see the `Hello`/`0x00`-flush note under "Telemetry output"), not a fault; persistent drops mean corruption or a `PROTOCOL_VERSION` skew. Closing the port drops DTR → firmware reverts to text, so no `t` needed on exit.
- **Don't hand-roll a wire decoder** (we burned time doing exactly this in a shell/Python one-off): the envelope is `Frame { t_ms: u32 (postcard varint, *first*), msg }` — forgetting the leading `t_ms` varint is what made a hand decode look like it had a mystery 4-byte prefix. Use `codec::Decoder`; it's `no_std` and already handles framing + resync.

## Releasing

Tagged releases ship a prebuilt `firmware.bin` as a GitHub release asset so a flasher needn't have the Rust toolchain. `firmware.bin` is git-ignored (`*.bin`) — the **release asset is the canonical binary** for a tag; it is *not* reproducible from a plain checkout without rebuilding. Steps (run on `main`, clean tree, at the commit you want to ship):

1. **Bump `version`** in `Cargo.toml` to match the tag (`cargo build` to refresh `Cargo.lock`), commit via PR. In the same PR, **roll `CHANGELOG.md`**: rename the `## [Unreleased]` heading to `## [x.y.z] - YYYY-MM-DD` and add a fresh empty `## [Unreleased]` above it.
2. **Build the release binary** from that exact commit — the same `objcopy` the flash flow uses:
   ```bash
   cargo objcopy --release -- -O binary firmware.bin
   ```
3. **Tag (annotated) and push:**
   ```bash
   git tag -a v0.1.0 -m "ark-discovery v0.1.0 — <one-line summary>"
   git push origin v0.1.0
   ```
4. **Compute the checksum** and put it in the notes (downloaders verify with the same command):
   ```bash
   shasum -a 256 firmware.bin
   ```
5. **Create the release and attach the binary:**
   ```bash
   gh release create v0.1.0 --title "v0.1.0 — <summary>" --notes-file <notes.md> --verify-tag
   gh release upload v0.1.0 firmware.bin
   ```
6. **Verify the asset matches the local build** (catches a stale/wrong upload):
   ```bash
   gh release download v0.1.0 -R wboayue/ark-fpv-discovery --pattern firmware.bin --output /tmp/dl.bin
   cmp /tmp/dl.bin firmware.bin && shasum -a 256 /tmp/dl.bin firmware.bin
   ```

Notes should state: it's a raw `objcopy` image (no DFU suffix), the flash command (`dfu-util -a 0 -s 0x08000000:leave -D firmware.bin`, then **tap NRST** — `:leave` auto-run is unreliable on this H7, see "Build & flash"), the SHA-256, and the honest hardware-status caveat (e.g. bench-verified / untested in flight). `gh release` commands need `-R wboayue/ark-fpv-discovery` when run outside the repo dir (e.g. from `/tmp`).

## Memory layout & clocks (the parts that bite)

- `memory.x` defines the regions the linker uses: `FLASH @ 0x08000000` (2048K), `RAM @ 0x20000000` (128K). The DFU flash address above must match `FLASH ORIGIN`.
- The HAL feature `stm32h743v` (in `Cargo.toml`) selects the chip variant; changing the MCU means changing this feature.
- USB clock: the SYSCFG/RCC setup in `init` deliberately routes USB off **HSI48** (`UsbClkSel::Hsi48`, with `hsi48_ck().expect(...)`). SYSCLK is 400 MHz. USB enumeration silently fails if this clock path is disturbed.

## RTIC structure (`src/main.rs`)

Everything lives in one `#[rtic::app]` module — there is no `main()`. Key conventions when editing:

- **`#[shared]` resources** (`usb_dev`, `serial`) are accessed only inside `.lock(|r| ...)` closures; RTIC enforces this for data-race freedom across priorities.
- **`#[local]` resources** belong to exactly one task (e.g. `counter` in `log_tick`).
- **`#[init]` local statics** (`ep_mem`, `usb_bus`) give `'static` backing storage for the USB allocator — the `usb_bus: Option<...> = None` + `.replace()` dance exists because the `SerialPort`/`UsbDevice` borrow from a bus that must outlive `init`.
- **Tasks**: `usb_irq` (`binds = OTG_FS`) pumps `usb_dev.poll` and reads the CDC RX, dispatching control bytes `r` (reboot-to-DFU), `d` (toggle diagnostics), and `b`/`t` (binary/text output — see "Telemetry output"). **`imu_drdy` (`binds = EXTI2`, `priority = 2`)** is the gyro-synchronous control loop — it fires on the IMU data-ready interrupt, reads a sample, acks it, and hands every Nth to `imu_log`. `log_tick`, `baro_sample`, `mag_sample`, and `imu_log` are `async` software tasks at the default priority (1); the high-priority `imu_drdy` never touches `serial`, so the fast loop is never blocked by USB. The `dispatchers = [FDCAN1_IT0]` list donates one interrupt vector for the priority-1 software tasks (enough while they share a priority) — add more for new priority levels.
- Timebase is SysTick via `systick_monotonic!(Mono, 1_000)` (1 kHz tick), started in `init` with the 400 MHz core clock.

## Status LEDs

Red `PE3` / green `PE4` / blue `PE5` (GPIOE), stored as an erased-pin array in `Local`. **Active-low** (confirmed on hardware): pin LOW = lit, HIGH = off. Configure with `into_push_pull_output_in_state(PinState::High)` to start off. `log_tick` blinks green as a heartbeat. Full pin map in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Sensors — IIM-42653 IMU (`src/sensors/imu.rs`)

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

## Sensors — BMP388/BMP390 barometer (`src/sensors/baro.rs`)

Second sensor. Register-level I2C2 driver, no external crate; composes the shared [`I2cRegs`](src/i2c_regs.rs) for register access. `Baro::bring_up` owns reset → settle → load calibration → configure → wait (the task just supplies an async delay). Things that bite:

- **I2C2 has NO kernel-clock trap** (unlike SPI1's PLL1_Q): I2C123 runs off `pclk1` (APB1), always live after `freeze()`. No rcc change needed.
- Pins SCL `PF1` / SDA `PF0` must be **AF4 open-drain** — use `into_alternate_open_drain::<4>()`, **not** `into_alternate::<4>()` (push-pull won't satisfy the `Pins<I2C2>` bound; it's a compile error, so at least it fails loud).
- Address `0x76`. CHIP_ID (`0x00`) = `0x50` (BMP388) or `0x60` (BMP390) — accept either.
- **PWR_CTRL (`0x1B`): `mode` is bits[5:4]** (normal = `0b11`), press_en bit0, temp_en bit1 → normal+both = **`0x33`**. NOT `0x0F` — that puts mode=`00`=sleep, and the bug is silent: in sleep the data registers return their reset default `0x800000`, which *compensates to a believable ~23 °C / ~817 hPa that never changes*. If baro readings are plausible but frozen, suspect the mode bits. (Verified the layout against Bosch `bmp3_defs.h`: `BMP3_OP_MODE_MSK 0x30`, pos 4.)
- Calibration: burst-read the 21-byte NVM block at `0x31..=0x45`, parse with the datasheet signedness (P5/P6 are u16; most other P's are i8/i16), and scale each coefficient by its power-of-two divisor (Bosch `parse_calib_data`). Compensation uses **f64** (matches the Bosch double API; only integer powers, so no `libm`).
- Data: 6 bytes from `0x04`, little-endian (XLSB/LSB/MSB), pressure then temperature.
- Sanity check on hardware: pressure ≈ 950–1030 hPa; temperature reads the sensor's *local* board temp (runs well above ambient near the H7 — ~50 °C observed), and breathing on the board swings it noticeably (toward breath temp).

## Sensors — IIS2MDC/LIS2MDL magnetometer (`src/sensors/mag.rs`)

Third sensor. Register-level I2C4 driver, no external crate; composes the shared [`I2cRegs`](src/i2c_regs.rs) for register access. IIS2MDC (ArduPilot naming) and LIS2MDL (Betaflight) are the same ST 3-axis magnetometer at `0x1E` with an identical register map. Things to know:

- **I2C4 has NO kernel-clock trap either** (like I2C2): it runs off `pclk4` (APB4, D3 domain), always live after `freeze()`. The HAL exposes it the same way — `dp.I2C4.i2c(..., ccdr.peripheral.I2C4, ...)`.
- Pins SCL `PF14` / SDA `PF15`, **AF4 open-drain** (`into_alternate_open_drain::<4>()`, same bound trap as the baro). Both are on GPIOF, already split for the IMU DRDY / baro.
- Address `0x1E`. **WHO_AM_I (`0x4F`) = `0x40`** (ST `LIS2MDL_ID`).
- **CFG_REG_A (`0x60`)** holds COMP_TEMP_EN[7], REBOOT[6], SOFT_RST[5], LP[4], ODR[3:2], MD[1:0]. Continuous mode is MD=`00`; the write value (50 Hz) is `COMP_TEMP_EN | ODR | MD_CONTINUOUS` = **`0x88`**. Soft reset = SOFT_RST bit5 alone (`0x20`); it self-clears, so we **poll** `reset_complete()` (CFG_A bit5 == 0) before configuring — a fixed delay isn't enough.
- **CFG_REG_B (`0x61`) = `0x02`**: OFF_CANC (bit1) only — offset cancellation every ODR, matching the ST example (we dropped the LPF bit to stay verbatim with it). **CFG_REG_C (`0x62`) = `0x10`**: BDU (bit4) so the output regs stay coherent across a multi-byte read.
- **The hard-won bit — the first continuous-mode write after a reset does NOT latch.** You write `0x88` to CFG_A and read back **`0x8b`**: COMP_TEMP_EN and ODR stick, but the MD bits revert to `11` (idle). The chip then sits idle — it does exactly one conversion (so the field looks like a plausible-but-frozen vector) and **temperature reads exactly `25.0 °C` (raw 0)**, the dead giveaway. A *second* CFG_A write makes continuous stick (verified: `cfgA` then holds `0x88`, `STATUS` `0x0f`, temp real). So `configure()` asserts continuous via `start_continuous()`, and **`Mag::bring_up`** (which the `mag_sample` task calls) **re-asserts + verifies with `is_continuous()`** (up to 10×) until it latches. Same "plausible but frozen" failure class as the baro's PWR_CTRL sleep bug — if mag data is believable but static and temp is pinned at 25.0, suspect the mode bits.
- Data: 6 bytes from OUTX_L (`0x68`), **little-endian** two's complement, X/Y/Z; temperature 2 bytes from `0x6E`. Scaling (ST `lis2mdl_reg.c`): **1.5 mgauss/LSB** — reported in **µT** as `raw * 0.15` (1 gauss = 100 µT); temp = `raw/8 + 25 °C`.
- **Rates in [`src/config.rs`](src/config.rs)**: `MAG_ODR` (native 10/20/50/100 Hz) + sample/log throttle. Polled like the baro (50 Hz), decoupled from logging. `data_ready()` (STATUS_REG `0x67` Zyxda bit3) is available but unused — we poll at the ODR with BDU.
- Sanity check on hardware (verified): field magnitude ≈ 25–65 µT (Earth's field; ~46 µT observed in a clean spot); a magnet or motor nearby swings it hard. Temp tracks the die (~31–38 °C observed), biased high near the H7 like the baro. **Magnitude is very sensitive to ambient hard-iron — on a metal/cluttered bench it reads 2–3× Earth's field (≈130 µT seen, steady X+Z bias, Y≈0) with no driver fault.** So when validating the mag, the *liveness* checks (data changing on all axes, temp real and ≠ exactly 25.0 °C) are the real "it works" signal; an out-of-range *magnitude* alone usually just means move the board away from metal and recheck.

## Sensor fusion (`src/fusion.rs`)

Fuses all three sensors into **attitude** (roll/pitch/yaw) via the [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs) crate and **altitude + vertical velocity** via [`fusion-altitude`](https://github.com/wboayue/fusion-altitude). `Fusion` wraps both estimators behind one `update(&ImuSample, Option<&MagSample>, Option<f32> pressure, dt) -> FusedState`; the `fusion_step` RTIC task just orchestrates (drain → update → store → log). Logs a throttled `fus roll=… pitch=… yaw=…deg alt=…m vz=…m/s` line and stores the latest `FusedState` in a `#[shared]` resource as the estimation endpoint (read by loggers / future host telemetry — a control law that consumes it is out of scope; see **Scope** above).

- **Unit match is exact** — the libraries want gyro **dps**, accel **g**, mag **µT**, which `ImuSample`/`MagSample` already produce; no conversion. `fusion-altitude` wants metres + gravity-compensated earth-frame +up m/s²: altitude comes from `pressure_to_altitude_m` (ISA formula `44330*(1-(p/p0)^(1/5.255))`, `libm::powf` since `powf` is std-only), and vertical accel from `ahrs.earth_acceleration().z * GRAVITY` (earth_acceleration is in g; `GRAVITY = 9.806_65`).
- **Delta-angle downsampling — the key architectural choice.** `imu_drdy` (1 kHz, priority 2) keeps its thin job but *also* accumulates running gyro/accel/temp **sums** into the shared `SensorState` (pure adds — no trig/nalgebra in the ISR). The `fusion_step` task (250 Hz, priority 1) drains the **mean** (`SensorState::drain_imu` → a real mean `ImuSample`) and runs the AHRS on it. This keeps full 1 kHz gyro fidelity (no aliasing) while all heavy `libm`/`nalgebra` math stays off the storm-sensitive latched-interrupt ISR. CPU cost is negligible (~0.1–0.3%); the reason to keep math out of `imu_drdy` is worst-case ISR latency / the interrupt-storm margin, not total CPU. After wiring fusion in, re-verify the DRDY counter still advances by exactly `IMU_LOG_DIV` per logged line (no storm).
- **Transport** is one `#[shared] latest: SensorState` copied in/out under `.lock()` (latest-wins, no queue). `imu_drdy` (prio 2) also touches it, so its ceiling rises to prio 2 — every critical section must be a tiny copy/adds, **never fusion math** (the math runs outside the lock). `SensorState` holds *raw* values only; all interpretation (averaging, pressure→altitude, axis remap) lives in `fusion.rs`, so the producer tasks stay fusion-agnostic.
- **9-DOF vs 6-DOF**: `config::FUSION_USE_MAG`. 9-DOF (`ahrs.update`) gives absolute yaw from the mag; 6-DOF (`ahrs.update_no_magnetometer`) drops the mag so yaw is relative and drifts. Flip to 6-DOF on a hard-iron bench to validate roll/pitch independently of bench yaw error.
- **Altitude is absolute ISA**, not re-zeroed: fixed `P0_REFERENCE = 1013.25` hPa; on the first valid baro sample `Fusion::update` *seeds* the estimator (`reset(baro_alt)`) so it starts converged at the current ISA altitude (≈145 m observed at ~996 hPa) rather than ramping from 0. Vertical velocity is a derivative, unaffected by the P0 offset — vz is the operationally meaningful signal. Set a local QNH in `P0_REFERENCE` for true MSL.
- **dt** is measured from `Mono::now().ticks()` deltas (1 kHz monotonic → ms), clamped to `[FUSION_DT_MIN_S, FUSION_DT_MAX_S]`. The 1 ms tick resolution caps the usable fusion rate (~250–333 Hz, so dt stays ≥3 ticks); a faster loop would need a finer `systick_monotonic!(Mono, …)`.
- **The altitude observer runs EVERY fusion tick, not just on fresh baro — and that bit us.** `AltitudeEstimator::update(vaccel, baro_alt, dt)` integrates over `dt` and its `dt` **must be the time since its previous call**. The first cut updated it only when a fresh baro sample arrived (~25 Hz) but passed the ~4 ms fusion-tick dt → the filter advanced at 1/10 real-time, so `vz` damped ~10× too slowly (rang for ~14 s at rest instead of settling in ~1–2 s). Fix: call `update` every tick with the tick dt, **holding the last baro altitude** (`Fusion::baro_alt_m`) between samples — high-rate accel predict + continuous baro correction, the standard complementary-filter structure. Rule of thumb: the estimator's call rate and its `dt` must agree.
- **`AltitudeSettings` is `#[non_exhaustive]`** — construct via `default()` then assign fields, not a struct literal (a literal is a compile error outside the crate). `AhrsSettings` is a plain struct (literal fine).
- **edition 2024**: this crate is edition 2024 (both fusion crates are; needs rustc ≥ 1.85). The migration made `#[link_section]` an unsafe attribute → it's now `#[unsafe(link_section = …)]` on `BOOT_FLAG`.
- **Axis/convention (`Fusion::to_body_frame`)**: `Convention::Nwu`. The ARK FPV's IIM-42653 reads accel ≈ (0,0,**−1**) g flat & level (its +Z points down), so `to_body_frame` applies a **180°-about-X** rotation `(x,−y,−z)` to gyro/accel/mag (verified on hardware — without it the AHRS reported roll ≈ 180° level). The mag is assumed co-framed with the IMU; if the yaw *heading* is wrong (roll/pitch unaffected) the mag needs its own remap. Rates/gains live in [`src/config.rs`](src/config.rs) under `// --- Fusion ---`.
- Sanity check on hardware: level at rest → roll/pitch ≈ 0 after ~1–2 s convergence, alt tracks absolute ISA height (~145 m at ~996 hPa, *not* 0), vz ≈ 0. A steadily growing vz ⇒ dt error or earth-accel sign/scale bug (at rest `earth_acceleration().z*GRAVITY` should be ≈0, **not** ≈9.81). Bench yaw may be biased by hard-iron even when correct — judge yaw by *tracking direction*, not absolute heading.
- Verified on hardware (bench, motors off): **static** — at rest vz mean ≈ 0 (unbiased), 1σ ≈ 0.056 m/s; alt 1σ ≈ 0.08 m (baro noise, ~0.02 hPa × 8.3 m/hPa). **dynamic lift test** — a ~0.45 m hand-lift gives a clean **+vz** transient (~+0.3 m/s peak) with alt rising, vz returning to ≈0 when held at the new height, and a symmetric **−vz** on lowering; well-damped, no ringing, correct sign (up = +, NWU). Both static and dynamic bench behavior are good. Still **untested: in flight** — prop vibration into the accel and prop-wash pressure into the baro dominate and a bench can't show them; foam the baro port and do a props-on/tethered capture before trusting altitude hold.

## Reboot to DFU (`r` command)

Sending `r` over the serial link reboots into the ROM bootloader so the board can be reflashed without touching BOOT0/RESET. Mechanism:

1. `reboot_to_bootloader()` writes `BOOTLOADER_MAGIC` to `BOOT_FLAG` (a `MaybeUninit<u32>` in the `.uninit` linker section, which cortex-m-rt does **not** zero, so it survives a soft reset), then `SCB::sys_reset()`.
2. After reset, `maybe_enter_bootloader()` runs as the **first line of `init`** — before any clock/peripheral setup, while clocks are at reset defaults — and if the magic is set, clears it and `cortex_m::asm::bootload(0x1FF0_9800)` into the H743 system-memory bootloader.

**Do NOT move this jump into `#[cortex_m_rt::pre_init]`.** That ran before RAM init, was unsound, and bricked the boot (no CDC, no LED, not even DFU — only BOOT0 recovered it). Checking the flag at the top of `init` is the working approach.

## Telemetry output: text / binary (`src/telemetry.rs`, `b`/`t` commands)

All serial output goes through **`mod telemetry`** — the presentation layer. The RTIC tasks stay thin: they call `log_imu` / `log_baro` / `log_mag` / `log_fused` (per-sample renderers) or `emit_line` (status/notice lines) and never hand-roll a `String`/`write!`/`serial.write` or touch the wire codec. Two runtime-selectable modes:

- **Text** (boot default, `config::DEFAULT_OUTPUT_BINARY = false`): today's human-readable `\r\n` lines. A terminal user (`screen`, `just monitor`) sees readable output with zero setup.
- **Binary**: postcard-encoded, COBS-framed [`discovery-telemetry`](https://github.com/wboayue/discovery-telemetry) `Frame`s, decoded losslessly by the host scope. Each `log_*` helper maps the sample 1:1 onto the wire payload (`ImuSample`→`wire::Imu`, etc.; note the `FusedState` field renames `vertical_velocity → vertical_speed_mps`, `baro_residual → baro_residual_m`).

The output mode is a lock-free `static OUTPUT_BINARY: AtomicBool` (mirrors `DIAG`), read via `output_is_binary()`. Control bytes in `usb_irq` (alongside `r`/`d`):

| Byte | Action |
|---|---|
| `b` | switch to **binary**; emits a `Hello` frame first (scope confirms the switch + checks `PROTOCOL_VERSION`) |
| `t` | switch to **text** (replies `text mode`) |

- **`Hello` must be preceded by a lone `0x00` flush — found on hardware.** When `b` switches mid-stream, the partial text line already sent sits *delimiter-less* in the host's COBS accumulator; the `Hello` bytes concatenate onto it and the first `0x00` (Hello's own delimiter) makes the decoder parse `[stale text + Hello]` as one frame and drop it — so the version handshake was lost on every connect. Fix: `usb_irq` writes a single `0x00` *before* the `Hello`, closing the stale text as its own discarded frame so `Hello` lands clean. (Verified: without it, 0 `Hello` + 1 drop; with it, `Hello proto=2 …` decodes; the 1–2 leading drops that remain are just the >64 B stale text overflowing the accumulator — benign.)
- **The status-message gap this closes.** The firmware's non-data lines (mode acks, sensor bring-up identity, errors like `baro: invalid ODR/OSR`) had no binary form — in a binary stream they'd be injected as raw text and dropped by the decoder. `discovery-telemetry` v0.2.0 adds `Msg::Status { level, text }`; `emit_line(serial, level, args)` renders one line as a `Status` frame (binary) or a text line (text). `PROTOCOL_VERSION` is **2**.
- **Encoding is split from the serial lock** — the hard-won bit. `usb_irq` runs *inside* `serial.lock`, and re-locking the same RTIC resource deadlocks. So the spawned tasks (which hold a `Mutex` proxy) use the locking `emit_frame`/`emit_line`/`log_*` wrappers, while `usb_irq` builds bytes with `encode_frame` and writes the **already-unlocked** port via `write_frame` directly. Never call a `&mut impl SerialMutex` helper from `usb_irq`.
- **`Hello.fw_git`** is the short git commit, injected at build time by `build.rs` (`GIT_HASH`, capped to 7 chars + NUL = `[u8; 8]`; `"unknown"` if `git` is unavailable). `Hello.board = Board::ArkDiscovery`.
- **DTR-revert:** `usb_irq` watches `serial.dtr()`; on a true→false transition (host/scope closed the port) it reverts to text, so the next terminal user gets readable lines without sending `t`. Best-effort (not every host signals on close; a re-enumeration also trips it).
- **Frame buffer:** `encode_frame` owns the one `[u8; codec::MAX_FRAME]` (64) buffer, as `log_fmt` owns `LOG_LINE_CAP`. `Status` (`text:[u8;48]`) is the largest frame (~57 B framed). **Partial USB writes are accepted as lossy:** a frame truncated by a full endpoint is dropped and the decoder resyncs at the next `0x00` (the protocol has no reliability layer — USB CDC already gives link-layer CRC + retransmit).
- After wiring binary in, **re-verify no interrupt storm**: the DRDY counter still advances by exactly `IMU_LOG_DIV` per logged sample (binary emit is all priority-1; `imu_drdy` never touches `serial`).

## Diagnostic mode (`d` command)

Sending `d` over the serial link toggles verbose sensor diagnostics at runtime. It flips a lock-free `static DIAG: AtomicBool` (in `telemetry`, read with `diag_enabled()`, no RTIC resource lock) which `usb_irq` toggles alongside the `r`/`b`/`t` handlers. The `d` **ack** itself honors the output mode: a `Status` frame in binary, the `diag on`/`diag off` text line otherwise. Two tasks honor the flag in **text mode** (the verbose dumps read live driver registers / estimator internals, so they're text-only and stay in the task; binary mode always emits the plain data frame):
- **`mag_sample`** — off → concise `mag field[uT]=… temp=…C`; on → `mag[diag] id=… cfgA=… B=… C=… field=… temp=… status=…` register dump. This is how the IIS2MDC continuous-mode-latch bug above was diagnosed on hardware without reflashing per probe.
- **`fusion_step`** — off → concise `fus roll=… pitch=… yaw=…deg alt=…m vz=…m/s`; on → `fus[diag] … resid=…m vacc=…m/s2 bias=…m/s2`, surfacing the vertical-channel signals (baro innovation `resid`, gravity-compensated vertical accel `vacc`, estimated accel `bias`) for characterizing the baro disturbance — e.g. sizing `r0` for a future adaptive-baro-trust scheme from a props-on capture.

Extensible to other tasks the same way.

## Naming note

The USB product string still reads `"RTIC STM32H743 USB Serial"` and old `h753-usb-serial` naming may appear in stray references — the canonical binary/crate name is `ark-discovery`. Treat the H753 references as stale.
