# ark-discovery

A discovery project for **reading and processing sensor data** from the
[ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller (STM32H743, Cortex-M7) —
bare-metal (`#![no_std]`) Rust firmware on the [RTIC 2](https://rtic.rs) async framework.

The aim is to bring up each onboard sensor in turn, stream its readings out over USB, and fuse
them into attitude and altitude estimates. The board carries:

| Sensor | Part | Bus |
| --- | --- | --- |
| IMU (gyro + accel) | IIM-42653 | SPI1 (CS `PI9`, DRDY `PF2`) |
| Barometer | BMP388/BMP390 | I2C2 @ `0x76` |
| Magnetometer | IIS2MDC / LIS2MDL | I2C4 @ `0x1E` |

Full pin map in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Current state

The transport, dev loop, all three onboard sensors, and sensor fusion are up.

- Enumerates as a USB CDC serial device; readings stream out over it.
- **IIM-42653 IMU on SPI1** (`src/imu.rs`): SPI1 (SCK `PA5` / MISO `PG9` / MOSI `PB5`, soft CS
  `PI9`, MODE_3), WHO_AM_I `0x56`, ±16g / ±2000 dps, anti-alias + UI filtering. Driven by its
  **hardware data-ready interrupt** (INT1 → `PF2` → EXTI) for a **gyro-synchronous control loop**
  at the configured ODR (default **1 kHz**) — low-jitter, deterministic. Serial logging is
  decoupled (every Nth sample) so USB never gates the loop.
- **BMP388/BMP390 barometer on I2C2** (`src/baro.rs`): I2C2 (SCL `PF1` / SDA `PF0`, AF4 open-drain)
  at `0x76`, CHIP_ID-checked (`0x50`/`0x60`), normal-mode, factory NVM calibration + Bosch float
  compensation. Polled at 25 Hz (low-bandwidth — it can't and needn't match the IMU rate).
- **IIS2MDC/LIS2MDL magnetometer on I2C4** (`src/mag.rs`): I2C4 (SCL `PF14` / SDA `PF15`, AF4
  open-drain) at `0x1E`, WHO_AM_I-checked (`0x40`), continuous-conversion mode with on-chip
  temperature compensation and offset cancellation. Reports the field in µT (1.5 mgauss/LSB) plus
  die temperature. Polled at 50 Hz (low-bandwidth, like the baro). The first continuous-mode write
  after reset doesn't latch, so bring-up re-asserts and verifies (see `CLAUDE.md`).
- **Sensor fusion** (`src/fusion.rs`): fuses all three sensors into **attitude** (roll/pitch/yaw)
  via [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs) and **altitude + vertical velocity**
  via [`fusion-altitude`](https://github.com/wboayue/fusion-altitude), in a dedicated 250 Hz task.
  `imu_drdy` accumulates gyro/accel at the full 1 kHz (pure adds in the ISR); the fusion task drains
  the mean each tick (**delta-angle downsampling** — full gyro fidelity, estimator decoupled, heavy
  math off the interrupt path). 9-DOF (mag for absolute yaw) or 6-DOF, toggled in `config`. Logs a
  `fus roll=… pitch=… yaw=…deg alt=…m vz=…m/s` line and keeps the latest estimate in a shared
  resource for a future control loop. Altitude is absolute ISA height (relative to a fixed P0),
  seeded at startup so it starts converged; vertical velocity is the meaningful relative signal.
- **Rates are configurable in [`src/config.rs`](src/config.rs)** — IMU ODR / log rate, baro ODR /
  oversampling / sample rate / log rate, mag ODR / sample rate / log rate. Sized for quad/VTOL
  control: the rate loop wants ≥400 Hz
  (gyro sampled ≥1 kHz, anti-aliased), the baro only ~25 Hz. The control *law* (PID/mixer/motors)
  is not implemented yet — this provides the timely data path it will run on.
- Streams a `tick` counter line once per second and cycles the status LEDs red → green → blue
  (one per tick) as a heartbeat.
- Sending `r` over the serial link reboots into the ROM bootloader for DFU reflashing; `d`
  toggles verbose per-sensor diagnostics (register dumps) at runtime.
- **Roadmap:** the control law (PID/mixer/motor outputs) on top of the fused estimate.

## Hardware

- **MCU:** STM32H743 (selected via the `stm32h743v` HAL feature in `Cargo.toml`), SYSCLK 400 MHz.
- **Status LEDs:** red `PE3` / green `PE4` / blue `PE5`, active-low (pin LOW = lit).
- **USB:** OTG2_HS on `PA11` (DM) / `PA12` (DP), clocked off HSI48.

See [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md) for the full pin map — sensors, UARTs,
motor outputs, and ADC power monitoring.

## Prerequisites

Host tooling (developed on macOS; Linux is the same — substitute the serial-device path, e.g.
`/dev/ttyACM*`):

- **Rust toolchain** — stable `rustc ≥ 1.85` (this crate is **edition 2024**). Install via
  [rustup](https://rustup.rs).
- **Bare-metal target:** `rustup target add thumbv7em-none-eabihf` (Cortex-M7F; pinned in
  [`.cargo/config.toml`](.cargo/config.toml), so a plain `cargo build` cross-compiles).
- **`cargo-binutils` + LLVM tools** — for `cargo objcopy` (ELF → raw `firmware.bin`):
  `cargo install cargo-binutils && rustup component add llvm-tools-preview`.
- **[`dfu-util`](https://dfu-util.sourceforge.net/)** ≥ 0.9 — flashes over USB DFU
  (`brew install dfu-util`, or `apt install dfu-util`).
- **[`just`](https://github.com/casey/just)** *(optional)* — wraps the build/flash/monitor flows
  (`brew install just`); the raw commands below work without it.
- **Hardware access:** a USB cable to the board, plus the **BOOT0** and **NRST** buttons/pads —
  needed for the first flash and for recovery.
- No `pyserial` required — the serial port is read with `stty` + `cat`.

## Build

Target and linker args are fixed in `.cargo/config.toml`, so plain cargo cross-compiles:

```bash
cargo build              # debug
cargo build --release    # release — use this for flashing (smaller, faster)
```

## Flash & bring up the board

The firmware runs from FLASH origin `0x08000000`; it's flashed as a raw binary over USB DFU. With
`just` the whole flow is two commands (`just --list` shows all recipes):

```bash
just flash      # release build → objcopy → dfu-util download (board must already be in DFU)
just monitor    # read the serial port
```

The raw steps below are the source of truth for the gotchas.

### 1. Get the board into DFU mode

- **First flash of a board, or recovery — BOOT0 + NRST (always works):** hold **BOOT0**, tap
  **NRST**, release BOOT0. Use this for the very first flash (the `r` trick below needs already-
  working firmware) and any time the board is hung.
- **Reflashing — `r` over serial (no buttons):** `printf 'r' > /dev/cu.usbmodem*` — running
  firmware reboots itself into the ROM bootloader (~2 s). **Not 100 % reliable on this H7**: it
  occasionally lands in neither DFU nor CDC. If `dfu-util -l` shows nothing within ~15 s, fall back
  to **BOOT0 + NRST**.

Confirm DFU is up: `dfu-util -l` lists `0483:df11` (two "Found DFU" entries).

### 2. Flash

```bash
cargo objcopy --release -- -O binary firmware.bin   # ELF → raw image
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin    # write to FLASH origin, then leave
```

Then **tap NRST to boot cleanly.** The `:leave` auto-run is unreliable on this H7 ROM bootloader
and often leaves the app hung (no CDC, no LED) — a manual NRST always boots. macOS re-enumerates
the CDC port slowly (up to ~15 s) — poll, don't assume failure. The
`Error during download get_status` (on leave) and `Invalid DFU suffix signature` warnings are both
**benign** (`File downloaded successfully` is the line that matters).

### 3. Verify (first light)

There is no host test harness (`#![no_std]`) — verify on the board by reading the serial port:

```bash
stty -f /dev/cu.usbmodem* 115200 raw -echo
cat /dev/cu.usbmodem*
```

Expect, once the CDC port enumerates:

- `hello from RTIC on STM32H743, tick N` once per second, LEDs stepping red → green → blue in sync;
- the sensor streams — `imu[...]`, `baro press=…hPa`, `mag field[uT]=…`, and fused
  `fus roll=… pitch=… yaw=…deg alt=…m vz=…m/s`.

`r` reboots into DFU for the next flash; `d` toggles verbose per-sensor diagnostics. See
[`CLAUDE.md`](CLAUDE.md) for the deeper bring-up notes (USB clock path, reboot-to-DFU mechanism,
per-sensor quirks, fusion axis/dt gotchas).

## Layout

| Path                     | What                                             |
| ------------------------ | ------------------------------------------------ |
| `src/main.rs`            | The `#[rtic::app]` module — init, tasks, peripheral wiring |
| `src/imu.rs`             | IIM-42653 IMU driver (SPI1)                       |
| `src/baro.rs`            | BMP388/BMP390 barometer driver (I2C2)             |
| `src/mag.rs`             | IIS2MDC/LIS2MDL magnetometer driver (I2C4)        |
| `src/fusion.rs`          | Sensor fusion: attitude (fusion-ahrs) + altitude (fusion-altitude) |
| `memory.x`               | Linker regions: FLASH @ `0x08000000`, RAM @ `0x20000000` |
| `.cargo/config.toml`     | Target + linker args                             |
| `docs/ark-fpv-board.md`  | ARK FPV pin map                                  |
| `CLAUDE.md`              | Deeper notes on RTIC structure, clocks, flashing gotchas |

See [`CLAUDE.md`](CLAUDE.md) for the hard-won details (USB clock path, reboot-to-DFU mechanism,
`pre_init` pitfall, RTIC resource conventions).

## References

Datasheets and reference drivers the sensor code is built from. Local PDF copies live in
[`docs/datasheets/`](docs/datasheets/).

- **Barometer (BMP388/BMP390)** — Bosch datasheets
  [`docs/datasheets/bmp388-ds001.pdf`](docs/datasheets/bmp388-ds001.pdf) and
  [`docs/datasheets/bmp390-ds002.pdf`](docs/datasheets/bmp390-ds002.pdf) (register map, CHIP_ID,
  PWR_CTRL/OSR bit fields, ODR/OSR timing), and the Bosch
  [BMP3_SensorAPI](https://github.com/boschsensortec/BMP3_SensorAPI) — source of the NVM
  calibration scaling and float compensation in `src/baro.rs`.
- **IMU (IIM-42653)** — TDK
  [IIM-42653 product page](https://invensense.tdk.com/products/smartindustrial/iim-42653/) (the
  datasheet PDF is gated, so it isn't vendored), the register-identical PX4
  [`InvenSense_ICM42688P_registers.hpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/imu/invensense/icm42688p/InvenSense_ICM42688P_registers.hpp)
  (source of the verified INT/ODR/filter register values), and Betaflight
  [`accgyro_mpu.h`](https://github.com/betaflight/betaflight/blob/master/src/main/drivers/accgyro/accgyro_mpu.h)
  (WHO_AM_I `0x56`).
- **Magnetometer (IIS2MDC/LIS2MDL)** — ST
  [LIS2MDL datasheet](https://www.st.com/resource/en/datasheet/lis2mdl.pdf) (the PDF is gated, so
  it isn't vendored) and ST's official driver
  [`lis2mdl-pid`](https://github.com/STMicroelectronics/lis2mdl-pid)
  (`lis2mdl_reg.h`/`lis2mdl_reg.c`) — source of the register map, CFG bit fields, WHO_AM_I `0x40`,
  and the LSB scaling (1.5 mgauss/LSB; temp `lsb/8 + 25 °C`) in `src/mag.rs`. IIS2MDC (ArduPilot)
  and LIS2MDL (Betaflight) are the same part at `0x1E`.
- **Sensor fusion** — [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs) (attitude; a Rust
  port of xioTechnologies' Fusion AHRS — gain/rejection settings follow its canonical example) and
  [`fusion-altitude`](https://github.com/wboayue/fusion-altitude) (a 3rd-order complementary
  observer for altitude + vertical velocity). The pressure→altitude step in `src/fusion.rs` uses the
  ISA/NOAA hypsometric formula `44330 * (1 - (p/p0)^(1/5.255))` (constants per the Bosch BMP3
  examples).
