# ark-discovery

A discovery project for **reading and processing sensor data** from the
[ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller (STM32H743, Cortex-M7) —
bare-metal (`#![no_std]`) Rust firmware on the [RTIC 2](https://rtic.rs) async framework.

The aim is to bring up each onboard sensor in turn, stream its readings out over USB, and build
toward fusing them. The board carries:

| Sensor | Part | Bus |
| --- | --- | --- |
| IMU (gyro + accel) | IIM-42653 | SPI1 (CS `PI9`, DRDY `PF2`) |
| Barometer | BMP388/BMP390 | I2C2 @ `0x76` |
| Magnetometer | IIS2MDC / LIS2MDL | I2C4 @ `0x1E` |

Full pin map in [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md).

## Current state

The transport, dev loop, and the first two sensors are up.

- Enumerates as a USB CDC serial device; readings stream out over it.
- **IIM-42653 IMU on SPI1** (`src/imu.rs`): SPI1 (SCK `PA5` / MISO `PG9` / MOSI `PB5`, soft CS
  `PI9`, MODE_3), WHO_AM_I `0x56`, ±16g / ±2000 dps, anti-alias + UI filtering. Driven by its
  **hardware data-ready interrupt** (INT1 → `PF2` → EXTI) for a **gyro-synchronous control loop**
  at the configured ODR (default **1 kHz**) — low-jitter, deterministic. Serial logging is
  decoupled (every Nth sample) so USB never gates the loop.
- **BMP388/BMP390 barometer on I2C2** (`src/baro.rs`): I2C2 (SCL `PF1` / SDA `PF0`, AF4 open-drain)
  at `0x76`, CHIP_ID-checked (`0x50`/`0x60`), normal-mode, factory NVM calibration + Bosch float
  compensation. Polled at 25 Hz (low-bandwidth — it can't and needn't match the IMU rate).
- **Rates are configurable in [`src/config.rs`](src/config.rs)** — IMU ODR / log rate, baro ODR /
  oversampling / sample rate / log rate. Sized for quad/VTOL control: the rate loop wants ≥400 Hz
  (gyro sampled ≥1 kHz, anti-aliased), the baro only ~25 Hz. The control *law* (PID/mixer/motors)
  is not implemented yet — this provides the timely data path it will run on.
- Streams a `tick` counter line once per second and cycles the status LEDs red → green → blue
  (one per tick) as a heartbeat.
- Sending `r` over the serial link reboots into the ROM bootloader for DFU reflashing.
- **Roadmap:** magnetometer (IIS2MDC/LIS2MDL on I2C4), then sensor fusion.

## Hardware

- **MCU:** STM32H743 (selected via the `stm32h743v` HAL feature in `Cargo.toml`), SYSCLK 400 MHz.
- **Status LEDs:** red `PE3` / green `PE4` / blue `PE5`, active-low (pin LOW = lit).
- **USB:** OTG2_HS on `PA11` (DM) / `PA12` (DP), clocked off HSI48.

See [`docs/ark-fpv-board.md`](docs/ark-fpv-board.md) for the full pin map — sensors, UARTs,
motor outputs, and ADC power monitoring.

## Build

Target (`thumbv7em-none-eabihf`) and linker args are fixed in `.cargo/config.toml`, so plain
cargo works:

```bash
cargo build              # debug
cargo build --release    # release — use this for flashing (smaller, faster)
```

## Flash (USB DFU)

```bash
cargo objcopy --release -- -O binary firmware.bin   # needs cargo-binutils + llvm-tools
printf 'r' > /dev/cu.usbmodem*                       # reboot running firmware into DFU (~2s)
dfu-util -l                                          # confirm 0483:df11 (2 "Found DFU" lines)
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin    # flash to FLASH origin, then leave
```

Two ways into DFU:

- **`r` over the serial port** (preferred) — the firmware reboots itself into the ROM bootloader.
- **BOOT0 + RESET** (manual) — always works, even when firmware is hung; use for the first DFU
  flash of a build or to recover a bricked boot.

After flashing, **press NRST manually**: the `:leave` auto-run is unreliable on this H7 ROM
bootloader and often leaves the app hung. macOS also re-enumerates the CDC port slowly (~15s) —
poll, don't assume failure. The `Error during download get_status` (on leave) and
`Invalid DFU suffix signature` warnings are both benign.

## Verify

There is no host test harness (`#![no_std]`). Verify by reading the serial port:

```bash
stty -f /dev/cu.usbmodem* 115200 raw -echo
cat /dev/cu.usbmodem*
```

Expect `hello from RTIC on STM32H743, tick N` once per second, with the LEDs stepping
red → green → blue in sync.

## Layout

| Path                     | What                                             |
| ------------------------ | ------------------------------------------------ |
| `src/main.rs`            | The `#[rtic::app]` module — init, tasks, peripheral wiring |
| `src/imu.rs`             | IIM-42653 IMU driver (SPI1)                       |
| `src/baro.rs`            | BMP388/BMP390 barometer driver (I2C2)             |
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
