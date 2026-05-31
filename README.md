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

The transport and dev-loop scaffolding is up; sensor drivers are the next step.

- Enumerates as a USB CDC serial device and streams a `tick` counter line once per second —
  this is the channel sensor readings will flow out over.
- Cycles the onboard status LEDs red → green → blue (one per tick) as a heartbeat.
- Sending `r` over the serial link reboots into the ROM bootloader for DFU reflashing.
- **No sensors are read yet** — IMU/baro/mag bring-up over SPI1 / I2C2 / I2C4 is the roadmap.

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
| `src/main.rs`            | The whole firmware — one `#[rtic::app]` module   |
| `memory.x`               | Linker regions: FLASH @ `0x08000000`, RAM @ `0x20000000` |
| `.cargo/config.toml`     | Target + linker args                             |
| `docs/ark-fpv-board.md`  | ARK FPV pin map                                  |
| `CLAUDE.md`              | Deeper notes on RTIC structure, clocks, flashing gotchas |

See [`CLAUDE.md`](CLAUDE.md) for the hard-won details (USB clock path, reboot-to-DFU mechanism,
`pre_init` pitfall, RTIC resource conventions).
