# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Generic sensor traits in `src/sensors.rs` (`Sensor`, `Identify`, `SoftReset`, `BringUp`) naming the shared driver surface. Definitions only — no `impl`s yet (drivers adopt them in a follow-up).

### Changed

- Sensor drivers grouped under a `sensors` module: `src/{imu,baro,mag}.rs` moved to `src/sensors/` behind a `src/sensors.rs` facade (no driver logic changed).

## [0.3.0] - 2026-06-03

### Added

- `tools/telem` — host-side binary-telemetry decoder (workspace member, not part of the firmware build). Opens the serial port, sends `b`, and pretty-prints decoded `discovery-telemetry` frames. Run with `just telem`.

### Changed

- Repo is now a Cargo workspace (firmware at root + `tools/`); `default-members = ["."]` keeps the firmware/flash/release flow firmware-only.

## [0.2.0] - 2026-06-03

### Changed

- Crate renamed `ark-discovery` → `ark-fpv-discovery`.
- `discovery-telemetry` now pulled from crates.io (`= "0.2.0"`) instead of the git tag `v0.2.0`.

## [0.1.0] - 2026-06-02

### Added

- Initial release: USB CDC serial device on the ARK FPV board (STM32H743), RTIC 2 async.
- IIM-42653 IMU driver (SPI1) with gyro-synchronous control loop off the data-ready interrupt.
- BMP388/BMP390 barometer driver (I2C2) and IIS2MDC/LIS2MDL magnetometer driver (I2C4).
- Sensor fusion: attitude (fusion-ahrs) + altitude/vertical-velocity (fusion-altitude).
- Text/binary telemetry output (discovery-telemetry frames) with `b`/`t` toggle, `d` diagnostics, `r` reboot-to-DFU.
