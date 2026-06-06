//! Register-level drivers for the three onboard sensors, plus the generic traits that name their
//! shared surface.
//!
//! Each `<sensor>` module is a self-contained, `no_std`, no-external-crate driver: `imu` on SPI1,
//! `baro`/`mag` composing the shared [`crate::i2c_regs::I2cRegs`] on I2C2/I2C4. The traits below
//! factor out the common shape (identify → reset → bring up → read a typed sample).
//!
//! **Trait definitions only** — there are no `impl` blocks yet. Wiring each driver onto these
//! traits is a deliberate follow-up, so this stays a pure, reviewable extraction with no
//! behaviour change. Every method takes `&mut self` because each access drives the bus (SPI
//! transfer / I2C write) through the owned peripheral.

// The traits are scaffolding for a not-yet-written set of impls; suppress the bin-crate
// dead-code warning until the drivers adopt them.
#![allow(dead_code)]

use core::future::Future;

pub(crate) mod baro;
pub(crate) mod imu;
pub(crate) mod mag;

/// Reads one typed sample from the device.
///
/// The unit-bearing sample struct (`ImuSample`, `BaroSample`, `MagSample`) is each driver's
/// contract with the fusion/telemetry layers, so it stays an **associated type** rather than a
/// shared struct — the three carry different fields and units.
pub trait Sensor {
    /// The driver's scaled, unit-bearing output (e.g. `ImuSample`).
    type Sample;

    /// Read the latest sample, applying the driver's scaling/compensation.
    fn read(&mut self) -> Self::Sample;
}

/// Device identification via its WHO_AM_I / CHIP_ID register.
///
/// `id_matches` is an associated fn (rather than a single `EXPECTED_ID` const) because the baro
/// accepts two IDs — BMP388 (`0x50`) or BMP390 (`0x60`) — behind one register-compatible driver.
pub trait Identify {
    /// Read the raw identity register (WHO_AM_I / CHIP_ID).
    fn read_id(&mut self) -> u8;

    /// True if `id` is an identity this driver accepts.
    fn id_matches(id: u8) -> bool;

    /// Convenience: read the id and check it in one call.
    fn identified(&mut self) -> bool {
        Self::id_matches(self.read_id())
    }
}

/// Soft-reset to a known state. Reboot/reflash is far more common than a power cycle here, so a
/// reset on bring-up matters. The settle/poll *after* the reset is the device's own quirk and
/// lives in [`BringUp`] (baro: fixed ~2 ms; mag: poll SOFT_RST self-clear).
pub trait SoftReset {
    fn soft_reset(&mut self);
}

/// Async, self-contained bring-up for a **polled** sensor: reset → settle → configure → wait,
/// driven by an injected async millisecond delay so the RTIC monotonic stays in the task while
/// the chip timing/retry quirks stay in the driver.
///
/// Implemented by the polled I2C sensors (baro, mag). The interrupt-driven IMU is deliberately
/// excluded: its bring-up is split between `init` (busy-wait soft-reset) and
/// `configure_control_mode`, gated on the EXTI/DRDY path rather than a delay closure.
///
/// `Config` and `Output` are associated because the parameters and result differ per device:
/// baro takes `(BaroOdr, Oversampling, Oversampling)` and yields `Result<(), ConfigError>`; mag
/// takes `MagOdr` and yields `bool` (whether continuous mode latched).
pub trait BringUp {
    /// Per-device configuration passed into bring-up (ODR, oversampling, …).
    type Config;
    /// Per-device result (`Result<(), ConfigError>` for baro, `bool` for mag).
    type Output;

    /// Run the full reset → settle → configure → wait sequence. `delay_ms` is an async
    /// millisecond delay (e.g. `|ms| Mono::delay(ms.millis())`).
    fn bring_up<F, Fut>(
        &mut self,
        config: Self::Config,
        delay_ms: F,
    ) -> impl Future<Output = Self::Output>
    where
        F: FnMut(u32) -> Fut,
        Fut: Future<Output = ()>;
}
