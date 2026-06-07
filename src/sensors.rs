//! # Sensing strategy — what each sensor contributes to the estimate
//!
//! A flight controller can't measure its attitude or altitude directly; it *infers* them by fusing
//! complementary sensors, each strong where another is weak:
//!
//! - **Gyroscope** (IMU, [`ImuSample::gyro_dps`]) — angular *rate*. Fast and clean enough to drive
//!   the control loop, but integrating it to an angle **drifts** (bias accumulates without bound).
//! - **Accelerometer** (IMU, [`ImuSample::accel_g`]) — senses the gravity vector at rest, giving an
//!   absolute **tilt** (roll/pitch) reference that corrects gyro drift. Can't see yaw (gravity is
//!   symmetric about vertical) and is corrupted by linear acceleration / vibration in flight.
//! - **Magnetometer** ([`MagSample::field_ut`]) — Earth's field gives an absolute **heading (yaw)**
//!   reference, the one axis the accel can't fix. Easily distorted by nearby iron/current, so it
//!   needs field calibration to be trustworthy (see "Calibration" below).
//! - **Barometer** ([`BaroSample::pressure_hpa`]) — air pressure → **altitude** and its derivative,
//!   vertical speed. Slow, noisy, and sensitive to prop-wash/wind, but the only onboard absolute
//!   altitude source.
//!
//! So: gyro for *rate*, accel to stop roll/pitch drifting, mag to stop yaw drifting, baro for
//! altitude. Turning all four into a single attitude + altitude estimate is [`crate::fusion`]'s job.
//!
//! **Sampling paradigm follows the physics.** The gyro must be sampled *synchronously* at a high,
//! fixed rate — jitter or aliasing in the rate signal feeds straight into the control loop — so the
//! IMU is **interrupt-driven**: it raises a data-ready line at its ODR and the [`Imu`] role has no
//! `bring_up`/poll method (the EXTI/DRDY path lives in `main`). The baro and mag change slowly (tens
//! of Hz), so they are **polled** on a timer and own an async `bring_up` that hides the chip's
//! reset/settle quirks. Interrupt for the fast, drift-critical sensor; polling for the slow ones —
//! the same split full flight stacks make.
//!
//! **Calibration comes in two kinds.** The baro loads *factory* trim ([`baro::Bmp3xx::read_calibration`])
//! and the mag enables *on-chip* offset/temperature compensation — but neither is **field
//! calibration**: gyro bias, accel six-position, and especially magnetometer hard/soft-iron
//! correction are vehicle-specific and still owed before the estimate is flight-trustworthy (on a
//! cluttered metal bench the mag already reads 2–3× Earth's field from hard-iron alone). Treat the
//! samples here as *raw, factory-compensated* — not yet *calibrated for this airframe*.
//!
//! ---
//!
//! The sensor **role layer**: the contract types every sensor of a given role produces, and the
//! role traits (`Imu`, `Baro`, `Mag`) that fusion/tasks can eventually be written against so the
//! concrete chip behind each role is swappable.
//!
//! The submodules are role slots — one concrete driver each (a firmware build has one IMU, one
//! baro, one mag). Self-contained, `no_std`, no-external-crate: `imu` (SPI1) and `baro`/`mag`
//! (composing the shared [`crate::i2c_regs::I2cRegs`] on I2C2/I2C4). The module names the role; the
//! struct names the part currently filling it (`imu::Iim42653`, `baro::Bmp3xx`, `mag::Lis2mdl`),
//! and each implements its role trait (`impl Imu for Iim42653`, etc.) and produces the contract
//! types defined here. Swapping the part means a new struct `impl`ing the same role trait and one
//! changed `::new()` line in `main`; the module path stays stable.
//!
//! **Contract vs. encoding.** The sample structs and the *logical* config enums (e.g.
//! `ImuOdr { Hz200, Hz500, Hz1000 }`) live here — they name a rate/quantity, not a register value.
//! Each driver maps them to its own chip registers privately (e.g. `imu::odr_reg`), so a second
//! IMU could satisfy the same `ImuOdr` with a different encoding. Datasheet citations for
//! the register values stay with those per-driver mappings.
//!
//! Each concrete driver `impl`s its role trait (plus the horizontal `Identify`/`SoftReset`); the
//! RTIC tasks in `main` call those trait methods on the owned concrete type. The remaining
//! follow-up for full swappability is to make `fusion` generic over `impl Imu`/`Baro`/`Mag` rather
//! than the sample structs. Every method takes `&mut self` because each access drives the bus (SPI
//! transfer / I2C write) through the owned peripheral.

use core::future::Future;

pub(crate) mod baro;
pub(crate) mod imu;
pub(crate) mod mag;

// =============================================================================
// Driver selection — the one place that names the concrete chip per role.
// =============================================================================

// Consumers (`main`'s `#[local]` resources, the sensor tasks) reference these aliases, never the
// part struct — so swapping a chip is an alias change here plus its new driver module (which also
// exports the identity constants the tasks read by module path: `EXPECTED_WHO_AM_I`, the baro's
// `CHIP_ID_*` + part-name match). The tasks can't be generic over the role traits (the
// `#[rtic::app]` macro needs concrete `#[local]` types, and the mag task uses inherent
// `status`/`read_cfg` diagnostics that aren't on `Mag`), and after the role-layer split the
// chip-specific logic already lives inside the trait methods — so an alias, not `impl Trait`, is
// the swap seam. `fusion` consumes the sample contract types, not drivers, so it's unaffected.
pub(crate) type ImuDriver = imu::Iim42653;
pub(crate) type BaroDriver = baro::Bmp3xx;
pub(crate) type MagDriver = mag::Lis2mdl;

// =============================================================================
// Contract types — the role layer's vocabulary (units + logical config).
// =============================================================================

/// One scaled IMU sample: accel in g, gyro in dps, temperature in °C.
///
/// Axes are the **raw chip frame** (the IIM-42653's own X/Y/Z as soldered on the board) — *not* the
/// vehicle body frame. The board-mounting rotation into the body/NWU frame happens once, downstream,
/// in [`crate::fusion`] (`to_body_frame`). Keeping the driver frame-agnostic is deliberate: the
/// remap depends on how the chip is mounted, which is a vehicle fact, not a sensor fact.
#[derive(Clone, Copy)]
pub struct ImuSample {
    pub accel_g: [f32; 3],
    pub gyro_dps: [f32; 3],
    pub temp_c: f32,
}

/// IMU output data rate. The data-ready interrupt fires at this rate, so it sets the
/// gyro-synchronous control-loop cadence. The driver maps it to its ODR register field.
// These enums are the full hardware-supported mode set; `config` selects one, so the rest are
// never *constructed*. Keep the complete capability surface rather than trimming to what's wired.
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum ImuOdr {
    Hz200,
    Hz500,
    Hz1000,
}

impl ImuOdr {
    /// Nominal rate in Hz (the logical quantity; const so `config` can derive log divisors).
    pub const fn hz(self) -> u32 {
        match self {
            ImuOdr::Hz200 => 200,
            ImuOdr::Hz500 => 500,
            ImuOdr::Hz1000 => 1000,
        }
    }
}

/// One compensated barometer sample: pressure in hPa, temperature in °C.
pub struct BaroSample {
    pub pressure_hpa: f32,
    pub temp_c: f32,
}

/// Barometer output data rate. Coupled to [`Oversampling`] by the measurement-time rule the driver
/// checks in bring-up (a faster ODR than the conversion can complete is rejected).
#[allow(dead_code)] // full hardware ODR set; config selects one (see ImuOdr)
#[derive(Clone, Copy)]
pub enum BaroOdr {
    Hz200,
    Hz100,
    Hz50,
    Hz25,
    Hz12_5,
}

impl BaroOdr {
    /// Sampling period in microseconds (the logical quantity used by the timing rule).
    pub const fn period_us(self) -> u32 {
        match self {
            BaroOdr::Hz200 => 5_000,
            BaroOdr::Hz100 => 10_000,
            BaroOdr::Hz50 => 20_000,
            BaroOdr::Hz25 => 40_000,
            BaroOdr::Hz12_5 => 80_000,
        }
    }
}

/// Oversampling factor (pressure or temperature). The driver maps it to its OSR register code.
#[allow(dead_code)] // full hardware oversampling set; config selects a subset (see ImuOdr)
#[derive(Clone, Copy)]
pub enum Oversampling {
    X1,
    X2,
    X4,
    X8,
    X16,
    X32,
}

impl Oversampling {
    /// Number of internal samples averaged (1/2/4/…/32) — the logical quantity used by the
    /// measurement-time rule. (The register *code* is `log2(factor)`, mapped in the driver.)
    pub const fn factor(self) -> u32 {
        match self {
            Oversampling::X1 => 1,
            Oversampling::X2 => 2,
            Oversampling::X4 => 4,
            Oversampling::X8 => 8,
            Oversampling::X16 => 16,
            Oversampling::X32 => 32,
        }
    }
}

/// Bring-up/configuration error: the requested ODR is too fast for the chosen oversampling
/// (the measurement can't complete within the ODR period).
#[derive(Debug)]
pub enum ConfigError {
    OdrTooFast,
}

/// One scaled magnetometer sample: field in µT, die temperature in °C.
///
/// Like [`ImuSample`], `field_ut` is in the **raw chip frame**, not the body frame; the remap is
/// applied downstream in [`crate::fusion`]. The mag is assumed co-framed with the IMU — if heading
/// (yaw) is wrong while roll/pitch are right, the mag needs its own axis remap.
#[derive(Clone, Copy)]
pub struct MagSample {
    pub field_ut: [f32; 3],
    pub temp_c: f32,
}

/// Magnetometer output data rate (the continuous-conversion rate). Poll at or below it. The driver
/// maps it to its ODR register field.
#[allow(dead_code)] // full hardware ODR set; config selects one (see ImuOdr)
#[derive(Clone, Copy)]
pub enum MagOdr {
    Hz10,
    Hz20,
    Hz50,
    Hz100,
}

// =============================================================================
// Horizontal traits — genuinely uniform across every role.
// =============================================================================

/// Device identification via its WHO_AM_I / CHIP_ID register.
///
/// `id_matches` is an associated fn (rather than a single `EXPECTED_ID` const) because the baro
/// accepts two IDs — BMP388 (`0x50`) or BMP390 (`0x60`) — behind one register-compatible driver.
pub trait Identify {
    /// Read the raw identity register (WHO_AM_I / CHIP_ID).
    fn read_id(&mut self) -> u8;

    /// True if `id` is an identity this driver accepts.
    fn id_matches(id: u8) -> bool;
}

/// Soft-reset to a known state. Reboot/reflash is far more common than a power cycle here, so a
/// reset on bring-up matters. The settle/poll *after* the reset is the device's own quirk and
/// lives in each role's bring-up (baro: fixed ~2 ms; mag: poll SOFT_RST self-clear).
pub trait SoftReset {
    fn soft_reset(&mut self);
}

// =============================================================================
// Role traits — one per sensor role, with concrete contract types.
// =============================================================================

/// An interrupt-driven 6-axis IMU: the gyro-synchronous control-loop source. No bring-up method
/// because its bring-up is split between `init` (busy-wait soft-reset) and `configure_control_mode`
/// and gated on the EXTI/DRDY path rather than a delay closure — unlike the polled sensors.
pub trait Imu {
    /// Configure range/ODR/filtering and route data-ready to the interrupt line, then power on.
    fn configure_control_mode(&mut self, odr: ImuOdr);

    /// Acknowledge the latched data-ready interrupt (drops the line so the next edge can fire).
    fn clear_interrupt(&mut self);

    /// Read one scaled sample.
    fn read(&mut self) -> ImuSample;
}

/// A polled barometer. `bring_up` owns the reset → settle → calibrate → configure → wait protocol,
/// driven by an injected async millisecond delay (the RTIC monotonic stays in the task).
pub trait Baro {
    /// Full polled bring-up. `delay_ms` is an async millisecond delay (e.g.
    /// `|ms| Mono::delay(ms.millis())`). Errors if the ODR is too fast for the oversampling.
    fn bring_up<F, Fut>(
        &mut self,
        odr: BaroOdr,
        osr_p: Oversampling,
        osr_t: Oversampling,
        delay_ms: F,
    ) -> impl Future<Output = Result<(), ConfigError>>
    where
        F: FnMut(u32) -> Fut,
        Fut: Future<Output = ()>;

    /// Read the latest compensated sample.
    fn read(&mut self) -> BaroSample;
}

/// A polled magnetometer. `bring_up` owns the reset → wait-for-self-clear → configure →
/// re-assert-until-latched protocol (the first continuous-mode write after reset often reverts).
pub trait Mag {
    /// Full polled bring-up. `delay_ms` is an async millisecond delay. Returns `true` once
    /// continuous conversion is confirmed running, `false` if it never latched.
    fn bring_up<F, Fut>(&mut self, odr: MagOdr, delay_ms: F) -> impl Future<Output = bool>
    where
        F: FnMut(u32) -> Fut,
        Fut: Future<Output = ()>;

    /// Read the latest field + die temperature.
    fn read(&mut self) -> MagSample;
}
