//! Single place to tune sensor/control-loop rates. See CLAUDE.md "Control-loop data path".

use crate::sensors::{BaroOdr, ImuOdr, MagOdr, Oversampling};

// --- IMU ---------------------------------------------------------------------
// The IMU loop is gyro-synchronous: its data-ready interrupt fires at IMU_ODR, so IMU_ODR *is*
// the control-loop rate. A quad/VTOL rate loop wants >=400 Hz (ArduCopter floor; PX4 ~1 kHz),
// with the gyro anti-alias-filtered, so 1 kHz is the default. Serial is logged far slower than
// the loop runs (decoupled), so USB never gates the fast path.
pub const IMU_ODR: ImuOdr = ImuOdr::Hz1000;
pub const IMU_LOG_HZ: u32 = 10;
/// Log one of every `IMU_LOG_DIV` samples (must divide evenly).
pub const IMU_LOG_DIV: u32 = IMU_ODR.hz() / IMU_LOG_HZ;

// --- Barometer ---------------------------------------------------------------
// Low-bandwidth; polled. Oversampling and ODR are coupled (the driver validates the pair): at
// pressure x8 the max valid ODR is ~25 Hz. Logged slower than sampled.
pub const BARO_ODR: BaroOdr = BaroOdr::Hz25;
pub const BARO_OSR_P: Oversampling = Oversampling::X8;
pub const BARO_OSR_T: Oversampling = Oversampling::X1;
pub const BARO_SAMPLE_HZ: u32 = 25;
pub const BARO_LOG_HZ: u32 = 5;
/// Log one of every `BARO_LOG_DIV` samples (must divide evenly).
pub const BARO_LOG_DIV: u32 = BARO_SAMPLE_HZ / BARO_LOG_HZ;

// --- Magnetometer ------------------------------------------------------------
// Low-bandwidth heading sensor; polled like the baro. ODR sets the continuous-conversion rate;
// sample below it. Logged slower than sampled.
pub const MAG_ODR: MagOdr = MagOdr::Hz50;
pub const MAG_SAMPLE_HZ: u32 = 50;
pub const MAG_LOG_HZ: u32 = 5;
/// Log one of every `MAG_LOG_DIV` samples (must divide evenly).
pub const MAG_LOG_DIV: u32 = MAG_SAMPLE_HZ / MAG_LOG_HZ;

// --- Fusion ------------------------------------------------------------------
// Attitude (fusion-ahrs) + altitude/vertical-velocity (fusion-altitude) run in a dedicated async
// task, decoupled from every sensor ODR. imu_drdy accumulates gyro/accel at IMU_ODR; the fusion
// task drains the mean each tick (delta-angle downsampling → full 1 kHz gyro fidelity at a 250 Hz
// estimator). Keep FUSION_RATE_HZ <= ~250-333 so the measured dt stays >= 3 SysTick (1 ms) ticks.
pub const FUSION_RATE_HZ: u32 = 250;
pub const FUSION_LOG_HZ: u32 = 10;
/// Log one of every `FUSION_LOG_DIV` fused states (must divide evenly).
pub const FUSION_LOG_DIV: u32 = FUSION_RATE_HZ / FUSION_LOG_HZ;
/// Measured-dt clamp (s): guards the first iteration and any scheduling gap from corrupting the
/// altitude velocity integration. Nominal period is 1/FUSION_RATE_HZ = 4 ms.
pub const FUSION_DT_MIN_S: f32 = 0.001;
pub const FUSION_DT_MAX_S: f32 = 0.050;

// 9-DOF (use the mag for absolute yaw) vs 6-DOF (gyro+accel; yaw is relative and drifts). The mag
// is hard-iron-sensitive on a cluttered bench (CLAUDE.md), so this toggle lets us validate
// roll/pitch independently of bench yaw error.
pub const FUSION_USE_MAG: bool = true;

// AHRS tuning (fusion-ahrs `AhrsSettings`). Values follow the canonical xioTechnologies Fusion
// example (gain 0.5; accel/mag rejection 10°; recovery after ~5 s of continuous rejection).
pub const AHRS_GAIN: f32 = 0.5;
pub const AHRS_GYRO_RANGE_DPS: f32 = 2000.0; // matches IMU FS_SEL=001 (±2000 dps); see src/imu.rs
pub const AHRS_ACCEL_REJECTION: f32 = 10.0;
pub const AHRS_MAG_REJECTION: f32 = 10.0;
pub const AHRS_RECOVERY_TRIGGER_PERIOD: u32 = 5 * FUSION_RATE_HZ; // in samples (~5 s)

// Altitude estimator gains (fusion-altitude `AltitudeSettings`) — its documented defaults.
pub const ALT_POSITION_GAIN: f32 = 2.40;
pub const ALT_VELOCITY_GAIN: f32 = 2.88;
pub const ALT_BIAS_GAIN: f32 = 0.675;

// --- Telemetry output --------------------------------------------------------
// Output mode at boot: false = human-readable text lines (a terminal user sees readable output
// with zero setup); true = binary postcard+COBS frames. The host scope switches to binary at
// runtime by sending 'b' ('t' switches back); see CLAUDE.md "Telemetry output".
pub const DEFAULT_OUTPUT_BINARY: bool = false;

// Reference sea-level pressure for the barometric formula (hPa). ISA standard. Altitude is
// absolute ISA height relative to this P0 (not re-zeroed at startup — the estimator is *seeded* to
// the first baro altitude so it starts converged). Set a local QNH here for true MSL altitude.
pub const P0_REFERENCE: f32 = 1013.25;
