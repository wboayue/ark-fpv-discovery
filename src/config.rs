//! Single place to tune sensor/control-loop rates. See CLAUDE.md "Control-loop data path".

use crate::baro::{BaroOdr, Oversampling};
use crate::imu::ImuOdr;

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
