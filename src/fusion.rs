//! Sensor fusion: attitude (roll/pitch/yaw) + altitude/vertical-velocity from the IMU, mag, and
//! baro.
//!
//! Wraps the [`fusion-ahrs`] and [`fusion-altitude`] crates behind one thin struct so the RTIC
//! `fusion_step` task stays an orchestrator (drain → update → store → log) and all the library
//! math lives here — the driver pattern used by `baro`/`mag` (module owns the device/library
//! logic, the task composes it). Inputs are the existing driver sample types; output is a flat,
//! `Copy` [`FusedState`].
//!
//! Unit match is exact: `fusion-ahrs` wants gyro in dps, accel in g, mag in µT — what
//! [`ImuSample`]/[`MagSample`] already carry. `fusion-altitude` wants altitude in metres and a
//! gravity-compensated, earth-frame, +up vertical acceleration in m/s² — we derive altitude from
//! pressure ([`pressure_to_altitude_m`], the crate has no helper) and the vertical accel from
//! `ahrs.earth_acceleration().z * GRAVITY` (`earth_acceleration` is in g).
//!
//! [`fusion-ahrs`]: https://github.com/wboayue/fusion-ahrs
//! [`fusion-altitude`]: https://github.com/wboayue/fusion-altitude

use nalgebra::Vector3;

use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
use fusion_altitude::{AltitudeEstimator, AltitudeSettings, GRAVITY};

use crate::config;
use crate::sensors::imu::ImuSample;
use crate::sensors::mag::MagSample;

/// Fused estimate. Flat and `Copy` so it drops straight into a `#[shared]` resource and the
/// logger. Angles in degrees, altitude in metres, velocity in m/s.
#[derive(Clone, Copy, Default)]
pub struct FusedState {
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub altitude_m: f32,
    pub vertical_velocity: f32,
    /// Baro innovation (baro altitude − filtered altitude), m. The disturbance signal a future
    /// adaptive-baro-trust scheme would gate on — log it to size the trust threshold (`r0`).
    pub baro_residual: f32,
}

/// Latest raw sensor inputs, shared between the producer tasks and the fusion consumer. Holds
/// *raw* values only — all interpretation (averaging, pressure→altitude, axis remap) is deferred
/// to `Fusion`, so the producer tasks carry no fusion concern. The IMU is kept as a running sum +
/// count (delta-angle downsampling): `imu_drdy` adds every 1 kHz sample (pure adds, cheap enough
/// for the ISR), and the fusion task drains the mean over its window.
#[derive(Clone, Copy, Default)]
pub struct SensorState {
    gyro_sum: [f32; 3],
    accel_sum: [f32; 3],
    temp_sum: f32,
    n: u32,
    mag: Option<MagSample>,
    baro_pressure_hpa: Option<f32>,
}

impl SensorState {
    /// Accumulate one IMU sample (called from `imu_drdy`). Pure adds — no trig/nalgebra, so it's
    /// safe in the high-priority latched-interrupt ISR.
    #[inline]
    pub fn accumulate(&mut self, s: &ImuSample) {
        for i in 0..3 {
            self.gyro_sum[i] += s.gyro_dps[i];
            self.accel_sum[i] += s.accel_g[i];
        }
        self.temp_sum += s.temp_c;
        self.n += 1;
    }

    /// Take the accumulated IMU mean as a real [`ImuSample`] and clear the accumulator. `None`
    /// until the gyro has started producing data-ready edges.
    pub fn drain_imu(&mut self) -> Option<ImuSample> {
        if self.n == 0 {
            return None;
        }
        let inv = 1.0 / self.n as f32;
        let mean = ImuSample {
            accel_g: [self.accel_sum[0] * inv, self.accel_sum[1] * inv, self.accel_sum[2] * inv],
            gyro_dps: [self.gyro_sum[0] * inv, self.gyro_sum[1] * inv, self.gyro_sum[2] * inv],
            temp_c: self.temp_sum * inv,
        };
        self.gyro_sum = [0.0; 3];
        self.accel_sum = [0.0; 3];
        self.temp_sum = 0.0;
        self.n = 0;
        Some(mean)
    }

    pub fn set_mag(&mut self, m: MagSample) {
        self.mag = Some(m);
    }
    pub fn set_pressure(&mut self, p_hpa: f32) {
        self.baro_pressure_hpa = Some(p_hpa);
    }
    pub fn take_mag(&mut self) -> Option<MagSample> {
        self.mag.take()
    }
    pub fn take_pressure(&mut self) -> Option<f32> {
        self.baro_pressure_hpa.take()
    }
}

/// The two estimators behind one update. Owns axis-remap, the pressure→altitude conversion, and
/// first-sample altitude seeding so the RTIC task carries none of it.
pub struct Fusion {
    ahrs: Ahrs,
    altitude: AltitudeEstimator,
    use_mag: bool,
    alt_seeded: bool,
    /// Most recent baro altitude (m), held between baro samples so the altitude observer can run
    /// every fusion tick with continuous baro correction (it's slower than the fusion rate).
    baro_alt_m: f32,
    /// Last gravity-compensated vertical acceleration fed to the altitude observer (m/s², +up).
    /// Retained only for diagnostics (the `fus[diag]` line) — shows accel/vibration coupling.
    vertical_accel_mps2: f32,
}

impl Fusion {
    pub fn new(use_mag: bool) -> Self {
        // AhrsSettings is a plain struct (literal OK). Convention NWU → earth-frame Z is up, so
        // earth_acceleration().z is +up for the altitude estimator.
        let ahrs = Ahrs::with_settings(AhrsSettings {
            convention: Convention::Nwu,
            gain: config::AHRS_GAIN,
            gyroscope_range: config::AHRS_GYRO_RANGE_DPS,
            acceleration_rejection: config::AHRS_ACCEL_REJECTION,
            magnetic_rejection: config::AHRS_MAG_REJECTION,
            recovery_trigger_period: config::AHRS_RECOVERY_TRIGGER_PERIOD,
        });
        // AltitudeSettings is #[non_exhaustive] → build from default(), then override.
        let mut alt = AltitudeSettings::default();
        alt.position_gain = config::ALT_POSITION_GAIN;
        alt.velocity_gain = config::ALT_VELOCITY_GAIN;
        alt.bias_gain = config::ALT_BIAS_GAIN;
        Self {
            ahrs,
            altitude: AltitudeEstimator::with_settings(alt),
            use_mag,
            alt_seeded: false,
            baro_alt_m: 0.0,
            vertical_accel_mps2: 0.0,
        }
    }

    /// Estimated accel bias (m/s²) — diagnostics: a drifting bias inflates the baro residual.
    pub fn accel_bias(&self) -> f32 {
        self.altitude.accel_bias()
    }
    /// Last gravity-compensated vertical accel (m/s², +up) — diagnostics: vibration coupling.
    pub fn vertical_accel(&self) -> f32 {
        self.vertical_accel_mps2
    }

    /// Map sensor axes → the NWU body frame the AHRS expects. The single documented place for the
    /// board-mount remap.
    ///
    /// ARK FPV mount (verified on hardware): flat & level the IIM-42653 reads accel ≈ (0, 0, −1) g,
    /// i.e. its +Z points *down*, so the sensor frame is rotated 180° about X from the body frame
    /// (which left the AHRS reporting roll ≈ 180° level). Undo it with the same 180°-about-X
    /// rotation `(x, y, z) → (x, −y, −z)` — a proper rotation (det +1), so gyro handedness and yaw
    /// direction stay consistent. Applied to all three sensors; the mag is assumed co-framed with
    /// the IMU here — revisit this if the yaw *heading* turns out wrong (roll/pitch are unaffected).
    #[inline]
    fn to_body_frame(
        gyro_dps: [f32; 3],
        accel_g: [f32; 3],
        field_ut: [f32; 3],
    ) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
        let flip = |v: [f32; 3]| Vector3::new(v[0], -v[1], -v[2]);
        (flip(gyro_dps), flip(accel_g), flip(field_ut))
    }

    /// One fusion step. `imu` is the accumulated *mean* over `dt`; `mag` and `pressure_hpa` are the
    /// latest raw values (`None` when no fresh sample arrived this tick — the mag/baro run far
    /// slower than the fusion rate). Returns the updated [`FusedState`].
    pub fn update(
        &mut self,
        imu: &ImuSample,
        mag: Option<&MagSample>,
        pressure_hpa: Option<f32>,
        dt: f32,
    ) -> FusedState {
        let field = mag.map(|m| m.field_ut).unwrap_or([0.0; 3]);
        let (gyro, accel, field) = Self::to_body_frame(imu.gyro_dps, imu.accel_g, field);

        // 9-DOF when the mag is enabled and fresh; otherwise integrate gyro+accel only.
        match (self.use_mag, mag) {
            (true, Some(_)) => self.ahrs.update(gyro, accel, field, dt),
            _ => self.ahrs.update_no_magnetometer(gyro, accel, dt),
        }

        let (roll, pitch, yaw) = self.ahrs.quaternion().euler_angles(); // radians

        // Refresh the held baro reference when a new sample arrives; seed on the first one.
        if let Some(p_hpa) = pressure_hpa {
            let alt = pressure_to_altitude_m(p_hpa, config::P0_REFERENCE);
            if !self.alt_seeded {
                // Seed the filter to the current baro altitude so it starts converged rather than
                // ramping up from 0. This is absolute ISA altitude (relative to P0_REFERENCE), not
                // a re-zero — vertical velocity is a derivative and unaffected by the offset.
                self.altitude.reset(alt);
                self.alt_seeded = true;
            }
            self.baro_alt_m = alt;
        }

        // Run the altitude observer EVERY fusion tick (high-rate accel predict + continuous baro
        // correction), once seeded. Critical: the estimator's `dt` must be the time since its
        // previous `update` — i.e. the fusion tick interval. Updating it only on fresh baro (~25 Hz)
        // while passing the ~4 ms tick dt advances the filter at 1/10 real-time, so velocity damps
        // ~10× too slowly (vz rings for seconds). The baro is held between samples — that's the
        // standard complementary-filter structure. Vertical accel is gravity-compensated, +up (NWU).
        if self.alt_seeded {
            let vertical_accel = self.ahrs.earth_acceleration().z * GRAVITY; // g → m/s²
            self.vertical_accel_mps2 = vertical_accel;
            self.altitude.update(vertical_accel, self.baro_alt_m, dt);
        }

        FusedState {
            roll_deg: roll.to_degrees(),
            pitch_deg: pitch.to_degrees(),
            yaw_deg: yaw.to_degrees(),
            altitude_m: self.altitude.altitude(),
            vertical_velocity: self.altitude.vertical_velocity(),
            baro_residual: self.altitude.baro_residual(),
        }
    }
}

/// International (ISA) barometric formula: metres above the `p0_hpa` reference level.
/// `h = 44330 * (1 - (p/p0)^(1/5.255))`. Source: NOAA/ISA hypsometric formula; the 44330 / 5.255
/// constants match the Bosch BMP3 examples. `powf` is std-only, so we use `libm` (no_std).
pub fn pressure_to_altitude_m(p_hpa: f32, p0_hpa: f32) -> f32 {
    44330.0 * (1.0 - libm::powf(p_hpa / p0_hpa, 1.0 / 5.255))
}
