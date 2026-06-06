//! Minimal register-level driver for the ARK FPV's BMP388/BMP390 barometer on I2C2.
//!
//! Polled, no_std, no external crate. Identifies the chip, reads its factory calibration,
//! configures continuous (normal-mode) sampling, and returns compensated pressure (hPa) and
//! temperature (°C). BMP388 and BMP390 are register-compatible and differ only in CHIP_ID.
//!
//! Register addresses and the soft-reset command come from the Bosch BMP388/BMP390 datasheets.
//! The calibration scaling (`read_calibration`) and the float compensation math (`read`) are
//! transcribed from the official Bosch BMP3_SensorAPI `bmp3.c` (`parse_calib_data`,
//! `compensate_temperature`, `compensate_pressure`):
//!   <https://github.com/boschsensortec/BMP3_SensorAPI/blob/master/bmp3.c>

use core::future::Future;

use stm32h7xx_hal as hal;

use super::{BaroOdr, BaroSample, ConfigError, Oversampling};
use crate::i2c_regs::I2cRegs;

type I2c2 = hal::i2c::I2c<hal::pac::I2C2>;

/// 7-bit I2C address (SDO tied low on this board).
const ADDR: u8 = 0x76;

// --- Registers (datasheet) ----------------------------------------------------
const REG_CHIP_ID: u8 = 0x00;
const REG_DATA: u8 = 0x04; // PRESS_XLSB; 6 bytes through TEMP_MSB (0x09)
const REG_PWR_CTRL: u8 = 0x1B;
const REG_OSR: u8 = 0x1C;
const REG_ODR: u8 = 0x1D;
const REG_CALIB: u8 = 0x31; // NVM_PAR_T1 .. NVM_PAR_P11, 21 bytes through 0x45
const REG_CMD: u8 = 0x7E;

const CMD_SOFT_RESET: u8 = 0xB6;

/// CHIP_ID values — accept either part.
pub const CHIP_ID_BMP388: u8 = 0x50;
pub const CHIP_ID_BMP390: u8 = 0x60;

// PWR_CTRL layout (datasheet / bmp3_defs.h): mode in bits[5:4] (normal=0b11), temp_en bit1,
// press_en bit0. So 0x33 = (0b11<<4) | temp_en | press_en. (NOT 0x0F — mode is not bits[1:0].)
const PWR_CTRL_NORMAL: u8 = 0x33;

// Bosch meas-time constants (bmp3_defs.h): t_meas ≈ 234 + (392 + 2^osr_p·2000) +
// (313 + 2^osr_t·2000) µs. Config is valid iff t_meas < the ODR period.
const SETTLE_PRESS_US: u32 = 392;
const SETTLE_TEMP_US: u32 = 313;
const ADC_CONV_US: u32 = 2000;
const MEAS_BASE_US: u32 = 234;

/// Map the logical [`BaroOdr`] to the ODR register (`0x1D`) code. Values per the Bosch BMP3
/// datasheet (`bmp3_defs.h` `BMP3_ODR_*`).
const fn odr_reg(odr: BaroOdr) -> u8 {
    match odr {
        BaroOdr::Hz200 => 0x00,
        BaroOdr::Hz100 => 0x01,
        BaroOdr::Hz50 => 0x02,
        BaroOdr::Hz25 => 0x03,
        BaroOdr::Hz12_5 => 0x04,
    }
}

/// Map the logical [`Oversampling`] to its OSR register field code (`log2(factor)`, 0..=5). Values
/// per the Bosch BMP3 datasheet (`bmp3_defs.h` `BMP3_OVERSAMPLING_*`).
const fn osr_reg(osr: Oversampling) -> u8 {
    match osr {
        Oversampling::X1 => 0,
        Oversampling::X2 => 1,
        Oversampling::X4 => 2,
        Oversampling::X8 => 3,
        Oversampling::X16 => 4,
        Oversampling::X32 => 5,
    }
}

/// Float-scaled calibration coefficients, per Bosch `parse_calib_data` (quantized form).
#[derive(Default)]
struct Calib {
    t1: f64,
    t2: f64,
    t3: f64,
    p1: f64,
    p2: f64,
    p3: f64,
    p4: f64,
    p5: f64,
    p6: f64,
    p7: f64,
    p8: f64,
    p9: f64,
    p10: f64,
    p11: f64,
}

pub struct Baro {
    regs: I2cRegs<I2c2>,
    calib: Calib,
}

impl Baro {
    /// Take ownership of the configured I2C2 bus. Calibration is zeroed until
    /// [`read_calibration`](Self::read_calibration) runs.
    pub fn new(i2c: I2c2) -> Self {
        Self {
            regs: I2cRegs::new(i2c, ADDR),
            calib: Calib::default(),
        }
    }

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) {
        self.regs.read_regs(reg, buf);
    }

    fn write_reg(&mut self, reg: u8, val: u8) {
        self.regs.write_reg(reg, val);
    }

    pub fn chip_id(&mut self) -> u8 {
        self.regs.read_reg(REG_CHIP_ID)
    }

    /// Soft-reset to a known state. Caller must wait ~2 ms afterwards.
    pub fn soft_reset(&mut self) {
        self.write_reg(REG_CMD, CMD_SOFT_RESET);
    }

    /// Read the 21-byte NVM trimming block and scale it to floating-point coefficients.
    /// Scaling divisors are verbatim from Bosch `parse_calib_data` (float variant).
    pub fn read_calibration(&mut self) {
        let mut b = [0u8; 21];
        self.read_regs(REG_CALIB, &mut b);

        // Raw NVM coefficients (little-endian; signedness per the datasheet struct).
        let t1 = u16::from_le_bytes([b[0], b[1]]);
        let t2 = u16::from_le_bytes([b[2], b[3]]);
        let t3 = b[4] as i8;
        let p1 = i16::from_le_bytes([b[5], b[6]]);
        let p2 = i16::from_le_bytes([b[7], b[8]]);
        let p3 = b[9] as i8;
        let p4 = b[10] as i8;
        let p5 = u16::from_le_bytes([b[11], b[12]]);
        let p6 = u16::from_le_bytes([b[13], b[14]]);
        let p7 = b[15] as i8;
        let p8 = b[16] as i8;
        let p9 = i16::from_le_bytes([b[17], b[18]]);
        let p10 = b[19] as i8;
        let p11 = b[20] as i8;

        // Quantized calibration: raw / 2^k (see BMP3_SensorAPI). Powers of two as exact f64.
        self.calib = Calib {
            t1: t1 as f64 / 0.003_906_25,           // / 2^-8  (= * 256)
            t2: t2 as f64 / 1_073_741_824.0,         // / 2^30
            t3: t3 as f64 / 281_474_976_710_656.0,   // / 2^48
            p1: (p1 as f64 - 16_384.0) / 1_048_576.0, // / 2^20
            p2: (p2 as f64 - 16_384.0) / 536_870_912.0, // / 2^29
            p3: p3 as f64 / 4_294_967_296.0,         // / 2^32
            p4: p4 as f64 / 137_438_953_472.0,       // / 2^37
            p5: p5 as f64 / 0.125,                   // / 2^-3  (= * 8)
            p6: p6 as f64 / 64.0,                    // / 2^6
            p7: p7 as f64 / 256.0,                   // / 2^8
            p8: p8 as f64 / 32_768.0,                // / 2^15
            p9: p9 as f64 / 281_474_976_710_656.0,   // / 2^48
            p10: p10 as f64 / 281_474_976_710_656.0, // / 2^48
            p11: p11 as f64 / 36_893_488_147_419_103_232.0, // / 2^65
        };
    }

    /// Set oversampling/ODR and start normal-mode sampling. Returns `Err(OdrTooFast)` if the
    /// measurement can't complete within the ODR period (Bosch timing rule) — the registers are
    /// left untouched in that case. Caller must wait for the first conversion before reading.
    pub fn configure(
        &mut self,
        odr: BaroOdr,
        osr_p: Oversampling,
        osr_t: Oversampling,
    ) -> Result<(), ConfigError> {
        let meas_us = MEAS_BASE_US
            + (SETTLE_PRESS_US + osr_p.factor() * ADC_CONV_US)
            + (SETTLE_TEMP_US + osr_t.factor() * ADC_CONV_US);
        if meas_us >= odr.period_us() {
            return Err(ConfigError::OdrTooFast);
        }
        self.write_reg(REG_OSR, (osr_reg(osr_t) << 3) | osr_reg(osr_p));
        self.write_reg(REG_ODR, odr_reg(odr));
        self.write_reg(REG_PWR_CTRL, PWR_CTRL_NORMAL);
        Ok(())
    }

    /// Full polled bring-up: soft-reset, settle, load factory calibration, start normal-mode
    /// sampling, then wait for the first conversion. Encapsulates the reset/settle timing so the
    /// caller only supplies an async millisecond delay (e.g. `|ms| Mono::delay(ms.millis())`).
    /// Returns `Err(OdrTooFast)` straight from [`configure`](Self::configure).
    pub async fn bring_up<F, Fut>(
        &mut self,
        odr: BaroOdr,
        osr_p: Oversampling,
        osr_t: Oversampling,
        mut delay_ms: F,
    ) -> Result<(), ConfigError>
    where
        F: FnMut(u32) -> Fut,
        Fut: Future<Output = ()>,
    {
        self.soft_reset();
        delay_ms(5).await; // ~2 ms reset + margin
        self.read_calibration();
        self.configure(odr, osr_p, osr_t)?;
        delay_ms(50).await; // let the first normal-mode conversion complete
        Ok(())
    }

    /// Read the latest sample and apply Bosch float compensation.
    pub fn read(&mut self) -> BaroSample {
        let mut b = [0u8; 6];
        self.read_regs(REG_DATA, &mut b);
        // 24-bit, little-endian: XLSB, LSB, MSB.
        let adc_p = u32::from_le_bytes([b[0], b[1], b[2], 0]) as f64;
        let adc_t = u32::from_le_bytes([b[3], b[4], b[5], 0]) as f64;

        let c = &self.calib;

        // compensate_temperature -> t_lin (°C)
        let d1 = adc_t - c.t1;
        let d2 = d1 * c.t2;
        let t_lin = d2 + d1 * d1 * c.t3;

        // compensate_pressure (Pa)
        let o1 = c.p5 + c.p6 * t_lin + c.p7 * t_lin * t_lin + c.p8 * t_lin * t_lin * t_lin;
        let o2 = adc_p * (c.p1 + c.p2 * t_lin + c.p3 * t_lin * t_lin + c.p4 * t_lin * t_lin * t_lin);
        let o3 = adc_p * adc_p * (c.p9 + c.p10 * t_lin) + adc_p * adc_p * adc_p * c.p11;
        let pressure_pa = o1 + o2 + o3;

        BaroSample {
            pressure_hpa: (pressure_pa / 100.0) as f32,
            temp_c: t_lin as f32,
        }
    }
}
