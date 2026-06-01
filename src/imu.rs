//! Minimal register-level driver for the ARK FPV's IIM-42653 6-axis IMU on SPI1.
//!
//! Register-level, no_std, no external crate. Verifies WHO_AM_I, configures the chip for a
//! control loop (selectable ODR, anti-alias + UI filtering, data-ready interrupt on INT1), and
//! reads scaled accel (g) / gyro (dps) / temp (°C). The IIM-42653 is the wide-range (±32g /
//! ±4000 dps) member of the ICM-426xx family; its FS_SEL table is shifted up one step vs the
//! ICM-42688, so FS_SEL=001 here selects ±16g / ±2000 dps. All config below is bank 0.
//!
//! INT/filter register values verified against the PX4 `InvenSense_ICM42688P_registers.hpp`
//! (register-identical part): INT_CONFIG `0x07` = active-high/push-pull/**latched** (INT1_MODE
//! bit2 set), INT_CONFIG1 `0x00` clears the default INT_ASYNC_RESET (bit4), INT_SOURCE0 `0x08`
//! routes UI data-ready to INT1. Latched (not pulsed) is deliberate: INT1 holds asserted until
//! `INT_STATUS` is read, giving exactly one clean STM32 EXTI edge per sample with an explicit
//! acknowledge — pulsed mode's brief edges glitched/rang into intermittent interrupt storms.
//! The anti-alias filter is left at its enabled default; only the UI filter bandwidth is set
//! (explicit AAF-cutoff tuning is a future refinement).

use stm32h7xx_hal::{
    self as hal,
    gpio::{ErasedPin, Output, PushPull},
    prelude::*,
};

type Spi1 = hal::spi::Spi<hal::pac::SPI1, hal::spi::Enabled>;

// --- Bank-0 registers ---------------------------------------------------------
const WHO_AM_I: u8 = 0x75;
const REG_BANK_SEL: u8 = 0x76;
const DEVICE_CONFIG: u8 = 0x11;
const INT_CONFIG: u8 = 0x14;
const PWR_MGMT0: u8 = 0x4E;
const GYRO_CONFIG0: u8 = 0x4F;
const ACCEL_CONFIG0: u8 = 0x50;
const GYRO_ACCEL_CONFIG0: u8 = 0x52; // UI filter bandwidth (accel [7:4], gyro [3:0])
const INT_STATUS: u8 = 0x2D; // reading it clears the latched INT1 (drops the line)
const INT_CONFIG1: u8 = 0x64;
const INT_SOURCE0: u8 = 0x65;
const TEMP_DATA: u8 = 0x1D; // burst start: TEMP, ACCEL X/Y/Z, GYRO X/Y/Z = 14 bytes (..=0x2A)

/// Expected WHO_AM_I for the IIM-42653 (per Betaflight `accgyro_mpu.h`; not the ICM-42688 0x47).
pub const EXPECTED_WHO_AM_I: u8 = 0x56;

// FS_SEL in bits [7:5], ODR in bits [3:0]. FS_SEL=001 on the IIM-42653 → ±16g / ±2000 dps.
const FS_SEL: u8 = 0b001 << 5;
const UI_FILT_BW_ODR_4: u8 = 0x11; // accel + gyro UI filter bandwidth ≈ ODR/4
const INT_CONFIG_VAL: u8 = 0x07; // INT1 active-high, push-pull, latched (held until INT_STATUS read)
const INT_SOURCE0_DRDY_INT1: u8 = 0x08; // route UI data-ready to INT1

// Sensitivities for the selected ranges (= 32768 / full-scale).
const ACCEL_LSB_PER_G: f32 = 2048.0; // ±16g
const GYRO_LSB_PER_DPS: f32 = 16.384; // ±2000 dps

const READ: u8 = 0x80; // OR into the address byte for a read transaction

/// Output data rate (gyro/accel). The DRDY interrupt fires at this rate, so it sets the
/// control-loop cadence. Field value goes in ACCEL_CONFIG0/GYRO_CONFIG0 bits[3:0].
#[derive(Clone, Copy)]
pub enum ImuOdr {
    Hz200,
    Hz500,
    Hz1000,
}

impl ImuOdr {
    const fn reg(self) -> u8 {
        match self {
            ImuOdr::Hz200 => 0x07,
            ImuOdr::Hz500 => 0x0F,
            ImuOdr::Hz1000 => 0x06,
        }
    }

    pub const fn hz(self) -> u32 {
        match self {
            ImuOdr::Hz200 => 200,
            ImuOdr::Hz500 => 500,
            ImuOdr::Hz1000 => 1000,
        }
    }
}

/// One scaled sample.
#[derive(Clone, Copy)]
pub struct ImuSample {
    pub accel_g: [f32; 3],
    pub gyro_dps: [f32; 3],
    pub temp_c: f32,
}

pub struct Imu {
    spi: Spi1,
    cs: ErasedPin<Output<PushPull>>,
}

impl Imu {
    /// Take ownership of the SPI1 bus and the (active-low) CS pin. CS idles high.
    pub fn new(spi: Spi1, mut cs: ErasedPin<Output<PushPull>>) -> Self {
        cs.set_high();
        Self { spi, cs }
    }

    fn read_reg(&mut self, reg: u8) -> u8 {
        let mut b = [reg | READ, 0];
        self.cs.set_low();
        let _ = self.spi.transfer(&mut b);
        self.cs.set_high();
        b[1]
    }

    fn write_reg(&mut self, reg: u8, val: u8) {
        self.cs.set_low();
        let _ = self.spi.write(&[reg, val]);
        self.cs.set_high();
    }

    /// Burst-read starting at `start`. `buf[0]` holds the command byte; data lands in `buf[1..]`.
    fn read_burst(&mut self, start: u8, buf: &mut [u8]) {
        buf[0] = start | READ;
        self.cs.set_low();
        let _ = self.spi.transfer(buf);
        self.cs.set_high();
    }

    pub fn who_am_i(&mut self) -> u8 {
        self.read_reg(WHO_AM_I)
    }

    /// Soft-reset to a known state (matters because we reboot/reflash often, not just power-cycle).
    /// Caller must wait ~2 ms afterwards before further access.
    pub fn soft_reset(&mut self) {
        self.write_reg(DEVICE_CONFIG, 0x01);
    }

    /// Configure for a control loop: set range/ODR + UI filtering, route data-ready to INT1, and
    /// power on gyro+accel in Low-Noise mode. All register writes happen while the sensors are
    /// still off (PWR_MGMT0 is written last), as the datasheet requires for the config registers.
    /// The DRDY interrupt then fires on INT1 at `odr` once the gyro has started (~50 ms).
    pub fn configure_control_mode(&mut self, odr: ImuOdr) {
        self.write_reg(REG_BANK_SEL, 0x00); // ensure bank 0
        let cfg = FS_SEL | odr.reg();
        self.write_reg(ACCEL_CONFIG0, cfg);
        self.write_reg(GYRO_CONFIG0, cfg);
        self.write_reg(GYRO_ACCEL_CONFIG0, UI_FILT_BW_ODR_4); // AAF stays enabled (default)
        self.write_reg(INT_CONFIG, INT_CONFIG_VAL);
        self.write_reg(INT_CONFIG1, 0x00); // clear default INT_ASYNC_RESET (bit4)
        self.write_reg(INT_SOURCE0, INT_SOURCE0_DRDY_INT1);
        self.write_reg(PWR_MGMT0, 0x0F); // GYRO_MODE=LN (bits[3:2]), ACCEL_MODE=LN (bits[1:0])
    }

    /// Acknowledge the latched data-ready interrupt by reading INT_STATUS, which drops INT1.
    /// Must be called each sample in latched mode, or INT1 stays asserted and no further edge fires.
    pub fn clear_interrupt(&mut self) {
        let _ = self.read_reg(INT_STATUS);
    }

    /// Read one scaled sample (big-endian 16-bit registers).
    pub fn read(&mut self) -> ImuSample {
        let mut b = [0u8; 15]; // 1 command + 14 data bytes
        self.read_burst(TEMP_DATA, &mut b);
        let be = |hi: usize, lo: usize| i16::from_be_bytes([b[hi], b[lo]]);

        let temp_raw = be(1, 2);
        let ax = be(3, 4);
        let ay = be(5, 6);
        let az = be(7, 8);
        let gx = be(9, 10);
        let gy = be(11, 12);
        let gz = be(13, 14);

        ImuSample {
            accel_g: [
                ax as f32 / ACCEL_LSB_PER_G,
                ay as f32 / ACCEL_LSB_PER_G,
                az as f32 / ACCEL_LSB_PER_G,
            ],
            gyro_dps: [
                gx as f32 / GYRO_LSB_PER_DPS,
                gy as f32 / GYRO_LSB_PER_DPS,
                gz as f32 / GYRO_LSB_PER_DPS,
            ],
            // IIM-42653 temperature transfer function.
            temp_c: temp_raw as f32 / 132.48 + 25.0,
        }
    }
}
