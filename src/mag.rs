//! Minimal register-level driver for the ARK FPV's IIS2MDC/LIS2MDL magnetometer on I2C4.
//!
//! Polled, no_std, no external crate. Identifies the chip, soft-resets it, configures
//! continuous-conversion mode with on-chip temperature compensation, and returns the magnetic
//! field (µT) and die temperature (°C). The IIS2MDC (ArduPilot) and LIS2MDL (Betaflight) are the
//! same ST 3-axis magnetometer family at the same address with an identical register map.
//!
//! Register addresses, bit layout, and the LSB scaling are transcribed from ST's official
//! `lis2mdl-pid` C driver (`lis2mdl_reg.h` / `lis2mdl_reg.c`): WHO_AM_I `0x4F` = `0x40`,
//! `lis2mdl_from_lsb_to_mgauss(lsb) = lsb * 1.5f`, `lis2mdl_from_lsb_to_celsius(lsb) = lsb/8 + 25`.
//!   <https://github.com/STMicroelectronics/lis2mdl-pid/blob/master/lis2mdl_reg.h>
//!   <https://github.com/STMicroelectronics/lis2mdl-pid/blob/master/lis2mdl_reg.c>

use core::future::Future;

use stm32h7xx_hal as hal;

use crate::i2c_regs::I2cRegs;

type I2c4 = hal::i2c::I2c<hal::pac::I2C4>;

/// 7-bit I2C address (fixed on this part; SDO/SA1 not used).
const ADDR: u8 = 0x1E;

// --- Registers (ST lis2mdl_reg.h) ---------------------------------------------
const REG_WHO_AM_I: u8 = 0x4F;
const REG_CFG_A: u8 = 0x60; // COMP_TEMP_EN[7], REBOOT[6], SOFT_RST[5], LP[4], ODR[3:2], MD[1:0]
const REG_CFG_B: u8 = 0x61; // OFF_CANC[1], LPF[0]
const REG_CFG_C: u8 = 0x62; // BDU[4]
const REG_STATUS: u8 = 0x67;
const REG_OUTX_L: u8 = 0x68; // X/Y/Z, 6 bytes through 0x6D, little-endian two's complement
const REG_TEMP_L: u8 = 0x6E; // 2 bytes, little-endian two's complement

/// Expected WHO_AM_I for IIS2MDC/LIS2MDL (ST `lis2mdl_reg.h` `LIS2MDL_ID`).
pub const EXPECTED_WHO_AM_I: u8 = 0x40;

// CFG_REG_A fields.
const CFG_A_COMP_TEMP_EN: u8 = 1 << 7; // on-chip hard-iron/temperature compensation
const CFG_A_SOFT_RST: u8 = 1 << 5; // reset config registers + user banks
const MD_CONTINUOUS: u8 = 0b00; // bits[1:0]: continuous-conversion mode

// CFG_REG_B fields: offset cancellation + digital low-pass filter (noise reduction in continuous
// mode; both recommended by the ST app note for a fixed-rate compass).
const CFG_B_OFF_CANC: u8 = 1 << 1;
const CFG_B_VAL: u8 = CFG_B_OFF_CANC; // match ST example exactly (offset cancel only, no LPF)

// CFG_REG_C: block data update — output regs aren't refreshed mid-read, so MSB/LSB stay coherent.
const CFG_C_BDU: u8 = 1 << 4;

/// STATUS_REG Zyxda bit: a full X/Y/Z set is ready. (We poll below the ODR, so it's normally set.)
const STATUS_ZYXDA: u8 = 1 << 3;

// LSB scaling (ST lis2mdl_reg.c). Fixed ±50 gauss full scale: 1 LSB = 1.5 mgauss. Reported in µT
// (1 gauss = 100 µT ⇒ 1 mgauss = 0.1 µT), so 1.5 mgauss = 0.15 µT/LSB.
const MAG_UT_PER_LSB: f32 = 0.15;
// Temperature: 8 LSB/°C, 25 °C reference.
const TEMP_LSB_PER_C: f32 = 8.0;
const TEMP_REF_C: f32 = 25.0;

/// Output data rate (CFG_REG_A bits[3:2]). Low-bandwidth sensor; poll at or below this rate.
#[derive(Clone, Copy)]
pub enum MagOdr {
    Hz10,
    Hz20,
    Hz50,
    Hz100,
}

impl MagOdr {
    const fn reg(self) -> u8 {
        // bits[3:2]
        match self {
            MagOdr::Hz10 => 0b00 << 2,
            MagOdr::Hz20 => 0b01 << 2,
            MagOdr::Hz50 => 0b10 << 2,
            MagOdr::Hz100 => 0b11 << 2,
        }
    }
}

/// One scaled sample.
pub struct MagSample {
    pub field_ut: [f32; 3],
    pub temp_c: f32,
}

pub struct Mag {
    regs: I2cRegs<I2c4>,
}

impl Mag {
    /// Take ownership of the configured I2C4 bus.
    pub fn new(i2c: I2c4) -> Self {
        Self {
            regs: I2cRegs::new(i2c, ADDR),
        }
    }

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) {
        self.regs.read_regs(reg, buf);
    }

    fn write_reg(&mut self, reg: u8, val: u8) {
        self.regs.write_reg(reg, val);
    }

    pub fn who_am_i(&mut self) -> u8 {
        self.regs.read_reg(REG_WHO_AM_I)
    }

    /// Soft-reset the config registers. SOFT_RST self-clears when the reset completes; the caller
    /// MUST poll [`reset_complete`](Self::reset_complete) until true before configuring. A fixed
    /// delay is not enough: if `configure` writes CFG_A before the reset finalizes, the reset then
    /// clobbers the freshly-written MD (mode) bits back to the idle default — the chip ends up in
    /// idle (no conversions, frozen data) even though COMP_TEMP_EN/ODR appear set.
    pub fn soft_reset(&mut self) {
        self.write_reg(REG_CFG_A, CFG_A_SOFT_RST);
    }

    /// True once the soft reset has finished (CFG_A SOFT_RST bit self-cleared).
    pub fn reset_complete(&mut self) -> bool {
        let mut b = [0u8; 1];
        self.read_regs(REG_CFG_A, &mut b);
        b[0] & CFG_A_SOFT_RST == 0
    }

    /// Configure block-data-update and offset cancellation, then assert continuous mode via
    /// [`start_continuous`](Self::start_continuous). CFG_REG_A (mode) is written last so continuous
    /// conversion starts only once the rest is set. NOTE: the first continuous write after a reset
    /// often fails to latch (the chip reverts MD to idle); callers MUST verify with
    /// [`is_continuous`](Self::is_continuous) and re-assert — see the bring-up loop in `main`.
    pub fn configure(&mut self, odr: MagOdr) {
        self.write_reg(REG_CFG_C, CFG_C_BDU);
        self.write_reg(REG_CFG_B, CFG_B_VAL);
        self.start_continuous(odr);
    }

    /// Full polled bring-up: soft-reset, wait for it to self-clear, configure, then re-assert
    /// continuous mode until it latches. Owns the two chip quirks the task shouldn't know about —
    /// the reset must finalize before configuring, and the first continuous write after reset
    /// often reverts MD to idle (see [`soft_reset`](Self::soft_reset) / [`configure`]). The caller
    /// supplies an async millisecond delay (e.g. `|ms| Mono::delay(ms.millis())`). Returns `true`
    /// once continuous conversion is confirmed running, `false` if it never latched.
    pub async fn bring_up<F, Fut>(&mut self, odr: MagOdr, mut delay_ms: F) -> bool
    where
        F: FnMut(u32) -> Fut,
        Fut: Future<Output = ()>,
    {
        self.soft_reset();
        for _ in 0..10 {
            delay_ms(2).await;
            if self.reset_complete() {
                break;
            }
        }
        self.configure(odr);
        for _ in 0..10 {
            delay_ms(10).await;
            if self.is_continuous() {
                return true;
            }
            self.start_continuous(odr);
        }
        false
    }

    /// Write CFG_REG_A = temperature-compensation + ODR + continuous mode. Idempotent; safe to
    /// call repeatedly to re-assert continuous mode until it latches (see [`configure`]).
    pub fn start_continuous(&mut self, odr: MagOdr) {
        self.write_reg(REG_CFG_A, CFG_A_COMP_TEMP_EN | odr.reg() | MD_CONTINUOUS);
    }

    /// True once CFG_REG_A reports continuous mode (MD bits == 0) — i.e. conversions are running.
    pub fn is_continuous(&mut self) -> bool {
        self.read_cfg().0 & 0b11 == MD_CONTINUOUS
    }

    /// True once a fresh X/Y/Z set is available (STATUS_REG Zyxda).
    pub fn data_ready(&mut self) -> bool {
        self.status() & STATUS_ZYXDA != 0
    }

    /// Raw STATUS_REG (`0x67`).
    pub fn status(&mut self) -> u8 {
        self.regs.read_reg(REG_STATUS)
    }

    /// Read back the three config registers (CFG_A, CFG_B, CFG_C) for bring-up diagnostics.
    pub fn read_cfg(&mut self) -> (u8, u8, u8) {
        let mut b = [0u8; 3];
        self.read_regs(REG_CFG_A, &mut b); // 0x60..=0x62 auto-increment
        (b[0], b[1], b[2])
    }

    /// Read the latest magnetic field (µT) and die temperature (°C).
    pub fn read(&mut self) -> MagSample {
        let mut b = [0u8; 6];
        self.read_regs(REG_OUTX_L, &mut b);
        let le = |lo: usize, hi: usize| i16::from_le_bytes([b[lo], b[hi]]) as f32;
        let field_ut = [
            le(0, 1) * MAG_UT_PER_LSB,
            le(2, 3) * MAG_UT_PER_LSB,
            le(4, 5) * MAG_UT_PER_LSB,
        ];

        let mut t = [0u8; 2];
        self.read_regs(REG_TEMP_L, &mut t);
        let temp_c = i16::from_le_bytes([t[0], t[1]]) as f32 / TEMP_LSB_PER_C + TEMP_REF_C;

        MagSample { field_ut, temp_c }
    }
}
