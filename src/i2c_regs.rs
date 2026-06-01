//! Shared register-access helper for the I2C sensor drivers (`baro`, `mag`).
//!
//! Both speak the same "write the register index, then read/write bytes" protocol over an
//! `embedded_hal::blocking::i2c` bus and previously re-inlined byte-identical `read_regs`/
//! `write_reg` helpers. This wraps a bus + 7-bit address so each driver *composes* one
//! `I2cRegs` instead of duplicating the access code.
//!
//! Generic over the bus type (the HAL's `I2c<I2C2>`, `I2c<I2C4>`, …); the bound is the
//! embedded-hal 0.2 blocking traits the HAL implements (re-exported as `stm32h7xx_hal::hal`).
//! All accesses are best-effort: bus errors are dropped, matching the original drivers.

use stm32h7xx_hal::hal::blocking::i2c::{Write, WriteRead};

pub struct I2cRegs<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C> I2cRegs<I2C> {
    /// Take ownership of a configured bus and the device's 7-bit address.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }
}

impl<I2C: Write + WriteRead> I2cRegs<I2C> {
    /// Read `buf.len()` bytes starting at `reg` (register auto-increment).
    pub fn read_regs(&mut self, reg: u8, buf: &mut [u8]) {
        let _ = self.i2c.write_read(self.addr, &[reg], buf);
    }

    /// Read a single register.
    pub fn read_reg(&mut self, reg: u8) -> u8 {
        let mut b = [0u8; 1];
        self.read_regs(reg, &mut b);
        b[0]
    }

    /// Write `val` to `reg`.
    pub fn write_reg(&mut self, reg: u8, val: u8) {
        let _ = self.i2c.write(self.addr, &[reg, val]);
    }
}
