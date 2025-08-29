//! ### IF_CONF - Interface configuration (`0x1A`, 1 byte, R/W)
//!
//! Contains configuration related to the communications interface that BMP390 supports (SPI/I2C).
//! Specifically, it is here you configure 3-wire SPI and the internal I2C watchdog.
//!
//! The I2C watchdog monitors I2C transactions to reset the I2C state machine if a transaction is not ended in a timely fashion.
//!
//! ### Default values
//! 0x00 (4-wire SPI and no I2C watchdog)
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::if_conf::{IfConf, IfConfFields, I2cWatchdogTimer};
//!
//! // Enable I2C watchdog with a short timeout
//! device.write::<IfConf>(&IfConfFields {
//!     i2c_wdt_en: true,
//!     i2c_wdt_sel: I2cWatchdogTimer::WdtShort,
//!     spi3: false,
//! }).await?;
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

/// Marker type for IF_CONF (0x1A) register
///
/// - **Length:** 1 byte
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<IfConf>()`] or [`Bmp390::write::<IfConf>()`]
pub struct IfConf;
impl Reg for IfConf { const ADDR: u8 = 0x1A; }

/// The payload for the IF_CONF (0x1A) register.
#[derive(Copy, Clone, Debug)]
pub struct IfConfFields {
    /// Determines if the SPI bus is 3-wire or 4-wire. Set to true for 3-wire, false for 4-wire.
    pub spi3: bool,

    /// Enables or disables the I2C watchdog timer.
    ///
    /// This will enable the BMP390 to detect I2C transactions where no proper end (STOP) is seen within
    /// a selected timeout, and will reset the sensors I2C state machine.
    pub i2c_wdt_en: bool,

    /// Sets the timer period for the I2C watchdog.
    ///
    /// This determines after how long the I2C state machine will be reset after it detects a transaction that is not properly ended.
    pub i2c_wdt_sel: I2cWatchdogTimer,
}

/// Represents the different timer periods for the I2C watchdog timer.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum I2cWatchdogTimer {
    /// I2C Watchdog will timeout after 1.25 ms.
    WdtShort = 0,
    /// I2C Watchdog will timeout after 40 ms.
    WdtLong = 1,
}

impl From<u8> for I2cWatchdogTimer {
    fn from(field: u8) -> Self {
        match field {
            0 =>  I2cWatchdogTimer::WdtShort,
            _ => I2cWatchdogTimer::WdtLong,
        }
    }
}

impl Readable for IfConf {
    type Out = IfConfFields;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(IfConfFields {
            spi3:       (b[0] & 0b001) != 0,
            i2c_wdt_en: (b[0] & 0b010) != 0,
            i2c_wdt_sel: I2cWatchdogTimer::from((b[0] >> 2) & 0b001)
        })
    }
}

impl Writable for IfConf {
    type In = IfConfFields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.spi3 { value |= 0b001; }
        if v.i2c_wdt_en { value |= 0b010; }
        value |= (v.i2c_wdt_sel as u8) << 2;

        out[0] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn if_conf_decode() {
        let reg = IfConf::decode(&[0b0000_0001]).unwrap();
        assert!(reg.spi3);
        assert_eq!(I2cWatchdogTimer::WdtShort, reg.i2c_wdt_sel);

        let reg = IfConf::decode(&[0b0000_0010]).unwrap();
        assert!(reg.i2c_wdt_en);

        let reg = IfConf::decode(&[0b0000_0100]).unwrap();
        assert_eq!(I2cWatchdogTimer::WdtLong, reg.i2c_wdt_sel);
    }

    #[test]
    fn if_conf_encode() {
        let mut buffer = [0u8; 1];

        IfConf::encode(&IfConfFields {
            spi3: false,
            i2c_wdt_en: false,
            i2c_wdt_sel: I2cWatchdogTimer::WdtShort,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        IfConf::encode(&IfConfFields {
            spi3: true,
            i2c_wdt_en: false,
            i2c_wdt_sel: I2cWatchdogTimer::WdtLong,
        }, &mut buffer);
        assert_eq!([0b0000_0101], buffer);

        IfConf::encode(&IfConfFields {
            spi3: true,
            i2c_wdt_en: true,
            i2c_wdt_sel: I2cWatchdogTimer::WdtLong,
        }, &mut buffer);
        assert_eq!([0b0000_0111], buffer);
    }
}