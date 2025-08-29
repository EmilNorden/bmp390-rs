//! ### PWR_CTRL - Oversampling configuration (`0x1B`, 1 byte, R/W)
//!
//! Configures power modes and enables/disables pressure and temperature sensors.
//!
//! ### Default values
//! 0x00 (Pressure and temperature sensors disabled. Power Mode = Sleep)
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::pwr_ctrl::{PwrCtrl, PwrCtrlCfg, PowerMode};
//!
//! // Enables pressure sensor and put device in Normal mode.
//! device.write::<PwrCtrl>(&PwrCtrlCfg { press_en: true, temp_en: false, mode: PowerMode::Normal }).await?;
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

/// Marker type for PWR_CTRL (0x1B) register
///
/// - **Length:** 1 byte
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<PwrCtrl>()`] or [`Bmp390::write::<PwrCtrl>()`]
pub struct PwrCtrl;
impl Reg for PwrCtrl { const ADDR:u8 = 0x1B; }

/// The payload for the PwrCtrl (0x1B) register.
#[derive(Copy, Clone, Debug)]
pub struct PwrCtrlCfg {
    /// Pressor sensor enabled/disabled.
    pub press_en: bool, 
    /// Temperature sensor enabled/disabled.
    pub temp_en: bool,
    /// The power mode of the device.
    pub mode: PowerMode,
}

impl Readable for PwrCtrl {
    type Out = PwrCtrlCfg;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        let v = b[0];
        Ok(PwrCtrlCfg {
            press_en:   v & 0b0000_0001 != 0,
            temp_en:    v & 0b0000_0010 != 0,
            mode:       PowerMode::from((v >> 4) & 0b11),
        })
    }
}

impl Writable for PwrCtrl {
    type In = PwrCtrlCfg;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.press_en {
            value |= 0b0000_0001;
        }
        if v.temp_en {
            value |= 0b0000_0010;
        }
        let mode: u8 = v.mode.into();
        value |= mode << 4;
        out[0] = value;
    }
}

/// Describes the different power modes that can be set in the PwrCtrl register.
///
/// For more information, see section 3.3 in the datasheet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerMode {
    /// Sleep mode.
    ///
    /// This is the default mode after power on reset.
    /// It is also this mode that the device returns to immediately after performing a measurement in [`Forced`] mode.
    Sleep,
    /// Forced mode.
    ///
    /// In this mode, a single measurement is performed after which the device returns to Sleep mode.
    Forced,
    /// Normal mode.
    ///
    /// In this mode, measurements are constantly performed at the rate set in the odr_sel register
    Normal,
}

impl From<u8> for PowerMode {
    fn from(field: u8) -> Self {
        match field {
            0b00 => PowerMode::Sleep,
            0b01 | 0b10 => PowerMode::Forced,
            _ => PowerMode::Normal,
        }
    }
}

impl Into<u8> for PowerMode {
    fn into(self) -> u8 {
        match self {
            PowerMode::Sleep => 0b00,
            PowerMode::Forced => 0b01,
            PowerMode::Normal => 0b11,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pwr_ctrl_decode() {
        let reg = PwrCtrl::decode(&[0b0000_0001]).unwrap();
        assert!(reg.press_en);
        assert_eq!(PowerMode::Sleep, reg.mode);

        let reg = PwrCtrl::decode(&[0b0001_0010]).unwrap();
        assert!(reg.temp_en);
        assert_eq!(PowerMode::Forced, reg.mode);

        let reg = PwrCtrl::decode(&[0b0010_0000]).unwrap();
        assert!(!reg.press_en);
        assert!(!reg.temp_en);
        assert_eq!(PowerMode::Forced, reg.mode);

        let reg = PwrCtrl::decode(&[0b0011_0000]).unwrap();

        assert_eq!(PowerMode::Normal, reg.mode);
    }

    #[test]
    fn pwr_ctrl_encode() {
        let mut buffer = [0u8; 1];
        PwrCtrl::encode(&PwrCtrlCfg {
            press_en: false,
            temp_en: false,
            mode: PowerMode::Sleep,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        PwrCtrl::encode(&PwrCtrlCfg {
            press_en: true,
            temp_en: false,
            mode: PowerMode::Forced,
        }, &mut buffer);
        assert_eq!([0b0001_0001], buffer);

        PwrCtrl::encode(&PwrCtrlCfg {
            press_en: false,
            temp_en: true,
            mode: PowerMode::Normal,
        }, &mut buffer);
        assert_eq!([0b0011_0010], buffer);
    }
}