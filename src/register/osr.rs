//! ### OSR - Oversampling configuration (`0x1D`, 1 byte, R/W)
//!
//! Configures oversampling for pressure and temperature measurements.
//!
//! ### Default values
//! 0x02 (x2 oversampling for pressure, no oversampling for temperature)
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::osr::{Osr, OsrCfg, Oversampling};
//!
//! // Get and print current oversampling settings
//! let osr = device.read::<Osr>().await?;
//! println!("Pressure oversampling {:?} Temperature oversampling: {:?}", osr.osr_p, osr.osr_t);
//! 
//! // Set maximum oversampling for both
//! device.write::<Osr>(&OsrCfg { osr_p: Oversampling::X32, osr_t: Oversampling::X32 }).await?;
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg, UnexpectedValue, Writable};

/// Marker type for OSR (0x1C) register
///
/// - **Length:** 1 byte
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<Osr>()`] or [`Bmp390::write::<Osr>()`]
pub struct Osr;
impl Reg for Osr { const ADDR:u8 = 0x1C; }

/// The payload for the OSR (0x1C) register.
#[derive(Copy, Clone, Debug)]
pub struct OsrCfg {
    /// Pressure oversampling
    pub osr_p: Oversampling,
    /// Temperature oversampling
    pub osr_t: Oversampling,
}

impl Readable for Osr {
    type Out = OsrCfg;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(OsrCfg {
            osr_p: Oversampling::try_from(b[0] & 0b0000_0111)
                .map_err(|e| InvalidRegisterField::new(Self::ADDR, e.0, 0))?,
            osr_t: Oversampling::try_from((b[0] >> 3) & 0b0000_0111)
                .map_err(|e| InvalidRegisterField::new(Self::ADDR, e.0, 3))?,
        })
    }
}

impl Writable for Osr {
    type In = OsrCfg;
    fn encode(v: &Self::In, out: &mut [u8]) {
        let osr_p: u8 = v.osr_p.into();
        let osr_t: u8 = v.osr_t.into();
        out[0] = osr_p | (osr_t << 3);
    }
}

/// This enum contains all possible oversampling settings for temperature and pressure.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Oversampling {
    /// No oversampling
    X1,
    /// x2 oversampling
    X2,
    /// x4 oversampling
    X4,
    /// x8 oversampling
    X8,
    /// x16 oversampling
    X16,
    /// x32 oversampling
    X32,
}

impl TryFrom<u8> for Oversampling {
    type Error = UnexpectedValue;
    fn try_from(field: u8) -> Result<Self, Self::Error> {
        match field {
            0b000 => Ok(Oversampling::X1),
            0b001 => Ok(Oversampling::X2),
            0b010 => Ok(Oversampling::X4),
            0b011 => Ok(Oversampling::X8),
            0b100 => Ok(Oversampling::X16),
            0b101 => Ok(Oversampling::X32),
            other => Err(UnexpectedValue(other))
        }
    }
}

impl Into<u8> for Oversampling {
    fn into(self) -> u8 {
        match self {
            Oversampling::X1 => 0b000,
            Oversampling::X2 => 0b001,
            Oversampling::X4 => 0b010,
            Oversampling::X8 => 0b011,
            Oversampling::X16 => 0b100,
            Oversampling::X32 => 0b101,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn osr_decode() {
        let reg = Osr::decode(&[0b0000_0000]).unwrap();
        assert_eq!(Oversampling::X1, reg.osr_p);
        assert_eq!(Oversampling::X1, reg.osr_t);

        let reg = Osr::decode(&[0b0000_0001]).unwrap();
        assert_eq!(Oversampling::X2, reg.osr_p);
        assert_eq!(Oversampling::X1, reg.osr_t);

        let reg = Osr::decode(&[0b0000_1000]).unwrap();
        assert_eq!(Oversampling::X1, reg.osr_p);
        assert_eq!(Oversampling::X2, reg.osr_t);
    }

    #[test]
    fn osr_encode() {
        let mut buffer = [0u8; 1];
        Osr::encode(&OsrCfg {
            osr_p: Oversampling::X1,
            osr_t: Oversampling::X1,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        Osr::encode(&OsrCfg {
            osr_p: Oversampling::X2,
            osr_t: Oversampling::X1,
        }, &mut buffer);
        assert_eq!([0b0000_0001], buffer);

        Osr::encode(&OsrCfg {
            osr_p: Oversampling::X1,
            osr_t: Oversampling::X2,
        }, &mut buffer);
        assert_eq!([0b0000_1000], buffer);
    }
}