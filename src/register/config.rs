//! ### CONFIG - IIR filter configuration (`0x1F`, 1 byte, R/W)
//!
//! Controls the IIR filter coefficients.
//!
//! ### Default values
//! 0x00 (Bypass mode / No filter)
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::config::{Config, ConfigFields, IIRFilterCoefficient};
//!
//! // Print current IIR filter coefficient
//! let data = device.read::<Config>().await?;
//! println!("{:?}", data.iir_filter);
//! 
//! // Write new coefficient:
//! device.write::<Config>(&ConfigFields { iir_filter: IIRFilterCoefficient::Coef3 }).await?;
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

/// Marker type for CONFIG (0x1F) register
pub struct Config;
impl Reg for Config { const ADDR: u8 = 0x1F; }

/// The payload for the CONFIG (0x1F) register.
#[derive(Copy, Clone, Debug)]
pub struct ConfigFields {
    /// The IIR filter coefficient.
    ///
    /// Read more about the IIR filter in the datasheet section 3.4.3
    pub iir_filter: IIRFilterCoefficient
}

impl Readable for Config {
    type Out = ConfigFields;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(ConfigFields {
            iir_filter: IIRFilterCoefficient::from((b[0] >> 1) & 0b111)
        })
    }
}

impl Writable for Config {
    type In = ConfigFields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let iir_filter:u8 = v.iir_filter.into();
        out[0] = (iir_filter & 0b111) << 1;
    }
}

/// This enum holds all configurable IIR filter coefficients.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum IIRFilterCoefficient {
    /// Filter coefficient is 0 => bypass-mode
    Coef0   = 0b000,
    /// Filter coefficient is 1
    Coef1   = 0b001,
    /// Filter coefficient is 3
    Coef3   = 0b010,
    /// Filter coefficient is 7
    Coef7   = 0b011,
    /// Filter coefficient is 15
    Coef15  = 0b100,
    /// Filter coefficient is 31
    Coef31  = 0b101,
    /// Filter coefficient is 63
    Coef63  = 0b110,
    /// Filter coefficient is 127
    Coef127 = 0b111,
}

impl Into<u8> for IIRFilterCoefficient
{
    fn into(self) -> u8 {
        match self {
            IIRFilterCoefficient::Coef0 =>      0b000,
            IIRFilterCoefficient::Coef1 =>      0b001,
            IIRFilterCoefficient::Coef3 =>      0b010,
            IIRFilterCoefficient::Coef7 =>      0b011,
            IIRFilterCoefficient::Coef15 =>     0b100,
            IIRFilterCoefficient::Coef31 =>     0b101,
            IIRFilterCoefficient::Coef63 =>     0b110,
            IIRFilterCoefficient::Coef127 =>    0b111,
        }
    }
}

impl From<u8> for IIRFilterCoefficient {
    fn from(field: u8) -> Self {
        match field {
            0b000 =>  IIRFilterCoefficient::Coef0,
            0b001 =>  IIRFilterCoefficient::Coef1,
            0b010 =>  IIRFilterCoefficient::Coef3,
            0b011 =>  IIRFilterCoefficient::Coef7,
            0b100 =>  IIRFilterCoefficient::Coef15,
            0b101 =>  IIRFilterCoefficient::Coef31,
            0b110 =>  IIRFilterCoefficient::Coef63,
            _ =>  IIRFilterCoefficient::Coef127,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_decode() {
        let reg = Config::decode(&[0b0000_0000]).unwrap();
        assert_eq!(IIRFilterCoefficient::Coef0, reg.iir_filter);

        let reg = Config::decode(&[0b0000_0010]).unwrap();
        assert_eq!(IIRFilterCoefficient::Coef1, reg.iir_filter);
    }

    #[test]
    fn config_encode() {
        let mut buffer = [0u8; 1];
        Config::encode(&ConfigFields {
            iir_filter: IIRFilterCoefficient::Coef0,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        Config::encode(&ConfigFields {
            iir_filter: IIRFilterCoefficient::Coef1,
        }, &mut buffer);
        assert_eq!([0b0000_0010], buffer);
    }
}