//! ### ODR - Output data rates (`0x1D`, 1 byte, R/W)
//!
//! Configures output data rate by subdivision/subsampling
//! The base sample rate for the BMP390 is 200 Hz, and by selecting different prescalers through this register it is possible to subdivide the output data rate.
//!
//! ### Default values
//! 0x00 (Prescaler = 1, Rate = 200 Hz)
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::odr::{Odr, OdrCfg, OutputDataRate};
//!
//! // Get and print current output data rate
//! let rate = device.read::<Odr>().await?;
//! println!("{:?}", rate);
//!
//! // Set a new rate of 12.5 Hz
//! device.write::<Odr>(&OdrCfg { odr_sel: OutputDataRate::R12p5Hz}).await?;
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg, UnexpectedValue, Writable};

/// Marker type for ODR (0x1D) register
///
/// - **Length:** 1 byte
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<Odr>()`] or [`Bmp390::write::<Odr>()`]
pub struct Odr;
impl Reg for Odr { const ADDR:u8 = 0x1D; }

/// The payload for the ODR (0x1D) register.
#[derive(Copy, Clone, Debug)]
pub struct OdrCfg {
    /// The output data rate as a subdivision of the base clock @ 200 Hz.
    pub odr_sel: OutputDataRate
}

impl Readable for Odr {
    type Out = OdrCfg;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(OdrCfg {
            odr_sel: OutputDataRate::try_from(b[0] & 0b0001_1111)
                .map_err(|e| InvalidRegisterField::new(Self::ADDR, e.0, 0))?
        })
    }
}

impl Writable for Odr {
    type In = OdrCfg;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let odr_sel:u8 = v.odr_sel.into();
        out[0] = odr_sel & 0b11111;
    }
}

/// This enum defines the different output data rates that can be set in the ODR (0x1D) register.
///
/// You can find more information under section 4.3.19 in the datasheet.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum OutputDataRate {
    /// 200Hz, sampling period 5ms.
    R200Hz      = 0x00,
    /// 100Hz, sampling period 10ms.
    R100Hz      = 0x01,
    /// 50Hz, sampling period 20ms.
    R50Hz       = 0x02,
    /// 25Hz, sampling period 40ms.
    R25Hz       = 0x03,
    /// 12.5Hz, sampling period 80ms.
    R12p5Hz     = 0x04,
    /// 6.25Hz, sampling period 160ms.
    R6p25Hz     = 0x05,
    /// 25/8Hz, sampling period 320ms.
    R3p1Hz      = 0x06,
    /// 25/16Hz, sampling period 640ms.
    R1p5Hz      = 0x07,
    /// 25/32Hz, sampling period 1.280s.
    R0p78Hz     = 0x08,
    /// 25/64Hz, sampling period 2.560s.
    R0p39Hz     = 0x09,
    /// 25/128Hz, sampling period 5.120s.
    R0p2Hz      = 0x0A,
    /// 25/256Hz, sampling period 10.24s.
    R0p1Hz      = 0x0B,
    /// 25/512Hz, sampling period 20.48s.
    R0p05Hz     = 0x0C,
    /// 25/1024Hz, sampling period 40.96s.
    R0p02Hz     = 0x0D,
    /// 25/2048Hz, sampling period 81.92s.
    R0p01Hz     = 0x0E,
    /// 25/4096Hz, sampling period 163.84s.
    R0p006Hz    = 0x0F,
    /// 25/8192Hz, sampling period 327.68s.
    R0p003Hz    = 0x10,
    /// 25/16384Hz, sampling period 655.36s.
    R0p0015Hz   = 0x11,
}

impl TryFrom<u8> for OutputDataRate {
    type Error = UnexpectedValue;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(OutputDataRate::R200Hz),
            0x01 => Ok(OutputDataRate::R100Hz),
            0x02 => Ok(OutputDataRate::R50Hz),
            0x03 => Ok(OutputDataRate::R25Hz),
            0x04 => Ok(OutputDataRate::R12p5Hz),
            0x05 => Ok(OutputDataRate::R6p25Hz),
            0x06 => Ok(OutputDataRate::R3p1Hz),
            0x07 => Ok(OutputDataRate::R1p5Hz),
            0x08 => Ok(OutputDataRate::R0p78Hz),
            0x09 => Ok(OutputDataRate::R0p39Hz),
            0x0A => Ok(OutputDataRate::R0p2Hz),
            0x0B => Ok(OutputDataRate::R0p1Hz),
            0x0C => Ok(OutputDataRate::R0p05Hz),
            0x0D => Ok(OutputDataRate::R0p02Hz),
            0x0E => Ok(OutputDataRate::R0p01Hz),
            0x0F => Ok(OutputDataRate::R0p006Hz),
            0x10 => Ok(OutputDataRate::R0p003Hz),
            0x11 => Ok(OutputDataRate::R0p0015Hz),
            _ => Err(UnexpectedValue(value))
        }
    }
}

impl Into<u8> for OutputDataRate {
    fn into(self) -> u8 {
        self as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odr_decode() {
        let reg = Odr::decode(&[0b0000_0000]).unwrap();
        assert_eq!(OutputDataRate::R200Hz, reg.odr_sel);

        let reg = Odr::decode(&[0b0000_0001]).unwrap();
        assert_eq!(OutputDataRate::R100Hz, reg.odr_sel);
    }

    #[test]
    fn odr_encode() {
        let mut buffer = [0u8; 1];
        Odr::encode(&OdrCfg {
            odr_sel: OutputDataRate::R200Hz,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        Odr::encode(&OdrCfg {
            odr_sel: OutputDataRate::R100Hz,
        }, &mut buffer);
        assert_eq!([0b0000_0001], buffer);
    }
}