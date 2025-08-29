//! ### Calibration - Calibration coefficients (`0x31`, 21 bytes, Read-only)
//!
//! To compensate for fluctuations in each individual sensing element and achieve the most accurate physical values,
//! a compensation formulae must be applied to the raw sensor output. The coefficients used in this formulae are stored in these registers.
//! These are stored as signed and unsigned integers of varying widths, and need to be transformed into floating point before they are usable.
//! This is all done by the [`Bmp390`] driver struct at initialization.
//! 
//! ### Default values
//! N/A. These are unique for each sensor so there is no *one* default value.
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::calibration::Calibration;
//!
//! // Read calibration coefficients
//! let data = device.read::<Calibration>().await?;
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for Calibration (0x31 - 0x46) registers
///
/// These registers range from 0x30 - 0x57 according to the datasheet section 4.3.22
/// however according to the memory map in section 3.11.1 the coefficients are stored in
/// 0x31 - 0x46, and those are the ones we are interested in.
pub struct Calibration;
impl Reg for Calibration { const ADDR:u8 = 0x31; }

/// The payload for the CMD (0x7E) register.
#[derive(Copy, Clone, Debug)]
pub struct CalibrationNvm {
    pub(crate) nvm_par_t1: u16,
    pub(crate) nvm_par_t2: u16,
    pub(crate) nvm_par_t3: i8,
    pub(crate) nvm_par_p1: i16,
    pub(crate) nvm_par_p2: i16,
    pub(crate) nvm_par_p3: i8,
    pub(crate) nvm_par_p4: i8,
    pub(crate) nvm_par_p5: u16,
    pub(crate) nvm_par_p6: u16,
    pub(crate) nvm_par_p7: i8,
    pub(crate) nvm_par_p8: i8,
    pub(crate) nvm_par_p9: i16,
    pub(crate) nvm_par_p10: i8,
    pub(crate) nvm_par_p11: i8,
}

impl Readable for Calibration {
    type Out = CalibrationNvm;

    const N: usize = 21;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(CalibrationNvm {
            nvm_par_t1: u16::from_le_bytes([b[0], b[1]]),
            nvm_par_t2: u16::from_le_bytes([b[2], b[3]]),
            nvm_par_t3: i8::from_le_bytes([b[4]]),
            nvm_par_p1: i16::from_le_bytes([b[5], b[6]]),
            nvm_par_p2: i16::from_le_bytes([b[7], b[8]]),
            nvm_par_p3: i8::from_le_bytes([b[9]]),
            nvm_par_p4: i8::from_le_bytes([b[10]]),
            nvm_par_p5: u16::from_le_bytes([b[11], b[12]]),
            nvm_par_p6: u16::from_le_bytes([b[13], b[14]]),
            nvm_par_p7: i8::from_le_bytes([b[15]]),
            nvm_par_p8: i8::from_le_bytes([b[16]]),
            nvm_par_p9: i16::from_le_bytes([b[17], b[18]]),
            nvm_par_p10: i8::from_le_bytes([b[19]]),
            nvm_par_p11: i8::from_le_bytes([b[20]]),
        })
    }
}

#[cfg(test)]
mod tests {
    // TODO Add tests here
}