//! ### CHIP_ID - Chip identification number (`0x00`, 1 byte, R)
//!
//! Contains the chip identification code, which will always be 0x60 for BMP390.
//!
//! ### Default values
//! 0x60
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::chip_id::ChipId;
//!
//! // Print chip id
//! let id = device.read::<ChipId>().await?;
//! println!("{:?}", id);
//!
//! # Ok(()) }
//! ```
#![doc(alias = "CHIP_ID")]
use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the CHIP_ID (0x00) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<ChipId>()`]
pub struct ChipId;
impl Reg for ChipId { const ADDR:u8 = 0x00; }

impl Readable for ChipId {
    type Out = u8;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0])
    }
}