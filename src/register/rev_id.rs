//! ### REV_ID - Revision ID (`0x01`, 1 byte, Read-only)
//!
//! Contains the mask revision of the ASIC. Where `CHIP_ID` can be used to determine device model, `REV_ID` lets you differentiate versions of that device.
//!
//! ### Default values
//! 0x01
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::rev_id::RevId;
//!
//! // Get and print this device revision ID
//! let rev_id = device.read::<RevId>().await?;
//! println!("{}", rev_id);
//!
//! # Ok(()) }
//! ```
use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for the REV_ID (0x01) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<RevId>()`]
pub struct RevId;
impl Reg for RevId { const ADDR:u8 = 0x01; }

impl Readable for RevId {
    type Out = u8;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0])
    }
}