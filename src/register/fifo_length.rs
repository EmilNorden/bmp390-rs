//! ### FIFO_LENGTH - FIFO buffer length in bytes (`0x12`, 2 byte, Read-only)
//!
//! Contains the FIFO buffer length. The FIFO buffer length is a 9-bit value split into two registers, `FIFO_LENGTH_0` and `FIFO_LENGTH_1`.
//! This module exposes a single marker type [`FifoLength`] with address `0x12` and length `2`, and handles the logistics of combining/splitting these during encoding/decoding.
//!
//! Note that the length is in *bytes*, not in the total number of frames in the FIFO. This means that the buffer is considered full when it is >= 504.
//!
//! ### Default values
//! 0x00
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::fifo_length::FifoLength;
//!
//! // Print length as a 16-bit unsigned integer
//! let length = device.read::<FifoLength>().await?;
//! println!("{}", length);
//!
//! # Ok(()) }
//! ```
//!
//! See also: [`Bmp390::fifo_length()`](crate::Bmp390::fifo_length())
use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for FIFO_LENGTH_0 / FIFO_LENGTH_1 (0x12-0x13) register.
///
/// - **Length:** 2 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<FifoLength>()`]
///
/// This register holds a 9-bit value split across two registers. This marker will read/write to both.
pub struct FifoLength;
impl Reg for FifoLength { const ADDR: u8 = 0x12; }

impl Readable for FifoLength {
    type Out = u16;

    const N: usize = 2;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u16::from_le_bytes([b[0], b[1] & 0b0000_0001]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_length_decode() {
        let reg = FifoLength::decode(&[0xFF, 0x01]).unwrap();

        assert_eq!(511, reg);
    }
}