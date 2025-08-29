//! ### FIFO_WTM - FIFO watermark level (`0x15`, 2 byte, R/W)
//!
//! Contains the FIFO "watermark level", i.e. the number of bytes that the FIFO must contain before the FIFO Watermark interrupt ([`Interrupts::fifo_watermark()`](crate::Interrupts::fifo_watermark())) is asserted.
//! 
//! This is useful if you don't want to wait for the FIFO to be full (using [`Interrupts::fifo_full()`](crate::Interrupts::fifo_full())), and instead want to be notified when it reaches another level.
//! 
//! The FIFO watermark level is a 9-bit value split into two registers, `FIFO_WTM_0` and `FIFO_WTM_1`.
//! This module exposes a single marker type [`FifoWtm`] with address `0x15` and length `2`, and handles the logistics of combining/splitting these during encoding/decoding.
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
//! use bmp390_rs::register::fifo_wtm::FifoWtm;
//! use bmp390_rs::Interrupts;
//!
//! // Sets a new watermark level at slightly below half full
//! device.write::<FifoWtm>(&200).await?;
//! 
//! // Enables the watermark interrupt and disables the other 2 interrupts.
//! device.mask_interrupts(Interrupts::new().fifo_full().data_ready()).await?;
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

/// Marker type for FIFO_WTM_0 / FIFO_WTM_1 (0x15-0x16) registers
///
/// - **Length:** 2 bytes
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<FifoWtm>()`] or [`Bmp390::write::<FifoWtm>()`]
///
/// This register holds a 9-bit value split across two registers. This marker will read/write to both.
pub struct FifoWtm;
impl Reg for FifoWtm { const ADDR: u8 = 0x15; }

impl Readable for FifoWtm {
    type Out = u16;

    const N: usize = 2;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u16::from_le_bytes([b[0], b[1] & 0b0000_0001]))
    }
}

impl Writable for FifoWtm {
    type In = u16;
    const N: usize = 2;

    fn encode(v: &Self::In, out: &mut [u8]) {
        out[0] = (v & 0xFF) as u8;
        out[1] = ((v >> 8) & 1u16) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_wtm_decode() {
        let reg = FifoWtm::decode(&[0xFF, 0x01]).unwrap();

        assert_eq!(511, reg);
    }

    #[test]
    fn fifo_wtm_encode() {
        let mut buffer = [0u8; 2];
        FifoWtm::encode(&511, &mut buffer);

        assert_eq!(buffer, [0xFF, 0x01]);
    }
}