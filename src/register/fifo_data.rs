//! ### FIFO_DATA - FIFO buffer data (`0x14`, 1 byte, Read-only)
//!
//! Contains the FIFO buffer data. Data in the FIFO is stored as frames, with a header and payload.
//!
//! This register behaves a little different from other registers,
//! in the sense that it is a 1-byte register, but it allows for multibyte reads. See documentation for [`FifoData`] for more information
//!
//! ### Default values
//! N/A
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::fifo_data::FifoData;
//!
//! // Read 1 byte of the FIFO frame to get the header.
//! let header = device.read::<FifoData<1>>().await?;
//!
//! // Once you have determined what type of frame it is based on the header, you can read the entire frame.
//! let frame = device.read::<FifoData<7>>().await?;
//!
//! # Ok(()) }
//! ```
//!
//! See also: [`Bmp390::read_fifo_frame()`](crate::Bmp390::read_fifo_frame())

use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for FIFO_DATA (0x14) register
///
/// - **Length:** Variable. Maximum 512.
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<FifoData<1>>()`] for a read of size 1.
///
/// The FIFO_DATA register is special in the sense that for all other registers, BMP390 will auto-increment the address counter when reading multiple bytes (burst-read).
/// But for FIFO_DATA, datasheet mentions this:
/// "During burst read, the address counter stops incrementing when FIFO_DATA address is reached; This allows a complete reading of the FIFO content within one burst read transaction.
///
/// For this reason, the FifoData has a const generic parameter [`BURST_SIZE`] to allow reading FIFO frames of different sizes, or even partial frames.
///
/// **Note:** Only when a frame has been read in its entirety is it removed from the FIFO. That is why if you read 1 byte at a time, you will keep reading the same header byte for the first FIFO frame.
pub struct FifoData<const BURST_SIZE: usize>;
impl<const BURST_SIZE: usize> Reg for FifoData<BURST_SIZE> { const ADDR: u8 = 0x14; }

impl<const BURST_SIZE: usize> Readable for FifoData<BURST_SIZE> {
    type Out = [u8; BURST_SIZE];

    const N: usize = BURST_SIZE;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0..BURST_SIZE].try_into().unwrap())
    }
}