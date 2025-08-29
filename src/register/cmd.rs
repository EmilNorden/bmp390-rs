//! ### CMD - Command register (`0x7E`, 1 byte, Write-only)
//!
//! Used to issue commands to the BMP390:
//! - Flush the FIFO buffer
//! - Soft reset the device
//!
//! ### Default values
//! N/A. This register is write-only.
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::cmd::{Cmd, CmdData::FifoFlush, CmdData::SoftReset};
//!
//! // Flush FIFO
//! device.write::<Cmd>(&FifoFlush).await?;
//!
//! // Soft reset
//! device.write::<Cmd>(&SoftReset).await?;
//!
//! # Ok(()) }
//! ```
//!
//! See also: [`Bmp390::soft_reset()`]
#![doc(alias = "CMD")]
use crate::register::{Reg, Writable};

/// Marker type for CMD (0x7E) register
pub struct Cmd;
impl Reg for Cmd { const ADDR:u8 = 0x7E; }

/// The payload for the CMD (0x7E) register.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CmdData {
    /// Flushes the FIFO buffer.
    ///
    /// Does not affect any FIFO configuration.
    FifoFlush,

    /// Triggers a reset.
    ///
    /// All user configuration settings are overwritten with their default state and the FIFO is flushed.
    SoftReset
}

impl Into<u8> for CmdData {
    fn into(self) -> u8 {
        match self {
            CmdData::FifoFlush => 0xB0,
            CmdData::SoftReset => 0xB6,
        }
    }
}

impl Writable for Cmd {
    type In = CmdData;
    fn encode(v: &Self::In, out: &mut [u8]) {
        out[0] = (*v).into();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cmd_encode() {
        let mut buffer = [0u8; 1];
        Cmd::encode(&CmdData::FifoFlush, &mut buffer);
        assert_eq!([0xB0], buffer);

        Cmd::encode(&CmdData::SoftReset, &mut buffer);
        assert_eq!([0xB6], buffer);
    }
}