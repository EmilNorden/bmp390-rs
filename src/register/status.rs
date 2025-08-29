//! ### STATUS - Sensor status flags (`0x03`, 1 byte, Read-only)
//!
//! Contains sensor status flags.
//!
//! [`StatusFlags::cmd_rdy`] can be used to determine if the device is ready for another command using the `CMD` ([`register::cmd::Cmd`](crate::register::cmd::Cmd)) register.
//!
//! The data ready flags [`StatusFlags::drdy_press`] and [`StatusFlags::drdy_temp`] are reset when pressure or temperature data is read out from the `DATA` ([`register::data::Data`](crate::register::data::Data)) register.
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
//! use bmp390_rs::register::status::Status;
//!
//! // Get and print status flags
//! let status_flags = device.read::<Status>().await?;
//! println!("{:?}", status_flags);
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for the STATUS (0x03) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Status>()`] or the convenience method
/// [`Bmp390::status`].
pub struct Status;

impl Reg for Status { const ADDR: u8 = 0x03; }

/// The payload for the STATUS (0x03) register.
#[derive(Copy, Clone, Debug)]
pub struct StatusFlags {
    /// Is the command decoder ready to accept a new command?
    ///
    /// [`false`] means that a command is already in progress.
    pub cmd_rdy: bool,

    /// Is there new pressure data to be read?
    ///
    /// This value is cleared when any of the pressure DATA registers are read.
    pub drdy_press: bool,

    /// Is there new temperature data to be read?
    ///
    /// This value is cleared when any of the temperature DATA registers are read.
    pub drdy_temp: bool,
}

impl Readable for Status {
    type Out = StatusFlags;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(StatusFlags {
            cmd_rdy:    (b[0] & 0b0010000) != 0,
            drdy_press: (b[0] & 0b0100000) != 0,
            drdy_temp:  (b[0] & 0b1000000) != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_decode() {
        let reg = Status::decode(&[0b0010000]).unwrap();
        assert_eq!([true, false, false], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);

        let reg = Status::decode(&[0b0100000]).unwrap();
        assert_eq!([false, true, false], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);

        let reg = Status::decode(&[0b1000000]).unwrap();
        assert_eq!([false, false, true], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);
    }
}