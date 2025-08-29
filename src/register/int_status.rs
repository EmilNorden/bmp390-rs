//! ### INT_STATUS - Interrupt status (`0x11`, 1 byte, Read-only)
//!
//! Contains flags that describes *which* interrupt asserted the INT pin:
//! - FIFO watermark
//! - FIFO full
//! - Data ready
//!
//! **Note:** The INT_STATUS register has clear-on-read semantics.
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
//! use bmp390_rs::register::int_status::IntStatus;
//!
//! // Get interrupt status
//! let int_status = device.read::<IntStatus>().await?;
//! println!("{:?}", int_status);
//!
//! # Ok(()) }
//! ```
use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for INT_STATUS (0x11) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<IntStatus>()`]
/// 
/// Note that this register has **clear-on-read** semantics.
pub struct IntStatus;
impl Reg for IntStatus { const ADDR: u8 = 0x11; }

/// The payload for the INT_STATUS (0x11) register.
#[derive(Copy, Clone, Debug)]
pub struct IntStatusFlags {
    /// Has a FIFO watermark interrupt been asserted?
    ///
    /// This value is cleared on **register** read.
    pub fwm_int: bool,

    /// Has a FIFO full interrupt been asserted?
    ///
    /// This value is cleared on **register** read.
    pub ffull_int: bool,

    /// Has a data ready interrupt asserted?
    ///
    /// This value is cleared on **register** read.
    pub drdy: bool,
}

impl Readable for IntStatus {
    type Out = IntStatusFlags;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(IntStatusFlags {
            fwm_int:    (b[0] & 0b0001) != 0,
            ffull_int:  (b[0] & 0b0010) != 0,
            drdy:       (b[0] & 0b1000) != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn int_status_decode() {
        let reg = IntStatus::decode(&[0b0001]).unwrap();
        assert!(reg.fwm_int);

        let reg = IntStatus::decode(&[0b0010]).unwrap();
        assert!(reg.ffull_int);

        let reg = IntStatus::decode(&[0b1000]).unwrap();
        assert!(reg.drdy);
    }
}