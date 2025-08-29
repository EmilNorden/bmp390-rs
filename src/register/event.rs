//! ### EVENT - Error conditions (`0x10`, 1 byte, Read-only)
//!
//! Contains sensor status flags.
//!
//! **Note:** The EVENT register has clear-on-read semantics.
//!
//! ### Default values
//! `por_detected = true`
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::event::Event;
//!
//! // Check for device power-up/soft-reset
//! let event = device.read::<Event>().await?;
//! if event.por_detected {
//!     println!("Good morning!");
//! }
//!
//! # Ok(()) }
//! ```

use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for the EVENT (0x10) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Event>()`]
pub struct Event;
impl Reg for Event { const ADDR: u8 = 0x10;}

/// The payload for the EVENT (0x10) register.
#[derive(Copy, Clone, Debug)]
pub struct EventFlags {
    /// Set to [`true`] after device power-up or after a soft-reset.
    ///
    /// This value is cleared on **register** read.
    pub por_detected: bool,

    /// Set to [`true`] if a serial interface transaction has occurred
    /// during a pressure or temperature conversion.
    ///
    /// This value is cleared on **register** read.
    pub itf_act_pt: bool,
}

impl Readable for Event {
    type Out = EventFlags;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(EventFlags {
            por_detected:   (b[0] & 0b01) != 0,
            itf_act_pt:     (b[0] & 0b10) != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_decode() {
        let reg = Event::decode(&[0b01]).unwrap();
        assert!(reg.por_detected);

        let reg = Event::decode(&[0b10]).unwrap();
        assert!(reg.itf_act_pt);
    }
}