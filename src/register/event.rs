use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the EVENT (0x10) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Event>()`] or the convenience method
/// [`Bmp390::status`].
pub struct Event;
impl Reg for Event { const ADDR: u8 = 0x10;}

#[derive(Copy, Clone, Debug)]
pub struct EventFlags {
    /// Returns [`true`] after device power-up or after a soft-reset.
    ///
    /// This value is cleared on **register** read.
    pub por_detected: bool,

    /// Returns [`true`] if a serial interface transaction has occurred
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