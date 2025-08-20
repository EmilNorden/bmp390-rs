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

pub struct EventFlags {
    por_detected: bool,
    itf_act_pt: bool,
}

impl EventFlags {
    pub fn new(por_detected: bool, itf_act_pt: bool) -> Self {
        Self { por_detected, itf_act_pt }
    }

    /// Returns [`true`] after device power-up or after a soft-reset.
    ///
    /// This value is cleared on **register** read.
    pub fn power_on_reset_detected(&self) -> bool { self.por_detected }

    /// Returns [`true`] if a serial interface transaction has occurred
    /// during a pressure or temperature conversion.
    ///
    /// This value is cleared on **register** read.
    pub fn transaction_on_pt_conversion(&self) -> bool { self.itf_act_pt }
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