use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the REV_ID (0x01) register
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