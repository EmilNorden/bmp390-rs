use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the CHIP_ID (0x00) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<ChipId>()`]
pub struct ChipId;
impl Reg for ChipId { const ADDR:u8 = 0x00; }

impl Readable for ChipId {
    type Out = u8;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0])
    }
}