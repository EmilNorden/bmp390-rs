use crate::register::{InvalidRegisterField, Readable, Reg};

pub struct FifoData;
impl Reg for FifoData { const ADDR: u8 = 0x14; }

impl Readable for FifoData {
    type Out = u8;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}