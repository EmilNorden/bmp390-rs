use crate::register::{InvalidRegisterField, Readable, Reg};

pub struct FifoLength;
impl Reg for FifoLength { const ADDR: u8 = 0x12; }

impl Readable for FifoLength {
    type Out = u16;

    const N: usize = 2;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u16::from_le_bytes([b[0], b[1] & 0b0000_0001]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_length_decode() {
        let reg = FifoLength::decode(&[0xFF, 0x01]).unwrap();

        assert_eq!(511, reg);
    }
}