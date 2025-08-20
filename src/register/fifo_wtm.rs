use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

pub struct FifoWtm;
impl Reg for FifoWtm { const ADDR: u8 = 0x15; }

impl Readable for FifoWtm {
    type Out = u16;

    const N: usize = 2;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u16::from_le_bytes([b[0], b[1] & 0b0000_0001]))
    }
}

impl Writable for FifoWtm {
    type In = u16;
    const N: usize = 2;

    fn encode(v: &Self::In, out: &mut [u8]) {
        out[0] = (v & 0xFF) as u8;
        out[1] = ((v >> 8) & 1u16) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_wtm_decode() {
        let reg = FifoWtm::decode(&[0xFF, 0x01]).unwrap();

        assert_eq!(511, reg);
    }

    #[test]
    fn fifo_wtm_encode() {
        let mut buffer = [0u8; 2];
        FifoWtm::encode(&511, &mut buffer);

        assert_eq!(buffer, [0xFF, 0x01]);
    }
}