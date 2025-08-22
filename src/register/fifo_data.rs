use crate::register::{InvalidRegisterField, Readable, Reg};

pub struct FifoData<const BURST_SIZE: usize>;
impl<const BURST_SIZE: usize> Reg for FifoData<BURST_SIZE> { const ADDR: u8 = 0x14; }

impl<const BURST_SIZE: usize> Readable for FifoData<BURST_SIZE> {
    type Out = [u8; BURST_SIZE];

    const N: usize = BURST_SIZE;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0..BURST_SIZE].try_into().unwrap())
    }
}