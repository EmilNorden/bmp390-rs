use crate::register::{Reg, Writable};

pub struct Cmd;
impl Reg for Cmd { const ADDR:u8 = 0x7E; }

#[derive(Copy, Clone)]
pub enum CmdData {
    FifoFlush,
    SoftReset
}

impl Into<u8> for CmdData {
    fn into(self) -> u8 {
        match self {
            CmdData::FifoFlush => 0xB0,
            CmdData::SoftReset => 0xB6,
        }
    }
}

impl Writable for Cmd {
    type In = CmdData;
    fn encode(v: &Self::In, out: &mut [u8]) {
        out[0] = (*v).into();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cmd_encode() {
        let mut buffer = [0u8; 1];
        Cmd::encode(&CmdData::FifoFlush, &mut buffer);
        assert_eq!([0xB0], buffer);

        Cmd::encode(&CmdData::SoftReset, &mut buffer);
        assert_eq!([0xB6], buffer);
    }
}