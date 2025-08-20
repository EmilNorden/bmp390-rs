use crate::register::{InvalidRegisterField, Readable, Reg};

pub struct SensorTime;
impl Reg for SensorTime { const ADDR: u8 = 0x0C; }

impl Readable for SensorTime {
    type Out = u32;
    const N: usize = 3;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u32::from_le_bytes([b[0], b[1], b[2], 0]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_time_decode() {
        let reg = SensorTime::decode(&[0xAA, 0xBB, 0xCC]).unwrap();

        assert_eq!(0xCCBBAA, reg);
    }
}