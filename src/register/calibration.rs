use crate::register::{InvalidRegisterField, Readable, Reg};

pub struct Calibration;
impl Reg for Calibration { const ADDR:u8 = 0x31; }

#[derive(Copy, Clone, Debug)]
pub struct CalibrationNvm {
    pub(crate) nvm_par_t1: u16,
    pub(crate) nvm_par_t2: u16,
    pub(crate) nvm_par_t3: i8,
    pub(crate) nvm_par_p1: i16,
    pub(crate) nvm_par_p2: i16,
    pub(crate) nvm_par_p3: i8,
    pub(crate) nvm_par_p4: i8,
    pub(crate) nvm_par_p5: u16,
    pub(crate) nvm_par_p6: u16,
    pub(crate) nvm_par_p7: i8,
    pub(crate)  nvm_par_p8: i8,
    pub(crate) nvm_par_p9: i16,
    pub(crate) nvm_par_p10: i8,
    pub(crate) nvm_par_p11: i8,
}

impl Readable for Calibration {
    type Out = CalibrationNvm;

    const N: usize = 21;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(CalibrationNvm {
            nvm_par_t1: u16::from_le_bytes([b[0], b[1]]),
            nvm_par_t2: u16::from_le_bytes([b[2], b[3]]),
            nvm_par_t3: i8::from_le_bytes([b[4]]),
            nvm_par_p1: i16::from_le_bytes([b[5], b[6]]),
            nvm_par_p2: i16::from_le_bytes([b[7], b[8]]),
            nvm_par_p3: i8::from_le_bytes([b[9]]),
            nvm_par_p4: i8::from_le_bytes([b[10]]),
            nvm_par_p5: u16::from_le_bytes([b[11], b[12]]),
            nvm_par_p6: u16::from_le_bytes([b[13], b[14]]),
            nvm_par_p7: i8::from_le_bytes([b[15]]),
            nvm_par_p8: i8::from_le_bytes([b[16]]),
            nvm_par_p9: i16::from_le_bytes([b[17], b[18]]),
            nvm_par_p10: i8::from_le_bytes([b[19]]),
            nvm_par_p11: i8::from_le_bytes([b[20]]),
        })
    }
}

#[cfg(test)]
mod tests {
    // TODO Add tests here
}