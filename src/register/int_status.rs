use crate::register::{InvalidRegisterField, Readable, Reg};

pub struct IntStatus;
impl Reg for IntStatus { const ADDR: u8 = 0x11; }

#[derive(Copy, Clone, Debug)]
pub struct IntStatusFlags {
    /// Has a FIFO watermark interrupt been asserted?
    ///
    /// This value is cleared on **register** read.
    pub fwm_int: bool,

    /// Has a FIFO full interrupt been asserted?
    ///
    /// This value is cleared on **register** read.
    pub ffull_int: bool,

    /// Has a data ready interrupt asserted?
    ///
    /// This value is cleared on **register** read.
    pub drdy: bool,
}

impl Readable for IntStatus {
    type Out = IntStatusFlags;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(IntStatusFlags {
            fwm_int:    (b[0] & 0b0001) != 0,
            ffull_int:  (b[0] & 0b0010) != 0,
            drdy:       (b[0] & 0b1000) != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn int_status_decode() {
        let reg = IntStatus::decode(&[0b0001]).unwrap();
        assert!(reg.fwm_int);

        let reg = IntStatus::decode(&[0b0010]).unwrap();
        assert!(reg.ffull_int);

        let reg = IntStatus::decode(&[0b1000]).unwrap();
        assert!(reg.drdy);
    }
}