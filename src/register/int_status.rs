use crate::register::{InvalidRegisterField, Readable, Reg};

pub struct IntStatus;
impl Reg for IntStatus { const ADDR: u8 = 0x11; }

pub struct IntStatusFlags {
    fwm_int: bool,
    ffull_int: bool,
    drdy: bool,
}

impl IntStatusFlags {
    pub fn new(fwm_int: bool, ffull_int: bool, drdy: bool) -> Self {
        Self { fwm_int, ffull_int, drdy }
    }
    /// Has a FIFO watermark interrupt triggered?
    ///
    /// This value is cleared on **register** read.
    pub fn fifo_watermark_interrupt(&self) -> bool { self.fwm_int }

    /// Has a FIFO full interrupt triggered?
    ///
    /// This value is cleared on **register** read.
    pub fn fifo_full_interrupt(&self) -> bool { self.ffull_int }

    /// Has a data ready interrupt triggered?
    ///
    /// This value is cleared on **register** read.
    pub fn data_ready_interrupt(&self) -> bool { self.drdy }
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