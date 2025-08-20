use crate::register::{InvalidRegisterField, Readable, Reg, UnexpectedValue, Writable};

pub struct Osr;
impl Reg for Osr { const ADDR:u8 = 0x1C; }

#[derive(Copy, Clone, Debug)]
pub struct OsrCfg {
    pub osr_p: Oversampling,
    pub osr_t: Oversampling,
}

impl Readable for Osr {
    type Out = OsrCfg;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(OsrCfg {
            osr_p: Oversampling::try_from(b[0] & 0b0000_0111)
                .map_err(|e| InvalidRegisterField::new(Self::ADDR, e.0, 0))?,
            osr_t: Oversampling::try_from((b[0] >> 3) & 0b0000_0111)
                .map_err(|e| InvalidRegisterField::new(Self::ADDR, e.0, 3))?,
        })
    }
}

impl Writable for Osr {
    type In = OsrCfg;
    fn encode(v: &Self::In, out: &mut [u8]) {
        let osr_p: u8 = v.osr_p.into();
        let osr_t: u8 = v.osr_t.into();
        out[0] = osr_p | (osr_t << 3);
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Oversampling {
    X1,
    X2,
    X4,
    X8,
    X16,
    X32,
}

impl TryFrom<u8> for Oversampling {
    type Error = UnexpectedValue;
    fn try_from(field: u8) -> Result<Self, Self::Error> {
        match field {
            0b000 => Ok(Oversampling::X1),
            0b001 => Ok(Oversampling::X2),
            0b010 => Ok(Oversampling::X4),
            0b011 => Ok(Oversampling::X8),
            0b100 => Ok(Oversampling::X16),
            0b101 => Ok(Oversampling::X32),
            other => Err(UnexpectedValue(other))
        }
    }
}

impl Into<u8> for Oversampling {
    fn into(self) -> u8 {
        match self {
            Oversampling::X1 => 0b000,
            Oversampling::X2 => 0b001,
            Oversampling::X4 => 0b010,
            Oversampling::X8 => 0b011,
            Oversampling::X16 => 0b100,
            Oversampling::X32 => 0b101,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn osr_decode() {
        let reg = Osr::decode(&[0b0000_0000]).unwrap();
        assert_eq!(Oversampling::X1, reg.osr_p);
        assert_eq!(Oversampling::X1, reg.osr_t);

        let reg = Osr::decode(&[0b0000_0001]).unwrap();
        assert_eq!(Oversampling::X2, reg.osr_p);
        assert_eq!(Oversampling::X1, reg.osr_t);

        let reg = Osr::decode(&[0b0000_1000]).unwrap();
        assert_eq!(Oversampling::X1, reg.osr_p);
        assert_eq!(Oversampling::X2, reg.osr_t);
    }

    #[test]
    fn osr_encode() {
        let mut buffer = [0u8; 1];
        Osr::encode(&OsrCfg {
            osr_p: Oversampling::X1,
            osr_t: Oversampling::X1,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        Osr::encode(&OsrCfg {
            osr_p: Oversampling::X2,
            osr_t: Oversampling::X1,
        }, &mut buffer);
        assert_eq!([0b0000_0001], buffer);

        Osr::encode(&OsrCfg {
            osr_p: Oversampling::X1,
            osr_t: Oversampling::X2,
        }, &mut buffer);
        assert_eq!([0b0000_1000], buffer);
    }
}