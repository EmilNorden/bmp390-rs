use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

pub struct Config;
impl Reg for Config { const ADDR: u8 = 0x1F; }

pub struct ConfigFields {
    pub(crate) iir_filter: IIRFilterCoefficient
}

impl Readable for Config {
    type Out = ConfigFields;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(ConfigFields {
            iir_filter: IIRFilterCoefficient::from((b[0] >> 1) & 0b111)
        })
    }
}

impl Writable for Config {
    type In = ConfigFields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let iir_filter:u8 = v.iir_filter.into();
        out[0] = (iir_filter & 0b111) << 1;
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum IIRFilterCoefficient {
    Coef0   = 0b000,
    Coef1   = 0b001,
    Coef3   = 0b010,
    Coef7   = 0b011,
    Coef15  = 0b100,
    Coef31  = 0b101,
    Coef63  = 0b110,
    Coef127 = 0b111,
}

impl Into<u8> for IIRFilterCoefficient
{
    fn into(self) -> u8 {
        match self {
            IIRFilterCoefficient::Coef0 =>      0b000,
            IIRFilterCoefficient::Coef1 =>      0b001,
            IIRFilterCoefficient::Coef3 =>      0b010,
            IIRFilterCoefficient::Coef7 =>      0b011,
            IIRFilterCoefficient::Coef15 =>     0b100,
            IIRFilterCoefficient::Coef31 =>     0b101,
            IIRFilterCoefficient::Coef63 =>     0b110,
            IIRFilterCoefficient::Coef127 =>    0b111,
        }
    }
}

impl From<u8> for IIRFilterCoefficient {
    fn from(field: u8) -> Self {
        match field {
            0b000 =>  IIRFilterCoefficient::Coef0,
            0b001 =>  IIRFilterCoefficient::Coef1,
            0b010 =>  IIRFilterCoefficient::Coef3,
            0b011 =>  IIRFilterCoefficient::Coef7,
            0b100 =>  IIRFilterCoefficient::Coef15,
            0b101 =>  IIRFilterCoefficient::Coef31,
            0b110 =>  IIRFilterCoefficient::Coef63,
            _ =>  IIRFilterCoefficient::Coef127,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_decode() {
        let reg = Config::decode(&[0b0000_0000]).unwrap();
        assert_eq!(IIRFilterCoefficient::Coef0, reg.iir_filter);

        let reg = Config::decode(&[0b0000_0010]).unwrap();
        assert_eq!(IIRFilterCoefficient::Coef1, reg.iir_filter);
    }

    #[test]
    fn config_encode() {
        let mut buffer = [0u8; 1];
        Config::encode(&ConfigFields {
            iir_filter: IIRFilterCoefficient::Coef0,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        Config::encode(&ConfigFields {
            iir_filter: IIRFilterCoefficient::Coef1,
        }, &mut buffer);
        assert_eq!([0b0000_0010], buffer);
    }
}