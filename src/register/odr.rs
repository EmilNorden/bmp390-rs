use crate::register::{InvalidRegisterField, Readable, Reg, UnexpectedValue, Writable};

pub struct Odr;
impl Reg for Odr { const ADDR:u8 = 0x1D; }

pub struct OdrCfg {
    pub odr_sel: OutputDataRate
}

impl Readable for Odr {
    type Out = OdrCfg;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(OdrCfg {
            odr_sel: OutputDataRate::try_from(b[0] & 0b0001_1111)
                .map_err(|e| InvalidRegisterField::new(Self::ADDR, e.0, 0))?
        })
    }
}

impl Writable for Odr {
    type In = OdrCfg;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let odr_sel:u8 = v.odr_sel.into();
        out[0] = odr_sel & 0b11111;
    }
}

/// This enum defines the different output data rates that can be set in the ODR (0x1D) register.
///
/// You can find more information under section 4.3.19 in the datasheet.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum OutputDataRate {
    R200Hz      = 0x00,
    R100Hz      = 0x01,
    R50Hz       = 0x02,
    R25Hz       = 0x03,
    R12p5Hz     = 0x04,
    R6p25Hz     = 0x05,
    R3p1Hz      = 0x06,
    R1p5Hz      = 0x07,
    R0p78Hz     = 0x08,
    R0p39Hz     = 0x09,
    R0p2Hz      = 0x0A,
    R0p1Hz      = 0x0B,
    R0p05Hz     = 0x0C,
    R0p02Hz     = 0x0D,
    R0p01Hz     = 0x0E,
    R0p006Hz    = 0x0F,
    R0p003Hz    = 0x10,
    R0p0015Hz   = 0x11,
}

impl TryFrom<u8> for OutputDataRate {
    type Error = UnexpectedValue;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(OutputDataRate::R200Hz),
            0x01 => Ok(OutputDataRate::R100Hz),
            0x02 => Ok(OutputDataRate::R50Hz),
            0x03 => Ok(OutputDataRate::R25Hz),
            0x04 => Ok(OutputDataRate::R12p5Hz),
            0x05 => Ok(OutputDataRate::R6p25Hz),
            0x06 => Ok(OutputDataRate::R3p1Hz),
            0x07 => Ok(OutputDataRate::R1p5Hz),
            0x08 => Ok(OutputDataRate::R0p78Hz),
            0x09 => Ok(OutputDataRate::R0p39Hz),
            0x0A => Ok(OutputDataRate::R0p2Hz),
            0x0B => Ok(OutputDataRate::R0p1Hz),
            0x0C => Ok(OutputDataRate::R0p05Hz),
            0x0D => Ok(OutputDataRate::R0p02Hz),
            0x0E => Ok(OutputDataRate::R0p01Hz),
            0x0F => Ok(OutputDataRate::R0p006Hz),
            0x10 => Ok(OutputDataRate::R0p003Hz),
            0x11 => Ok(OutputDataRate::R0p0015Hz),
            _ => Err(UnexpectedValue(value))
        }
    }
}

impl Into<u8> for OutputDataRate {
    fn into(self) -> u8 {
        self as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odr_decode() {
        let reg = Odr::decode(&[0b0000_0000]).unwrap();
        assert_eq!(OutputDataRate::R200Hz, reg.odr_sel);

        let reg = Odr::decode(&[0b0000_0001]).unwrap();
        assert_eq!(OutputDataRate::R100Hz, reg.odr_sel);
    }

    #[test]
    fn odr_encode() {
        let mut buffer = [0u8; 1];
        Odr::encode(&OdrCfg {
            odr_sel: OutputDataRate::R200Hz,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        Odr::encode(&OdrCfg {
            odr_sel: OutputDataRate::R100Hz,
        }, &mut buffer);
        assert_eq!([0b0000_0001], buffer);
    }
}