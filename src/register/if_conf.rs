use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

pub struct IfConf;
impl Reg for IfConf { const ADDR: u8 = 0x1A; }

pub struct IfConfFields {
    spi3: bool,
    i2c_wdt_en: bool,
    i2c_wdt_sel: I2cWatchdogTimer,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum I2cWatchdogTimer {
    WdtShort = 0,
    WdtLong = 1,
}

impl From<u8> for I2cWatchdogTimer {
    fn from(field: u8) -> Self {
        match field {
            0 =>  I2cWatchdogTimer::WdtShort,
            _ => I2cWatchdogTimer::WdtLong,
        }
    }
}

impl Readable for IfConf {
    type Out = IfConfFields;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(IfConfFields {
            spi3:       (b[0] & 0b001) != 0,
            i2c_wdt_en: (b[0] & 0b010) != 0,
            i2c_wdt_sel: I2cWatchdogTimer::from((b[0] >> 2) & 0b001)
        })
    }
}

impl Writable for IfConf {
    type In = IfConfFields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.spi3 { value |= 0b001; }
        if v.i2c_wdt_en { value |= 0b010; }
        value |= (v.i2c_wdt_sel as u8) << 2;

        out[0] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn if_conf_decode() {
        let reg = IfConf::decode(&[0b0000_0001]).unwrap();
        assert!(reg.spi3);
        assert_eq!(I2cWatchdogTimer::WdtShort, reg.i2c_wdt_sel);

        let reg = IfConf::decode(&[0b0000_0010]).unwrap();
        assert!(reg.i2c_wdt_en);

        let reg = IfConf::decode(&[0b0000_0100]).unwrap();
        assert_eq!(I2cWatchdogTimer::WdtLong, reg.i2c_wdt_sel);
    }

    #[test]
    fn if_conf_encode() {
        let mut buffer = [0u8; 1];

        IfConf::encode(&IfConfFields {
            spi3: false,
            i2c_wdt_en: false,
            i2c_wdt_sel: I2cWatchdogTimer::WdtShort,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        IfConf::encode(&IfConfFields {
            spi3: true,
            i2c_wdt_en: false,
            i2c_wdt_sel: I2cWatchdogTimer::WdtLong,
        }, &mut buffer);
        assert_eq!([0b0000_0101], buffer);

        IfConf::encode(&IfConfFields {
            spi3: true,
            i2c_wdt_en: true,
            i2c_wdt_sel: I2cWatchdogTimer::WdtLong,
        }, &mut buffer);
        assert_eq!([0b0000_0111], buffer);
    }
}