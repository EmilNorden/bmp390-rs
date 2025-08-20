use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

pub struct IntCtrl;
impl Reg for IntCtrl { const ADDR: u8 = 0x19; }

pub struct IntCtrlCfg {
    int_od: bool,
    int_level: bool,
    int_latch: bool,
    fwtm_en: bool,
    ffull_en: bool,
    int_ds: bool,
    drdy_en: bool,
}

impl Readable for IntCtrl {
    type Out = IntCtrlCfg;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(IntCtrlCfg {
            int_od:     (b[0] & 0b0000_0001) != 0,
            int_level:  (b[0] & 0b0000_0010) != 0,
            int_latch:  (b[0] & 0b0000_0100) != 0,
            fwtm_en:    (b[0] & 0b0000_1000) != 0,
            ffull_en:   (b[0] & 0b0001_0000) != 0,
            int_ds:     (b[0] & 0b0010_0000) != 0,
            drdy_en:    (b[0] & 0b0100_0000) != 0,
        })
    }
}

impl Writable for IntCtrl {
    type In = IntCtrlCfg;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.int_od { value |= 0b0000_0001; }
        if v.int_level { value |= 0b0000_0010;}
        if v.int_latch { value |= 0b0000_0100; }
        if v.fwtm_en { value |= 0b0000_1000; }
        if v.ffull_en { value |= 0b0001_0000; }
        if v.int_ds { value |= 0b0010_0000; }
        if v.drdy_en { value |= 0b0100_0000; }

        out[0] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn int_ctrl_decode() {
        let reg = IntCtrl::decode(&[0b0000_0001]).unwrap();
        assert!(reg.int_od);

        let reg = IntCtrl::decode(&[0b0000_0010]).unwrap();
        assert!(reg.int_level);

        let reg = IntCtrl::decode(&[0b0000_0100]).unwrap();
        assert!(reg.int_latch);

        let reg = IntCtrl::decode(&[0b0000_1000]).unwrap();
        assert!(reg.fwtm_en);

        let reg = IntCtrl::decode(&[0b0001_0000]).unwrap();
        assert!(reg.ffull_en);

        let reg = IntCtrl::decode(&[0b0010_0000]).unwrap();
        assert!(reg.int_ds);

        let reg = IntCtrl::decode(&[0b0100_0000]).unwrap();
        assert!(reg.drdy_en);
    }

    fn int_ctrl_encode() {
        let mut buffer = [0u8; 1];
        IntCtrl::encode(&IntCtrlCfg {
            int_od: true,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_0001], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: true,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_0010], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: true,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_0100], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: true,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_1000], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: true,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0001_0000], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: true,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0010_0000], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: true,
        }, &mut buffer);
        assert_eq!([0b0100_0000], buffer);
    }
}