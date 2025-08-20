use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

pub struct FifoConfig1;
impl Reg for FifoConfig1 { const ADDR: u8 = 0x17; }

pub struct FifoConfig1Fields {
    pub fifo_mode: bool,
    pub fifo_stop_on_full: bool,
    pub fifo_time_en: bool,
    pub fifo_press_en: bool,
    pub fifo_temp_en: bool,
}
impl Readable for FifoConfig1 {
    type Out = FifoConfig1Fields;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(FifoConfig1Fields {
            fifo_mode:          (b[0] & 0b00001) != 0,
            fifo_stop_on_full:  (b[0] & 0b00010) != 0,
            fifo_time_en:       (b[0] & 0b00100) != 0,
            fifo_press_en:      (b[0] & 0b01000) != 0,
            fifo_temp_en:       (b[0] & 0b10000) != 0,
        })
    }
}

impl Writable for FifoConfig1 {
    type In = FifoConfig1Fields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.fifo_mode { value |= 0b1;}
        if v.fifo_stop_on_full { value |= 0b10; }
        if v.fifo_time_en { value |= 0b100; }
        if v.fifo_press_en { value |= 0b1000; }
        if v.fifo_temp_en { value |= 0b10000; }

        out[0] = value;
    }
}

pub struct FifoConfig2;
impl Reg for FifoConfig2 { const ADDR: u8 = 0x18; }

pub struct FifoConfig2Fields {
    fifo_subsampling: u8,
    data_select: FifoDataSource,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FifoDataSource {
    Unfiltered = 0,
    Filtered = 1,
    Reserved = 2,
}

impl From<u8> for FifoDataSource {
    fn from(field: u8) -> Self {
        match field {
            0b00 => FifoDataSource::Unfiltered,
            0b01 => FifoDataSource::Filtered,
            _ => FifoDataSource::Reserved
        }
    }
}

impl Readable for FifoConfig2 {
    type Out = FifoConfig2Fields;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(FifoConfig2Fields {
            fifo_subsampling: b[0] & 0b111,
            data_select: FifoDataSource::from((b[0] & 0b11000) >> 3),
        })
    }
}

impl Writable for FifoConfig2 {
    type In = FifoConfig2Fields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = v.fifo_subsampling & 0b111;
        value |= (v.data_select as u8) << 3;

        out[0] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_config1_decode() {
        let reg = FifoConfig1::decode(&[0b00001]).unwrap();
        assert!(reg.fifo_mode);

        let reg = FifoConfig1::decode(&[0b00010]).unwrap();
        assert!(reg.fifo_stop_on_full);

        let reg = FifoConfig1::decode(&[0b00100]).unwrap();
        assert!(reg.fifo_time_en);

        let reg = FifoConfig1::decode(&[0b01000]).unwrap();
        assert!(reg.fifo_press_en);

        let reg = FifoConfig1::decode(&[0b10000]).unwrap();
        assert!(reg.fifo_temp_en);
    }

    #[test]
    fn fifo_config1_encode() {
        let mut buffer = [0u8; 1];

        FifoConfig1::encode(&FifoConfig1Fields {
            fifo_mode: false,
            fifo_stop_on_full: false,
            fifo_time_en: false,
            fifo_press_en: false,
            fifo_temp_en: false,
        }, &mut buffer);
        assert_eq!([0], buffer);

        FifoConfig1::encode(&FifoConfig1Fields {
            fifo_mode: true,
            fifo_stop_on_full: false,
            fifo_time_en: true,
            fifo_press_en: false,
            fifo_temp_en: true,
        }, &mut buffer);
        assert_eq!([0b10101], buffer)
    }

    #[test]
    fn fifo_config2_decode() {
        let reg = FifoConfig2::decode(&[0b000_0100]).unwrap();
        assert_eq!(0b100, reg.fifo_subsampling);
        assert_eq!(FifoDataSource::Unfiltered, reg.data_select);

        let reg = FifoConfig2::decode(&[0b000_1010]).unwrap();
        assert_eq!(0b010, reg.fifo_subsampling);
        assert_eq!(FifoDataSource::Filtered, reg.data_select);

        let reg = FifoConfig2::decode(&[0b001_1000]).unwrap();
        assert_eq!(0, reg.fifo_subsampling);
        assert_eq!(FifoDataSource::Reserved, reg.data_select);
    }

    #[test]
    fn fifo_config2_encode() {
        let mut buffer = [0u8; 1];
        FifoConfig2::encode(&FifoConfig2Fields {
            fifo_subsampling: 0,
            data_select: FifoDataSource::Unfiltered,
        }, &mut buffer);
        assert_eq!([0], buffer);

        FifoConfig2::encode(&FifoConfig2Fields {
            fifo_subsampling: 5,
            data_select: FifoDataSource::Filtered,
        }, &mut buffer);
        assert_eq!([0b0000_1101], buffer);

        FifoConfig2::encode(&FifoConfig2Fields {
            fifo_subsampling: 5,
            data_select: FifoDataSource::Reserved,
        }, &mut buffer);
        assert_eq!([0b0001_0101], buffer);
    }
}