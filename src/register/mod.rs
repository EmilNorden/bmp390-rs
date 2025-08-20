pub mod chip_id;
pub mod rev_id;
pub mod err_reg;
pub mod status;
pub mod data;
pub mod sensor_time;
pub mod event;
pub mod int_status;
pub mod fifo_length;
pub mod fifo_data;
pub mod fifo_wtm;
pub mod fifo_config;
pub mod int_ctrl;
pub mod if_conf;
pub mod pwr_ctrl;
pub mod osr;
pub mod odr;
pub mod config;
pub mod calibration;
pub mod cmd;

#[derive(Debug)]
pub struct InvalidRegisterField{
    pub register: u8,
    pub value: u8,
    pub bit_offset: u8,
}

impl InvalidRegisterField {
    pub fn new(register: u8, value: u8, bit_offset: u8) -> Self {
        Self { register, value, bit_offset }
    }
}

pub struct UnexpectedValue(pub u8);

pub trait Reg { const ADDR: u8; }

pub trait Readable: Reg {
    type Out;
    const N: usize = 1;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField>;
}

pub trait Writable: Reg {
    type In;
    const N: usize = 1;
    fn encode(v: &Self::In, out: &mut [u8]);
}