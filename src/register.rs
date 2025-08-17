
#[derive(Copy, Clone)]
pub enum OutputDataRate {
    R200Hz,
    R100Hz,
    R50Hz,
    R25Hz,
    R12p5Hz,
    R6p25Hz,
    R3p1Hz,
    R1p5Hz,
    R0p78Hz,
    R0p39Hz,
    R0p2Hz,
    R0p1Hz,
    R0p05Hz,
    R0p02Hz,
    R0p01Hz,
    R0p006Hz,
    R0p003Hz,
    R0p0015Hz
}

impl OutputDataRate {
   pub(crate)  fn register_value(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum IIRFilterCoefficient {
    Coef0,
    Coef1,
    Coef3,
    Coef7,
    Coef15,
    Coef31,
    Coef63,
    Coef127,
}

impl IIRFilterCoefficient {
    pub(crate) fn register_value(&self) -> u8 {
        *self as u8
    }
}

pub enum Oversampling {
    X1,
    X2,
    X4,
    X8,
    X16,
    X32,
}

impl Oversampling {
    pub(crate) fn register_value(&self) -> u8 {
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

pub enum PowerMode {
    Sleep,
    Forced,
    Normal,
}

impl PowerMode {
    pub(crate) fn register_value(&self) -> u8 {
        match self {
            PowerMode::Sleep => 0b00,
            PowerMode::Forced => 0b10,
            PowerMode::Normal => 0b11,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Register {
    ChipId                  = 0x00,
    RevId                   = 0x01,
    ErrReg                  = 0x02,
    Status                  = 0x03,
    Data0                   = 0x04,
    Data1                   = 0x05,
    Data2                   = 0x06,
    Data3                   = 0x07,
    Data4                   = 0x08,
    Data5                   = 0x09,
    SensorTime0             = 0x0C,
    SensorTime1             = 0x0D,
    SensorTime2             = 0x0E,
    Event                   = 0x10,
    IntStatus               = 0x11,
    FifoLength0             = 0x12,
    FifoLength1             = 0x13,
    FifoData                = 0x14,
    FifoWatemark0           = 0x15,
    FifoWatemark1           = 0x16,
    FifoConfig1             = 0x17,
    FifoConfig2             = 0x18,
    IntCtrl                 = 0x19,
    IfConfig                = 0x1A,
    PwrCtrl                 = 0x1B,
    Osr                     = 0x1C,
    Odr                     = 0x1D,
    Config                  = 0x1F,
    CalibrationDataStart    = 0x31,
    // 0x30...0x57 calibration data.
    Cmd                     = 0x7E,
}

impl Register {
    pub(crate) fn addr(&self) -> u8 {
        *self as u8
    }
}
