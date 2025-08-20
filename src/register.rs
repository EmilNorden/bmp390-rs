use core::fmt::Write;
use crate::Bmp390;

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

/// Marker struct for the CHIP_ID (0x00) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<ChipId>()`]
pub struct ChipId;
impl Reg for ChipId { const ADDR:u8 = 0x00; }

impl Readable for ChipId {
    type Out = u8;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0])
    }
}

/// Marker struct for the REV_ID (0x01) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<RevId>()`]
pub struct RevId;
impl Reg for RevId { const ADDR:u8 = 0x01; }

impl Readable for RevId {
    type Out = u8;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0])
    }
}

/// Marker struct for the ERR_REG (0x02) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<ErrReg>()`] or the convenience method
/// [`Bmp390::error_flags`].
pub struct ErrReg;
impl Reg for ErrReg {  const ADDR:u8 = 0x02; }

pub struct ErrorFlags {
    fatal_err: bool,
    cmd_err: bool,
    conf_err: bool,
}

impl ErrorFlags {
    pub fn new(fatal_err: bool, cmd_err: bool, conf_err: bool) -> Self {
        Self { fatal_err, cmd_err, conf_err }
    }

    /// A fatal error occurred.
    pub fn fatal_error(&self) -> bool { self.fatal_err }

    /// Command execution failed.
    ///
    /// This value is cleared on **register** read.
    pub fn command_error(&self) -> bool { self.cmd_err }

    /// Sensor configuration error detected.
    ///
    /// This can only happen in [`Normal´] power mode.
    /// This value is cleared on **register** read.
    pub fn configuration_error(&self) -> bool { self.conf_err }
}
impl Readable for ErrReg {
    type Out = ErrorFlags;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(ErrorFlags {
            fatal_err: (b[0] & 0b001) != 0,
            cmd_err: (b[0] & 0b010) != 0,
            conf_err: (b[0] & 0b100) != 0,
        })
    }
}

/// Marker struct for the STATUS (0x03) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Status>()`] or the convenience method
/// [`Bmp390::status`].
pub struct Status;
impl Reg for Status { const ADDR: u8 = 0x03; }

pub struct StatusFlags {
    cmd_rdy: bool,
    drdy_press: bool,
    drdy_temp: bool,
}

impl StatusFlags {
    pub fn new(cmd_rdy: bool, drdy_press: bool, drdy_temp: bool) -> Self {
        Self { cmd_rdy, drdy_press, drdy_temp }
    }

    /// Is the command decoder ready to accept a new command?
    ///
    /// [`false`] means that a command is already in progress.
    pub fn command_decoder_ready(&self) -> bool { self.cmd_rdy }

    /// Is there new pressure data to be read?
    ///
    /// This value is cleared when any of the pressure DATA registers are read.
    pub fn pressure_data_ready(&self) -> bool { self.drdy_press }

    /// Is there new temperature data to be read?
    ///
    /// This value is cleared when any of the temperature DATA registers are read.
    pub fn temperature_data_ready(&self) -> bool { self.drdy_temp }
}

impl Readable for Status {
    type Out = StatusFlags;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(StatusFlags {
            cmd_rdy:    (b[0] & 0b0010000) != 0,
            drdy_press: (b[0] & 0b0100000) != 0,
            drdy_temp:  (b[0] & 0b1000000) != 0,
        })
    }
}

/// Marker struct for the DATA_0 - DATA_5 (0x04 - 0x09) registers.
/// The BMP390 will auto-increment on multiple reads, so reading 6 bytes from 0x04 will read
/// pressure and temperature in one burst read as recommended by the datasheet, section 3.10.
/// Note that this will return the raw uncalibrated measurement data. So for most use cases
/// calling [`Bmp390::read_sensor_data()`] is recommended as it will calibrate the data for you.
///
/// - **Length:** 6 bytes
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Measurement>()`] or the method
/// [`Bmp390::read_sensor_data()`].
pub struct Measurement;
impl Reg for Measurement { const ADDR: u8 = 0x04;}

pub struct MeasurementData {
    pressure: u32,
    temperature: u32,
}

impl MeasurementData {
    pub fn new(pressure: u32, temperature: u32) -> Self {
        Self { pressure, temperature }
    }

    /// Returns the raw uncalibrated pressure data from the DATA_0, DATA_1 and DATA_2 registers
    pub fn pressure(&self) -> u32 { self.pressure }

    /// Returns the raw uncalibrated temperature data from the DATA_3, DATA_4 and DATA_5 registers
    pub fn temperature(&self) -> u32 { self.temperature }
}

impl Readable for Measurement {
    type Out = MeasurementData;

    const N: usize = 6;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(MeasurementData {
            pressure: u32::from_le_bytes([b[0], b[1], b[2], 0]),
            temperature: u32::from_le_bytes([b[3], b[4], b[5], 0]),
        })
    }
}

pub struct SensorTime;
impl Reg for SensorTime { const ADDR: u8 = 0x0C; }

impl Readable for SensorTime {
    type Out = u32;
    const N: usize = 3;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u32::from_le_bytes([b[0], b[1], b[2], 0]))
    }
}

/// Marker struct for the EVENT (0x10) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Event>()`] or the convenience method
/// [`Bmp390::status`].
pub struct Event;
impl Reg for Event { const ADDR: u8 = 0x10;}

pub struct EventFlags {
    por_detected: bool,
    itf_act_pt: bool,
}

impl EventFlags {
    pub fn new(por_detected: bool, itf_act_pt: bool) -> Self {
        Self { por_detected, itf_act_pt }
    }

    /// Returns [`true`] after device power-up or after a soft-reset.
    ///
    /// This value is cleared on **register** read.
    pub fn power_on_reset_detected(&self) -> bool { self.por_detected }

    /// Returns [`true`] if a serial interface transaction has occurred
    /// during a pressure or temperature conversion.
    ///
    /// This value is cleared on **register** read.
    pub fn transaction_on_pt_conversion(&self) -> bool { self.itf_act_pt }
}

impl Readable for Event {
    type Out = EventFlags;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(EventFlags {
            por_detected:   (b[0] & 0b01) != 0,
            itf_act_pt:     (b[0] & 0b10) != 0,
        })
    }
}

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

pub struct FifoLength;
impl Reg for FifoLength { const ADDR: u8 = 0x12; }

impl Readable for FifoLength {
    type Out = u16;

    const N: usize = 2;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u16::from_le_bytes([b[0], b[1] & 0b0000_0001]))
    }
}

pub struct FifoData;
impl Reg for FifoData { const ADDR: u8 = 0x14; }

impl Readable for FifoData {
    type Out = u8;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(b[0])
    }
}

pub struct FifoWtm;
impl Reg for FifoWtm { const ADDR: u8 = 0x15; }

impl Readable for FifoWtm {
    type Out = u16;

    const N: usize = 2;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u16::from_le_bytes([b[0], b[1] & 0b0000_0001]))
    }
}

impl Writable for FifoWtm {
    type In = u16;
    const N: usize = 2;

    fn encode(v: &Self::In, out: &mut [u8]) {
        out[0] = (v & 0xFF) as u8;
        out[1] = ((v >> 8) & 1u16) as u8;
    }
}

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
            int_od:     (b[0] & 0b0000001) != 0,
            int_level:  (b[0] & 0b0000010) != 0,
            int_latch:  (b[0] & 0b0000100) != 0,
            fwtm_en:    (b[0] & 0b0001000) != 0,
            ffull_en:   (b[0] & 0b0010000) != 0,
            int_ds:     (b[0] & 0b0100000) != 0,
            drdy_en:    (b[0] & 0b1000000) != 0,
        })
    }
}

impl Writable for IntCtrl {
    type In = IntCtrlCfg;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.int_od { value |= 0b1; }
        if v.int_level { value |= 0b10;}
        if v.int_latch { value |= 0b100; }
        if v.fwtm_en { value |= 0b1000; }
        if v.ffull_en { value |= 0b10000; }
        if v.int_ds { value |= 0b0100000; }
        if v.drdy_en { value |= 0b1000000; }

        out[0] = value;
    }
}

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
pub struct PwrCtrl;
impl Reg for PwrCtrl { const ADDR:u8 = 0x1B; }

pub struct PwrCtrlCfg {
    pub press_en: bool,
    pub temp_en: bool,
    pub mode: PowerMode,
}

impl Readable for PwrCtrl {
    type Out = PwrCtrlCfg;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        let v = b[0];
        Ok(PwrCtrlCfg {
            press_en:   v & 0b0000_0001 != 0,
            temp_en:    v & 0b0000_0010 != 0,
            mode:       PowerMode::from((v >> 4) & 0b11),
        })
    }
}

impl Writable for PwrCtrl {
    type In = PwrCtrlCfg;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.press_en {
            value |= 0b0000_0001;
        }
        if v.temp_en {
            value |= 0b0000_0010;
        }
        let mode: u8 = v.mode.into();
        value |= mode << 4;
        out[0] = value;
    }
}

pub struct Osr;
impl Reg for Osr { const ADDR:u8 = 0x1C; }

pub struct OsrCfg {
    pub osr_p: Oversampling,
    pub osr_t: Oversampling,
}

impl Readable for Osr {
    type Out = OsrCfg;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(OsrCfg {
            osr_p: Oversampling::try_from(b[0] & 0b111)
                .map_err(|e| InvalidRegisterField::new(Self::ADDR, e.0, 0))?,
            osr_t: Oversampling::try_from((b[0] >> 3) & 0b111)
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

pub struct Calibration;
impl Reg for Calibration { const ADDR:u8 = 0x31; }

pub struct CalibrationNvm {
    pub(crate) nvm_par_t1: u16,
    pub(crate) nvm_par_t2: u16,
    pub(crate) nvm_par_t3: i8,
    pub(crate) nvm_par_p1: i16,
    pub(crate) nvm_par_p2: i16,
    pub(crate) nvm_par_p3: i8,
    pub(crate) nvm_par_p4: i8,
    pub(crate) nvm_par_p5: u16,
    pub(crate) nvm_par_p6: u16,
    pub(crate) nvm_par_p7: i8,
    pub(crate)  nvm_par_p8: i8,
    pub(crate) nvm_par_p9: i16,
    pub(crate) nvm_par_p10: i8,
    pub(crate) nvm_par_p11: i8,
}

impl Readable for Calibration {
    type Out = CalibrationNvm;

    const N: usize = 21;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(CalibrationNvm {
            nvm_par_t1: u16::from_le_bytes([b[0], b[1]]),
            nvm_par_t2: u16::from_le_bytes([b[2], b[3]]),
            nvm_par_t3: i8::from_le_bytes([b[4]]),
            nvm_par_p1: i16::from_le_bytes([b[5], b[6]]),
            nvm_par_p2: i16::from_le_bytes([b[7], b[8]]),
            nvm_par_p3: i8::from_le_bytes([b[9]]),
            nvm_par_p4: i8::from_le_bytes([b[10]]),
            nvm_par_p5: u16::from_le_bytes([b[11], b[12]]),
            nvm_par_p6: u16::from_le_bytes([b[13], b[14]]),
            nvm_par_p7: i8::from_le_bytes([b[15]]),
            nvm_par_p8: i8::from_le_bytes([b[16]]),
            nvm_par_p9: i16::from_le_bytes([b[17], b[18]]),
            nvm_par_p10: i8::from_le_bytes([b[19]]),
            nvm_par_p11: i8::from_le_bytes([b[20]]),
        })
    }
}

pub struct Cmd;
impl Reg for Cmd { const ADDR:u8 = 0x7E; }

#[derive(Copy, Clone)]
pub enum CmdData {
    FifoFlush,
    SoftReset
}

impl Into<u8> for CmdData {
    fn into(self) -> u8 {
        match self {
            CmdData::FifoFlush => 0xB0,
            CmdData::SoftReset => 0xB6,
        }
    }
}

impl Writable for Cmd {
    type In = CmdData;
    fn encode(v: &Self::In, out: &mut [u8]) {
        out[0] = (*v).into();
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

impl IIRFilterCoefficient {
    pub(crate) fn register_value(&self) -> u8 {
        *self as u8
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

/// Describes the different power modes that can be set in the PwrCtrl register.
///
/// For more information, see section 3.3 in the datasheet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerMode {
    /// Sleep mode. This is the default mode after power on reset.
    Sleep,
    /// Forced mode. In this mode, a single measurement is performed after which the device returns to Sleep mode.
    Forced,
    // Normal mode. In this mode, measurements are constantly performed at the rate set in the odr_sel register
    Normal,
}

impl From<u8> for PowerMode {
    fn from(field: u8) -> Self {
        match field {
            0b00 => PowerMode::Sleep,
            0b01 | 0b10 => PowerMode::Forced,
            _ => PowerMode::Normal,
        }
    }
}

impl Into<u8> for PowerMode {
    fn into(self) -> u8 {
        match self {
            PowerMode::Sleep => 0b00,
            PowerMode::Forced => 0b01,
            PowerMode::Normal => 0b11,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn err_reg_decode() {
        let reg = ErrReg::decode(&[0b001]).unwrap();
        assert_eq!([true, false, false], [reg.fatal_err, reg.cmd_err, reg.conf_err]);

        let reg = ErrReg::decode(&[0b010]).unwrap();
        assert_eq!([false, true, false], [reg.fatal_err, reg.cmd_err, reg.conf_err]);

        let reg = ErrReg::decode(&[0b100]).unwrap();
        assert_eq!([false, false, true], [reg.fatal_err, reg.cmd_err, reg.conf_err]);
    }

    #[test]
    fn status_decode() {
        let reg = Status::decode(&[0b0010000]).unwrap();
        assert_eq!([true, false, false], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);

        let reg = Status::decode(&[0b0100000]).unwrap();
        assert_eq!([false, true, false], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);

        let reg = Status::decode(&[0b1000000]).unwrap();
        assert_eq!([false, false, true], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);
    }

    #[test]
    fn measurement_decode() {
        let reg = Measurement::decode(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]).unwrap();

        assert_eq!(0xCCBBAA, reg.pressure);
        assert_eq!(0xFFEEDD, reg.temperature);
    }

    #[test]
    fn sensor_time_decode() {
        let reg = SensorTime::decode(&[0xAA, 0xBB, 0xCC]).unwrap();

        assert_eq!(0xCCBBAA, reg);
    }

    #[test]
    fn event_decode() {
        let reg = Event::decode(&[0b01]).unwrap();
        assert!(reg.por_detected);

        let reg = Event::decode(&[0b10]).unwrap();
        assert!(reg.itf_act_pt);
    }

    #[test]
    fn int_status_decode() {
        let reg = IntStatus::decode(&[0b0001]).unwrap();
        assert!(reg.fwm_int);

        let reg = IntStatus::decode(&[0b0010]).unwrap();
        assert!(reg.ffull_int);

        let reg = IntStatus::decode(&[0b1000]).unwrap();
        assert!(reg.drdy);
    }

    #[test]
    fn fifo_length_decode() {
        let reg = FifoLength::decode(&[0xFF, 0x01]).unwrap();

        assert_eq!(511, reg);
    }

    #[test]
    fn fifo_wtm_decode() {
        let reg = FifoWtm::decode(&[0xFF, 0x01]).unwrap();

        assert_eq!(511, reg);
    }

    #[test]
    fn fifo_wtm_encode() {
        let mut buffer = [0u8; 2];
        FifoWtm::encode(&511, &mut buffer);

        assert_eq!(buffer, [0xFF, 0x01]);
    }

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

    #[test]
    fn pwr_ctrl_decode() {
        let reg = PwrCtrl::decode(&[0b0000_0001]).unwrap();
        assert!(reg.press_en);
        assert_eq!(PowerMode::Sleep, reg.mode);

        let reg = PwrCtrl::decode(&[0b0001_0010]).unwrap();
        assert!(reg.temp_en);
        assert_eq!(PowerMode::Forced, reg.mode);

        let reg = PwrCtrl::decode(&[0b0010_0000]).unwrap();
        assert!(!reg.press_en);
        assert!(!reg.temp_en);
        assert_eq!(PowerMode::Forced, reg.mode);

        let reg = PwrCtrl::decode(&[0b0011_0000]).unwrap();

        assert_eq!(PowerMode::Normal, reg.mode);
    }

    #[test]
    fn pwr_ctrl_encode() {
        let mut buffer = [0u8; 1];
        PwrCtrl::encode(&PwrCtrlCfg {
            press_en: false,
            temp_en: false,
            mode: PowerMode::Sleep,
        }, &mut buffer);
        assert_eq!([0b0000_0000], buffer);

        PwrCtrl::encode(&PwrCtrlCfg {
            press_en: true,
            temp_en: false,
            mode: PowerMode::Forced,
        }, &mut buffer);
        assert_eq!([0b0001_0001], buffer);

        PwrCtrl::encode(&PwrCtrlCfg {
            press_en: false,
            temp_en: true,
            mode: PowerMode::Normal,
        }, &mut buffer);
        assert_eq!([0b0011_0010], buffer);
    }

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

    #[test]
    fn cmd_encode() {
        let mut buffer = [0u8; 1];
        Cmd::encode(&CmdData::FifoFlush, &mut buffer);
        assert_eq!([0xB0], buffer);

        Cmd::encode(&CmdData::SoftReset, &mut buffer);
        assert_eq!([0xB6], buffer);
    }
}