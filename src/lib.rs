#![no_std]

use core::fmt::{Debug};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::SevenBitAddress;

pub trait Bus {
    type Error;
    fn write(&mut self, data: &[u8]) -> impl Future<Output = Result<(), Self::Error>>;
    fn read(&mut self, data: &mut [u8]) -> impl Future<Output = Result<(), Self::Error>>;
    fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> impl Future<Output = Result<(), Self::Error>>;
}

pub struct I2c<I2cType> {
    i2c: I2cType,
    address: SevenBitAddress
}

impl<I2cType> I2c<I2cType>
where
    I2cType: embedded_hal_async::i2c::I2c
{
    pub(crate) fn new(i2c: I2cType, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }
}

impl<I2cType> Bus for I2c<I2cType>
where
    I2cType: embedded_hal_async::i2c::I2c,
{
    type Error = <I2cType as embedded_hal_async::i2c::ErrorType>::Error;

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, data).await?;

        Ok(())
    }

    async fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, data).await?;

        Ok(())
    }

    async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, write, read).await?;

        Ok(())
    }
}

pub struct Spi<SpiType> {
    spi: SpiType,
}

impl<SpiType> Spi<SpiType>
where
    SpiType: embedded_hal_async::spi::SpiDevice
{
    pub(crate) fn new(spi: SpiType) -> Self {
        Self { spi }
    }

}

impl<SpiType> Bus for Spi<SpiType>
where
    SpiType: embedded_hal_async::spi::SpiDevice,
{
    type Error = <SpiType as embedded_hal_async::spi::ErrorType>::Error;

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        todo!()
    }

    async fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        todo!()
    }

    async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi.transaction(
            &mut [Operation::Write(write), Operation::Read(read)],
        ).await?;

        Ok(())
    }
}

#[derive(Debug)]
pub enum Bmp390Error<BusError> {
    Bus(BusError)
}

type Bmp390Result<T, BusError> = Result<T, Bmp390Error<BusError>>;

pub struct Bmp390<B> {
    bus: B,
    calibration_data: CalibrationData,
}

impl<I2cType> Bmp390<I2c<I2cType>>
where
    I2cType: embedded_hal_async::i2c::I2c,
    I2c<I2cType>: Bus,
{
    pub async fn new_i2c<D: DelayNs>(
        i2c: I2cType,
        address: SevenBitAddress,
        config: Configuration,
        delay: &mut D) -> Bmp390Result<Self, <I2c<I2cType> as Bus>::Error> {
        Self::new(I2c::new(i2c, address), config, delay).await
    }
}

impl<SpiType> Bmp390<Spi<SpiType>>
where
    SpiType: embedded_hal_async::spi::SpiDevice,
    Spi<SpiType>: Bus,
{
    pub async fn new_spi<D: DelayNs>(
        spi: SpiType,
        config: Configuration,
        delay: &mut D) -> Bmp390Result<Self, <Spi<SpiType> as Bus>::Error> {
        Self::new(Spi::new(spi), config, delay).await
    }
}

pub struct Measurement {
    pub pressure: f32,
    pub temperature: f32,
}

impl<B> Bmp390<B>
where
    B: Bus,
{
    async fn new<D: DelayNs>(mut bus: B, config: Configuration, delay: &mut D) -> Bmp390Result<Self, B::Error>  {
        let calibration_data = Self::load_calibration_data(&mut bus).await?;
        let mut device = Bmp390 { bus, calibration_data };

        device.write_register(
            Register::PwrCtrl,
            (config.enable_pressure as u8) |
                (config.enable_temperature as u8) << 1 |
                config.mode.register_value() << 4
        ).await?;

        device.write_register(
            Register::Osr,
            config.pressure_oversampling.register_value() |
                config.temperature_oversampling.register_value() << 3
        ).await?;

        device.write_register(
            Register::Odr,
            config.output_data_rate.register_value()
        ).await?;

        device.write_register(
            Register::Config,
            config.iir_filter_coefficient.register_value()
        ).await?;


        Ok(device)
    }

    async fn load_calibration_data(bus: &mut B) -> Bmp390Result<CalibrationData, B::Error> {
        let mut buf = [0u8; 21];
        bus.write_read(&[Register::CalibrationDataStart.addr()], &mut buf)
            .await
            .map_err(Bmp390Error::Bus)?;

        let t1 = u16::from_le_bytes([buf[0], buf[1]]);
        let t2 = u16::from_le_bytes([buf[2], buf[3]]);
        let t3 = i8::from_le_bytes([buf[4]]);
        let p1 = i16::from_le_bytes([buf[5], buf[6]]);
        let p2 = i16::from_le_bytes([buf[7], buf[8]]);
        let p3 = i8::from_le_bytes([buf[9]]);
        let p4 = i8::from_le_bytes([buf[10]]);
        let p5 = u16::from_le_bytes([buf[11], buf[12]]);
        let p6 = u16::from_le_bytes([buf[13], buf[14]]);
        let p7 = i8::from_le_bytes([buf[15]]);
        let p8 = i8::from_le_bytes([buf[16]]);
        let p9 = i16::from_le_bytes([buf[17], buf[18]]);
        let p10 = i8::from_le_bytes([buf[19]]);
        let p11 = i8::from_le_bytes([buf[20]]);

        Ok(CalibrationData {
            par_t1: (t1 as f32) * pow2f(8),
            par_t2: (t2 as f32) * pow2f(-30),
            par_t3: (t3 as f32) * pow2f(-48),
            par_p1: (p1 as f32 - pow2f(14)) * pow2f(-20),
            par_p2: (p2 as f32 - pow2f(14)) * pow2f(-29),
            par_p3: (p3 as f32) * pow2f(-32),
            par_p4: (p4 as f32) * pow2f(-37),
            par_p5: (p5 as f32) * pow2f(3),
            par_p6: (p6 as f32) * pow2f(-6),
            par_p7: (p7 as f32) * pow2f(-8),
            par_p8: (p8 as f32) * pow2f(-15),
            par_p9: (p9 as f32) * pow2f(-48),
            par_p10: (p10 as f32) * pow2f(-48),
            par_p11: (p11 as f32) * pow2f(-65),
        })
    }

    pub async fn read_sensor_data(&mut self) -> Bmp390Result<Measurement, B::Error> {
        let mut buffer = [0u8; 6];
        self.read_register(Register::Data0, &mut buffer).await?;
        let uncompensated_pressure = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], 0]);
        let uncompensated_temperature = u32::from_le_bytes([buffer[3], buffer[4], buffer[5], 0]);

        let compensated_temperature = self.compensate_temperature(uncompensated_temperature);
        let compensated_pressure = self.compensate_pressure(uncompensated_pressure, compensated_temperature);

        Ok(Measurement {
            pressure: compensated_pressure,
            temperature: compensated_temperature,
        })
    }

    fn compensate_temperature(&self, uncomp_temperature: u32) -> f32 {
        let partial_data1 = uncomp_temperature as f32 - self.calibration_data.par_t1;
        let partial_data2 = partial_data1 * self.calibration_data.par_t2;

        partial_data2 + (partial_data1 * partial_data1) * self.calibration_data.par_t3
    }

    fn compensate_pressure(&self, uncompensated_pressure: u32, compensated_temperature: f32) -> f32 {
        let partial_data1 = self.calibration_data.par_p6 * compensated_temperature;
        let partial_data2 = self.calibration_data.par_p7 * (compensated_temperature * compensated_temperature);
        let partial_data3 = self.calibration_data.par_p8 * (compensated_temperature * compensated_temperature * compensated_temperature);
        let partial_out1 = self.calibration_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = self.calibration_data.par_p2 * compensated_temperature;
        let partial_data2 = self.calibration_data.par_p3 * (compensated_temperature * compensated_temperature);
        let partial_data3 = self.calibration_data.par_p4 * (compensated_temperature * compensated_temperature * compensated_temperature);
        let partial_out2 = uncompensated_pressure as f32 * (self.calibration_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

        let partial_data1 = uncompensated_pressure as f32 * uncompensated_pressure as f32;
        let partial_data2 = self.calibration_data.par_p9 + self.calibration_data.par_p10 * compensated_temperature;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + (uncompensated_pressure as f32 * uncompensated_pressure as f32 * uncompensated_pressure as f32) * self.calibration_data.par_p11;
        partial_out1 + partial_out2 + partial_data4
    }

    async fn read_register(&mut self, reg: Register, data: &mut [u8]) -> Bmp390Result<(), B::Error> {
        self.bus.write_read(&[reg.addr()], data).await.map_err(Bmp390Error::Bus)?;

        Ok(())
    }

    async fn write_register(&mut self, reg: Register, value: u8) -> Bmp390Result<(), B::Error> {
        self.bus.write(&[reg.addr(), value])
            .await
            .map_err(Bmp390Error::Bus)?;

        Ok(())
    }

    pub async fn chip_id(&mut self) -> Bmp390Result<u8, B::Error> {
        self.bus.write(&[Register::ChipId.addr()])
            .await
            .map_err(Bmp390Error::Bus)?;

        let mut buf = [0u8; 1];
        self.bus.read(&mut buf)
            .await
            .map_err(Bmp390Error::Bus)?;

        Ok(buf[0])
    }

}

pub struct Configuration {
    enable_pressure: bool,
    enable_temperature: bool,
    mode: PowerMode,
    output_data_rate: OutputDataRate,
    pressure_oversampling: Oversampling,
    temperature_oversampling: Oversampling,
    iir_filter_coefficient: IIRFilterCoefficient,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            enable_pressure: true,
            enable_temperature: true,
            mode: PowerMode::Normal,
            output_data_rate: OutputDataRate::R50Hz,
            iir_filter_coefficient: IIRFilterCoefficient::Coef15,
            pressure_oversampling: Oversampling::X8,
            temperature_oversampling: Oversampling::X1,
        }
    }
}

impl Configuration {
    pub fn enable_pressure_measurement(&mut self, enable: bool) {
        self.enable_pressure = enable;
    }

    pub fn enable_temperature_measurement(&mut self, enable: bool) {
        self.enable_temperature = enable;
    }

    pub fn power_mode(&mut self, power_mode: PowerMode) {
        self.mode = power_mode;
    }

    pub fn output_data_rate(&mut self, output_data_rate: OutputDataRate) {
        self.output_data_rate = output_data_rate;
    }

    pub fn iir_filter_coefficient(&mut self, filter_coefficient: IIRFilterCoefficient) {
        self.iir_filter_coefficient = filter_coefficient;
    }

    pub fn pressure_oversampling(&mut self, pressure_oversampling: Oversampling) {
        self.pressure_oversampling = pressure_oversampling;
    }

    pub fn temperature_oversampling(&mut self, temperature_oversampling: Oversampling) {
        self.temperature_oversampling = temperature_oversampling;
    }
}

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
    fn register_value(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum IIRFilterCoefficient {
    Coef0,
    Coef1,
    COef3,
    Coef7,
    Coef15,
    Coef31,
    Coef63,
    Coef127,
}

impl IIRFilterCoefficient {
    fn register_value(&self) -> u8 {
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
    fn register_value(&self) -> u8 {
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
    fn register_value(&self) -> u8 {
        match self {
            PowerMode::Sleep => 0b00,
            PowerMode::Forced => 0b10,
            PowerMode::Normal => 0b11,
        }
    }
}

#[derive(Clone, Copy)]
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

pub struct CalibrationData {
    par_t1: f32,
    par_t2: f32,
    par_t3: f32,
    par_p1: f32,
    par_p2: f32,
    par_p3: f32,
    par_p4: f32,
    par_p5: f32,
    par_p6: f32,
    par_p7: f32,
    par_p8: f32,
    par_p9: f32,
    par_p10: f32,
    par_p11: f32,
}

#[inline]
const fn pow2f(e: i32) -> f32 {
    match e {
        // Normal numbers: exponent E = e + 127, mantissa = 0
        -126..=127 => f32::from_bits(((e + 127) as u32) << 23),

        // Subnormals: exponent bits = 0, value = F * 2^-149
        // Choose F = 1 << (e + 149) so value = 2^e
        -149..=-127 => f32::from_bits(1u32 << (e + 149)),

        // Underflow / overflow
        e if e < -149 => 0.0,
        _ => f32::INFINITY,
    }
}