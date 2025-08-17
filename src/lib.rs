#![no_std]
extern crate alloc;

pub mod config;
mod register;
pub mod bus;
mod calibration;

use core::fmt::{Debug};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::SevenBitAddress;
use crate::bus::{Bus, I2c, Spi};
use crate::calibration::CalibrationData;
use crate::config::Configuration;
use crate::register::Register;

const BMP390_CHIP_ID:u8 = 0x60;

#[derive(Debug)]
pub enum Bmp390Error<BusError> {
    Bus(BusError),
    NotConnected
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

impl<B> Bmp390<B>
where
    B: Bus,
{
    async fn probe_ready<D: DelayNs>(bus: &mut B, delay: &mut D, attempts: u32) -> Bmp390Result<(), B::Error> {
        for _ in 0..attempts {
            let mut buf = [0u8; 1];
            if let Ok(id) = bus.read_register(Register::ChipId, &mut buf).await {
                if buf[0] == BMP390_CHIP_ID {
                    return Ok(())
                }
            }

            delay.delay_ms(1).await;
        }

        Err(Bmp390Error::NotConnected)
    }
    async fn new<D: DelayNs>(mut bus: B, config: Configuration, delay: &mut D) -> Bmp390Result<Self, B::Error>  {
        // The datasheet (Section 1, table 2) specifies 2ms start-up time after VDD/VDDIO > 1.8V
        Self::probe_ready(&mut bus, delay, 5).await?;

        let calibration_data = CalibrationData::new(&mut bus).await?;

        bus.write_register(
            Register::PwrCtrl,
            (config.enable_pressure as u8) |
                (config.enable_temperature as u8) << 1 |
                config.mode.register_value() << 4
        ).await.map_err(Bmp390Error::Bus)?;

        bus.write_register(
            Register::Osr,
            config.pressure_oversampling.register_value() |
                config.temperature_oversampling.register_value() << 3
        ).await.map_err(Bmp390Error::Bus)?;

        bus.write_register(
            Register::Odr,
            config.output_data_rate.register_value()
        ).await.map_err(Bmp390Error::Bus)?;

        bus.write_register(
            Register::Config,
            config.iir_filter_coefficient.register_value()
        ).await.map_err(Bmp390Error::Bus)?;

        Ok(Bmp390 { bus, calibration_data })
    }

    pub async fn is_connected(&mut self) -> Bmp390Result<bool, B::Error> {
        let mut buf = [0u8; 1];
        self.bus.read_register(Register::ChipId, &mut buf)
            .await
            .map_err(Bmp390Error::Bus)?;

        Ok(buf[0] == BMP390_CHIP_ID)
    }

    pub async fn read_sensor_data(&mut self) -> Bmp390Result<Measurement, B::Error> {
        let mut buffer = [0u8; 6];
        self.bus.read_register(Register::Data0, &mut buffer)
            .await
            .map_err(Bmp390Error::Bus)?;

        let uncompensated_pressure = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], 0]);
        let uncompensated_temperature = u32::from_le_bytes([buffer[3], buffer[4], buffer[5], 0]);

        let compensated_temperature =
            self.calibration_data.compensate_temperature(uncompensated_temperature);
        let compensated_pressure =
            self.calibration_data.compensate_pressure(uncompensated_pressure);

        Ok(Measurement {
            pressure: compensated_pressure,
            temperature: compensated_temperature,
        })
    }
}

pub struct Measurement {
    pub pressure: f32,
    pub temperature: f32,
}
