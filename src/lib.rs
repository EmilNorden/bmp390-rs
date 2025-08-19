#![no_std]

pub mod config;
pub mod register;
pub mod bus;
mod calibration;

use core::fmt::{Debug};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::SevenBitAddress;
use crate::bus::{Bus, I2c, Spi};
use crate::calibration::CalibrationData;
use crate::config::Configuration;
use crate::register::{InvalidRegisterField};

const BMP390_CHIP_ID:u8 = 0x60;

#[derive(Debug)]
pub enum Bmp390Error<BusError> {
    Bus(BusError),
    NotConnected,
    UnexpectedRegisterData(InvalidRegisterField),
}

type Bmp390Result<T, BusError> = Result<T, Bmp390Error<BusError>>;

type Bmp390I2c<T> = Bmp390<I2c<T>>;
type Bmp390Spi<T> = Bmp390<Spi<T>>;

pub struct Bmp390<B> {
    bus: B,
    calibration_data: CalibrationData,
}

impl<T> Bmp390I2c<T>
where
    T: embedded_hal_async::i2c::I2c,
    I2c<T>: Bus,
{
    pub async fn new_i2c<D: DelayNs>(
        i2c: T,
        address: SevenBitAddress,
        config: Configuration,
        delay: &mut D) -> Bmp390Result<Self, <I2c<T> as Bus>::Error> {
        Self::new(I2c::new(i2c, address), config, delay).await
    }
}

impl<T> Bmp390Spi<T>
where
    T: embedded_hal_async::spi::SpiDevice,
    Spi<T>: Bus,
{
    pub async fn new_spi<D: DelayNs>(
        spi: T,
        config: Configuration,
        delay: &mut D) -> Bmp390Result<Self, <Spi<T> as Bus>::Error> {
        Self::new(Spi::new(spi), config, delay).await
    }
}

impl<B> Bmp390<B>
where
    B: Bus,
{
    async fn probe_ready<D: DelayNs>(bus: &mut B, delay: &mut D, attempts: u32) -> Bmp390Result<(), B::Error> {
        for _ in 0..attempts {
            if let Ok(id) = bus.read::<register::ChipId>().await {
                if id == BMP390_CHIP_ID {
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

        bus.write::<register::PwrCtrl>(&register::PwrCtrlCfg {
            press_en: config.enable_pressure,
            temp_en: config.enable_temperature,
            mode: config.mode,
        }).await?;

        bus.write::<register::Osr>(&register::OsrCfg {
            osr_p: config.pressure_oversampling,
            osr_t: config.temperature_oversampling,
        }).await?;

        bus.write::<register::Odr>(&register::OdrCfg {
            odr_sel: config.output_data_rate
        }).await?;

        bus.write::<register::Config>(&register::ConfigFields {
            iir_filter: config.iir_filter_coefficient
        }).await?;

        Ok(Bmp390 { bus, calibration_data })
    }

    pub async fn is_connected(&mut self) -> Bmp390Result<bool, B::Error> {
        let id = self.bus.read::<register::ChipId>().await?;

        Ok(id == BMP390_CHIP_ID)
    }

    /// Sets the power mode of the device by writing to the PwrCtrl (0x1B) register
    ///
    /// As described in section 3.3.4 of the datasheet, these are the valid state transitions:
    ///
    /// Sleep => Normal
    ///
    /// Normal => Sleep
    ///
    /// Sleep => Forced => Sleep (Forced is a transient state and the device will return to Sleep when the measurement is finished)
    ///
    /// The device ignores any attempt to perform an invalid state transition.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// device.set_mode(PowerMode::Normal).await?
    pub async fn set_mode(&mut self, mode: register::PowerMode) -> Bmp390Result<(), B::Error> {
        let mut pwr_ctrl = self.bus.read::<register::PwrCtrl>().await?;
        pwr_ctrl.mode = mode;
        self.bus.write::<register::PwrCtrl>(&pwr_ctrl).await?;
        Ok(())
    }
    /// Reads the current power mode from the PwrCtrl (0x1B) register
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// device.mode()?
    pub async fn mode(&mut self) -> Bmp390Result<register::PowerMode, B::Error> {
        Ok(self.bus.read::<register::PwrCtrl>().await?.mode)
    }

    pub async fn read_sensor_data(&mut self) -> Bmp390Result<Measurement, B::Error> {
        let measurement = self.bus.read::<register::Measurement>().await?;

        let compensated_temperature =
            self.calibration_data.compensate_temperature(measurement.temperature);
        let compensated_pressure =
            self.calibration_data.compensate_pressure(measurement.pressure);

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
