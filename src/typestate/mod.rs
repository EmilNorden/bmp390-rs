mod normal;
mod sample;
mod forced;

use crate::bus::{Bus, I2c, Spi};
use crate::config::Configuration;
use crate::register::pwr_ctrl::PowerMode;
use crate::{Bmp390, Bmp390Error, SdoPinState};
use core::marker::PhantomData;
use core::pin::Pin;
use embedded_hal::digital::{Error, ErrorKind, ErrorType, InputPin};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;

/// Marker for the "No bus chosen yet" state in Bmp390Builder
pub struct NoBus;

pub struct NoPin;

#[derive(Debug)]
pub struct NoPinError;

impl Error for NoPinError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for NoPin { type Error = NoPinError; }

impl Wait for NoPin {

    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub struct Bmp390Builder<Out = NoOutput, B = NoBus, P = NoPin> {
    bus: Option<B>,
    config: Configuration,
    irq_pin: Option<P>,
    _phantom_data: PhantomData<Out>,
    _phantom_data_p: PhantomData<P>
}

impl Bmp390Builder<NoOutput, NoBus> {
    pub fn new() -> Self {
        Self {
            bus: None,
            config: Configuration::default()
                .enable_pressure_measurement(false)
                .enable_temperature_measurement(false),
            irq_pin: None,
            _phantom_data: PhantomData,
            _phantom_data_p: PhantomData
        }
    }
}

impl<Out> Bmp390Builder<Out, NoBus> {
    pub fn use_i2c<I2cType>(
        self,
        i2c: I2cType,
        sdo_pin_state: SdoPinState,
    ) -> Bmp390Builder<Out, I2c<I2cType>>
    where
        I2cType: embedded_hal_async::i2c::I2c,
    {
        Bmp390Builder {
            bus: Some(I2c::new(i2c, sdo_pin_state.into())),
            config: self.config,
            irq_pin: self.irq_pin,
            _phantom_data: self._phantom_data,
            _phantom_data_p: self._phantom_data_p,
        }
    }

    pub fn use_spi<SpiType>(self, spi: SpiType) -> Bmp390Builder<Out, Spi<SpiType>>
    where
        SpiType: embedded_hal_async::spi::SpiDevice,
    {
        Bmp390Builder {
            bus: Some(Spi::new(spi)),
            config: self.config,
            irq_pin: self.irq_pin,
            _phantom_data: self._phantom_data,
            _phantom_data_p: self._phantom_data_p,
        }
    }
}

impl<Out, B: Bus> Bmp390Builder<Out, B, NoPin> {
    pub fn use_irq<Pin: Wait + InputPin>(self, pin: Pin) -> Bmp390Builder<Out, B, Pin> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config,
            irq_pin: Some(pin),
            _phantom_data: self._phantom_data,
            _phantom_data_p: PhantomData
        }
    }
}

impl<Out, B: Bus, P: Wait + InputPin> Bmp390Builder<Out, B, P> {
    pub async fn into_normal<D: DelayNs>(
        self,
        delay: &mut D,
    ) -> Result<normal::NormalDevice<Out, B>, BuilderError<B::Error>> {
        let bus = self.bus.ok_or(BuilderError::NoBus)?;
        let config = Configuration::default().power_mode(PowerMode::Normal);
        let device = Bmp390::new(bus, config, delay)
            .await
            .map_err(|e| BuilderError::DeviceError(e))?;

        Ok(normal::NormalDevice {
            device,
            _phantom_data: self._phantom_data,
        })
    }

    pub async fn into_forced<D: DelayNs>(
        self,
        delay: &mut D,
    ) -> Result<forced::ForcedDevice<Out, B, P>, BuilderError<B::Error>> {
        let bus = self.bus.ok_or(BuilderError::NoBus)?;
        let config = Configuration::default().power_mode(PowerMode::Sleep);
        let device = Bmp390::new(bus, config, delay)
            .await
            .map_err(BuilderError::DeviceError)?;

        Ok(forced::ForcedDevice::new(device, self.irq_pin).await.map_err(BuilderError::DeviceError)?)
        /*Ok(forced::ForcedDevice {
            device,
            irq_pin: self.irq_pin,
            _phantom_data: self._phantom_data
        })*/
    }

}

impl<B: Bus> Bmp390Builder<NoOutput, B> {
    pub fn enable_pressure(self) -> Bmp390Builder<Pressure, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_pressure_measurement(true),
            irq_pin: self.irq_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p
        }
    }

    pub fn enable_temperature(self) -> Bmp390Builder<Temperature, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_temperature_measurement(true),
            irq_pin: self.irq_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p
        }
    }
}

impl<B: Bus> Bmp390Builder<Pressure, B> {
    pub fn enable_temperature(self) -> Bmp390Builder<PressureAndTemperature, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_temperature_measurement(true),
            irq_pin: self.irq_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p
        }
    }
}

impl<B: Bus> Bmp390Builder<Temperature, B> {
    pub fn enable_pressure(self) -> Bmp390Builder<PressureAndTemperature, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_pressure_measurement(true),
            irq_pin: self.irq_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p
        }
    }
}

#[derive(Debug)]
pub enum BuilderError<B> {
    NoBus,
    DeviceError(Bmp390Error<B>),
}

pub struct IdleDevice<B> {
    device: Bmp390<B>,
}

pub struct FifoDevice<B> {
    device: Bmp390<B>,
}

pub struct NoOutput;

pub struct Temperature;
pub struct Pressure;
pub struct PressureAndTemperature;