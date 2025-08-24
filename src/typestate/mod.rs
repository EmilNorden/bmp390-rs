mod normal;

use crate::bus::{Bus, I2c, Spi};
use crate::config::Configuration;
use crate::register::pwr_ctrl::PowerMode;
use crate::{Bmp390, Bmp390Error, SdoPinState};
use core::marker::PhantomData;
use embedded_hal_async::delay::DelayNs;

/// Marker for the "No bus chosen yet" state in Bmp390Builder
pub struct NoBus;

pub struct Bmp390Builder<Out = NoOutput, B = NoBus> {
    bus: Option<B>,
    _phantom_data: PhantomData<Out>,
}

impl Bmp390Builder<NoOutput, NoBus> {
    pub fn new() -> Self {
        Self {
            bus: None,
            _phantom_data: PhantomData,
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
            _phantom_data: self._phantom_data,
        }
    }

    pub fn use_spi<SpiType>(self, spi: SpiType) -> Bmp390Builder<Out, Spi<SpiType>>
    where
        SpiType: embedded_hal_async::spi::SpiDevice,
    {
        Bmp390Builder {
            bus: Some(Spi::new(spi)),
            _phantom_data: self._phantom_data,
        }
    }
}

impl<Out, B: Bus> Bmp390Builder<Out, B> {
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
}

impl<B: Bus> Bmp390Builder<NoOutput, B> {
    pub fn enable_pressure(self) -> Bmp390Builder<Pressure, B> {
        Bmp390Builder {
            bus: self.bus,
            _phantom_data: PhantomData,
        }
    }

    pub fn enable_temperature(self) -> Bmp390Builder<Temperature, B> {
        Bmp390Builder {
            bus: self.bus,
            _phantom_data: PhantomData,
        }
    }
}

impl<B: Bus> Bmp390Builder<Pressure, B> {
    pub fn enable_temperature(self) -> Bmp390Builder<PressureAndTemperature, B> {
        Bmp390Builder {
            bus: self.bus,
            _phantom_data: PhantomData,
        }
    }
}

impl<B: Bus> Bmp390Builder<Temperature, B> {
    pub fn enable_pressure(self) -> Bmp390Builder<PressureAndTemperature, B> {
        Bmp390Builder {
            bus: self.bus,
            _phantom_data: PhantomData,
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

pub struct ForcedDevice<B> {
    device: Bmp390<B>,
}

pub struct FifoDevice<B> {
    device: Bmp390<B>,
}

pub struct NoOutput;
pub struct Temperature;
pub struct Pressure;
pub struct PressureAndTemperature;