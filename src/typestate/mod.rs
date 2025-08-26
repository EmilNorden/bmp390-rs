mod fifo;
mod forced;
mod measurement;
mod normal;

pub use fifo::FifoOutput;

use crate::bus::{Bus, I2c, Spi};
use crate::config::Configuration;
use crate::register::int_status::IntStatus;
use crate::register::pwr_ctrl::PowerMode;
use crate::typestate::measurement::Measurement;
use crate::{Bmp390, Bmp390Error, SdoPinState};
use core::marker::PhantomData;
use embedded_hal::digital::{Error, ErrorKind, ErrorType, InputPin};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;

/// Marker for the "No bus chosen yet" state in Bmp390Builder
pub struct NoBus;

/// Marker for the "No interrupt pin configured yet" state in the Bmp390Builder
pub struct NoPin;

#[derive(Debug)]
pub struct NoPinError;

impl Error for NoPinError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for NoPin {
    type Error = NoPinError;
}

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

impl InputPin for NoPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(false)
    }
}

#[derive(Debug)]
pub enum TypeStateError<BusError, PinError> {
    Device(Bmp390Error<BusError>),
    Pin(PinError),
    FifoConfigError,
}

pub type TypeStateResult<T, BusError, PinError> = Result<T, TypeStateError<BusError, PinError>>;

pub struct Bmp390Builder<Out = NoOutput, B = NoBus, IntPin = NoPin> {
    bus: Option<B>,
    config: Configuration,
    int_pin: Option<IntPin>,
    _phantom_data: PhantomData<Out>,
    _phantom_data_p: PhantomData<IntPin>,
}

impl Bmp390Builder<NoOutput, NoBus> {
    pub fn new() -> Self {
        Self {
            bus: None,
            config: Configuration::default()
                .enable_pressure_measurement(false)
                .enable_temperature_measurement(false),
            int_pin: None,
            _phantom_data: PhantomData,
            _phantom_data_p: PhantomData,
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
            int_pin: self.int_pin,
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
            int_pin: self.int_pin,
            _phantom_data: self._phantom_data,
            _phantom_data_p: self._phantom_data_p,
        }
    }
}

impl<Out, B: Bus> Bmp390Builder<Out, B, NoPin> {
    pub fn use_irq<IntPin: Wait + InputPin>(self, pin: IntPin) -> Bmp390Builder<Out, B, IntPin> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config,
            int_pin: Some(pin),
            _phantom_data: self._phantom_data,
            _phantom_data_p: PhantomData,
        }
    }
}

impl<Out, B: Bus, IntPin: Wait + InputPin> Bmp390Builder<Out, B, IntPin> {

    pub async fn into_fifo<D: DelayNs>(
        self,
        mut delay: D,
    ) -> Result<fifo::FifoDevice<Out, B, IntPin, D>, BuilderError<B::Error>> {
        let bus = self.bus.ok_or(BuilderError::NoBus)?;
        let config = Configuration::default().power_mode(PowerMode::Normal);
        let device = Bmp390::new(bus, config, &mut delay)
            .await
            .map_err(|e| BuilderError::DeviceError(e))?;

        Ok(fifo::FifoDevice::new(device, self.int_pin, delay))
    }

    pub async fn into_normal<D: DelayNs>(
        self,
        mut delay: D,
    ) -> Result<normal::NormalDevice<Out, B, IntPin, D>, BuilderError<B::Error>> {
        let bus = self.bus.ok_or(BuilderError::NoBus)?;
        let config = Configuration::default().power_mode(PowerMode::Normal);
        let device = Bmp390::new(bus, config, &mut delay)
            .await
            .map_err(|e| BuilderError::DeviceError(e))?;

        Ok(normal::NormalDevice::new(device, self.int_pin, delay)
            .await
            .map_err(BuilderError::DeviceError)?)
    }

    pub async fn into_forced<D: DelayNs>(
        self,
        mut delay: D,
    ) -> Result<forced::ForcedDevice<Out, B, IntPin, D>, BuilderError<B::Error>> {
        let bus = self.bus.ok_or(BuilderError::NoBus)?;
        let config = Configuration::default().power_mode(PowerMode::Sleep);
        let device = Bmp390::new(bus, config, &mut delay)
            .await
            .map_err(BuilderError::DeviceError)?;

        Ok(forced::ForcedDevice::new(device, self.int_pin, delay)
            .await
            .map_err(BuilderError::DeviceError)?)
    }
}

impl<B: Bus> Bmp390Builder<NoOutput, B> {
    pub fn enable_pressure(self) -> Bmp390Builder<Pressure, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_pressure_measurement(true),
            int_pin: self.int_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p,
        }
    }

    pub fn enable_temperature(self) -> Bmp390Builder<Temperature, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_temperature_measurement(true),
            int_pin: self.int_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p,
        }
    }
}

impl<B: Bus> Bmp390Builder<Pressure, B> {
    pub fn enable_temperature(self) -> Bmp390Builder<PressureAndTemperature, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_temperature_measurement(true),
            int_pin: self.int_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p,
        }
    }
}

impl<B: Bus> Bmp390Builder<Temperature, B> {
    pub fn enable_pressure(self) -> Bmp390Builder<PressureAndTemperature, B> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_pressure_measurement(true),
            int_pin: self.int_pin,
            _phantom_data: PhantomData,
            _phantom_data_p: self._phantom_data_p,
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

/// Waits for data to be ready in the Data registers.
///
/// This method will prioritize using interrupts, and will fall back to Waiting the maximum measurement time if no interrupt pin as been configured.
async fn wait_for_data<B: Bus, IntPin: Wait + InputPin, D: DelayNs, Out>(
    device: &mut Bmp390<B>,
    int_pin: &mut Option<IntPin>,
    delay: &mut D,
) -> TypeStateResult<Measurement<Out>, B::Error, IntPin::Error> {
    if let Some(int_pin) = int_pin {
        loop {
            while int_pin.is_high().map_err(TypeStateError::Pin)? {
                let int_status = device
                    .read::<IntStatus>()
                    .await
                    .map_err(TypeStateError::Device)?;
                if int_status.drdy {
                    let data = device
                        .read_sensor_data()
                        .await
                        .map_err(TypeStateError::Device)?;
                    return Ok(Measurement::new(data.temperature, data.pressure));
                }
            }

            int_pin
                .wait_for_rising_edge()
                .await
                .map_err(TypeStateError::Pin)?;
        }
    } else {
        delay.delay_us(device.maximum_measurement_time_us).await;

        let data = device
            .read_sensor_data()
            .await
            .map_err(TypeStateError::Device)?;
        Ok(Measurement::new(data.temperature, data.pressure))
    }
}
