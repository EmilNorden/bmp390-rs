//! This module provides a high-level interface that encodes BMP390 modes in the type system.
//!
//! Where [`Bmp390`] is the main driver struct for the core interface, the [`Bmp390Mode`] struct is the main type of the typestate interface.
//!
//! The entrypoint to this interface is using the [`Bmp390Builder`] type, which lets you perform multiple configurations that will affect the resulting [`Bmp390Mode`] type.:
//! - **Normal mode**
//! 
//!     In `Normal` mode, the resulting type will provide methods to read the last measurement or wait[^1] for the next measurement.
//! 
//!     Enable this mode by calling [`Bmp390Builder::into_normal`].
//!
//!     **Planned feature:** The aim is that `Normal` mode should feel like dealing with a stream of measurements, so in the future I will look into implementing either async iterators (once stable), or Streams.
//! - **Forced mode**
//! 
//!     In `Forced` mode, the resulting type will provide methods to perform "one-shot" measurements and wait[^1] for the data before returning.
//!     
//!     Enable this mode by calling [`Bmp390Builder::into_forced`].
//! 
//! - **FIFO mode**
//! 
//!     While not strictly a mode, `FIFO` can be enabled together with either `Normal` or `Forced` mode. This will change the abstraction to let you view the BMP390 device as a queue.
//! 
//! [^1] When waiting for the measurement data the [`Bmp390Mode`] will use interrupts if configured in the [`Bmp390Builder`] (using [`Bmp390Builder::use_irq`]), otherwise it will delay period of time based on the output data rate..
mod fifo;
mod forced;
mod measurement;
mod normal;
mod builder;

use crate::bus::Bus;
use crate::fifo::{ControlFrameType, FifoConfiguration, FifoFrame, SensorFrameType};
use crate::register::int_status::IntStatus;
use crate::register::osr::{Osr, OsrCfg};
use crate::typestate::measurement::Measurement;
use crate::{Bmp390, Bmp390Result};
use core::marker::PhantomData;
use embedded_hal::digital::{Error, ErrorKind, ErrorType, InputPin};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;

pub use builder::Bmp390Builder;
use crate::error::Bmp390Error;

/// This represents all possible errors that can occur when using the typestate interface.
#[derive(Debug)]
pub enum TypeStateError<BusError, PinError> {
    /// An error has occurred in the core driver.
    Device(Bmp390Error<BusError>),
    /// An error has occurred with the interrupt pin.
    Pin(PinError),
    /// This error occurs when attempting to read from the FIFO and a control frame of type configuration error is returned.
    FifoConfigError,
}

/// Type alias used to simplify return types throughout the typestate interface.
pub type TypeStateResult<T, BusError, PinError> = Result<T, TypeStateError<BusError, PinError>>;

/// Marker struct for the Forced mode
pub struct Forced;

/// Marker struct for the Normal mode
pub struct Normal;

/// Encapsulates the main [`Bmp390´] driver and applies abstractions depending on the configured mode. This is the main type
/// you will interact with if you are using the typestate API.
///
/// This struct is created and configured using the [`Bmp390Builder`] struct.
/// The available modes are:
/// - Normal
///
///     The Bmp390 device in normal mode performs measurement continuously and places the results into the Data register.
///     This abstraction allows you to get the latest performed measurement, wait for the next measurement, or stream them using futures_core (**NOT IMPLEMENTED**)
///
///     Use [`Bmp390Builder::into_normal()`] to configure this mode.
/// - Forced
///
///     The Bmp390 device in forced mode will remain idle until a measurement is explicitly requested.
///
///     Use [`Bmp390Builder::into_forced()`] to configure this mode.
/// - Fifo
///
///     The Bmp390 device in FIFO mode will continuously perform measurements and store them in a 512 byte on-board FIFO buffer for later readout.
///     This abstraction lets you view the Bmp390 as a read-only queue.
///
///     Use [`Bmp390Builder::into_fifo()`] to configure this mode.
pub struct Bmp390Mode<Mode, Out, B: Bus, IntPin, Delay, const USE_FIFO: bool> {
    device: Bmp390<B>,
    int_pin: Option<IntPin>,
    delay: Delay,
    _phantom_data: PhantomData<(Out, Mode)>,
}

impl<Mode, Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs, const USE_FIFO: bool>
    Bmp390Mode<Mode, Out, B, IntPin, Delay, USE_FIFO>
{
    /// Waits for data to be ready in the Data registers.
    ///
    /// This method will prioritize using interrupts, and will fall back to Waiting the maximum measurement time if no interrupt pin as been configured.
    async fn wait_for_data(
        &mut self,
    ) -> TypeStateResult<Measurement<Out>, B::Error, IntPin::Error> {
        if let Some(int_pin) = &mut self.int_pin {
            loop {
                while int_pin.is_high().map_err(TypeStateError::Pin)? {
                    let int_status = self
                        .device
                        .read::<IntStatus>()
                        .await
                        .map_err(TypeStateError::Device)?;
                    if int_status.drdy {
                        let data = self
                            .device
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
            self.delay
                .delay_us(self.device.max_measurement_time_us())
                .await;

            let data = self
                .device
                .read_sensor_data()
                .await
                .map_err(TypeStateError::Device)?;
            Ok(Measurement::new(data.temperature, data.pressure))
        }
    }

    /// Applies initial configuration. Must be called by Bmp390<Normal> and Bmp390<Forced> when initializing.
    async fn configure(device: &mut Bmp390<B>) -> Bmp390Result<(), B::Error> {
        if USE_FIFO {
            device.set_fifo_configuration(FifoConfiguration::default()
                .set_fifo_enabled(true)
                .set_time_enabled(true)
                .set_temperature_enabled(Out::TEMPERATURE)
                .set_pressure_enabled(Out::PRESSURE)).await?;
        }

        Ok(())
    }

    /// Returns the oversampling configuration from the OSR (0x1C) register.
    pub async fn oversampling_config(&mut self) -> Bmp390Result<OsrCfg, B::Error> {
        Ok(self.device.read::<Osr>().await?)
    }

    /// Writes oversampling configuration to the OSR (0x1C) register.
    pub async fn set_oversampling_config(
        &mut self,
        oversampling: &OsrCfg,
    ) -> Bmp390Result<(), B::Error> {
        Ok(self.device.write::<Osr>(oversampling).await?)
    }
}

/// FIFO related functionality that is common for both Normal and Forced mode
impl<Mode, Out, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs, const USE_FIFO: bool>
Bmp390Mode<Mode, Out, B, IntPin, Delay, USE_FIFO>
{
    /*pub async fn options(&mut self) -> TypeStateResult<FifoDeviceOptions, B::Error, IntPin::Error> {
        Ok(FifoDeviceOptions::from(self.device.fifo_configuration().await.map_err(TypeStateError::Device)?))
    }

    pub async fn set_options(&mut self, options: FifoDeviceOptions) -> TypeStateResult<(), B::Error, IntPin::Error> {
        let fifo_config = self.device.fifo_configuration().await.map_err(TypeStateError::Device)?;

        fifo_config.set_fifo_full_behavior(options.fifo_full_behavior)
            .set_subsampling(options.subsampling)
            .set_apply_iir_filter(options.apply_iir_filter);

        self.device.set_fifo_configuration(fifo_config).await.map_err(TypeStateError::Device)?;

        Ok(())
    }*/

    /// Returns the length (in bytes) of the internal FIFO buffer.
    pub async fn length(&mut self) -> TypeStateResult<u16, B::Error, IntPin::Error> {
        Ok(self.device.fifo_length().await.map_err(TypeStateError::Device)?)
    }

    /// Returns true if [`Bmp390Mode::length´] == 0
    pub async fn is_empty(&mut self) -> TypeStateResult<bool, B::Error, IntPin::Error> {
        Ok(self.length().await? == 0)
    }

    /// Dequeues a frame from the FIFO.
    ///
    /// If there are no more frames ([`Bmp390Mode::length`] == 0), ([`FifoOutput::SensorTime`]) is returned.
    pub async fn dequeue(&mut self) -> TypeStateResult<FifoOutput<Out>, B::Error, IntPin> {
        loop {
            let frame = self
                .device
                .read_fifo_frame()
                .await
                .map_err(TypeStateError::Device)?;
            match frame {
                FifoFrame::SensorFrame(sf) => {
                    return match sf {
                        SensorFrameType::SensorTime(time) => Ok(FifoOutput::SensorTime(time)),
                        SensorFrameType::Pressure(p) => {
                            Ok(FifoOutput::Measurement(Measurement::new(0.0, p)))
                        }
                        SensorFrameType::Temperature(t) => {
                            Ok(FifoOutput::Measurement(Measurement::new(t, 0.0)))
                        }
                        SensorFrameType::PressureAndTemperature {
                            pressure,
                            temperature,
                        } => Ok(FifoOutput::Measurement(Measurement::new(
                            temperature,
                            pressure,
                        ))),
                        SensorFrameType::Empty => Ok(FifoOutput::Empty),
                    };
                }
                FifoFrame::ControlFrame(ctrl) => match ctrl {
                    ControlFrameType::ConfigError => return Err(TypeStateError::FifoConfigError),
                    ControlFrameType::ConfigChange => {} // Do nothing, read next frame instead
                },
            }
        }
    }
}

/// Represents the different FIFO frames that can be read from the on-board FIFO.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FifoOutput<Out> {
    /// Sensor measurement frame. The [`Measurement´] type that this enum variant holds is generic over the output configured when creating the [`Bmp390Mode`].
    Measurement(Measurement<Out>),
    /// Sensor time frame. This frame is produced when there are no more frames in the FIFO and sensor time is enabled.
    SensorTime(u32),
    /// Empty FIFO frame. This frame is produced when there are no more frames in the FIFO and sensor time is not enabled.
    Empty,
}

/// Trait implemented by the marker types [`NoOutput`], [`Temperature`], [`Pressure`] and [`PressureAndTemperature`]
/// It is used when configuring the BMP390 device, to convey which outputs should be enabled.
pub trait OutputConfig {
    /// Should pressure output be enabled?
    const PRESSURE: bool = false;
    /// Should temperature output be enabled?
    const TEMPERATURE: bool = false;
}
/// Marker type for [`Bmp390Mode`] that has not yet been configured to output anything.
pub struct NoOutput;
impl OutputConfig for NoOutput {}

/// Marker type for [`Bmp390Mode`] configured to output temperature.
pub struct Temperature;
impl OutputConfig for Temperature {
    const TEMPERATURE: bool = true;
}
/// Marker type for [`Bmp390Mode`] configured to output pressure.
pub struct Pressure;
impl OutputConfig for Pressure {
    const PRESSURE: bool = true;
}
/// Marker type for [`Bmp390Mode`] configured to output both pressure and temperature.
pub struct PressureAndTemperature;
impl OutputConfig for PressureAndTemperature {
    const PRESSURE: bool = true;
    const TEMPERATURE: bool = true;
}


/// Marker struct for the "No mode chosen yet" state in the Bmp390Builder
pub struct NoMode;

/// Marker struct for the "No bus chosen yet" state in Bmp390Builder
pub struct NoBus;

/// Marker struct for the "No interrupt pin configured yet" state in the Bmp390Builder
#[derive(Debug)]
pub struct NoPin;

/// Error type for [`NoPin`]. Required for [`NoPin`] to implement the [`embedded_hal_async::digital::Wait`] and [`embedded_hal::digital::InputPin`] traits.
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
