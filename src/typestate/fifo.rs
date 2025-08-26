use crate::Bmp390;
use crate::bus::Bus;
use crate::fifo::{ControlFrameType, FifoFrame, SensorFrameType};
use crate::register::fifo_length::FifoLength;
use crate::register::pwr_ctrl::PowerMode;
use crate::typestate::forced::ForcedDevice;
use crate::typestate::measurement::Measurement;
use crate::typestate::{TypeStateError, TypeStateResult};
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;

/// Represents a BMP390 device with the FIFO enabled.
///
/// While [`SleepDevice`], [`NormalDevice`] and [`ForcedDevice`] all map to a specific power mode, the [`FifoDevice`] does not.
/// [`FifoDevice`] puts a queue abstraction layer on top of the BMP390 device.
///
/// # Examples
///
/// ```rust,ignore
/// # tokio_test::block_on(async {
/// use bmp390_rs::typestate::Bmp390Builder;
///
/// let spi = setup_spi();
/// let delay = setup_delay();
///
/// // Creates a ForcedDevice that uses SPI and outputs both pressure and temperature.
///
/// let forced_device = Bmp390Builder::new()
///     .use_spi(spi)
///     .enable_pressure()
///     .enable_temperature()
///     .into_forced(delay);
///
/// # });
pub struct FifoDevice<Out, B, IntPin, Delay> {
    device: Bmp390<B>,
    int_pin: Option<IntPin>,
    delay: Delay,
    buffer: [u8; 512],
    _phantom_data: PhantomData<Out>,
}

impl<Out, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs> FifoDevice<Out, B, IntPin, Delay> {
    pub fn new(device: Bmp390<B>, int_pin: Option<IntPin>, delay: Delay) -> Self {
        Self {
            device,
            int_pin,
            delay,
            buffer: [0; 512],
            _phantom_data: PhantomData,
        }
    }

    /// Returns the length (in bytes) of the internal FIFO buffer.
    pub async fn length(&mut self) -> TypeStateResult<u16, B::Error, IntPin::Error> {
        Ok(self
            .device
            .read::<FifoLength>()
            .await
            .map_err(TypeStateError::Device)?)
    }

    pub async fn is_empty(&mut self) -> TypeStateResult<bool, B::Error, IntPin::Error> {
        Ok(self.length().await? == 0)
    }

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

pub enum FifoOutput<Out> {
    Measurement(Measurement<Out>),
    SensorTime(u32),
    Empty,
}
