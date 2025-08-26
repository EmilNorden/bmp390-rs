use crate::bus::Bus;
use crate::register::pwr_ctrl::PowerMode;
use crate::typestate::measurement::Measurement;
use crate::typestate::TypeStateResult;
use crate::{Bmp390, Bmp390Result, Interrupts};
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;

/// Represents a BMP390 device in [`PowerMode::Normal`].
///
/// In this mode, the device will continuously produce new measurements for readout in the Data registers.
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
/// // Creates a NormalDevice that uses SPI and outputs both pressure and temperature.
///
/// let forced_device = Bmp390Builder::new()
///     .use_spi(spi)
///     .enable_pressure()
///     .enable_temperature()
///     .into_normal(delay);
///
/// # });
pub struct NormalDevice<Out, B, IntPin, Delay> {
    device: Bmp390<B>,
    int_pin: Option<IntPin>,
    delay: Delay,
    _phantom_data: PhantomData<Out>,
}

impl<Out, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs> NormalDevice<Out, B, IntPin, Delay> {

    pub(crate) async fn new(
        mut device: Bmp390<B>,
        int_pin: Option<IntPin>,
        delay: Delay,
    ) -> Bmp390Result<Self, B::Error> {
        // If an interrupt pin is given, we want to setup the device to enable DRDY interrupts and disable FIFO interrupts.
        if int_pin.is_some() {
            device
                .mask_interrupts(Interrupts::new().fifo_full().fifo_watermark())
                .await?;
        }
        Ok(Self {
            device,
            int_pin,
            delay,
            _phantom_data: PhantomData,
        })
    }

    /// Reads the latest measurement from the device.
    ///
    /// This method will not synchronize data readout with the configured measurement rate, so if you're calling
    /// this method faster than the device measurement rate, you will read the same data multiple times.
    pub async fn read_latest_measurement(&mut self) -> Bmp390Result<Measurement<Out>, B::Error> {
        let data = self.device.read_sensor_data().await?;

        Ok(Measurement::new(data.temperature, data.pressure))
    }

    /// Reads the next measurement from the device.
    ///
    /// If an interrupt pin was configured using the [`Bmp390Builder::use_irq`] method, this method will use the
    /// [`Interrupts::data_ready()`] interrupt to wait for data. If no interrupt pin was configured, it will simply wait the
    /// maximum measurement time as described by the datasheet section 3.9 before reading the data.
    pub async fn read_next_measurement(&mut self) -> TypeStateResult<Measurement<Out>, B::Error, IntPin::Error> {
        Ok(super::wait_for_data(
            &mut self.device,
            &mut self.int_pin,
            &mut self.delay).await?)
    }
}
/*
TODO: Iterators are synchronous by nature. AsyncIterators seems to be experimental. Look into Streams instead. Perhaps feature-gated since it looks like it will pull in extra dependencies.
pub struct MeasurementIterator<'a, B: Bus, Out, IntPin, Delay> {
    int_pin: &'a mut Option<IntPin>,
    device:  &'a mut Bmp390<B>,
    delay:   &'a mut Delay,
    _phantom: PhantomData<Out>,
}
*/

