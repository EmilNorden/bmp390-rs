use crate::bus::Bus;
use crate::register::pwr_ctrl::PowerMode;
use crate::typestate::measurement::Measurement;
use crate::typestate::{TypeStateError, TypeStateResult};
use crate::{Bmp390, Bmp390Result, Interrupts};
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;

/// Represents a BMP390 device in [`PowerMode::Forced`].
///
/// Technically, the device spends more time in [`PowerMode::Sleep`], as Forced is only a transient state that triggers a measurement and then returns to Sleep mode.
/// You can construct a [`ForcedDevice`] through a [`Bmp390Builder`].
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
pub struct ForcedDevice<Out, B, IntPin, Delay> {
    device: Bmp390<B>,
    int_pin: Option<IntPin>,
    delay: Delay,
    _phantom_data: PhantomData<Out>,
}

impl<Out, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs> ForcedDevice<Out, B, IntPin, Delay> {
    pub(crate) async fn new(
        mut device: Bmp390<B>,
        int_pin: Option<IntPin>,
        delay: Delay,
    ) -> Bmp390Result<ForcedDevice<Out, B, IntPin, Delay>, B::Error> {
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

    /// Triggers a new measurement and waits for the results.
    ///
    /// If an interrupt pin was configured using the [`Bmp390Builder::use_irq`] method, this method will use the
    /// [`Interrupts::data_ready()`] interrupt to wait for data. If no interrupt pin was configured, it will simply wait the
    /// maximum measurement time as described by the datasheet section 3.9 before reading the data.
    ///
    /// The returned [`Measurement`] type will only provide you with the data you configured the device to return using the [`Bmp390Builder::enable_pressure`] and [`Bmp390Builder::enable_temperature`] methods.
    pub async fn read_measurement(
        &mut self,
    ) -> TypeStateResult<Measurement<Out>, B::Error, IntPin::Error> {
        self.device
            .set_mode(PowerMode::Forced)
            .await
            .map_err(TypeStateError::Device)?;

        Ok(super::wait_for_data(&mut self.device, &mut self.int_pin, &mut self.delay).await?)
    }
}
