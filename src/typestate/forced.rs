use crate::bus::Bus;
use crate::register::pwr_ctrl::PowerMode;
use crate::typestate::measurement::Measurement;
use crate::typestate::{Bmp390Mode, Forced, OutputConfig, TypeStateError, TypeStateResult};
use crate::{Bmp390, Bmp390Result};
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use crate::bmp390::Interrupts;

impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs, const USE_FIFO: bool>
    Bmp390Mode<Forced, Out, B, IntPin, Delay, USE_FIFO>
{
    pub(crate) async fn new(
        mut device: Bmp390<B>,
        int_pin: Option<IntPin>,
        delay: Delay,
    ) -> Bmp390Result<Bmp390Mode<Forced, Out, B, IntPin, Delay, USE_FIFO>, B::Error> {
        // If an interrupt pin is given, we want to setup the device to enable DRDY interrupts and disable FIFO interrupts.
        if int_pin.is_some() {
            device
                .mask_interrupts(Interrupts::new().fifo_full().fifo_watermark())
                .await?;
        }

        Self::configure(&mut device).await?;

        Ok(Self {
            device,
            int_pin,
            delay,
            _phantom_data: PhantomData,
        })
    }
}

impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs>
Bmp390Mode<Forced, Out, B, IntPin, Delay, false>
{
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

        Ok(self.wait_for_data().await?)
    }
}

impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs>
Bmp390Mode<Forced, Out, B, IntPin, Delay, true>
{
    /// Triggers a new measurement and stores the result in the FIFO for later consumption.'
    ///
    /// The measurement can be read-out using the [`Bmp390Mode::dequeue()´] method.
    pub async fn enqueue_measurement(&mut self) -> TypeStateResult<(), B::Error, IntPin::Error> {
        self.device
            .set_mode(PowerMode::Forced)
            .await
            .map_err(TypeStateError::Device)?;

        Ok(())
    }
}