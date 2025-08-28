use crate::bus::Bus;
use crate::typestate::measurement::Measurement;
use crate::typestate::{Bmp390Mode, Normal, OutputConfig, TypeStateResult};
use crate::{Bmp390, Bmp390Result};
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use crate::bmp390::Interrupts;

impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs, const USE_FIFO: bool>
    Bmp390Mode<Normal, Out, B, IntPin, Delay, USE_FIFO>
{
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

        Self::configure(&mut device).await?;
        
        Ok(Self {
            device,
            int_pin,
            delay,
            _phantom_data: PhantomData,
        })
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
impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs>
Bmp390Mode<Normal, Out, B, IntPin, Delay, false>
{
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
    pub async fn read_next_measurement(
        &mut self,
    ) -> TypeStateResult<Measurement<Out>, B::Error, IntPin::Error> {
        Ok(self.wait_for_data().await?)
    }
}