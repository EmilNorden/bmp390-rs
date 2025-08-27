/*use crate::bus::Bus;
use crate::fifo::{ControlFrameType, FifoConfiguration, FifoFrame, FifoFullBehavior, SensorFrameType};
use crate::register::fifo_length::FifoLength;
use crate::typestate::measurement::Measurement;
use crate::typestate::{Bmp390Mode, Forced, Normal, OutputConfig, TypeStateError, TypeStateResult};
use crate::{Bmp390, Bmp390Result};
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;

impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, Delay: DelayNs>
    Bmp390Mode<Fifo, Out, B, IntPin, Delay>
{
    pub(crate) async fn new(
        mut device: Bmp390<B>,
        int_pin: Option<IntPin>,
        delay: Delay,
    ) -> Bmp390Result<Bmp390Mode<Fifo, Out, B, IntPin, Delay>, B::Error> {

        let mut fifo_config = device.fifo_configuration().await?;
        /*if let Some(options) = options {
            fifo_config = fifo_config
                .set_subsampling(options.subsampling)
                .set_fifo_full_behavior(options.fifo_full_behavior)
                .set_apply_iir_filter(options.apply_iir_filter)
        }*/

        device
            .set_fifo_configuration(
                fifo_config
                    .set_fifo_enabled(true)
                    .set_pressure_enabled(Out::PRESSURE)
                    .set_temperature_enabled(Out::TEMPERATURE)
                    .set_time_enabled(true),
            )
            .await?;

        Ok(Self {
            device,
            int_pin,
            delay,
            _phantom_data: PhantomData,
        })
    }




}*/