use core::marker::PhantomData;
use crate::{Bmp390, Bmp390Result};
use crate::bus::Bus;
use crate::typestate::sample::Sample;

pub struct NormalDevice<Out, B> {
    pub(crate) device: Bmp390<B>,
    pub(crate) _phantom_data: PhantomData<Out>,
}

impl<Out, B: Bus> NormalDevice<Out, B> {
    pub async fn read_measurement(&mut self) -> Bmp390Result<Sample<Out>, B::Error> {
        let data = self.device.read_sensor_data().await?;

        Ok(Sample::new(data.temperature, data.pressure))
    }
}