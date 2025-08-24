use core::marker::PhantomData;
use uom::si::f32;
use uom::si::length::meter;
use uom::si::pressure::pascal;
use uom::si::thermodynamic_temperature::degree_celsius;
use crate::{Bmp390, Bmp390Result};
use crate::bus::Bus;
use crate::typestate::{Pressure, PressureAndTemperature, Temperature};

pub struct NormalDevice<Out, B> {
    pub(crate) device: Bmp390<B>,
    pub(crate) _phantom_data: PhantomData<Out>,
}

impl<B: Bus> NormalDevice<Pressure, B> {
    pub async fn read_sensor_data(&mut self) -> Bmp390Result<f32::Pressure, B::Error> {
        let data = self.device.read_sensor_data().await?;

        Ok(f32::Pressure::new::<pascal>(data.pressure))
    }
}

impl<B: Bus> NormalDevice<Temperature, B> {
    pub async fn read_sensor_data(&mut self) -> Bmp390Result<f32::ThermodynamicTemperature, B::Error> {
        let data = self.device.read_sensor_data().await?;

        Ok(f32::ThermodynamicTemperature::new::<degree_celsius>(data.temperature))
    }
}

impl<B: Bus> NormalDevice<PressureAndTemperature, B> {
    pub async fn read_sensor_data(&mut self) -> Bmp390Result<(f32::ThermodynamicTemperature, f32::Pressure), B::Error> {
        let data = self.device.read_sensor_data().await?;

        Ok((f32::ThermodynamicTemperature::new::<degree_celsius>(data.temperature), f32::Pressure::new::<pascal>(data.pressure)))
    }
}

struct PressureReading(f32);