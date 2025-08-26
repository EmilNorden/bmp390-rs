use core::fmt::{Debug, Formatter};
use core::marker::PhantomData;
use crate::typestate::{Pressure, PressureAndTemperature, Temperature};

pub struct Measurement<Out> {
    temperature_c: f32,
    pressure_pa: f32,
    _phantom: PhantomData<Out>,
}

impl<Out> Measurement<Out> {
    pub fn new(temperature_c: f32, pressure_pa: f32) -> Self {
        Self { temperature_c, pressure_pa, _phantom: PhantomData }
    }
}

impl Measurement<Pressure> {
    pub fn pressure_pascal(&self) -> f32 {
        self.pressure_pa
    }
}

impl Debug for Measurement<Pressure> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Measurement")
            .field("pressure_pa",  &self.pressure_pa)
            .finish()
    }
}

impl Measurement<Temperature> {
    pub fn temperature_celsius(&self) -> f32 {
        self.temperature_c
    }
}

impl Debug for Measurement<Temperature> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Sample")
            .field("temperature_c",  &self.temperature_c)
            .finish()
    }
}

impl Measurement<PressureAndTemperature> {
    pub fn pressure_pascal(&self) -> f32 {
        self.pressure_pa
    }

    pub fn temperature_celsius(&self) -> f32 {
        self.temperature_c
    }
}

impl Debug for Measurement<PressureAndTemperature> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Sample")
            .field("pressure_pa",  &self.pressure_pa)
            .field("temperature_c",  &self.temperature_c)
            .finish()
    }
}

#[cfg(feature = "uom")]
impl Measurement<Pressure> {
    pub fn pressure_uom(&self) -> uom::si::f32::Pressure {
        uom::si::pressure::Pressure::new::<uom::si::pressure::pascal>(self.pressure_pa)
    }
}
#[cfg(feature = "uom")]
impl Measurement<Temperature> {
    pub fn temperature_uom(&self) -> uom::si::f32::ThermodynamicTemperature {
        uom::si::thermodynamic_temperature::ThermodynamicTemperature::new::<uom::si::thermodynamic_temperature::degree_celsius>(self.temperature_c)
    }
}

#[cfg(feature = "uom")]
impl Measurement<PressureAndTemperature> {
    pub fn pressure_uom(&self) -> uom::si::f32::Pressure {
        uom::si::pressure::Pressure::new::<uom::si::pressure::pascal>(self.pressure_pa)
    }

    pub fn temperature_uom(&self) -> uom::si::f32::ThermodynamicTemperature {
        uom::si::thermodynamic_temperature::ThermodynamicTemperature::new::<uom::si::thermodynamic_temperature::degree_celsius>(self.temperature_c)
    }
}