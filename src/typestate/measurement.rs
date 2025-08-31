use crate::typestate::{Pressure, PressureAndTemperature, Temperature};
use core::fmt::{Debug, Formatter};
use core::marker::PhantomData;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Measurement<TPress, TTemp, Out> {
    temperature: TTemp,
    pressure: TPress,
    _phantom: PhantomData<Out>,
}

impl<TPress: Copy, TTemp: Copy, Out> Measurement<TPress, TTemp, Out> {
    pub fn new(temperature_c: TTemp, pressure_pa: TPress) -> Self {
        Self {
            temperature: temperature_c,
            pressure: pressure_pa,
            _phantom: PhantomData,
        }
    }
}

impl<TPress: Copy, TTemp: Copy> Measurement<TPress, TTemp, Pressure> {
    pub fn pressure_pascal(&self) -> TPress {
        self.pressure
    }
}

impl<TPress: Copy + Debug, TTemp: Copy + Debug> Debug for Measurement<TPress, TTemp, Pressure> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Measurement")
            .field("pressure_pa", &self.pressure)
            .finish()
    }
}

impl<TPress: Copy, TTemp: Copy> Measurement<TPress, TTemp, Temperature> {
    pub fn temperature_celsius(&self) -> TTemp {
        self.temperature
    }
}

impl<TPress: Copy + Debug, TTemp: Copy + Debug> Debug for Measurement<TPress, TTemp, Temperature> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Sample")
            .field("temperature_c", &self.temperature)
            .finish()
    }
}

impl<TPress: Copy, TTemp: Copy> Measurement<TPress, TTemp, PressureAndTemperature> {
    pub fn pressure_pascal(&self) -> TPress {
        self.pressure
    }

    pub fn temperature_celsius(&self) -> TTemp {
        self.temperature
    }
}

impl<TPress: Copy + Debug, TTemp: Copy + Debug> Debug for Measurement<TPress, TTemp, PressureAndTemperature> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Sample")
            .field("pressure_pa", &self.pressure)
            .field("temperature_c", &self.temperature)
            .finish()
    }
}

#[cfg(feature = "uom")]
impl<Out> Measurement<f32, f32, Out> {
    pub fn into_uom(self) -> Measurement<uom::si::f32::Pressure, uom::si::f32::ThermodynamicTemperature, Out> {
        use uom::si::thermodynamic_temperature::{ThermodynamicTemperature, degree_celsius};
        use uom::si::pressure::{Pressure, pascal};
        Measurement::new(
            ThermodynamicTemperature::new::<degree_celsius>(self.temperature),
            Pressure::new::<pascal>(self.pressure)
        )
    }
}