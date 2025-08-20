use crate::register::config::IIRFilterCoefficient;
use crate::register::odr::OutputDataRate;
use crate::register::osr::Oversampling;
use crate::register::pwr_ctrl::PowerMode;

pub struct Configuration {
    pub(crate) enable_pressure: bool,
    pub(crate) enable_temperature: bool,
    pub(crate) mode: PowerMode,
    pub(crate) output_data_rate: OutputDataRate,
    pub(crate) pressure_oversampling: Oversampling,
    pub(crate) temperature_oversampling: Oversampling,
    pub(crate) iir_filter_coefficient: IIRFilterCoefficient,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            enable_pressure: true,
            enable_temperature: true,
            mode: PowerMode::Normal,
            output_data_rate: OutputDataRate::R50Hz,
            iir_filter_coefficient: IIRFilterCoefficient::Coef15,
            pressure_oversampling: Oversampling::X8,
            temperature_oversampling: Oversampling::X1,
        }
    }
}

impl Configuration {
    pub fn enable_pressure_measurement(mut self, enable: bool) -> Self {
        self.enable_pressure = enable;

        self
    }

    /// Enables or disables temperature measurements.
    /// Temperature is needed to perform compensation on pressure readings, and since this driver
    /// caches the latest temperature reading, it is recommended to perform a temperature reading at least once.
    pub fn enable_temperature_measurement(mut self, enable: bool) -> Self {
        self.enable_temperature = enable;

        self
    }

    pub fn power_mode(mut self, power_mode: PowerMode) -> Self {
        self.mode = power_mode;

        self
    }

    pub fn output_data_rate(mut self, output_data_rate: OutputDataRate) -> Self {
        self.output_data_rate = output_data_rate;

        self
    }

    pub fn iir_filter_coefficient(mut self, filter_coefficient: IIRFilterCoefficient) -> Self {
        self.iir_filter_coefficient = filter_coefficient;

        self
    }

    pub fn pressure_oversampling(mut self, pressure_oversampling: Oversampling) -> Self {
        self.pressure_oversampling = pressure_oversampling;

        self
    }

    pub fn temperature_oversampling(mut self, temperature_oversampling: Oversampling) -> Self {
        self.temperature_oversampling = temperature_oversampling;

        self
    }

    pub fn from_preset(p: Preset)  -> Self {
        match p {
            Preset::HandheldLowPower => Configuration::default()
                .iir_filter_coefficient(IIRFilterCoefficient::Coef3)
                .output_data_rate(OutputDataRate::R12p5Hz),
            Preset::HandheldDynamic => Configuration::default()
                .pressure_oversampling(Oversampling::X4),
            Preset::WeatherMonitoring => Configuration::default()
                .power_mode(PowerMode::Forced)
                .pressure_oversampling(Oversampling::X1)
                .iir_filter_coefficient(IIRFilterCoefficient::Coef0)
                .output_data_rate(OutputDataRate::R0p01Hz),
            Preset::DropDetection => Configuration::default()
                .pressure_oversampling(Oversampling::X2)
                .iir_filter_coefficient(IIRFilterCoefficient::Coef0)
                .output_data_rate(OutputDataRate::R100Hz),
            Preset::IndoorNavigation => Configuration::default()
                .pressure_oversampling(Oversampling::X16)
                .temperature_oversampling(Oversampling::X2)
                .iir_filter_coefficient(IIRFilterCoefficient::Coef15)
                .output_data_rate(OutputDataRate::R25Hz),
            Preset::Drone => Configuration::default()
                .iir_filter_coefficient(IIRFilterCoefficient::Coef3),
            Preset::IndoorLocalization => Configuration::default()
                .pressure_oversampling(Oversampling::X1)
                .output_data_rate(OutputDataRate::R0p78Hz)
        }
    }
}

pub enum Preset {
    HandheldLowPower,
    HandheldDynamic,
    WeatherMonitoring,
    DropDetection,
    IndoorNavigation,
    Drone,
    IndoorLocalization,
}

