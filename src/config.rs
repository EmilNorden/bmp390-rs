//! This module contains types for configuring the initial state of the Bmp390.

use crate::register::config::IIRFilterCoefficient;
use crate::register::odr::OutputDataRate;
use crate::register::osr::Oversampling;
use crate::register::pwr_ctrl::PowerMode;

/// Holds the initial configuration of the Bmp390 driver.
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
    /// Enables os disables pressure sensor.
    pub fn enable_pressure_measurement(mut self, enable: bool) -> Self {
        self.enable_pressure = enable;

        self
    }

    /// Enables or disables temperature sensor.
    /// 
    /// Temperature is needed to perform compensation on pressure readings, and since this driver
    /// caches the latest temperature reading, it is recommended to perform a temperature reading at least once.
    pub fn enable_temperature_measurement(mut self, enable: bool) -> Self {
        self.enable_temperature = enable;

        self
    }

    /// Sets the power mode.
    /// 
    /// Note that [`PowerMode::Forced`] is transient and will almost immediately return to [`PowerMode::Sleep`], after a measurement is taken.
    /// 
    /// See datasheet section 3.3 for more information
    pub fn power_mode(mut self, power_mode: PowerMode) -> Self {
        self.mode = power_mode;

        self
    }

    /// Sets the output data rate.
    ///
    /// See datasheet section 3.3.3 and 4.3.19 for more information.
    pub fn output_data_rate(mut self, output_data_rate: OutputDataRate) -> Self {
        self.output_data_rate = output_data_rate;

        self
    }

    /// Set the IIR filter coefficient.
    ///
    /// See datasheet section 3.4.3 for more information.
    pub fn iir_filter_coefficient(mut self, filter_coefficient: IIRFilterCoefficient) -> Self {
        self.iir_filter_coefficient = filter_coefficient;

        self
    }

    /// Set the amount of pressure oversampling.
    ///
    /// See datasheet section 3.4.4 for more information.
    pub fn pressure_oversampling(mut self, pressure_oversampling: Oversampling) -> Self {
        self.pressure_oversampling = pressure_oversampling;

        self
    }

    /// Set the amount of temperature oversampling.
    ///
    /// See datasheet section 3.4.4 for more information.
    pub fn temperature_oversampling(mut self, temperature_oversampling: Oversampling) -> Self {
        self.temperature_oversampling = temperature_oversampling;

        self
    }

    /// Applies a pre-defined configuration based on a preset (or use-case), as defined by the BMP390 datasheet section 3.5 Table 10.
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

/// Defines a number of use cases where some preset configurations are suggested by the BMP390 datasheet.
///
/// Use these together with the [`Configuration::from_preset`].
pub enum Preset {
    /// Handheld low-power device (e.g. Android)
    HandheldLowPower,
    /// Handheld dynamic device (e.g. Android)
    HandheldDynamic,
    /// Weather monitoring (lowest power)
    WeatherMonitoring,
    /// Drop detection
    DropDetection,
    /// Indoor navigation
    IndoorNavigation,
    /// Drone
    Drone,
    /// Indoor localization
    IndoorLocalization,
}

