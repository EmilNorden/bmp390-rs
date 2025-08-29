//! This module contains FIFO-related types and functions.

/// Holds all configurable parameters related to FIFO functionality of the BMP390.
/// 
/// This is a combination of fields from the FIFO_CONFIG_1 and FIFO_CONFIG_2 registers.
#[derive(Copy, Clone, Debug)]
pub struct FifoConfiguration {
    fifo_enabled: bool,
    pressure_enabled: bool,
    temperature_enabled: bool,
    time_enabled: bool,
    full_behavior: FifoFullBehavior,
    subsampling: u8,
    apply_iir_filter: bool,
}

impl Default for FifoConfiguration {
    /// Returns a default configuration that is identical to the default configuration of the BMP930.
    fn default() -> Self {
        Self {
            fifo_enabled: false,
            pressure_enabled: false,
            temperature_enabled: false,
            time_enabled: false,
            full_behavior: FifoFullBehavior::Stop,
            subsampling: 0x02,
            apply_iir_filter: false,
        }
    }
}

impl FifoConfiguration {
    
    /// Creates a new instance of the [`FifoConfiguration`] struct with the given initial configuration.
    pub fn new(
        fifo_enabled: bool,
        pressure_enabled: bool,
        temperature_enabled: bool,
        time_enabled: bool,
        full_behavior: FifoFullBehavior,
        subsampling: u8,
        apply_iir_filter: bool)
        -> Self {
        Self {
            fifo_enabled,
            pressure_enabled,
            temperature_enabled,
            time_enabled,
            full_behavior,
            subsampling,
            apply_iir_filter,
        }
    }

    /// Returns true if the FIFO functionality of the BMP390 should be enabled.
    pub fn fifo_enabled(&self) -> bool { self.fifo_enabled }
    
    /// Enables or disables writing measurements to FIFO.
    /// 
    /// If the FIFO is disabled, no measurements will be stored. It is however still possible to read from the FIFO.
    pub fn set_fifo_enabled(mut self, enabled: bool) -> Self {
        self.fifo_enabled = enabled;
        
        self
    }

    /// Returns true if pressure should be measured and stored in the FIFO.
    pub fn pressure_enabled(&self) -> bool { self.pressure_enabled }
    
    /// Sets whether pressure should be measured and stored in the FIFO.
    pub fn set_pressure_enabled(mut self, enabled: bool) -> Self {
        self.pressure_enabled = enabled;
        
        self
    }

    /// Returns true if temperature should be measured and stored in the FIFO.
    pub fn temperature_enabled(&self) -> bool { self.temperature_enabled }

    /// Sets whether temperature should be measured and stored in the FIFO.
    pub fn set_temperature_enabled(mut self, enabled: bool) -> Self {
        self.temperature_enabled = enabled;
        
        self
    }

    /// Returns true if sensor timestamps should be appended to the last frame of the FIFO. (i.e., when the FIFO is fully drained)
    pub fn time_enabled(&self) -> bool { self.time_enabled }

    /// Sets whether sensor timestamps should be appended to the last frame of the FIFO. (i.e., when the FIFO is fully drained)
    pub fn set_time_enabled(mut self, enabled: bool) -> Self {
        self.time_enabled = enabled;
        
        self
    }

    /// Returns the configured behavior for when new measurements are made with a full FIFO.
    pub fn fifo_full_behavior(&self) -> FifoFullBehavior { self.full_behavior }

    /// Sets the behavior for when new measurements are made with a full FIFO.
    pub fn set_fifo_full_behavior(mut self, behavior: FifoFullBehavior) -> Self{
        self.full_behavior = behavior;
        
        self
    }

    /// Returns the level of subsampling that will be performed.
    pub fn subsampling(&self) -> u8 { self.subsampling }

    /// Set the level of subsampling that should be performed.
    ///
    /// This value is clamped to the range 0-7.
    /// Read more about FIFO subsampling in the datasheet section 3.6.2
    pub fn set_subsampling(mut self, subsampling: u8) -> Self{
        self.subsampling = subsampling;
        
        self
    }

    /// Returns true if the IIR filter should be applied to FIFO measurements.
    pub fn apply_iir_filter(&self) -> bool { self.apply_iir_filter }

    /// Sets whether the IIR filter should be applied to FIFO measurements or not.
    ///
    /// Read more about the IIR filter in the datasheet section 3.4.3
    pub fn set_apply_iir_filter(mut self, apply_iir_filter: bool) -> Self {
        self.apply_iir_filter = apply_iir_filter;
        
        self
    }
}

/// This enum describes the different behaviors that are configurable for when the FIFO is full.
///
/// It is used in conjunction with the [`FifoConfiguration`] struct and the [`Bmp390::fifo_configuration`] and [`Bmp390::set_fifo_configuration`] methods.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FifoFullBehavior {
    /// Stop writing to FIFO if it is full.
    Stop,

    /// When the FIFO is full, new measurements will overwrite the oldest one.
    OverwriteOldest,
}
pub(crate) struct FifoHeader {
    pub fh_mode: u8,
    pub fh_param: u8,
}

impl FifoHeader {
    pub fn is_sensor_frame(&self) -> bool {
        self.fh_mode == 0b10
    }

    pub fn is_control_frame(&self) -> bool {
        self.fh_mode == 0b01
    }

    pub fn sensor_time_flag(&self) -> bool {
        (self.fh_param & 0b1000) != 0
    }

    pub fn temperature_flag(&self) -> bool {
        (self.fh_param & 0b0100) != 0
    }

    pub fn pressure_flag(&self) -> bool {
        (self.fh_param & 0b0001) != 0
    }

    pub fn config_error_flag(&self) -> bool {
        self.fh_param == 1
    }

    pub fn config_change_flag(&self) -> bool {
        self.fh_param == 2
    }
}

impl From<u8> for FifoHeader {
    fn from(raw: u8) -> Self {
        FifoHeader {
            fh_mode: (raw & 0b1100_0000) >> 6,
            fh_param: (raw & 0b0011_1100) >> 2,
        }
    }
}

/// Represents the frames that can be read from the FIFO
#[derive(PartialEq, Debug)]
pub enum FifoFrame {
    /// Sensor frame
    SensorFrame(SensorFrameType),

    /// Control frame
    ControlFrame(ControlFrameType),
}

/// Represents the sensor frames that can be read from the FIFO
#[derive(PartialEq, Debug)]
pub enum SensorFrameType {
    /// Sensor timestamp. This is appended to the *last* frame of the FIFO only if sensor time has been enabled.
    SensorTime(u32),
    /// Calibrated pressure. This frame is produced if only pressure output is enabled.
    Pressure(f32),
    /// Calibrated temperature. This frame is produced if only temperature output is enabled.
    Temperature(f32),

    /// Calibrated pressure and temperature. This frame is produced if *both* pressure and temperature output is enabled.
    PressureAndTemperature {
        /// Calibrated pressure.
        pressure: f32,

        /// Calibrated temperature.
        temperature: f32,
    },

    /// Empty frame. This frame is returned when the FIFO is empty and sensor time is disabled.
    Empty
}

/// Represents the control frames that can be read from the FIFO
#[derive(PartialEq, Debug)]
pub enum ControlFrameType {
    /// A configuration error was detected while the FIFO is enabled.
    ConfigError,
    /// Changes has been made to any of these registers/fields while the FIFO is enabled:
    /// - FIFO_CONFIG_1
    /// - FIFO_CONFIG_2
    /// - OSR
    /// - ODR
    /// - CONFIG
    /// - PWR_CTRL (press_en/temp_en)
    ConfigChange,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fifo_header_from_u8() {
        let header = FifoHeader::from(0b1001_0100);
        assert_eq!(header.fh_mode, 0b10);
        assert_eq!(header.fh_param, 0b0101);
    }

    #[test]
    fn fifo_header_is_control_frame() {
        let header = FifoHeader::from(0b1001_0100);
        assert!(!header.is_control_frame());

        let header = FifoHeader::from(0b0101_0100);
        assert!(header.is_control_frame());
    }

    #[test]
    fn fifo_header_is_sensor_frame() {
        let header = FifoHeader::from(0b0101_0100);
        assert!(!header.is_sensor_frame());

        let header = FifoHeader::from(0b1001_0100);
        assert!(header.is_sensor_frame());
    }

    #[test]
    fn fifo_header_config_error_flag() {
        let header = FifoHeader::from(0b0100_0000);
        assert!(!header.config_error_flag());

        let header = FifoHeader::from(0b0100_0100);
        assert!(header.config_error_flag());
    }

    #[test]
    fn fifo_header_config_change_flag() {
        let header = FifoHeader::from(0b0100_0000);
        assert!(!header.config_change_flag());

        let header = FifoHeader::from(0b0100_1000);
        assert!(header.config_change_flag());
    }

    #[test]
    fn fifo_header_sensor_time_flag() {
        let header = FifoHeader::from(0b1000_0000);
        assert!(!header.sensor_time_flag());

        let header = FifoHeader::from(0b1010_0000);
        assert!(header.sensor_time_flag());
    }

    #[test]
    fn fifo_header_temperature_flag() {
        let header = FifoHeader::from(0b1000_0000);
        assert!(!header.temperature_flag());

        let header = FifoHeader::from(0b1001_0000);
        assert!(header.temperature_flag());
    }

    #[test]
    fn fifo_header_pressure_flag() {
        let header = FifoHeader::from(0b1000_0000);
        assert!(!header.pressure_flag());

        let header = FifoHeader::from(0b1000_0100);
        assert!(header.pressure_flag());
    }
}