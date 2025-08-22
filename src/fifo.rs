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

#[derive(PartialEq, Debug)]
pub enum FifoFrame {
    SensorFrame(SensorFrameType),
    ControlFrame(ControlFrameType),
}

#[derive(PartialEq, Debug)]
pub enum SensorFrameType {
    SensorTime(u32),
    Pressure(f32),
    Temperature(f32),
    PressureAndTemperature {
        pressure: f32,
        temperature: f32,
    },
    Empty
}

#[derive(PartialEq, Debug)]
pub enum ControlFrameType {
    ConfigError,
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