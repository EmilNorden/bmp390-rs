use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the DATA_0 - DATA_5 (0x04 - 0x09) registers.
/// The BMP390 will auto-increment on multiple reads, so reading 6 bytes from 0x04 will read
/// pressure and temperature in one burst read as recommended by the datasheet, section 3.10.
/// Note that this will return the raw uncalibrated measurement data. So for most use cases
/// calling [`Bmp390::read_sensor_data()`] is recommended as it will calibrate the data for you.
///
/// - **Length:** 6 bytes
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Data>()`] or the method
/// [`Bmp390::read_sensor_data()`].
pub struct Data;
impl Reg for Data { const ADDR: u8 = 0x04;}

pub struct DataSample {
    pressure: u32,
    temperature: u32,
}

impl DataSample {
    pub fn new(pressure: u32, temperature: u32) -> Self {
        Self { pressure, temperature }
    }

    /// Returns the raw uncalibrated pressure data from the DATA_0, DATA_1 and DATA_2 registers
    pub fn pressure(&self) -> u32 { self.pressure }

    /// Returns the raw uncalibrated temperature data from the DATA_3, DATA_4 and DATA_5 registers
    pub fn temperature(&self) -> u32 { self.temperature }
}

impl Readable for Data {
    type Out = DataSample;

    const N: usize = 6;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(DataSample {
            pressure: u32::from_le_bytes([b[0], b[1], b[2], 0]),
            temperature: u32::from_le_bytes([b[3], b[4], b[5], 0]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn data_decode() {
        let reg = Data::decode(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]).unwrap();

        assert_eq!(0xCCBBAA, reg.pressure);
        assert_eq!(0xFFEEDD, reg.temperature);
    }
}