//! ### DATA - Sensor data (`0x04`, 6 bytes, R)
//!
//! Contains pressure and temperature data stored in consecutive registers:
//! - Pressure: `DATA_0 (0x04)`, `DATA_1 (0x05)`, `DATA_2 (0x06)`
//! - Temperature: `DATA_3 (0x07)`, `DATA_4 (0x08)`, `DATA_5 (0x09)`
//!
//! For a total of 6 bytes, or 2 24-bit values.
//!
//! Instead of exposing each individual data register, this module exposes a single marker type [`Data`] with address `0x04` and length `6` since these
//! registers need to be burst read all at once to prevent mix-up of bytes belonging to different measurements.
//!
//! **Note:** Reading from this register will yield uncalibrated raw sensor data. In order for them to be usable, they need to be calibrated.
//! All sensor data read through any of the exposed methods in this driver will always return *calibrated* data unless specified otherwise.
//!
//! ### Default values
//! N/A
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::data::Data;
//!
//! // Print raw uncalibrated measurements
//! let sensor_data = device.read::<Data>().await?;
//! println!("temp: {}, pressure: {}", sensor_data.temperature, sensor_data.pressure);
//!
//! # Ok(()) }
//! ```
use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the DATA_0 - DATA_5 (0x04 - 0x09) registers.
///
/// - **Length:** 6 bytes
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Data>()`] or the method
/// [`Bmp390::read_sensor_data()`].
///
/// The BMP390 will auto-increment on multiple reads, so reading 6 bytes from 0x04 will read
/// pressure and temperature in one burst read as recommended by the datasheet, section 3.10.
/// Note that this will return the raw uncalibrated measurement data. So for most use cases
/// calling [`Bmp390::read_sensor_data()`] is recommended as it will calibrate the data for you.
pub struct Data;
impl Reg for Data { const ADDR: u8 = 0x04;}

/// The payload for the Data (0x04 - 0x09) registers.
#[derive(Copy, Clone, Debug)]
pub struct DataSample {
    /// The 24-bit uncalibrated pressure data from the DATA_0, DATA_1 and DATA_2 registers.
    pub pressure: u32,

    /// The 24-bit uncalibrated temperature data from the DATA_3, DATA_4 and DATA_5 registers.
    pub temperature: u32,
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