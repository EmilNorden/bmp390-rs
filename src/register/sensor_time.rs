//! ### SENSOR_TIME - Sensor time data (`0x0C`, 3 byte, Read-only)
//!
//! Contains a 24-bit sensor timestamp stored in consecutive registers: `SENSOR_TIME_0 (0x0C)`, `SENSOR_TIME_1 (0x0D)` `SENSOR_TIME_2 (0x0E)`
//! For a total of 3 bytes, or 1 24-bit value.
//!
//! Instead of exposing each individual SENSOR_DATA register, this module exposes a single marker type [`SensorTime`] with address `0x0C` and length `3` since these
//! registers need to be burst read all at once to prevent mix-up of bytes.
//!
//! The datasheet does not specify a tick period for the sensor time, so it can only be used for relative timing/ordering.
//!
//! ### Default values
//! 0x00
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::sensor_time::SensorTime;
//!
//! // Get current sensor time
//! let time = device.read::<SensorTime>().await?;
//! println!("{:?}", time);
//!
//! # Ok(()) }
//! ```
use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker type for the SENSOR_TIME (0x0C-0x0E) registers
///
/// - **Length:** 3 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<SensorTime>()`]
pub struct SensorTime;
impl Reg for SensorTime { const ADDR: u8 = 0x0C; }

impl Readable for SensorTime {
    type Out = u32;
    const N: usize = 3;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(u32::from_le_bytes([b[0], b[1], b[2], 0]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_time_decode() {
        let reg = SensorTime::decode(&[0xAA, 0xBB, 0xCC]).unwrap();

        assert_eq!(0xCCBBAA, reg);
    }
}