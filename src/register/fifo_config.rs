//! ### FIFO_CONFIG_1/2 - FIFO configuration (`0x17/0x18`, 1 byte, R/W)
//!
//! Contains FIFO configuration.
//!
//! ### Default values
//! FIFO_CONFIG_1:
//! `fifo_stop_on_full = true`
//!
//! FIFO_CONFIG_2:
//! `fifo_subsampling = 0x02`
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::fifo_config::{FifoConfig1, FifoConfig2};
//!
//! // Enable FIFO with pressure measurements
//! let mut cfg = device.read::<FifoConfig1>().await?;
//! cfg.fifo_mode = true;
//! cfg.fifo_press_en = true;
//! device.write::<FifoConfig1>(&cfg).await?;
//!
//! // Set a different downsampling level
//! let mut cfg = device.read::<FifoConfig2>().await?;
//! cfg.fifo_subsampling = 0x0A;
//! device.write::<FifoConfig2>(&cfg).await?;
//!
//! # Ok(()) }
//! ```
//!
//! See also: [`crate::Bmp390::fifo_configuration()`] and [`crate::Bmp390::set_fifo_configuration()`]

use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

/// Marker type for FIFO_CONFIG_1 (0x17) register
///
/// - **Length:** 1 byte
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<FifoConfig1>()`] or [`Bmp390::write::<FifoConfig1>()`]
pub struct FifoConfig1;
impl Reg for FifoConfig1 { const ADDR: u8 = 0x17; }

/// The payload for the FIFO_CONFIG_1 (0x17) register.
#[derive(Copy, Clone, Debug)]
pub struct FifoConfig1Fields {
    /// Enables or disables writing measurements to FIFO.
    ///
    /// If the FIFO is disabled, no measurements will be stored. It is however still possible to read from the FIFO.
    pub fifo_mode: bool,

    /// Determines the behavior when there are new frames and the FIFO is full.
    ///
    /// If true, no frames will be written to FIFO if it is full. If false, the oldest frame will be overwritten.
    pub fifo_stop_on_full: bool,

    /// Determines whether sensor timestamps should be appended to the last frame of the FIFO. (i.e., when the FIFO is fully drained)
    pub fifo_time_en: bool,

    /// Should pressure frames be written to the FIFO?
    pub fifo_press_en: bool,

    /// Should temperature frames be written to the FIFO?
    pub fifo_temp_en: bool,
}
impl Readable for FifoConfig1 {
    type Out = FifoConfig1Fields;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(FifoConfig1Fields {
            fifo_mode:          (b[0] & 0b00001) != 0,
            fifo_stop_on_full:  (b[0] & 0b00010) != 0,
            fifo_time_en:       (b[0] & 0b00100) != 0,
            fifo_press_en:      (b[0] & 0b01000) != 0,
            fifo_temp_en:       (b[0] & 0b10000) != 0,
        })
    }
}

impl Writable for FifoConfig1 {
    type In = FifoConfig1Fields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.fifo_mode { value |= 0b1;}
        if v.fifo_stop_on_full { value |= 0b10; }
        if v.fifo_time_en { value |= 0b100; }
        if v.fifo_press_en { value |= 0b1000; }
        if v.fifo_temp_en { value |= 0b10000; }

        out[0] = value;
    }
}

/// Marker type for FIFO_CONFIG_2 (0x18) register
///
/// - **Length:** 1 byte
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<FifoConfig2>()`] or [`Bmp390::write::<FifoConfig2>()`]
pub struct FifoConfig2;
impl Reg for FifoConfig2 { const ADDR: u8 = 0x18; }

/// The payload for the FIFO_CONFIG_2 (0x18) register.
#[derive(Copy, Clone, Debug)]
pub struct FifoConfig2Fields {
    /// Downsampling of pressure and temperature data.
    /// The factor is 2^fifo_subsampling, where fifo_subsampling is clamped between 0-7.
    pub fifo_subsampling: u8,

    /// Data source for the pressure and temperature data.
    pub data_select: FifoDataSource,
}

/// Specifies the available data sources for pressure and temperature.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FifoDataSource {
    /// Unfiltered data
    Unfiltered = 0,
    /// Filtered data
    Filtered = 1,
    /// Reserved.
    Reserved = 2,
}

impl From<u8> for FifoDataSource {
    fn from(field: u8) -> Self {
        match field {
            0b00 => FifoDataSource::Unfiltered,
            0b01 => FifoDataSource::Filtered,
            _ => FifoDataSource::Reserved
        }
    }
}

impl Readable for FifoConfig2 {
    type Out = FifoConfig2Fields;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(FifoConfig2Fields {
            fifo_subsampling: b[0] & 0b111,
            data_select: FifoDataSource::from((b[0] & 0b11000) >> 3),
        })
    }
}

impl Writable for FifoConfig2 {
    type In = FifoConfig2Fields;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = v.fifo_subsampling & 0b111;
        value |= (v.data_select as u8) << 3;

        out[0] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_config1_decode() {
        let reg = FifoConfig1::decode(&[0b00001]).unwrap();
        assert!(reg.fifo_mode);

        let reg = FifoConfig1::decode(&[0b00010]).unwrap();
        assert!(reg.fifo_stop_on_full);

        let reg = FifoConfig1::decode(&[0b00100]).unwrap();
        assert!(reg.fifo_time_en);

        let reg = FifoConfig1::decode(&[0b01000]).unwrap();
        assert!(reg.fifo_press_en);

        let reg = FifoConfig1::decode(&[0b10000]).unwrap();
        assert!(reg.fifo_temp_en);
    }

    #[test]
    fn fifo_config1_encode() {
        let mut buffer = [0u8; 1];

        FifoConfig1::encode(&FifoConfig1Fields {
            fifo_mode: false,
            fifo_stop_on_full: false,
            fifo_time_en: false,
            fifo_press_en: false,
            fifo_temp_en: false,
        }, &mut buffer);
        assert_eq!([0], buffer);

        FifoConfig1::encode(&FifoConfig1Fields {
            fifo_mode: true,
            fifo_stop_on_full: false,
            fifo_time_en: true,
            fifo_press_en: false,
            fifo_temp_en: true,
        }, &mut buffer);
        assert_eq!([0b10101], buffer)
    }

    #[test]
    fn fifo_config2_decode() {
        let reg = FifoConfig2::decode(&[0b000_0100]).unwrap();
        assert_eq!(0b100, reg.fifo_subsampling);
        assert_eq!(FifoDataSource::Unfiltered, reg.data_select);

        let reg = FifoConfig2::decode(&[0b000_1010]).unwrap();
        assert_eq!(0b010, reg.fifo_subsampling);
        assert_eq!(FifoDataSource::Filtered, reg.data_select);

        let reg = FifoConfig2::decode(&[0b001_1000]).unwrap();
        assert_eq!(0, reg.fifo_subsampling);
        assert_eq!(FifoDataSource::Reserved, reg.data_select);
    }

    #[test]
    fn fifo_config2_encode() {
        let mut buffer = [0u8; 1];
        FifoConfig2::encode(&FifoConfig2Fields {
            fifo_subsampling: 0,
            data_select: FifoDataSource::Unfiltered,
        }, &mut buffer);
        assert_eq!([0], buffer);

        FifoConfig2::encode(&FifoConfig2Fields {
            fifo_subsampling: 5,
            data_select: FifoDataSource::Filtered,
        }, &mut buffer);
        assert_eq!([0b0000_1101], buffer);

        FifoConfig2::encode(&FifoConfig2Fields {
            fifo_subsampling: 5,
            data_select: FifoDataSource::Reserved,
        }, &mut buffer);
        assert_eq!([0b0001_0101], buffer);
    }
}