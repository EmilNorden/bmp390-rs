//! Errors that can occur when using the BMP390 device.
//!
//! This module provides an error type that encapsulates all possible errors that can occur during communication with BMP390.
//! It is generic over the underlying bus (spi/i2c) error type.

use crate::register::InvalidRegisterField;

/// This represents all possible errors that can occur when using the BMP390 device.
#[derive(Debug)]
pub enum Bmp390Error<BusError> {
    /// An error has occurred in the SPI / I2C driver
    Bus(BusError),

    /// Unable to communicate with BMP390
    ///
    /// Could possibly indicate an error with pin configuration and/or wiring.
    NotConnected,

    /// Reading from a register returned unexpected data. This should not happen in normal circumstances.
    ///
    /// Could possibly indicate a bug in the driver, or less likely, a faulty chip or interference.
    UnexpectedRegisterData(InvalidRegisterField),

    /// Reading from FIFO returned unexpected data. This should not happen in normal circumstances.
    ///
    /// Could possibly indicate a bug in the driver, or less likely, a faulty chip or interference.
    UnexpectedFifoData,
}
