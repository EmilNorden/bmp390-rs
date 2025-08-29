//! This module provides types for strongly typed register access.
//! 
//! The registers are split up into submodules, containing:
//! - One *marker struct* for the register, usually directly named after the register such as [`ErrReg`](err_reg::ErrReg)
//!
//!     This marker struct will implement [`Readable`] for readable registers and [`Writable`] for writable registers.
//! - One value type for the payload of the struct. Continuing with the example of [`ErrReg`](err_reg::ErrReg), that type is [`ErrorFlags`](err_reg::ErrorFlags)
//! - One or more supporting types.

pub mod chip_id;
pub mod rev_id;
pub mod err_reg;
pub mod status;
pub mod data;
pub mod sensor_time;
pub mod event;
pub mod int_status;
pub mod fifo_length;
pub mod fifo_data;
pub mod fifo_wtm;
pub mod fifo_config;
pub mod int_ctrl;
pub mod if_conf;
pub mod pwr_ctrl;
pub mod osr;
pub mod odr;
pub mod config;
pub mod calibration;
pub mod cmd;

/// Error type describing an invalid or unexpected value in a field of one of the registers.
///
/// This is an error that *should* in theory not happen. It could be caused by something like:
/// - A faulty chip.
/// - A bug in this code.
/// - Interference on the bus.
/// - Putting the BMP390 device inside a running microwave oven.
#[derive(Debug)]
pub struct InvalidRegisterField{
    /// The address of the register that was read.
    pub register: u8,
    /// The invalid value.
    pub value: u8,
    /// The bit offset into the register where the value was read. This should be used to identify which field was read.
    pub bit_offset: u8,
}

impl InvalidRegisterField {
    /// Creates a new instance of the struct for the given register address. The value parameter describes the invalid value read, and the bit_offset is the offset into the register where the value was read.
    pub fn new(register: u8, value: u8, bit_offset: u8) -> Self {
        Self { register, value, bit_offset }
    }
}

/// Error type used to signal unexpected values when parsing register fields.
///
/// It contains the invalid value.
pub struct UnexpectedValue(pub u8);

/// Register super-trait for [`Readable`] and [`Writable`] traits.
pub trait Reg {
    /// The address of this register (or register block as in the case of DATA etc.)
    const ADDR: u8;
}

/// Trait implemented by readable registers.
pub trait Readable: Reg {
    /// The type that this register value is serialized/encoded to.
    type Out;

    /// Number of **data** bytes (payload) read for this register/block.
    /// Does **not** include the register address byte.
    const N: usize = 1;

    /// Decodes [`Self::N`] bytes from [`b`] into a value of type [`Self::Out`]
    ///
    /// The [`b`] slice must contain at least [`Self::N`] bytes.
    ///
    /// In case of unexpected values in register fields, this function will return an error of type [`InvalidRegisterField`].
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField>;
}

/// Trait implemented by writable registers.
pub trait Writable: Reg {
    /// The type that this register value is serialized/encoded from.
    type In;

    /// Number of **data** bytes (payload) written for this register/block.
    /// Does **not** include the register address byte.
    const N: usize = 1;

    /// Encodes values of type [`Self::In`] as [`Self::N`] bytes to [`out`]
    ///
    /// The [`out`] slice must be of at least length [`N`]
    fn encode(v: &Self::In, out: &mut [u8]);
}