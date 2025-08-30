#![cfg_attr(
    not(doctest),
    doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md"))
)]
#![no_std]
#![warn(missing_docs)]

pub mod bus;
mod calibration;
pub mod config;
pub mod register;

mod bmp390;
pub mod error;
pub mod fifo;
#[cfg(test)]
pub mod testing;
pub mod typestate;

pub use crate::bmp390::{Bmp390, Bmp390Result, Interrupts, SdoPinState, ResetPolicy};
