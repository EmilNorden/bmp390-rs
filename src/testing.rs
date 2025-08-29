//! This module provides Fakes to be used in unit tests.

use crate::bus::{Bus, MAX_REG_BYTES};
use crate::error::Bmp390Error;
use crate::register::{Readable, Writable};
use embedded_hal_async::delay::DelayNs;
use heapless::LinearMap;

#[derive(Debug)]
enum RegisterValue {
    Data {bytes: [u8; MAX_REG_BYTES], len: usize },
    DontCare
}

/// A fake bus implementation used by unit tests.
///
/// It provides functionality to inject register values to ease testing.
pub struct FakeBus<const N: usize> {
    regs: LinearMap<(u8, usize), RegisterValue, N>,
    scratch: [u8; MAX_REG_BYTES],
}

/// A fake delay implementation used by unit tests
pub struct FakeDelay{}

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, _: u32) {

    }
}

impl<const N: usize> FakeBus<N> {

    /// Creates a new instance of [`FakeBus`]
    pub fn new() -> Self {
        FakeBus {
            regs: LinearMap::new(),
            scratch: [0u8; MAX_REG_BYTES],
        }
    }

    /// Make the [`FakeBus`] respond with the given data when reading the given register.
    pub fn with_response<R: Readable>(&mut self, data: &[u8]){
        let mut register_value = [0u8; MAX_REG_BYTES];
        register_value[..data.len()].copy_from_slice(data);
        self.regs.insert((R::ADDR, R::N), RegisterValue::Data { bytes: register_value, len: data.len()}).unwrap();
    }

    /// Make the [`FakeBus`] respond with garbage data for the given register.
    ///
    /// This method is suitable if you dont want the [`FakeBus`] to panic,
    /// but you are confident that the contents of the register does not matter for your test case
    pub fn with_any_response<R: Readable>(&mut self) {
        self.regs.insert((R::ADDR, R::N), RegisterValue::DontCare).unwrap();
    }
}

impl<const N: usize> Bus for FakeBus<N> {
    type Error = ();

    async fn read<R: Readable>(&mut self) -> Result<R::Out, Bmp390Error<Self::Error>> {
        if let Some(value) = self.regs.get(&(R::ADDR, R::N)) {
            match value {
                RegisterValue::Data { bytes, len} => {
                    if *len == R::N {
                        return Ok(R::decode(&bytes[..R::N]).unwrap())
                    }
                },
                RegisterValue::DontCare => {
                    let data = &self.scratch[0..R::N];
                    return Ok(R::decode(data).unwrap())
                }
            }

        }

        panic!("No mocked value for register 0x{:x} and length {}", R::ADDR, R::N)
    }

    async fn write<W: Writable>(&mut self, _v: &W::In) -> Result<(), Bmp390Error<Self::Error>> {
        Ok(())
    }
}