use crate::bus::{Bus, MAX_REG_BYTES};
use crate::config::Configuration;
use crate::error::Bmp390Error;
use crate::register::{Readable, Writable};
use crate::Bmp390;
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;
use heapless::LinearMap;

#[derive(Debug)]
enum RegisterValue {
    Data {bytes: [u8; MAX_REG_BYTES], len: usize },
    DontCare
}
pub struct FakeBus<const N: usize> {
    regs: LinearMap<(u8, usize), RegisterValue, N>,
    scratch: [u8; MAX_REG_BYTES],
}

pub struct FakeDelay{}

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, _: u32) {

    }
}

impl<const N: usize> FakeBus<N> {
    pub fn new() -> Self {
        FakeBus {
            regs: LinearMap::new(),
            scratch: [0u8; MAX_REG_BYTES],
        }
    }

    pub fn with_response<R: Readable>(&mut self, data: &[u8]){
        let mut register_value = [0u8; MAX_REG_BYTES];
        register_value[..data.len()].copy_from_slice(data);
        self.regs.insert((R::ADDR, R::N), RegisterValue::Data { bytes: register_value, len: data.len()}).unwrap();
    }

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