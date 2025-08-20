use embassy_time::Delay;
use heapless::LinearMap;
use crate::{Bmp390, Bmp390Error};
use crate::bus::Bus;
use crate::config::Configuration;
use crate::register::{Readable, Writable};

pub struct FakeBus<'a, const N: usize> {
    regs: LinearMap<u8, &'a[u8], N>,
}

impl<'a, const N: usize> FakeBus<'a, N> {
    pub fn new() -> Self {
        FakeBus {
            regs: LinearMap::new(),
        }
    }

    pub fn mock_register(&mut self, reg: u8, data: &'a [u8]) {
        self.regs.insert(reg, data).unwrap();
    }
}

impl<const N: usize> Bus for FakeBus<'_, N> {
    type Error = ();

    async fn read<R: Readable>(&mut self) -> Result<R::Out, Bmp390Error<Self::Error>> {
        if let Some(bytes) = self.regs.get(&R::ADDR) {
            if bytes.len() == R::N {
                return Ok(R::decode(bytes).unwrap())
            }
        }

        panic!("No mocked value for register {}", R::ADDR)
    }

    async fn write<W: Writable>(&mut self, _: &W::In) -> Result<(), Bmp390Error<Self::Error>> {
        todo!()
    }
}

pub async fn dummy_device() -> Bmp390<FakeBus<'static, 10>> {
   Bmp390::new(
       FakeBus::new(),
       Configuration::default(),
       &mut Delay{}
   ).await.unwrap()
}
