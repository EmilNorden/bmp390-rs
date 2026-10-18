//! This module contains traits and implementations to abstract away the I2C or SPI implementations to let the driver only care about
//! which registers it wants to read or write.
use crate::error::Bmp390Error;
use crate::register::{Readable, Writable};

pub(crate) const MAX_REG_BYTES:usize = 22;

/// Abstraction for the underlying bus (I2C/SPI) that provides high-level methods for reading and writing registers.
pub trait Bus {
    /// The underlying bus error type.
    type Error;

    /// Call with a register marker type that implements [`Readable`] to read its value from the bus.
    ///
    /// This will read [`R::N`] bytes from the bus and uses [`Readable::decode`] to produce an object of type [`R::Out`]
    fn read<R: Readable>(&mut self) -> impl Future<Output = Result<R::Out, Bmp390Error<Self::Error>>>;

    /// Writes [`W::In`] to register address [`W::ADDR`] by calling its [`Writable::encode`] implementation on the given value and writing the resulting [`[u8]`] slice to the bus.
    fn write<W: Writable>(&mut self, v: &W::In) -> impl Future<Output = Result<(), Bmp390Error<Self::Error>>>;
    
}

/// Wrapper for the `embedded_hal_async::i2c::I2c` that implements the [`Bus`] trait
pub struct I2c<I2cType> {
    i2c: I2cType,
    address: u8,
    scratch: [u8; MAX_REG_BYTES],
}

impl<I2cType> I2c<I2cType>
where
    I2cType: embedded_hal_async::i2c::I2c
{
    pub(crate) fn new(i2c: I2cType, address: u8) -> Self {
        Self { i2c, address, scratch: [0; MAX_REG_BYTES] }
    }
}

impl<I2cType> Bus for I2c<I2cType>
where
    I2cType: embedded_hal_async::i2c::I2c,
{
    type Error = <I2cType as embedded_hal_async::i2c::ErrorType>::Error;

    async fn read<R: Readable>(&mut self) -> Result<R::Out, Bmp390Error<Self::Error>> {
        let buf = &mut self.scratch[..R::N];
        self.i2c.write_read(self.address, &[R::ADDR], buf).await.map_err(Bmp390Error::Bus)?;

        Ok(R::decode(&buf).map_err(Bmp390Error::UnexpectedRegisterData)?)
    }

    async fn write<W: Writable>(&mut self, v: &W::In) -> Result<(), Bmp390Error<Self::Error>> {
        let buf = address_data_pairs::<W>(v, &mut self.scratch);

        self.i2c.write(self.address, buf).await.map_err(Bmp390Error::Bus)?;

        Ok(())
    }
}

/// Encodes `v` as pairs of register address and data in `scratch`.
///
/// The BMP390 does not auto-increment the address on writes: both I2C and SPI writes are
/// sequences of address/data pairs (datasheet sections 5.2.1 and 5.3.1). All register
/// addresses are below 0x80, so the SPI R/W bit of each address byte is 0 (write).
fn address_data_pairs<'a, W: Writable>(v: &W::In, scratch: &'a mut [u8]) -> &'a [u8] {
    let mut data = [0u8; MAX_REG_BYTES];
    W::encode(v, &mut data[..W::N]);

    let buf = &mut scratch[..2 * W::N];
    for (i, &byte) in data[..W::N].iter().enumerate() {
        buf[2 * i] = W::ADDR + i as u8;
        buf[2 * i + 1] = byte;
    }
    buf
}

/// Wrapper for the `embedded_hal_async::spi::SpiDevice` that implements the [`Bus`] trait
pub struct Spi<SpiType> {
    spi: SpiType,
    scratch: [u8; MAX_REG_BYTES + 1], // BMP390 send 1 initial dummy byte in responses, so account for that.
}

impl<SpiType> Spi<SpiType>
where
    SpiType: embedded_hal_async::spi::SpiDevice
{
    pub(crate) fn new(spi: SpiType) -> Self {
        Self { spi, scratch: [0; MAX_REG_BYTES + 1] }
    }

}

impl<SpiType> Bus for Spi<SpiType>
where
    SpiType: embedded_hal_async::spi::SpiDevice,
{
    type Error = <SpiType as embedded_hal_async::spi::ErrorType>::Error;

    async fn read<R: Readable>(&mut self) -> Result<R::Out, Bmp390Error<Self::Error>> {
        use embedded_hal_async::spi::Operation;
        let buf = &mut self.scratch[..R::N + 1]; // +1 to account for dummy byte sent by BMP390 in SPI mode
        // BMP390 uses the MSB bit of the register address to denote reads and writes. 1=Read, 0=Write.
        self.spi.transaction(
            &mut [Operation::Write(&[R::ADDR | 0x80]), Operation::Read(buf)],
        ).await.map_err(Bmp390Error::Bus)?;


        Ok(R::decode(&buf[1..]).map_err(Bmp390Error::UnexpectedRegisterData)?)
    }

    async fn write<W: Writable>(&mut self, v: &W::In) -> Result<(), Bmp390Error<Self::Error>> {
        let buf = address_data_pairs::<W>(v, &mut self.scratch);

        self.spi.write(buf).await.map_err(Bmp390Error::Bus)?;

        Ok(())
    }
}
