use crate::error::Bmp390Error;
use crate::register::{Readable, Writable};

pub(crate) const MAX_REG_BYTES:usize = 22;

pub trait Bus {
    type Error;
    
    fn read<R: Readable>(&mut self) -> impl Future<Output = Result<R::Out, Bmp390Error<Self::Error>>>;

    fn write<W: Writable>(&mut self, v: &W::In) -> impl Future<Output = Result<(), Bmp390Error<Self::Error>>>;
    
}

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
        debug_assert!(W::N + 1 <= self.scratch.len());
        let buf = &mut self.scratch[..W::N + 1];
        buf[0] = W::ADDR;
        W::encode(&v, &mut buf[1..W::N+1]);

        self.i2c.write(self.address, buf).await.map_err(Bmp390Error::Bus)?;

        Ok(())
    }
}

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
        debug_assert!(W::N + 1 <= self.scratch.len());

        let buf = &mut self.scratch[..W::N + 1];
        buf[0] = W::ADDR & 0x7F; // MSB should be 0 for writes.
        W::encode(&v, &mut buf[1..W::N+1]);

        self.spi.write(buf).await.map_err(Bmp390Error::Bus)?;

        Ok(())
    }
}
