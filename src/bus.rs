use crate::Bmp390Error;
use crate::register::Register;

pub trait Bus {
    type Error;
    fn write_register(&mut self, reg: Register, data: u8) ->  impl Future<Output = Result<(), Self::Error>>;

    fn read_register(&mut self, reg: Register, data: &mut [u8]) -> impl Future<Output = Result<(), Self::Error>>;
}

pub struct I2c<I2cType> {
    i2c: I2cType,
    address: u8
}

impl<I2cType> I2c<I2cType>
where
    I2cType: embedded_hal_async::i2c::I2c
{
    pub(crate) fn new(i2c: I2cType, address: u8) -> Self {
        Self { i2c, address }
    }
}

impl<I2cType> Bus for I2c<I2cType>
where
    I2cType: embedded_hal_async::i2c::I2c,
{
    type Error = <I2cType as embedded_hal_async::i2c::ErrorType>::Error;

    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address,&[reg.addr(), value]).await?;

        Ok(())
    }

    async fn read_register(&mut self, reg: Register, data: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address,&[reg.addr()], data).await?;

        Ok(())
    }
}

pub struct Spi<SpiType> {
    spi: SpiType,
}

impl<SpiType> Spi<SpiType>
where
    SpiType: embedded_hal_async::spi::SpiDevice
{
    pub(crate) fn new(spi: SpiType) -> Self {
        Self { spi }
    }

}

impl<SpiType> Bus for Spi<SpiType>
where
    SpiType: embedded_hal_async::spi::SpiDevice,
{
    type Error = <SpiType as embedded_hal_async::spi::ErrorType>::Error;

    async fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg.addr(), value]).await?;

        Ok(())
    }

    async fn read_register(&mut self, reg: Register, data: &mut [u8]) -> Result<(), Self::Error>{
        use embedded_hal_async::spi::Operation;
        self.spi.transaction(
            &mut [Operation::Write(&[reg.addr()]), Operation::Read(data)],
        ).await?;

        Ok(())
    }
}
