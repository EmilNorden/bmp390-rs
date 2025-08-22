#![no_std]

pub mod bus;
mod calibration;
pub mod config;
pub mod register;

pub mod fifo;
pub mod testing;

use crate::bus::{Bus, I2c, Spi};
use crate::calibration::CalibrationData;
use crate::config::Configuration;
use crate::fifo::{ControlFrameType, FifoConfiguration, FifoFrame, FifoFullBehavior, FifoHeader, SensorFrameType};
use crate::register::fifo_data::FifoData;
use crate::register::{
    InvalidRegisterField, Readable, Writable, chip_id, data, err_reg, odr, osr,
    pwr_ctrl, status,
};
use core::fmt::Debug;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::SevenBitAddress;
use crate::register::fifo_config::{FifoConfig1, FifoConfig1Fields, FifoConfig2, FifoConfig2Fields, FifoDataSource};
use crate::register::fifo_length::FifoLength;

const BMP390_CHIP_ID: u8 = 0x60;

#[derive(Debug)]
pub enum Bmp390Error<BusError> {
    Bus(BusError),
    NotConnected,
    UnexpectedRegisterData(InvalidRegisterField),
    FormatError,
}

type Bmp390Result<T, BusError> = Result<T, Bmp390Error<BusError>>;

type Bmp390I2c<T> = Bmp390<I2c<T>>;
type Bmp390Spi<T> = Bmp390<Spi<T>>;

pub struct Bmp390<B> {
    bus: B,
    calibration_data: CalibrationData,
}

impl<T> Bmp390I2c<T>
where
    T: embedded_hal_async::i2c::I2c,
    I2c<T>: Bus,
{
    pub async fn new_i2c<D: DelayNs>(
        i2c: T,
        address: SevenBitAddress,
        config: Configuration,
        delay: &mut D,
    ) -> Bmp390Result<Self, <I2c<T> as Bus>::Error> {
        Self::new(I2c::new(i2c, address), config, delay).await
    }
}

impl<T> Bmp390Spi<T>
where
    T: embedded_hal_async::spi::SpiDevice,
    Spi<T>: Bus,
{
    pub async fn new_spi<D: DelayNs>(
        spi: T,
        config: Configuration,
        delay: &mut D,
    ) -> Bmp390Result<Self, <Spi<T> as Bus>::Error> {
        Self::new(Spi::new(spi), config, delay).await
    }
}

impl<B> Bmp390<B>
where
    B: Bus,
{
    async fn probe_ready<D: DelayNs>(
        bus: &mut B,
        delay: &mut D,
        attempts: u32,
    ) -> Bmp390Result<(), B::Error> {
        for _ in 0..attempts {
            if let Ok(id) = bus.read::<register::chip_id::ChipId>().await {
                if id == BMP390_CHIP_ID {
                    return Ok(());
                }
            }

            delay.delay_ms(1).await;
        }

        Err(Bmp390Error::NotConnected)
    }
    async fn new<D: DelayNs>(
        mut bus: B,
        config: Configuration,
        delay: &mut D,
    ) -> Bmp390Result<Self, B::Error> {
        // The datasheet (Section 1, table 2) specifies 2ms start-up time after VDD/VDDIO > 1.8V
        Self::probe_ready(&mut bus, delay, 5).await?;

        let calibration_data = CalibrationData::new(&mut bus).await?;

        bus.write::<pwr_ctrl::PwrCtrl>(&pwr_ctrl::PwrCtrlCfg {
            press_en: config.enable_pressure,
            temp_en: config.enable_temperature,
            mode: config.mode,
        })
        .await?;

        bus.write::<osr::Osr>(&osr::OsrCfg {
            osr_p: config.pressure_oversampling,
            osr_t: config.temperature_oversampling,
        })
        .await?;

        bus.write::<odr::Odr>(&odr::OdrCfg {
            odr_sel: config.output_data_rate,
        })
        .await?;

        bus.write::<register::config::Config>(&register::config::ConfigFields {
            iir_filter: config.iir_filter_coefficient,
        })
        .await?;

        Ok(Bmp390 {
            bus,
            calibration_data,
        })
    }

    pub async fn read<R: Readable>(&mut self) -> Result<R::Out, Bmp390Error<B::Error>> {
        Ok(self.bus.read::<R>().await?)
    }

    pub async fn write<W: Writable>(&mut self, v: &W::In) -> Result<(), Bmp390Error<B::Error>> {
        Ok(self.bus.write::<W>(v).await?)
    }

    pub async fn is_connected(&mut self) -> Bmp390Result<bool, B::Error> {
        let id = self.bus.read::<chip_id::ChipId>().await?;

        Ok(id == BMP390_CHIP_ID)
    }

    /// Returns the error flags from the ERR_REG (0x02) register.
    ///
    /// These flags are
    pub async fn error_flags(&mut self) -> Bmp390Result<err_reg::ErrorFlags, B::Error> {
        Ok(self.bus.read::<err_reg::ErrReg>().await?)
    }

    /// Returns the status from the STATUS (0x03) register.
    pub async fn status(&mut self) -> Bmp390Result<status::StatusFlags, B::Error> {
        Ok(self.bus.read::<status::Status>().await?)
    }

    /// Returns the current FIFO configuration. This is a combination of the fields stored in registers FIFO_CONFIG_1 (0x17) and FIFO_CONFIG_2 (0x18).
    ///
    /// You could theoretically access the same information using [`Bmp390::read::<FifoConfig1>`] and [`Bmp390::read::<FifoConfig2>`].
    pub async fn fifo_configuration(&mut self) -> Bmp390Result<FifoConfiguration, B::Error> {
        let config1 = self.read::<FifoConfig1>().await?;
        let config2 = self.read::<FifoConfig2>().await?;

        Ok(FifoConfiguration::new(
            config1.fifo_mode,
            config1.fifo_press_en,
            config1.fifo_temp_en,
            config1.fifo_time_en,
            if config1.fifo_stop_on_full { FifoFullBehavior::Stop } else { FifoFullBehavior::OverwriteOldest },
            config2.fifo_subsampling,
            config2.data_select == FifoDataSource::Filtered
        ))
    }

    /// Writes a new FIFO configuration to the device.
    ///
    /// This equates to updating registers FIFO_CONFIG_1 (0x17) and FIFO_CONFIG_2 (0x18)
    pub async fn set_fifo_configuration(&mut self, cfg: FifoConfiguration) -> Bmp390Result<(), B::Error> {
        self.bus.write::<FifoConfig1>(&FifoConfig1Fields {
            fifo_mode: cfg.enable_fifo(),
            fifo_stop_on_full: cfg.fifo_full_behavior() == FifoFullBehavior::Stop,
            fifo_time_en: cfg.enable_time(),
            fifo_press_en: cfg.enable_pressure(),
            fifo_temp_en: cfg.enable_temperature(),
        }).await?;

        self.bus.write::<FifoConfig2>(&FifoConfig2Fields {
            fifo_subsampling: cfg.subsampling() & 0b0000_0111,
            data_select: if cfg.apply_iir_filter() { FifoDataSource::Filtered } else { FifoDataSource::Unfiltered },
        }).await?;

        Ok(())
    }

    /// Returns the number of bytes in the FIFO buffer.
    ///
    /// Note that this does not return the number of data frames in the buffer.
    /// The number of frames in the FIFO depends on the size of each frame, which can vary depending on your configuration.
    pub async fn fifo_length(&mut self) -> Bmp390Result<u16, B::Error> {
        Ok(self.bus.read::<FifoLength>().await?)
    }

    /// Pops a frame from the top of the FIFO.
    ///
    /// Note that the FIFO needs to be enabled in order for frames to be written to it, but it is still possible to read out frames even when the FIFO is disabled.
    /// Enabling the FIFO is done via [`Bmp390::set_fifo_configuration`].
    /// [`Bmp390::fifo_length`] can be used to determine the current size (in bytes, not frames) of the FIFO buffer.
    /// If there are no frames ([`Bmp390::fifo_length`] == 0) ([`FifoFrame::SensorFrame`] with a value of [`SensorFrameType::Empty`] is returned.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # tokio_test::block_on(async {
    /// use bmp390_rs::fifo::{FifoFrame, ControlFrameType, SensorFrameType};
    /// # let mut device = bmp390_rs::testing::dummy_device().await;
    /// let frame = device.read_fifo_frame().await.unwrap();
    /// match frame {
    ///     FifoFrame::SensorFrame(frame) => {
    ///         match frame {
    ///             SensorFrameType::SensorTime(t) =>
    ///                 { println!("Read sensor time {}", t);}
    ///             SensorFrameType::Pressure(p) =>
    ///                 { println!("Read pressure {}", p);}
    ///             SensorFrameType::Temperature(t) =>
    ///                 { println!("Read temperature {}", t); }
    ///             SensorFrameType::PressureAndTemperature{ pressure, temperature } =>
    ///                 { println!("Read pressure {} AND temperature {}", pressure, temperature) }
    ///             SensorFrameType::Empty =>
    ///                 { println!("Read an empty frame"); }
    ///         }
    ///     }
    ///     FifoFrame::ControlFrame(ctrl_frame) => {
    ///         match ctrl_frame {
    ///             ControlFrameType::ConfigError =>
    ///                 { println!("Read control frame: Config error"); }
    ///             ControlFrameType::ConfigChange =>
    ///                 { println!("Read control frame: Config change"); }
    ///         }
    ///     }
    /// }
    /// # });
    pub async fn read_fifo_frame(&mut self) -> Bmp390Result<FifoFrame, B::Error> {
        const SENSOR_SMALL_FRAME_SIZE: usize = 4;
        const SENSOR_LARGE_FRAME_SIZE: usize = 7;
        const SENSOR_EMPTY_FRAME_SIZE: usize = 2;
        const CONTROL_FRAME_SIZE: usize = 2;

        let header = FifoHeader::from(self.bus.read::<FifoData<1>>().await?[0]);

        if header.is_control_frame() {
            // Ideally, we would burst read FIFO_DATA and *not* end the transaction, that way we could
            // read the header, figure out the frame size, and just continue reading in the same burst.
            // But since the SPI/I2C traits in embedded_hal_async do not include the possibility to
            // begin a transaction and allow arbitrary read/writes before ending the transaction
            // (i.e in the case if SPI, assert the CS pin and leave it asserted)
            // We have to ask for the entire frame, which of course redundantly gives us the header again. Like so:
            let _ = self.bus.read::<FifoData<CONTROL_FRAME_SIZE>>().await?;
            if header.config_error_flag() {
                Ok(FifoFrame::ControlFrame(ControlFrameType::ConfigError))
            } else if header.config_change_flag() {
                Ok(FifoFrame::ControlFrame(ControlFrameType::ConfigChange))
            } else {
                Err(Bmp390Error::FormatError)
            }
        } else if header.is_sensor_frame() {
            if header.sensor_time_flag() {
                let frame = self.bus.read::<FifoData<SENSOR_SMALL_FRAME_SIZE>>().await?;

                let value = u32::from_le_bytes([frame[1], frame[2], frame[3], 0]);

                Ok(FifoFrame::SensorFrame(SensorFrameType::SensorTime(value)))
            } else {
                if header.temperature_flag() && header.pressure_flag() {
                    let frame = self.bus.read::<FifoData<SENSOR_LARGE_FRAME_SIZE>>().await?;
                    let temperature = u32::from_le_bytes([frame[1], frame[2], frame[3], 0]);
                    let pressure = u32::from_le_bytes([frame[4], frame[5], frame[6], 0]);

                    let temperature = self.calibration_data.compensate_temperature(temperature);
                    let pressure = self.calibration_data.compensate_pressure(pressure);

                    Ok(FifoFrame::SensorFrame(
                        SensorFrameType::PressureAndTemperature {
                            pressure,
                            temperature,
                        },
                    ))
                } else if header.temperature_flag() {
                    let frame = self.bus.read::<FifoData<SENSOR_SMALL_FRAME_SIZE>>().await?;

                    let temperature = u32::from_le_bytes([frame[1], frame[2], frame[3], 0]);

                    Ok(FifoFrame::SensorFrame(SensorFrameType::Temperature(
                        self.calibration_data.compensate_temperature(temperature),
                    )))
                } else if header.pressure_flag() {
                    let frame = self.bus.read::<FifoData<SENSOR_SMALL_FRAME_SIZE>>().await?;

                    let pressure = u32::from_le_bytes([frame[1], frame[2], frame[3], 0]);

                    Ok(FifoFrame::SensorFrame(SensorFrameType::Pressure(
                        self.calibration_data.compensate_pressure(pressure),
                    )))
                } else {
                    // Read and discard frame.
                    let _ = self.bus.read::<FifoData<SENSOR_EMPTY_FRAME_SIZE>>().await?;

                    Ok(FifoFrame::SensorFrame(SensorFrameType::Empty))
                }
            }
        } else {
            Err(Bmp390Error::FormatError)
        }
    }

    /// Sets the power mode of the device by writing to the PwrCtrl (0x1B) register
    ///
    /// As described in section 3.3.4 of the datasheet, these are the valid state transitions:
    ///
    /// Sleep => Normal
    ///
    /// Normal => Sleep
    ///
    /// Sleep => Forced => Sleep (Forced is a transient state and the device will return to Sleep when the measurement is finished)
    ///
    /// The device ignores any attempt to perform an invalid state transition.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # tokio_test::block_on(async {
    /// use bmp390_rs::register::pwr_ctrl::PowerMode;
    /// # let mut device = bmp390_rs::testing::dummy_device().await;
    /// device.set_mode(PowerMode::Normal).await;
    /// # });
    pub async fn set_mode(&mut self, mode: pwr_ctrl::PowerMode) -> Bmp390Result<(), B::Error> {
        let mut pwr_ctrl = self.bus.read::<pwr_ctrl::PwrCtrl>().await?;
        pwr_ctrl.mode = mode;
        self.bus.write::<pwr_ctrl::PwrCtrl>(&pwr_ctrl).await?;
        Ok(())
    }
    /// Reads the current power mode from the PwrCtrl (0x1B) register
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # tokio_test::block_on(async {
    /// # let mut device = bmp390_rs::testing::dummy_device().await;
    /// device.mode().await;
    /// # });
    pub async fn mode(&mut self) -> Bmp390Result<pwr_ctrl::PowerMode, B::Error> {
        Ok(self.bus.read::<pwr_ctrl::PwrCtrl>().await?.mode)
    }

    /// Reads the **calibrated** pressure and temperature from the data (0x04 - 0x09) registers.
    /// This method will read data from these registers and calibrate them using the NVM-stored calibration coefficients
    /// before returning them to the caller.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # tokio_test::block_on(async {
    /// # let mut device = bmp390_rs::testing::dummy_device().await;
    /// let data = device.read_sensor_data().await.unwrap();
    /// println!("The current pressure and temperature is {} and {}", data.pressure, data.temperature);
    /// # });
    pub async fn read_sensor_data(&mut self) -> Bmp390Result<Measurement, B::Error> {
        let sample = self.bus.read::<data::Data>().await?;

        let compensated_temperature = self
            .calibration_data
            .compensate_temperature(sample.temperature);
        let compensated_pressure = self.calibration_data.compensate_pressure(sample.pressure);

        Ok(Measurement {
            pressure: compensated_pressure,
            temperature: compensated_temperature,
        })
    }
}

/// Holds calibrated pressure and temperature samples.
pub struct Measurement {
    pub pressure: f32,
    pub temperature: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::register::{calibration::Calibration, chip_id::ChipId};
    use crate::testing::{FakeBus, FakeDelay};
    #[tokio::test]
    async fn bmp390_read_empty_fifo_frame() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_any_response::<Calibration>();
        bus.with_response::<FifoData<1>>(&[0b1000_0000]);
        bus.with_response::<FifoData<2>>(&[0b1000_0000, 0]);

        let mut device = Bmp390::new(bus, Configuration::default(), &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(FifoFrame::SensorFrame(SensorFrameType::Empty), frame);
    }

    #[tokio::test]
    async fn bmp390_read_sensor_time_fifo_frame() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_any_response::<Calibration>();
        bus.with_response::<FifoData<1>>(&[0b1010_0000]);
        bus.with_response::<FifoData<4>>(&[0b1010_0000, 0x78, 0x56, 0x34]);

        let mut device = Bmp390::new(bus, Configuration::default(), &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(
            FifoFrame::SensorFrame(SensorFrameType::SensorTime(0x345678)),
            frame
        );
    }

    #[tokio::test]
    async fn bmp390_read_temp_fifo_frame() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_response::<Calibration>(&[1u8; 21]);
        bus.with_response::<FifoData<1>>(&[0b1001_0000]);
        bus.with_response::<FifoData<4>>(&[0b1001_0000, 0x78, 0x56, 0x34]);

        let mut device = Bmp390::new(bus, Configuration::default(), &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(
            FifoFrame::SensorFrame(SensorFrameType::Temperature(0.8454342)),
            frame
        );
    }

    #[tokio::test]
    async fn bmp390_read_pressure_fifo_frame() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_response::<Calibration>(&[1u8; 21]);
        bus.with_response::<FifoData<1>>(&[0b1000_0100]);
        bus.with_response::<FifoData<4>>(&[0b1000_0100, 0x78, 0x56, 0x34]);

        let mut device = Bmp390::new(bus, Configuration::default(), &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(
            FifoFrame::SensorFrame(SensorFrameType::Pressure(-50685.363)),
            frame
        );
    }

    #[tokio::test]
    async fn bmp390_read_pressure_and_temp_fifo_frame() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_response::<Calibration>(&[1u8; 21]);
        bus.with_response::<FifoData<1>>(&[0b1001_0100]);
        bus.with_response::<FifoData<7>>(&[0b1000_0100, 0x98, 0x76, 0x54, 0x32, 0x21, 0x10]);

        let mut device = Bmp390::new(bus, Configuration::default(), &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(
            FifoFrame::SensorFrame(SensorFrameType::PressureAndTemperature {
                pressure: -14239.893,
                temperature: 1.4154308
            }),
            frame
        );
    }

    #[tokio::test]
    async fn bmp390_read_control_frame_config_error() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_any_response::<Calibration>();
        bus.with_response::<FifoData<1>>(&[0b0100_0100]);
        bus.with_response::<FifoData<2>>(&[0b0100_0100, 0xAB]);

        let mut device = Bmp390::new(bus, Configuration::default(), &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(FifoFrame::ControlFrame(ControlFrameType::ConfigError), frame);
    }

    #[tokio::test]
    async fn bmp390_read_control_frame_config_change() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_any_response::<Calibration>();
        bus.with_response::<FifoData<1>>(&[0b0100_1000]);
        bus.with_response::<FifoData<2>>(&[0b0100_1000, 0xCD]);

        let mut device = Bmp390::new(bus, Configuration::default(), &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(FifoFrame::ControlFrame(ControlFrameType::ConfigChange), frame);
    }
}
