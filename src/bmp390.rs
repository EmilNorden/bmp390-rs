use crate::bus::{Bus, I2c, Spi};
use crate::calibration::CalibrationData;
use crate::config::Configuration;
use crate::error::Bmp390Error;
use crate::fifo::{
    ControlFrameType, FifoConfiguration, FifoFrame, FifoFullBehavior, FifoHeader, SensorFrameType,
};
use crate::register;
use crate::register::fifo_config::{
    FifoConfig1, FifoConfig1Fields, FifoConfig2, FifoConfig2Fields, FifoDataSource,
};
use crate::register::fifo_data::FifoData;
use crate::register::fifo_length::FifoLength;
use crate::register::int_ctrl::IntCtrl;
use crate::register::osr::{Osr, OsrCfg, Oversampling};
use crate::register::{
    chip_id, cmd, data, err_reg, int_status, odr, osr, pwr_ctrl, status, Readable, Writable,
};
use embedded_hal::i2c::SevenBitAddress;
use embedded_hal_async::delay::DelayNs;

/// Type alias for a Bmp390 chip communicating over I2C
type Bmp390I2c<T> = Bmp390<I2c<T>>;

/// Type alias for a Bmp390 chip communicating over SPI
type Bmp390Spi<T> = Bmp390<Spi<T>>;

const BMP390_CHIP_ID: u8 = 0x60;

/// Main Bmp390 driver struct
pub struct Bmp390<B> {
    bus: B,
    calibration_data: CalibrationData,
    max_measurement_time_us: u32,
}

/// Type alias used to simplify return types throughout the driver
pub type Bmp390Result<T, BusError> = Result<T, Bmp390Error<BusError>>;

impl<T> Bmp390I2c<T>
where
    T: embedded_hal_async::i2c::I2c,
    I2c<T>: Bus,
{
    /// Constructs a new Bmp390 driver instance with a given configuration that communicates over I2C
    ///
    /// This function will:
    /// - Probe for a connected BMP390 device.
    /// - Perform a soft reset if `reset` == [`ResetPolicy::Soft`]
    /// - Apply the given configuration
    /// - Load calibration coefficients from NVM
    ///
    /// # Examples
    ///
    /// ```rust
    /// # use embedded_hal_async::delay::DelayNs;
    /// # use embedded_hal_async::i2c::I2c;
    /// # use bmp390_rs::Bmp390Result;
    ///  use bmp390_rs::{Bmp390, SdoPinState, ResetPolicy};
    ///  use bmp390_rs::config::Configuration;
    /// # async fn demo<I: I2c, D: DelayNs>(i2c: I, mut delay: D) -> Bmp390Result<(), I::Error> {
    ///
    ///  let device = Bmp390::new_i2c(
    ///     i2c,
    ///     SdoPinState::High,
    ///     Configuration::default(),
    ///     ResetPolicy::Soft,
    ///     &mut delay
    ///  ).await?;
    /// # Ok(())
    /// # }
    pub async fn new_i2c<D: DelayNs>(
        i2c: T,
        sdo_pin_state: SdoPinState,
        config: Configuration,
        reset: ResetPolicy,
        delay: &mut D,
    ) -> Bmp390Result<Self, <I2c<T> as Bus>::Error> {
        Self::new(I2c::new(i2c, sdo_pin_state.into()), config, reset, delay).await
    }
}

impl<T> Bmp390Spi<T>
where
    T: embedded_hal_async::spi::SpiDevice,
    Spi<T>: Bus,
{
    /// Constructs a new Bmp390 driver instance with a given configuration that communicates over SPI
    ///
    /// This function will:
    /// - Probe for a connected BMP390 device.
    /// - Perform a soft reset if `reset` == [`ResetPolicy::Soft`]
    /// - Apply the given configuration
    /// - Load calibration coefficients from NVM
    ///
    /// # Examples
    ///
    /// ```rust
    /// # use embedded_hal_async::delay::DelayNs;
    /// # use embedded_hal_async::spi::SpiDevice;
    /// # use bmp390_rs::Bmp390Result;
    ///  use bmp390_rs::{Bmp390, ResetPolicy};
    ///  use bmp390_rs::config::Configuration;
    /// # async fn demo<S: SpiDevice, D: DelayNs>(spi: S, mut delay: D) -> Bmp390Result<(), S::Error> {
    ///
    ///  let device = Bmp390::new_spi(
    ///     spi,
    ///     Configuration::default(),
    ///     ResetPolicy::Soft,
    ///     &mut delay
    ///  ).await?;
    /// # Ok(())
    /// # }
    pub async fn new_spi<D: DelayNs>(
        spi: T,
        config: Configuration,
        reset: ResetPolicy,
        delay: &mut D,
    ) -> Bmp390Result<Self, <Spi<T> as Bus>::Error> {
        Self::new(Spi::new(spi), config, reset, delay).await
    }
}

impl<B> Bmp390<B>
where
    B: Bus,
{
    /// Probes if the device is ready by attempting to read ChipId [`attempts`] times with a 1 ms delay.
    ///
    /// Returns [`Bmp390Error::NotConnected`] if no response is received.
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

    /// Creates a new instance of the Bmp390 driver struct with the given configuration.
    pub(crate) async fn new<D: DelayNs>(
        mut bus: B,
        config: Configuration,
        reset: ResetPolicy,
        delay: &mut D,
    ) -> Bmp390Result<Self, B::Error> {
        // The datasheet (Section 1, table 2) specifies 2ms start-up time after VDD/VDDIO > 1.8V
        Self::probe_ready(&mut bus, delay, 5).await?;

        let calibration_data = CalibrationData::new(&mut bus).await?;

        let mut device = Bmp390 {
            bus,
            calibration_data,
            max_measurement_time_us: calculate_maximum_measurement_time(
                config.enable_pressure,
                config.enable_temperature,
                config.pressure_oversampling,
                config.temperature_oversampling,
            ),
        };

        if reset == ResetPolicy::Soft {
            device.soft_reset().await?;
        }

        device.apply_configuration(&config).await?;

        Ok(device)
    }

    /// Applies the given configurations by writing to their corresponding registers.
    pub async fn apply_configuration(&mut self, config: &Configuration) -> Bmp390Result<(), B::Error> {
        self.bus.write::<pwr_ctrl::PwrCtrl>(&pwr_ctrl::PwrCtrlCfg {
            press_en: config.enable_pressure,
            temp_en: config.enable_temperature,
            mode: config.mode,
        })
            .await?;

        self.bus.write::<osr::Osr>(&osr::OsrCfg {
            osr_p: config.pressure_oversampling,
            osr_t: config.temperature_oversampling,
        })
            .await?;

        self.bus.write::<odr::Odr>(&odr::OdrCfg {
            odr_sel: config.output_data_rate,
        })
            .await?;

        self.bus.write::<register::config::Config>(&register::config::ConfigFields {
            iir_filter: config.iir_filter_coefficient,
        })
            .await?;

        Ok(())
    }

    /// Read a register (or fixed-size register block) using a **typed marker**.
    ///
    /// This is the low-level, register-accurate entry point. You pass a marker type
    /// from [`crate::register`] (e.g. `register::pwr_ctrl::PwrCtrl`), and you get back its
    /// decoded value (`R::Out`).
    ///
    /// - The bus transfer length and address come from `R::N` and `R::ADDR`.
    /// - Bytes are decoded by `R::decode(...)`, which may return
    ///   [`InvalidRegisterField`] if reserved/invalid bit patterns are observed.
    /// - Some registers have **clear-on-read** semantics (e.g. `INT_STATUS`,
    ///   data-ready flags). Consult the register’s docs.
    ///
    /// For most users, the convenience methods (e.g. [`error_flags`](Self::error_flags))
    /// are easier to discover and have concrete return types. This generic is here
    /// when you want full control.
    ///
    /// # Examples
    /// Read CHIP_ID (0x00):
    /// ```rust,no_run
    /// # use bmp390_rs::{register, Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// let id: u8 = device.read::<register::chip_id::ChipId>().await?;
    /// assert_eq!(id, 0x60);
    /// # Ok(()) }
    /// ```
    ///
    /// Read PWR_CTRL (0x1B) as a typed struct:
    /// ```rust,no_run
    /// # use bmp390_rs::{register, Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// use bmp390_rs::register::pwr_ctrl::{PwrCtrl, PwrCtrlCfg};
    /// let cfg: PwrCtrlCfg = device.read::<PwrCtrl>().await?;
    /// # Ok(()) }
    /// ```
    ///
    /// Read the pressure+temperature data block (0x04..0x09):
    /// ```rust,no_run
    /// # use bmp390_rs::{register, Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// let pt = device.read::<register::data::Data>().await?;
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`crate::register`] — register catalog (addresses, lengths, value types)
    /// - Convenience helpers like [`error_flags`](Self::error_flags)
    pub async fn read<R: Readable>(&mut self) -> Bmp390Result<R::Out, B::Error> {
        Ok(self.bus.read::<R>().await?)
    }

    /// Write a register (or fixed-size register block) using a **typed marker**.
    ///
    /// You pass a marker type from [`crate::register`] (e.g. `register::pwr_ctrl::PwrCtrl`) and
    /// a value of its input type (`W::In`). The value is encoded by `W::encode(...)`
    /// and written to `W::ADDR`.
    ///
    /// This performs a **direct write** of the provided fields. If you need to
    /// preserve unrelated bits (e.g. reserved fields), prefer a read-modify-write:
    /// read the struct, change the fields you care about, then write it back.
    ///
    /// **Side-effects:** some writes reset internal pipelines (e.g. IIR seed when
    /// changing filter/OSR), and `CMD` (0x7E) commands require polling `STATUS.cmd_rdy`.
    /// See the register docs for details.
    ///
    /// For most users, the convenience methods (e.g. [`set_mode`](Self::set_mode))
    /// are easier to discover and have concrete argument types. This generic is here
    /// when you want full control.
    ///
    /// # Examples
    /// Put the device in Sleep with both sensors enabled:
    /// ```rust,no_run
    /// # use bmp390_rs::{Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    /// # use bmp390_rs::error::Bmp390Error;
    ///
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// use crate::bmp390_rs::register::pwr_ctrl::{PwrCtrlCfg, PwrCtrl, PowerMode};
    /// let cfg = PwrCtrlCfg { press_en: true, temp_en: true, mode: PowerMode::Sleep };
    /// device.write::<PwrCtrl>(&cfg).await?;
    /// # Ok(()) }
    /// ```
    ///
    /// Issue a soft reset via CMD (0x7E):
    /// ```rust,no_run
    /// # use bmp390_rs::{Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    /// # use bmp390_rs::error::Bmp390Error;
    ///
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// use crate::bmp390_rs::register::cmd::{Cmd, CmdData::SoftReset};
    /// device.write::<Cmd>(&SoftReset).await?;
    /// // then poll `STATUS.cmd_rdy`
    /// # Ok(()) }
    /// ```
    ///
    /// # See also
    /// - [`crate::register`] — register catalog (addresses, lengths, value types)
    /// - Convenience helpers like [`set_mode`](Self::set_mode)
    pub async fn write<W: Writable>(&mut self, v: &W::In) -> Bmp390Result<(), B::Error> {
        Ok(self.bus.write::<W>(v).await?)
    }

    /// Determines if the BMP390 device is connected by attempting to read the [`ChipId`] (0x00) register.
    pub async fn is_connected(&mut self) -> Bmp390Result<bool, B::Error> {
        let id = self.bus.read::<chip_id::ChipId>().await?;

        Ok(id == BMP390_CHIP_ID)
    }

    /// Returns true if the command decoder is ready to accept a new command.
    ///
    /// This only applies to the `CMD` register commands [`FifoFlush`](cmd::CmdData::FifoFlush) and [`SoftReset`](cmd::CmdData::SoftReset)
    pub async fn command_ready(&mut self) -> Bmp390Result<bool, B::Error> {
        let status = self.read::<status::Status>().await?;

        Ok(status.cmd_rdy)
    }

    async fn wait_command_ready(&mut self, max_polls: u8) -> Bmp390Result<(), B::Error> {
        for _ in 0..max_polls {
            if self.command_ready().await? { return Ok(()) }
        }

        Err(Bmp390Error::Timeout)
    }

    /// Triggers a soft reset
    ///
    /// All user settings are reset to their default state.
    ///
    /// Before issuing a soft reset command, this method will wait for the command decoder to accept new commands. Likewise, after the soft reset the method will wait for the command decoder to be ready again.
    /// If these does not happen in a timely fashion, an error of type [`Bmp390Error::Timeout`] will be returned.
    ///
    /// **Note:** This resets the chip to factory defaults, not to the configuration that was provided when constructing the driver.
    pub async fn soft_reset(&mut self) -> Bmp390Result<(), B::Error> {
        // Is command decoder ready to accept a new command? Poll it max 32 times (non-scientifically chosen number)
        self.wait_command_ready(32).await?;

        // Issue soft reset command
        self.write::<cmd::Cmd>(&cmd::CmdData::SoftReset).await?;

        // Wait until command decoder is idle again
        self.wait_command_ready(32).await?;

        Ok(())
    }

    /// Returns the error flags from the ERR_REG (0x02) register.
    ///
    /// **Note:** These flags are cleared upon read:
    /// - cmd_err
    /// - conf_err
    pub async fn error_flags(&mut self) -> Bmp390Result<err_reg::ErrorFlags, B::Error> {
        Ok(self.bus.read::<err_reg::ErrReg>().await?)
    }

    /// Returns the status from the STATUS (0x03) register.
    pub async fn status(&mut self) -> Bmp390Result<status::StatusFlags, B::Error> {
        Ok(self.bus.read::<status::Status>().await?)
    }

    /// Returns the interrupt status from the INT_STATUS (0x11) register.
    ///
    /// Use this to determine what caused an interrupt to be generated.
    ///
    /// **Note:** The INT_STATUS register is cleared upon read.
    pub async fn interrupt_status(&mut self) -> Bmp390Result<int_status::IntStatusFlags, B::Error> {
        Ok(self.bus.read::<int_status::IntStatus>().await?)
    }

    /// Returns the oversampling configuration from the OSR (0x1C) register.
    pub async fn oversampling_config(&mut self) -> Bmp390Result<OsrCfg, B::Error> {
        Ok(self.bus.read::<Osr>().await?)
    }

    /// Writes oversampling configuration to the OSR (0x1C) register.
    pub async fn set_oversampling_config(
        &mut self,
        oversampling: &OsrCfg,
    ) -> Bmp390Result<(), B::Error> {
        Ok(self.bus.write::<Osr>(oversampling).await?)
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
            if config1.fifo_stop_on_full {
                FifoFullBehavior::Stop
            } else {
                FifoFullBehavior::OverwriteOldest
            },
            config2.fifo_subsampling,
            config2.data_select == FifoDataSource::Filtered,
        ))
    }

    /// Writes a new FIFO configuration to the device.
    ///
    /// This equates to updating registers FIFO_CONFIG_1 (0x17) and FIFO_CONFIG_2 (0x18)
    pub async fn set_fifo_configuration(
        &mut self,
        cfg: FifoConfiguration,
    ) -> Bmp390Result<(), B::Error> {
        self.bus
            .write::<FifoConfig1>(&FifoConfig1Fields {
                fifo_mode: cfg.fifo_enabled(),
                fifo_stop_on_full: cfg.fifo_full_behavior() == FifoFullBehavior::Stop,
                fifo_time_en: cfg.time_enabled(),
                fifo_press_en: cfg.pressure_enabled(),
                fifo_temp_en: cfg.temperature_enabled(),
            })
            .await?;

        self.bus
            .write::<FifoConfig2>(&FifoConfig2Fields {
                fifo_subsampling: cfg.subsampling() & 0b0000_0111,
                data_select: if cfg.apply_iir_filter() {
                    FifoDataSource::Filtered
                } else {
                    FifoDataSource::Unfiltered
                },
            })
            .await?;

        Ok(())
    }

    /// Returns the number of bytes in the FIFO buffer.
    ///
    /// Note that this does not return the number of data frames in the buffer.
    /// The number of frames in the FIFO depends on the size of each frame, which can vary depending on your configuration.
    pub async fn fifo_length(&mut self) -> Bmp390Result<u16, B::Error> {
        Ok(self.bus.read::<FifoLength>().await?)
    }

    /// Returns a frame from the FIFO
    ///
    /// Note that the FIFO needs to be enabled in order for frames to be written to it, but it is still possible to read out frames even when the FIFO is disabled.
    /// Enabling the FIFO is done via [`Bmp390::set_fifo_configuration`].
    /// [`Bmp390::fifo_length`] can be used to determine the current size (in bytes, not frames) of the FIFO buffer.
    /// If there are no frames ([`Bmp390::fifo_length`] == 0) one of two things will happen:
    /// - If sensor timestamps are enabled ([`FifoConfiguration::set_time_enabled`]) then a [`FifoFrame::SensorFrame`] with a value of [`SensorFrameType::SensorTime`] is returned.
    /// - If sensor timestamps are disabled, a [`FifoFrame::SensorFrame`] with a value of [`SensorFrameType::Empty`] is returned.
    ///
    /// This is important to note; If you want to correlate reading with the sensor timestamp, you need to completely drain the FIFO.
    /// Timestamps are not stored in the FIFO, they are only appended after the *last* frame.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # use bmp390_rs::{Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    ///
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// use bmp390_rs::fifo::{FifoFrame, ControlFrameType, SensorFrameType};
    ///
    /// let frame = device.read_fifo_frame().await?;
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
    /// # Ok(()) }
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
                Err(Bmp390Error::UnexpectedFifoData)
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
            Err(Bmp390Error::UnexpectedFifoData)
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
    /// # use bmp390_rs::{Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    ///
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// use bmp390_rs::register::pwr_ctrl::PowerMode;
    ///
    /// device.set_mode(PowerMode::Normal).await?;
    /// # Ok(()) };
    pub async fn set_mode(&mut self, mode: pwr_ctrl::PowerMode) -> Bmp390Result<(), B::Error> {
        let mut pwr_ctrl = self.bus.read::<pwr_ctrl::PwrCtrl>().await?;
        pwr_ctrl.mode = mode;
        self.bus.write::<pwr_ctrl::PwrCtrl>(&pwr_ctrl).await?;
        Ok(())
    }
    /// Reads the current power mode from the PwrCtrl (0x1B) register
    pub async fn mode(&mut self) -> Bmp390Result<pwr_ctrl::PowerMode, B::Error> {
        Ok(self.bus.read::<pwr_ctrl::PwrCtrl>().await?.mode)
    }

    /// Reads the latest **calibrated** pressure and temperature measurement stored in the Data (0x04 - 0x09) registers.
    ///
    /// This method will read data from these registers and calibrate them using the NVM-stored calibration coefficients.
    ///
    /// Ideally, you will want to call this method only when *new* measurements have been stored, otherwise you will be wasting time reading the same data multiple times.
    /// This can be achieved by using the interrupt pin on the BMP390 device and enabling the [`Interrupts::data_ready()`] interrupt using [`Bmp390::mask_interrupts()`].
    /// If for some reason you are unable to use interrupts, an option is to poll this method at a rate of [`Bmp390::max_measurement_time_us()`] microseconds.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # use bmp390_rs::{Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    ///
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// let data = device.read_sensor_data().await?;
    /// println!("The current pressure and temperature is {} and {}", data.pressure, data.temperature);
    /// # Ok(()) };
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

    /// Masks the given interrupts so they are not reflected on the INT pin
    ///
    /// The BMP390 exposes 3 interrupts:
    /// - Data ready (DRDY)
    /// - FIFO Full
    /// - FIFO Watermark
    ///
    /// By masking any of these interrups you can control when the INT pin is activated.
    ///
    /// # Examples
    /// ```rust, no_run
    /// # use bmp390_rs::{Bmp390, Bmp390Result};
    /// # use bmp390_rs::bus::Bus;
    ///
    /// # async fn demo<B: Bus>(mut device: Bmp390<B>) -> Bmp390Result<(), B::Error> {
    /// use bmp390_rs::Interrupts;
    /// // Allow all interrupts
    /// let interrupts = Interrupts::new().none();
    /// device.mask_interrupts(interrupts).await?;
    ///
    /// // Mask FIFO full and FIFO watermark, leaving only Data ready.
    /// let interrupts = Interrupts::new().fifo_full().fifo_watermark();
    /// device.mask_interrupts(interrupts).await?;
    /// # Ok(()) };
    pub async fn mask_interrupts(&mut self, ints: Interrupts) -> Bmp390Result<(), B::Error> {
        let mut int_ctrl = self.bus.read::<IntCtrl>().await?;

        int_ctrl.drdy_en = !ints.is_data_ready_set();
        int_ctrl.ffull_en = !ints.is_fifo_full_set();
        int_ctrl.fwtm_en = !ints.is_fifo_watermark_set();
        self.bus.write::<IntCtrl>(&int_ctrl).await?;

        Ok(())
    }

    /// Returns the estimated maximum measurement time in microseconds.
    ///
    /// The maximum measurement time is the period between measurements in the BMP390.
    /// It is described in detail in datasheet section 3.9.
    /// Section 3.9.2 of the datasheet holds the equation for the *typical* measurement time. The value returned from this
    /// method is produced by multiplying that value by 1.2.
    pub fn max_measurement_time_us(&self) -> u32 {
        self.max_measurement_time_us
    }
}

/// Calculates an estimation of the maximum measurement time
///
/// See section 3.9.2 "Measurement rate in forced mode and normal mode" of the datasheet for an explanation of the equation.
///
/// This equation calculates the *typical* measurement time, so I multiply the result by 1.2 to get within range of the maximum
/// time as denoted by table 23 in the datasheet.
fn calculate_maximum_measurement_time(
    pressure_enabled: bool,
    temperature_enabled: bool,
    pressure_oversampling: Oversampling,
    temperature_oversampling: Oversampling,
) -> u32 {
    let osr_p: u8 = pressure_oversampling.into();
    let osr_t: u8 = temperature_oversampling.into();
    let typical_time = 234u32
        + pressure_enabled as u32 * (392 + 2u32.pow(osr_p as u32) * 2020)
        + temperature_enabled as u32 * (163 + 2u32.pow(osr_t as u32) * 2020);

    (typical_time as f32 * 1.2) as u32
}

/// This enum should reflect the physical state of the SDO pin. This is used to determine the I2C address
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SdoPinState {
    /// SDO is pulled high by connection to VDDIO
    High,
    /// SDO is pulled low by connection to GND
    Low,
}

impl Into<SevenBitAddress> for SdoPinState {
    fn into(self) -> SevenBitAddress {
        match self {
            SdoPinState::High => 0x77,
            SdoPinState::Low => 0x76,
        }
    }
}

/// Represents the interrupts available on the BMP390 device.
pub struct Interrupts(u8);

impl Interrupts {
    const FIFO_FULL: u8 = 1 << 0;
    const FIFO_WATERMARK: u8 = 1 << 1;
    const DATA_READY: u8 = 1 << 2;

    /// Creates a new instance with no interrupts chosen.
    pub fn new() -> Self {
        Self(0)
    }

    /// Add the FIFO full interrupt to this set.
    ///
    /// The FIFO full interrupt is used to signal when the number of unread bytes in the FIFO is >= 504.
    /// See datasheet section 3.7.5.2 for more information.
    pub fn fifo_full(mut self) -> Self {
        self.0 |= Self::FIFO_FULL;

        self
    }

    /// Add the FIFO watermark interrupt to this set.
    ///
    /// The FIFO watermark interrupt is used to signal that the number of unread bytes in the FIFO has reached
    /// a pre-set limit. This limit is set by using [`Bmp390::set_fifo_configuration`].
    /// See datasheet section 3.7.5.1 for more information.
    pub fn fifo_watermark(mut self) -> Self {
        self.0 |= Self::FIFO_WATERMARK;

        self
    }

    /// Add the data ready (DRDY) interrupt to this set.
    ///
    /// DRDY is used to signal that a pressure/temperature measurement has ended and results have been stored in data registers and FIFO.
    /// See datasheet section 3.7.5.3 for more information.
    pub fn data_ready(mut self) -> Self {
        self.0 |= Self::DATA_READY;

        self
    }

    /// Remove all interrupts from the set.
    pub fn none(mut self) -> Self {
        self.0 = 0;

        self
    }

    pub(crate) fn is_data_ready_set(&self) -> bool {
        self.0 & Self::DATA_READY != 0
    }

    pub(crate) fn is_fifo_watermark_set(&self) -> bool {
        self.0 & Self::FIFO_WATERMARK != 0
    }

    pub(crate) fn is_fifo_full_set(&self) -> bool {
        self.0 & Self::FIFO_FULL != 0
    }
}

/// Holds calibrated pressure and temperature samples.
#[derive(Copy, Clone, Debug)]
pub struct Measurement {
    pub pressure: f32,
    pub temperature: f32,
}

/// What to do at startup before applying [`Configuration`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ResetPolicy {
    /// Issue CMD=0xB6 and wait for `STATUS.cmd_rdy` (recommended default).
    Soft,
    /// Don’t reset; leave the chip as-is (faster resume, preserves FIFO/IIR history).
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::register::data::Data;
    use crate::register::{calibration::Calibration, chip_id::ChipId};
    use crate::testing::{FakeBus, FakeDelay};

    #[tokio::test]
    async fn bmp390_read_sensor_data() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_response::<Calibration>(&[
            0x98, 0x6E, 0x13, 0x4D, 0xF9, 0xB0, 0x1B, 0xC0, 0x15, 0x06, 0x01, 0x92, 0x4A, 0xAE,
            0x5D, 0x03, 0xFA, 0x08, 0x0F, 0x06, 0xF5,
        ]);
        bus.with_response::<Data>(&[0x92, 0x51, 0x65, 0x79, 0xCE, 0x83]);

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None, &mut FakeDelay {})
            .await
            .unwrap();

        let measurement = device.read_sensor_data().await.unwrap();
        assert_eq!(100548.42, measurement.pressure);
        assert_eq!(25.498167, measurement.temperature);
    }

    #[tokio::test]
    async fn bmp390_read_empty_fifo_frame() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_any_response::<Calibration>();
        bus.with_response::<FifoData<1>>(&[0b1000_0000]);
        bus.with_response::<FifoData<2>>(&[0b1000_0000, 0]);

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None,&mut FakeDelay {})
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

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None,&mut FakeDelay {})
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

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None, &mut FakeDelay {})
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

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None, &mut FakeDelay {})
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

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None, &mut FakeDelay {})
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

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None, &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(
            FifoFrame::ControlFrame(ControlFrameType::ConfigError),
            frame
        );
    }

    #[tokio::test]
    async fn bmp390_read_control_frame_config_change() {
        let mut bus: FakeBus<10> = FakeBus::new();
        bus.with_response::<ChipId>(&[96]);
        bus.with_any_response::<Calibration>();
        bus.with_response::<FifoData<1>>(&[0b0100_1000]);
        bus.with_response::<FifoData<2>>(&[0b0100_1000, 0xCD]);

        let mut device = Bmp390::new(bus, Configuration::default(), ResetPolicy::None, &mut FakeDelay {})
            .await
            .unwrap();

        let frame = device.read_fifo_frame().await.unwrap();
        assert_eq!(
            FifoFrame::ControlFrame(ControlFrameType::ConfigChange),
            frame
        );
    }

    #[test]
    fn calculate_maximum_measurement_time() {
        // Test against the specified times in datasheet table 23.
        let time = super::calculate_maximum_measurement_time(
            true,
            true,
            Oversampling::X1,
            Oversampling::X1,
        );

        assert!(time > 5700);

        let time = super::calculate_maximum_measurement_time(
            true,
            true,
            Oversampling::X2,
            Oversampling::X1,
        );

        assert!(time > 7960);

        let time = super::calculate_maximum_measurement_time(
            true,
            true,
            Oversampling::X4,
            Oversampling::X1,
        );

        assert!(time > 12480);

        let time = super::calculate_maximum_measurement_time(
            true,
            true,
            Oversampling::X8,
            Oversampling::X1,
        );

        assert!(time > 21530);

        let time = super::calculate_maximum_measurement_time(
            true,
            true,
            Oversampling::X16,
            Oversampling::X2,
        );

        assert!(time > 41890);

        let time = super::calculate_maximum_measurement_time(
            true,
            true,
            Oversampling::X32,
            Oversampling::X2,
        );

        assert!(time > 78090);
    }
}
