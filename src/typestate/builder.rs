use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use crate::{Bmp390, Bmp390Result, SdoPinState};
use crate::bus::{Bus, I2c, Spi};
use crate::config::Configuration;
use crate::register::pwr_ctrl::PowerMode;
use crate::typestate::{Bmp390Mode, Forced, NoBus, NoMode, NoOutput, NoPin, Normal, OutputConfig, Pressure, PressureAndTemperature, Temperature};

/// The main entry-point into the typestate API. The [`Bmp390Builder`] lets you configure and build instances of [`Bmp390Mode`]
pub struct Bmp390Builder<
    Mode = NoMode,
    Out = NoOutput,
    B = NoBus,
    IntPin = NoPin,
    const USE_FIFO: bool = false,
> {
    bus: Option<B>,
    config: Configuration,
    int_pin: Option<IntPin>,
    _mode: PhantomData<Mode>,
    _out: PhantomData<Out>,
}

/// Methods available on Bmp390Builder when no bus or output has been configured yet.
impl Bmp390Builder<NoMode, NoOutput, NoBus, NoPin> {
    pub fn new() -> Self {
        Self {
            bus: None,
            config: Configuration::default()
                .enable_pressure_measurement(false)
                .enable_temperature_measurement(false),
            int_pin: None,
            _out: PhantomData,
            _mode: PhantomData,
        }
    }
}


/// Methods available on the Bmp390Builder when FIFO has not been enabled
impl<Mode, Out, B, IntPin> Bmp390Builder<Mode, Out, B, IntPin, false> {
    pub fn use_fifo(self) -> Bmp390Builder<Mode, Out, B, IntPin, true> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config,
            int_pin: self.int_pin,
            _mode: self._mode,
            _out: self._out,
        }
    }
}

/// Methods available on Bmp390Builder when no bus (SPI/I2C) has been configured yet.
impl<Mode, Out, IntPin, const USE_FIFO: bool> Bmp390Builder<Mode, Out, NoBus, IntPin, USE_FIFO> {
    /// Configures the underlying [`Bmp390`] driver to use I2C
    ///
    /// The `i2c` parameter should be an object that implements the [`embedded_hal_async::i2c::I2c`] trait.
    ///
    /// Since the I2C address on the Bmp390 device is derived from the state of the SDO pin, you need to tell the driver
    /// if it is connected to GND or VDDIO using the parameter [`sdo_pin_state`].
    pub fn use_i2c<I2cType>(
        self,
        i2c: I2cType,
        sdo_pin_state: SdoPinState,
    ) -> Bmp390Builder<Mode, Out, I2c<I2cType>, IntPin, USE_FIFO>
    where
        I2cType: embedded_hal_async::i2c::I2c,
    {
        Bmp390Builder {
            bus: Some(I2c::new(i2c, sdo_pin_state.into())),
            config: self.config,
            int_pin: self.int_pin,
            _out: self._out,
            _mode: self._mode,
        }
    }

    /// Configures the underlying [`Bmp390`] driver to use SPI
    ///
    /// The `spi` parameter should be an object that implements the [`embedded_hal_async::spi::SpiDevice`] trait.
    pub fn use_spi<SpiType>(
        self,
        spi: SpiType,
    ) -> Bmp390Builder<Mode, Out, Spi<SpiType>, IntPin, USE_FIFO>
    where
        SpiType: embedded_hal_async::spi::SpiDevice,
    {
        Bmp390Builder {
            bus: Some(Spi::new(spi)),
            config: self.config,
            int_pin: self.int_pin,
            _out: self._out,
            _mode: self._mode,
        }
    }
}

/// Methods available on Bmp390Builder when no interrupt pin has been configured yet.
impl<Mode, Out, B, const USE_FIFO: bool> Bmp390Builder<Mode, Out, B, NoPin, USE_FIFO> {
    /// Configures the [`Bmp390Mode`] instance to use interrupts whenever possible using the given interrupt pin.
    ///
    /// [`pin`] must implement both the [`embedded_hal_async::digital::Wait`] trait and the [`embedded_hal::digital::InputPin`] trait
    pub fn use_irq<IntPin: Wait + InputPin>(
        self,
        pin: IntPin,
    ) -> Bmp390Builder<Mode, Out, B, IntPin, USE_FIFO> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config,
            int_pin: Some(pin),
            _out: PhantomData,
            _mode: PhantomData,
        }
    }
}

/// Methods available on Bmp390Builder when no mode has been configured yet.
impl<Out, B, IntPin, const USE_FIFO: bool> Bmp390Builder<NoMode, Out, B, IntPin, USE_FIFO> {
    pub fn into_normal(self) -> Bmp390Builder<Normal, Out, B, IntPin, USE_FIFO> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config,
            int_pin: self.int_pin,
            _mode: PhantomData,
            _out: self._out,
        }
    }

    pub fn into_forced(self) -> Bmp390Builder<Forced, Out, B, IntPin, USE_FIFO> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config,
            int_pin: self.int_pin,
            _mode: PhantomData,
            _out: self._out,
        }
    }
}

/// Methods available on Bmp390Builder when no output has been configured yet.
impl<Mode, B, IntPin, const USE_FIFO: bool> Bmp390Builder<Mode, NoOutput, B, IntPin, USE_FIFO> {
    pub fn enable_pressure(self) -> Bmp390Builder<Mode, Pressure, B, IntPin, USE_FIFO> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_pressure_measurement(true),
            int_pin: self.int_pin,
            _out: PhantomData,
            _mode: self._mode,
        }
    }

    pub fn enable_temperature(self) -> Bmp390Builder<Mode, Temperature, B, IntPin, USE_FIFO> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_temperature_measurement(true),
            int_pin: self.int_pin,
            _out: PhantomData,
            _mode: self._mode,
        }
    }
}

/// Methods available on Bmp390Builder when pressure output has been configured.
impl<Mode, B, IntPin, const USE_FIFO: bool> Bmp390Builder<Mode, Pressure, B, IntPin, USE_FIFO> {
    pub fn enable_temperature(
        self,
    ) -> Bmp390Builder<Mode, PressureAndTemperature, B, IntPin, USE_FIFO> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_temperature_measurement(true),
            int_pin: self.int_pin,
            _out: PhantomData,
            _mode: self._mode,
        }
    }
}

/// Methods available on Bmp390Builder when temperature output has been configured.
impl<Mode, B, IntPin, const USE_FIFO: bool> Bmp390Builder<Mode, Temperature, B, IntPin, USE_FIFO> {
    pub fn enable_pressure(
        self,
    ) -> Bmp390Builder<Mode, PressureAndTemperature, B, IntPin, USE_FIFO> {
        Bmp390Builder {
            bus: self.bus,
            config: self.config.enable_pressure_measurement(true),
            int_pin: self.int_pin,
            _out: PhantomData,
            _mode: self._mode,
        }
    }
}

/// Methods available on Bmp390Builder only when a bus has
/// been configured with [`Bmp390Builder::use_spi`] or [`Bmp390Builder::use_i2c`] AND [`Bmp390Builder::into_normal`]
impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, const USE_FIFO: bool>
Bmp390Builder<Normal, Out, B, IntPin, USE_FIFO>
{
    pub async fn build<D: DelayNs>(
        self,
        mut delay: D,
    ) -> Bmp390Result<Bmp390Mode<Normal, Out, B, IntPin, D, USE_FIFO>, B::Error> {
        let bus = self.bus.unwrap();
        let config = Configuration::default().power_mode(PowerMode::Normal);
        let device = Bmp390::new(bus, config, &mut delay).await?;

        Ok(Bmp390Mode::<Normal, _, _, _, _, USE_FIFO>::new(device, self.int_pin, delay).await?)
    }
}

/// Methods available on Bmp390Builder only when a bus has
/// been configured with [`Bmp390Builder::use_spi`] or [`Bmp390Builder::use_i2c`] AND [`Bmp390Builder::into_forced`]
impl<Out: OutputConfig, B: Bus, IntPin: Wait + InputPin, const USE_FIFO: bool>
Bmp390Builder<Forced, Out, B, IntPin, USE_FIFO>
{
    pub async fn build<D: DelayNs>(
        self,
        mut delay: D,
    ) -> Bmp390Result<Bmp390Mode<Forced, Out, B, IntPin, D, USE_FIFO>, B::Error> {
        let bus = self.bus.unwrap();
        let config = Configuration::default().power_mode(PowerMode::Sleep);
        let device = Bmp390::new(bus, config, &mut delay).await?;

        Ok(Bmp390Mode::<Forced, _, _, _, _, USE_FIFO>::new(device, self.int_pin, delay).await?)
    }
}
