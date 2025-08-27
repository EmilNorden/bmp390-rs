# Rust BMP390 driver


This crate provides a platform-agnostic async driver for the Bosch BMP390 pressure (and temperature!) sensor using the [`embedded-hal-async`](https://github.com/rust-embedded/embedded-hal) traits.

The BMP390 device supports both I2C and SPI communication, and this driver does as well.

Feedback or contributions to this crate is highly welcome.

### Highlighted features

- `no_std` + `embedded-hal-async` support
- I²C and SPI support
- Ergonomic high-level API (no register twiddling needed, unless you want to)
- Strongly-typed register access via `read::<regs::...>()` / `write::<regs::...>()` for those who cant stay away.
- FIFO support.

### Roadmap
- Stabilize the public API (typestate vs. ergonomic helpers).
- Add blocking API (embedded-hal) alongside async.
- Increasing developer ergonomics.
- Documentation.

# Device
The Bosch BMP390 is a pressure sensor with an accuracy of ±3 Pascal, equivalent to 0.25m altitude.

#### [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf)

# Usage

Add this crate as a dependency in your `Cargo.toml`:

```toml
[dependencies]
bmp390-rs = "0.3"
```


## Pick your interface
This driver provides *two* different interfaces: The *core* interface and the *typestate* interface
### Core (register-level)
Direct, typed access to the chip’s registers—plus ergonomic helpers. Great when you want full control.

```rust
use bmp390_rs::{Bmp390, config::Configuration, register};

// Connect via SPI. Use Bmp390::new_i2c for I2C.
let mut device = Bmp390::new_spi(spi_device, Configuration::default(), &mut delay).await?

// Read pressure/temperature data
let data = device.read_sensor_data().await?;
info!("Measurement: {:?}", data);

// Read from registers
let osr = device.read::<register::osr::Osr>().await?;

// Write to registers
device.write::<register::odr::Odr>(&register::odr::OdrCfg {
    odr_sel: register::odr::OutputDataRate::R25Hz,
}).await?

```

### Typestate
Higher-level wrapper that encodes BMP390 modes in the type system.
```rust
use bmp390_rs::typestate::Bmp390Builder;

let mut normal_device = Bmp390Builder::new()
        .use_spi(device)        // Communicate over SPI
        .enable_temperature()   // Enable both temperature...
        .enable_pressure()      // ...and pressure
        .into_normal()          // Put the device into normal mode
        .build(delay).await?;

// Reads the next measurement. This will wait for interrupt (if configured) or delay for one measurement cycle.
let measurement = normal_device.read_next_measurement().await?;
info!("{:?}", measurement);

// Read the latest measurement. This does not wait, but will instead read whatever is in the Data register right now.
let measurement = normal_device.read_latest_measurement().await?;
info!("{:?}", measurement);

```
Here is an example of a Bmp390 device in *forced* mode and with the on-board FIFO enabled:
```rust
use bmp390_rs::typestate::{Bmp390Builder, FifoOutput};

/* Creates a forced mode Bmp390 that uses the FIFO. 
    This will give you a queue abstraction over the core driver. */
let mut device = Bmp390Builder::new()
        .use_irq(irq_pin)       // Use interrupts whenever possible to synchronize data output
        .use_spi(spi_device)    // Communicate over SPI
        .enable_pressure()      // We are only interested in pressure, so enable that
        .into_forced()          // Put the device into forced mode
        .use_fifo()             // And enable FIFO
        .build(delay).await?;

// Now you can ask the Bmp390 device to perform measurements and put them in the on-board FIFO..
device.enqueue_measurement().await.unwrap();

// And read from the FIFO at a later point:
match forced_device.dequeue().await? {
    FifoOutput::Measurement(x) => info!("Got measurement: {:?}", x),
    FifoOutput::SensorTime(x) => info!("Got sensor time {}", x),
    FifoOutput::Empty => {},
}
```

## License

This project is licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
