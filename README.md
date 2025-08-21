# Rust BMP390 driver

**This driver is still under development and is far from feature-complete. SPI mode is not yet working, and FIFO functionality is not implemented.**

This crate provides a platform-agnostic async driver for the Bosch BMP390 pressure (and temperature!) sensor using the [`embedded-hal-async`](https://github.com/rust-embedded/embedded-hal) traits.

The BMP390 driver supports both I2C and SPI communication, and this driver does as well.

# Device
The Bosch BMP390 is a pressure sensor with an accuracy of ±3 Pascal, equivalent to 0.25m altitude.

#### [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf)

# Usage

Add this crate as a dependency in your `Cargo.toml`:

```toml
[dependencies]
bmp390-rs = "0.1"
```

The driver is initialized by creating an instance of the `Bmp390` struct. Use `Bmp390::new_i2c` to create a driver that uses I2C, or `Bmp390::new_spi` for SPI.

When creating the driver, you need to pass an `Configuration` instance that specifies how the data should be sampled. If you don't care and want sane defaults, you can always use
`Configuration::default()` OR `Configuration::from_preset(..)` for a configuration that matches the presets that Bosch includes in the datasheet.
# Examples
This example shows how to continuously read pressure and temperature data from the driver.
```rust
use bmp390_rs::Bmp390;
use bmp390_rs::config::Configuration;

async fn main() {

    let peripherals = get_peripherals();

    let i2c = init_i2c(); // Initialize async I2C

    let mut bmp = Bmp390::new_i2c(
        i2c,
        0x77,
        Configuration::default(),
        &mut Delay{})
        .await
        .unwrap();

    loop {
        let reading = bmp.read_sensor_data()
            .await
            .unwrap();
        println!("Pressure: {:?} temp: {:?}", reading.pressure, reading.temperature);
    }
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
