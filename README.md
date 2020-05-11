# DPS422 embedded-hal I2C driver crate

A platform agnostic driver to interface with the DPS422 barometric pressure & temp sensor.
This driver uses I2C via [embedded-hal](https://docs.rs/embedded-hal). Note that the DPS422 also supports SPI, however that is not (yet) implemented in this driver.

## Usage

Include this [crate](https://crates.io/crates/dps422) as a dependency in your Cargo.toml


```
[dependencies.dps422]
version = "<version>"
```

Use embedded-hal implementation to get I2C, then create a driver instance

```rust
use dps422::{DPS422, self};


let address = 0x76;
let mut dps = DPS422::new(i2c, address, &dps422::Config::new()).unwrap();

dps.trigger_measurement(true, true, false).unwrap();

if dps.data_ready().unwrap() {
    let pressure = dps.read_pressure_calibrated().unwrap();
    let temp = dps.read_temp_calibrated().unwrap();
    writeln!(usart, "pressure: {:.1} [kPa]\t temp: {:.1} [˚C]", pressure, temp).unwrap();
}
```