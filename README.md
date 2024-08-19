# BLE CO2 Sensor

This embedded project measures CO2 concentration from a MH-Z19B sensor and
publishes the result via BLE.

It is built for the Makerdiary [nRF52840-MDK](https://wiki.makerdiary.com/nrf52840-mdk/) hardware.


## Toolchain setup

Prerequisites: rust and cargo

1. `cargo install probe-rs-tools --locked`
2. `rustup target add thumbv7em-none-eabihf`


## BLE

Download and unzip the [S140 v7.3.0](https://www.nordicsemi.com/Products/Development-software/s140/download) SoftDevice from Nordic.

Flash to chip using: `probe-rs download --verify --binary-format hex --chip nRF52840_xxAA <path to softdevice>.hex`

The memory.x file expects this SoftDevice to be flashed on the chip - the
application will not start properly without it.


## Run application

Use the following command to debug the application:

`DEFMT_LOG=debug cargo embed`

Flash a release build with:

`cargo embed --release`


## References

Pinout:

![pinout](https://wiki.makerdiary.com/nrf52840-mdk/images/nrf52840-mdk-pinout.jpg)
