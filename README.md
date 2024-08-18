# mdk-blink


## Toolchain setup

Prerequisites: rust and cargo

1. `cargo install cargo-embed`
2. `rustup target add thumbv7em-none-eabihf`


## BLE

Download and unzip the [S140 v7.3.0](https://www.nordicsemi.com/Products/Development-software/s140/download) SoftDevice from Nordic.

Flash to chip using: `probe-rs download --verify --binary-format hex --chip nRF52840_xxAA <path to softdevice>.hex`

The memory.x file expects this SoftDevice to be flashed on the chip - the
application will not start properly without it.


## Run application

`DEFMT_LOG=debug cargo embed`
