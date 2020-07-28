#![no_std]
#![no_main]

use panic_halt as _;
use nrf52840_hal as hal;
use hal::gpio::Level;
use rtt_target::{rprintln, rtt_init_print};

use hal::prelude::OutputPin;

mod sensor;

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();

    let device = hal::pac::Peripherals::take().unwrap();
    let p0 = hal::gpio::p0::Parts::new(device.P0);

    // onboard RGB LED pins
    let mut rgb_led = (
        p0.p0_23.into_push_pull_output(Level::High),
        p0.p0_22.into_push_pull_output(Level::High),
        p0.p0_24.into_push_pull_output(Level::High),
    ); // 0 = R, 1 = G, 2 = B - active low
    rgb_led.0.set_low().unwrap();

    let mut uart = hal::Uarte::new(
        device.UARTE0,
        hal::uarte::Pins {
            txd: p0.p0_06.into_push_pull_output(Level::High).degrade(),
            rxd: p0.p0_07.into_floating_input().degrade(),
            cts: None,
            rts: None,
        },
        hal::uarte::Parity::EXCLUDED,
        hal::uarte::Baudrate::BAUD9600,
    );

    rprintln!("Reading CO2 value...");
    let read_cmd: [u8; 9] = [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79];
    let mut read_buffer: [u8; 9] = [0x00; 9];
    uart.write(&read_cmd).unwrap();
    uart.read(&mut read_buffer).unwrap();
    let concentration = (read_buffer[2] as u16) << 8 | (read_buffer[3] as u16);
    let temperature = read_buffer[4] as i16 - 40;
    rprintln!("Got CO2 value {:?} at {:?} deg C", concentration, temperature);

    loop {
        rprintln!("Hello, world!");
    }
}
