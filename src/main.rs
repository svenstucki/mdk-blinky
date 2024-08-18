#![no_main]
#![no_std]

use core::panic::PanicInfo;
use cortex_m as _;
use cortex_m_rt::entry;
use defmt;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use hal::gpio::Level;
use nrf52840_hal as hal;

mod sensor;


#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    if let Some(location) = info.location() {
        defmt::error!("\n###### Panic occured in file '{}' at line {}", location.file(), location.line());
    } else {
        defmt::error!("\n###### Panic occured at unknown location");
    }
    let message = info.message();
    defmt::error!("{}", message.as_str());

    loop {}
}

#[entry]
fn main() -> ! {
    defmt::info!("Initializing..");

    let core_device = hal::pac::CorePeripherals::take().unwrap();
    let device = hal::pac::Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(device.P0);

    // onboard RGB LED pins
    let mut rgb_led = (
        port0.p0_23.into_push_pull_output(Level::High),
        port0.p0_22.into_push_pull_output(Level::High),
        port0.p0_24.into_push_pull_output(Level::High),
    ); // pins are (R, G, B) - active low
    rgb_led.0.set_low().unwrap();
    rgb_led.1.set_low().unwrap();

    let mut uart = hal::Uarte::new(
        device.UARTE0,
        hal::uarte::Pins {
            txd: port0.p0_06.into_push_pull_output(Level::High).degrade(),
            rxd: port0.p0_07.into_floating_input().degrade(),
            cts: None,
            rts: None,
        },
        hal::uarte::Parity::EXCLUDED,
        hal::uarte::Baudrate::BAUD9600,
    );

    let mut delay = hal::delay::Delay::new(core_device.SYST);

    loop {
        defmt::info!("Reading CO2 value...");
        let read_cmd: [u8; 9] = [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79];
        let mut read_buffer: [u8; 9] = [0x00; 9];
        uart.write(&read_cmd).unwrap();
        uart.read(&mut read_buffer).unwrap();
        let concentration = (read_buffer[2] as u16) << 8 | (read_buffer[3] as u16);
        let temperature = read_buffer[4] as i16 - 40;
        defmt::info!(
            "Got CO2 value {:?} at {:?} deg C",
            concentration,
            temperature
        );

        delay.delay_ms(2000);
    }
}
