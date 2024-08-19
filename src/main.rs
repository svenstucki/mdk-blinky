#![no_main]
#![no_std]

use assign_resources::assign_resources;
use core::mem;
use core::panic::PanicInfo;
use cortex_m as _;
use defmt;
use defmt_rtt as _;
use embassy_nrf::{
    interrupt::{self, InterruptExt},
    bind_interrupts,
    uarte,
    gpio::{Level, Output, OutputDrive},
    pwm::{Prescaler, SimplePwm},
    peripherals,
    self,
};
use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_sync::{signal::Signal, blocking_mutex::raw::ThreadModeRawMutex};
use nrf_softdevice::ble::advertisement_builder::{
    Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
};
use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};

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

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[nrf_softdevice::gatt_service(uuid = "b22d7f14-9361-4309-932e-ffbdefed97fe")]
struct CO2Service {
    #[characteristic(uuid = "c7d0c8a8-db04-4199-869d-5f80091f2036", read, notify, indicate)]
    concentration: u16,
    #[characteristic(uuid = "c7d0c8a8-db04-4199-869d-5f80091f2037", read, notify, indicate)]
    temperature: i16,
    #[characteristic(uuid = "c7d0c8a8-db04-4199-869d-5f80091f2038", read, write)]
    rgb_led: u32,
}

#[nrf_softdevice::gatt_server]
struct Server {
    co2: CO2Service,
}

#[embassy_executor::task]
async fn led_pwm_task(rgb_leds: Leds) {
    defmt::debug!("PWM initialising");
    let mut pwm = SimplePwm::new_3ch(rgb_leds.pwm, rgb_leds.r, rgb_leds.g, rgb_leds.b);
    pwm.set_prescaler(Prescaler::Div64);
    pwm.set_max_duty(255);
    pwm.set_duty(0, 0);
    pwm.set_duty(1, 0);
    pwm.set_duty(2, 0);
    defmt::debug!("PWM initialised");

    loop {
        let color = RGB_LED_COLOR.wait().await;
        pwm.set_duty(0, (color & 0xff) as u16);
        pwm.set_duty(1, ((color >> 8) & 0xff) as u16);
        pwm.set_duty(2, ((color >> 16) & 0xff) as u16);
        defmt::debug!("PWM set");
    }
}

#[embassy_executor::task]
async fn co2_sensor_task(sensor_uart: SensorUart) {
    defmt::debug!("UART initialising");
    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD9600;
    let mut uart = uarte::Uarte::new(sensor_uart.uart, Irqs, sensor_uart.rx, sensor_uart.tx, config);
    defmt::debug!("UART initialised");

    loop {
        defmt::info!("Reading CO2 value...");
        let read_cmd: [u8; 9] = [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79];
        defmt::unwrap!(uart.write(&read_cmd).await);
        let mut read_buffer: [u8; 9] = [0x00; 9];
        defmt::unwrap!(uart.read(&mut read_buffer).await);

        let concentration = (read_buffer[2] as u16) << 8 | (read_buffer[3] as u16);
        let temperature = read_buffer[4] as i16 - 40;
        defmt::info!(
            " got concentration {} ppm at {} deg C",
            concentration,
            temperature
        );

        Timer::after_secs(2).await;
    }
}

assign_resources! {
    leds: Leds {
        r: P0_23,
        g: P0_22,
        b: P0_24,
        pwm: PWM0,
    },
    sensor_uart: SensorUart {
        rx: P0_07,
        tx: P0_06,
        uart: UARTE0,
    },
}

bind_interrupts!(struct Irqs {
    UARTE0_UART0 => uarte::InterruptHandler<peripherals::UARTE0>;
});

static RGB_LED_COLOR: Signal<ThreadModeRawMutex, u32> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("Initialising");

    // initialize HAL
    let mut config = embassy_nrf::config::Config::default();
    // lower interrupt priority due to SD (it reserves 0, 1 and 4)
    config.gpiote_interrupt_priority = interrupt::Priority::P2;
    config.time_interrupt_priority = interrupt::Priority::P2;
    interrupt::UARTE0_UART0.set_priority(interrupt::Priority::P3);
    // split resources
    let peripherals = embassy_nrf::init(config);
    let resources = split_resources!(peripherals);

    // spawn LED task
    defmt::unwrap!(spawner.spawn(led_pwm_task(resources.leds)));

    // spawn sensor task
    defmt::unwrap!(spawner.spawn(co2_sensor_task(resources.sensor_uart)));

    // initialize SD and spawn task
    let sd_config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"RustCO2Sensor" as *const u8 as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };
    let sd = Softdevice::enable(&sd_config);
    let server = defmt::unwrap!(Server::new(sd));
    defmt::unwrap!(spawner.spawn(softdevice_task(sd)));

    static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
        .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
        .full_name("RustCO2Sensor")
        .build();
    static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
        .services_128(
            ServiceList::Complete,
            &[0xb22d7f14_9361_4309_932e_ffbdefed97fe_u128.to_le_bytes()],
            )
        .build();

    loop {
        defmt::info!("start advertising");
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &ADV_DATA,
            scan_data: &SCAN_DATA,
        };
        let conn = defmt::unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);
        defmt::info!("got connection, stop advertising");

        // Run the GATT server on the connection. This returns when the connection gets disconnected.
        //
        // Event enums (ServerEvent's) are generated by nrf_softdevice::gatt_server
        // proc macro when applied to the Server struct above
        let gatt_future = gatt_server::run(&conn, &server, |e| match e {
            ServerEvent::Co2(e) => match e {
                CO2ServiceEvent::ConcentrationCccdWrite {
                    indications,
                    notifications,
                } => {
                    defmt::info!("concentration indications: {}, notifications: {}", indications, notifications)
                },
                CO2ServiceEvent::TemperatureCccdWrite {
                    indications,
                    notifications,
                } => {
                    defmt::info!("temperature indications: {}, notifications: {}", indications, notifications)
                },
                CO2ServiceEvent::RgbLedWrite(val) => {
                    defmt::info!("got value {} for RGB LED", val);
                    RGB_LED_COLOR.signal(val)
                },
            },
        });
        let e = gatt_future.await;
        defmt::info!("gatt_server run exited with error: {:?}", e);
    }
}
