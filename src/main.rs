#![no_std]
#![no_main]

use core::str::{from_utf8, FromStr};
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config as NetConfig, DhcpConfig, IpEndpoint, Stack, StackResources};
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIN_23, PIN_25, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Instant, Timer};

use heapless::String;

use rand_core::RngCore;
use static_cell::StaticCell;

use crate::error::Error;
use crate::sensor::*;
use {defmt_rtt as _, panic_probe as _};

mod error;
pub(crate) mod networking;
mod sensor;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

#[cortex_m_rt::pre_init]
unsafe fn before_main() {
    // Soft-reset doesn't clear spinlocks. Clear the one used by critical-section
    // before we hit main to avoid deadlocks when using a debugger
    embassy_rp::pac::SIO.spinlock(31).write_value(1);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Program start");
    let mut p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_22, Level::Low);

    // I2C Setup
    info!("Starting I2C Setup");
    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c: I2c<I2C0, embassy_rp::i2c::Async> = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);
    info!("I2C Initialized");

    // Setup MCP9808 Temperature Sensor

    let mut mcp9808 = new_mcp9808(i2c);

    // ADC Setup

    let mut adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let mut temp_channel = Channel::new_temp_sensor(&mut p.ADC_TEMP_SENSOR);
    let mut adc_channel_pin26 = Channel::new_pin(&mut p.PIN_26, Pull::None);

    // Configure PIO and CYW43

    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(networking::wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let seed: u64 = RoscRng.next_u64();
    info!("Random seed value seeded to {=u64:#X}", seed);

    let wifi_ssid = env!("WIFI_SSID");
    let wifi_password = env!("WIFI_PASSWORD");
    const SERVER_NAME: &str = "pi2b";
    const CLIENT_NAME: &str = "picow";
    const COMMS_PORT: u16 = 9932;

    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = Some(heapless::String::from_str(CLIENT_NAME).unwrap());
    let net_config = NetConfig::dhcpv4(dhcp_config);

    static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new(); // Increase this if you start getting socket ring errors.
    let stack = &*STACK.init(Stack::new(
        net_device,
        net_config,
        RESOURCES.init(StackResources::<4>::new()),
        seed,
    ));
    let mac_addr = stack.hardware_address();
    info!("Hardware configured. MAC Address is {}", mac_addr);

    unwrap!(spawner.spawn(networking::net_task(stack))); // Start networking services thread
    info!(
        "About to connect to '{}' with pw '{}'",
        wifi_ssid, wifi_password
    );
    control.join_wpa2(wifi_ssid, wifi_password).await.unwrap();

    let start = Instant::now().as_millis();
    loop {
        let elapsed = Instant::now().as_millis() - start;
        if elapsed > 10000 {
            core::panic!("Couldn't get network up after 10 seconds");
        } else if stack.is_config_up() {
            info!("Network stack config completed after about {} ms", elapsed);
            break;
        } else {
            Timer::after_millis(10).await;
        }
    }

    match stack.config_v4() {
        Some(a) => info!("IP Address appears to be: {}", a.address),
        None => core::panic!("DHCP completed but no IP address was assigned!"),
    }

    let server_address = stack
        .dns_query(SERVER_NAME, embassy_net::dns::DnsQueryType::A)
        .await
        .unwrap();

    let dest = server_address.first().unwrap().clone();
    info!(
        "Our server named {} resolved to the address {}",
        SERVER_NAME, dest
    );

    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];
    let mut msg_buffer = [0; 128];

    let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);
    socket
        .connect(IpEndpoint::new(dest, COMMS_PORT))
        .await
        .unwrap();

    let tx_size = socket.write("test".as_bytes()).await.unwrap();
    info!("Wrote {} byes to the server", tx_size);
    let rx_size = socket.read(&mut msg_buffer).await.unwrap();
    let response = from_utf8(&msg_buffer[..rx_size]).unwrap();
    info!("Server replied with {}", response);

    socket.close();

    let mut udp_rx_meta = [PacketMetadata::EMPTY; 16];
    let mut udp_rx_buffer = [0; 1024];
    let mut udp_tx_meta = [PacketMetadata::EMPTY; 16];
    let mut udp_tx_buffer = [0; 1024];
    // I'll reuse the earlier msg_buffer since we're done with the TCP part

    let mut udp_socket = UdpSocket::new(
        &stack,
        &mut udp_rx_meta,
        &mut udp_rx_buffer,
        &mut udp_tx_meta,
        &mut udp_tx_buffer,
    );

    udp_socket.bind(0).unwrap();

    loop {
        info!("external LED on, onboard LED off!");
        led.set_high();
        control.gpio_set(0, false).await;

        // Time to take our readings...

        let chip_reading = adc
            .read(&mut temp_channel)
            .await
            .map(TempReading::new_from_internal)
            .map_err(Error::from);

        let tmp36_reading = adc
            .read(&mut adc_channel_pin26)
            .await
            .map(TempReading::new_from_tmp36)
            .map_err(Error::from);

        let mcp9808_reading = read_mcp9808(&mut mcp9808).map_err(Error::from);

        let mut json: String<80> = String::new();
        let readings: [Result<TempReading, Error<_>>; 3] =
            [chip_reading, tmp36_reading, mcp9808_reading];
        for r in &readings {
            match r {
                Ok(t) => info!("Read {} at {} °F", t.sensor, t.temp),
                Err(_) => error!("Sensor reading error"),
            }
        }
        format(&mut json, readings.into_iter().filter_map(|r| r.ok()))
            .expect("Couldn't format the readings into a JSON. Maybe the heapless string wasn't big enough?");

        // info!(
        //     "Temp readings:  MCP9808: {}°F, TMP36: {}°F, OnChip: {}°F",
        //     mcp9808_reading.unwrap().get_fahrenheit(),
        //     chip_reading.unwrap().get_fahrenheit(),
        //     tmp36_reading.unwrap().get_fahrenheit()
        // );

        info!("sending UDP packet");
        udp_socket
            .send_to("test".as_bytes(), IpEndpoint::new(dest, COMMS_PORT))
            .await
            .unwrap();
        Timer::after(Duration::from_secs(1)).await;

        info!("external LED off, onboard LED on!");
        led.set_low();
        control.gpio_set(0, true).await;
        if udp_socket.may_recv() {
            let (rx_size, from_addr) = udp_socket.recv_from(&mut msg_buffer).await.unwrap();
            let response = from_utf8(&msg_buffer[..rx_size]).unwrap();
            info!("Server replied with {} from {}", response, from_addr);
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}
