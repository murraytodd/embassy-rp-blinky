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

use rand_core::RngCore;
use static_cell::StaticCell;

use mcp9808::reg_conf::{Configuration, ShutdownMode};
use mcp9808::reg_res::ResolutionVal;
use mcp9808::reg_temp_generic::*;
use mcp9808::MCP9808;

use {defmt_rtt as _, panic_probe as _};

const REFERENCE_VOLTAGE: f32 = 3.3;
const STEPS_12BIT: f32 = 4096 as f32;

/// Basic Celsius-to-Fahrenheit conversion
fn c_to_f(c: f32) -> f32 {
    (c * 9.0 / 5.0) + 32.0
}

/// Convert ADC binary value to a float voltage value.
///
/// The ADC has a 12-bit resolution of voltage, meaning that there
/// are 2^12 or 4096 unique levels from OFF (0V) to FULL (3V). This
/// function converts the ADC reading into a float measurement in volts.
fn adc_reading_to_voltage(reading_12bit: u16) -> f32 {
    (reading_12bit as f32 / STEPS_12BIT) * REFERENCE_VOLTAGE
}

/// Convert the voltage from a TMP36 sensor into a temperature reading.
///
/// The sensor returns 0.5V at 0°C and voltage changes ±0.01V for every
/// degree Celcius with higher temps resolting in higher voltages within
/// the range of -40°C to 125°C.
fn tmp36_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c = (100.0 * voltage) - 50.0;
    c_to_f(c)
}

/// Convert the voltage from the onboard temp sensor into a temp reading.
///
/// From §4.9.5 from the rp2040-datasheet.pdf, the temperature can be
/// approximated as T = 27 - (ADC_voltage - 0.706) / 0.001721.
fn chip_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c: f32 = 27.0 - ((voltage - 0.706) / 0.001721);
    c_to_f(c)
}

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

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
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
    let i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);
    info!("I2C Initialized");

    // Setup MCP9808 Temperature Sensor

    let mut mcp9808 = MCP9808::new(i2c);
    let mut mcp_9808_conf = mcp9808.read_configuration().unwrap();
    mcp_9808_conf.set_shutdown_mode(ShutdownMode::Continuous);

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
    unwrap!(spawner.spawn(wifi_task(runner)));

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

    unwrap!(spawner.spawn(net_task(stack))); // Start networking services thread

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
        let chip_voltage_24bit: u16 = adc.read(&mut temp_channel).await.unwrap();
        let tmp36_voltage_24bit: u16 = adc.read(&mut adc_channel_pin26).await.unwrap();

        let mcp9808_reading_c: f32 = mcp9808
            .read_temperature()
            .unwrap()
            .get_celsius(ResolutionVal::Deg_0_0625C);

        info!(
            "Temp readings:  MCP9808: {}°F, TMP36: {}°F, OnChip: {}°F",
            c_to_f(mcp9808_reading_c),
            chip_f(chip_voltage_24bit),
            tmp36_f(tmp36_voltage_24bit)
        );

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
