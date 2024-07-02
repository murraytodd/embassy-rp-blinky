#![no_std]
#![no_main]

use core::str::FromStr;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::{Config as NetConfig, DhcpConfig, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_rp::clocks::RoscRng;
use rand_core::RngCore;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
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
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_22, Level::Low);

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

    loop {
        info!("external LED on, onboard LED off!");
        led.set_high();
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_secs(1)).await;

        info!("external LED off, onboard LED on!");
        led.set_low();
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_secs(1)).await;
    }
}
