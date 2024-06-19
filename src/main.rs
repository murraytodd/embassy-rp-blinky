#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{self, Pull};
use embassy_rp::i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Timer};
use gpio::{Level, Output};
use mcp9808::reg_conf::Configuration;
use mcp9808::reg_conf::ShutdownMode;
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
async fn main(_spawner: Spawner) {
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
    let mut adc_channel_pin26 = Channel::new_pin(&mut p.PIN_26, Pull::None); // TODO try up, down, none

    loop {
        info!("led on!");
        led.set_high();
        Timer::after(Duration::from_secs(1)).await;

        info!("led off!");
        led.set_low();
        Timer::after(Duration::from_secs(1)).await;

        let chip_voltage_24bit: u16 = adc.blocking_read(&mut temp_channel).unwrap();
        let tmp36_voltage_24bit: u16 = adc.blocking_read(&mut adc_channel_pin26).unwrap();

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
    }
}
