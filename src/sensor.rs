use embassy_rp::i2c::{Async, I2c};
use mcp9808::reg_conf::{Configuration, ShutdownMode};
use mcp9808::reg_res::ResolutionVal;
use mcp9808::reg_temp_generic::*;
use mcp9808::MCP9808;

const REFERENCE_VOLTAGE: f32 = 3.3;
const STEPS_12BIT: f32 = 4096 as f32;

/// Basic Celsius-to-Fahrenheit conversion
pub fn c_to_f(c: f32) -> f32 {
    (c * 9.0 / 5.0) + 32.0
}

pub fn f_to_c(f: f32) -> f32 {
    (f - 32.0) * 5.0 / 9.0
}

/// Convert ADC binary value to a float voltage value.
///
/// The ADC has a 12-bit resolution of voltage, meaning that there
/// are 2^12 or 4096 unique levels from OFF (0V) to FULL (3V). This
/// function converts the ADC reading into a float measurement in volts.
pub fn adc_reading_to_voltage(reading_12bit: u16) -> f32 {
    (reading_12bit as f32 / STEPS_12BIT) * REFERENCE_VOLTAGE
}

/// Convert the voltage from a TMP36 sensor into a temperature reading.
///
/// The sensor returns 0.5V at 0°C and voltage changes ±0.01V for every
/// degree Celcius with higher temps resolting in higher voltages within
/// the range of -40°C to 125°C.
pub fn tmp36_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c = (100.0 * voltage) - 50.0;
    c_to_f(c)
}

/// Convert the voltage from the onboard temp sensor into a temp reading.
///
/// From §4.9.5 from the rp2040-datasheet.pdf, the temperature can be
/// approximated as T = 27 - (ADC_voltage - 0.706) / 0.001721.
pub fn chip_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c: f32 = 27.0 - ((voltage - 0.706) / 0.001721);
    c_to_f(c)
}

pub enum TemperatureScale {
    C,
    F,
}

pub struct TempReading {
    temp: f32,
    scale: TemperatureScale,
}

impl TempReading {
    pub fn get_celsius(&self) -> f32 {
        match self.scale {
            TemperatureScale::C => self.temp,
            TemperatureScale::F => f_to_c(self.temp),
        }
    }
    pub fn get_fahrenheit(&self) -> f32 {
        match self.scale {
            TemperatureScale::C => c_to_f(self.temp),
            TemperatureScale::F => self.temp,
        }
    }
}

pub fn new_mcp9808<I: embassy_rp::i2c::Instance>(i2c: I2c<I, Async>) -> MCP9808<I2c<I, Async>> {
    let mut mcp9808 = MCP9808::new(i2c);
    let mut mcp_9808_conf = mcp9808.read_configuration().unwrap();
    mcp_9808_conf.set_shutdown_mode(ShutdownMode::Continuous);
    mcp9808
}

pub fn read_mcp9808<I: embassy_rp::i2c::Instance>(
    sensor: &mut MCP9808<I2c<I, Async>>,
) -> core::result::Result<TempReading, mcp9808::error::Error<embassy_rp::i2c::Error>> {
    sensor.read_temperature().map(|r| TempReading {
        temp: r.get_celsius(ResolutionVal::Deg_0_0625C),
        scale: TemperatureScale::C,
    })
}
