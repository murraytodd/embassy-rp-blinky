use crate::error::Error;
use core::convert::identity;
use core::fmt::Write;
use embassy_rp::i2c::{Async, I2c};
use heapless::String;
use mcp9808::reg_conf::{Configuration, ShutdownMode};
use mcp9808::reg_res::ResolutionVal;
use mcp9808::reg_temp_generic::*;
use mcp9808::MCP9808;

const REFERENCE_VOLTAGE: f32 = 3.3;
const STEPS_12BIT: f32 = 4096 as f32;

/// Convert ADC binary value to a float voltage value.
///
/// The ADC has a 12-bit resolution of voltage, meaning that there
/// are 2^12 or 4096 unique levels from OFF (0V) to FULL (3V). This
/// function converts the ADC reading into a float measurement in volts.
fn adc_reading_to_voltage(reading_12bit: u16) -> f32 {
    (reading_12bit as f32 / STEPS_12BIT) * REFERENCE_VOLTAGE
}

fn f_to_c(f: f32) -> f32 {
    (f - 32.0) * 5.0 / 9.0
}

fn c_to_f(c: f32) -> f32 {
    (c * 9.0 / 5.0) + 32.0
}

#[derive(defmt::Format)]
pub enum TemperatureScale {
    C,
    F,
}

#[derive(defmt::Format)]
pub struct TempReading {
    pub temp: f32,
    pub scale: TemperatureScale,
    pub sensor: &'static str,
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
    pub fn new_from_tmp36(adc_reading: u16) -> Self {
        let voltage: f32 = adc_reading_to_voltage(adc_reading);
        let c = (100.0 * voltage) - 50.0;
        TempReading {
            temp: c,
            scale: TemperatureScale::C,
            sensor: "TMP36",
        }
    }
    pub fn new_from_internal(adc_reading: u16) -> Self {
        let voltage: f32 = adc_reading_to_voltage(adc_reading);
        let c: f32 = 27.0 - ((voltage - 0.706) / 0.001721);
        TempReading {
            temp: c,
            scale: TemperatureScale::C,
            sensor: "On-chip",
        }
    }
    pub fn read_mcp9808<I: embassy_rp::i2c::Instance>(
        sensor: &mut MCP9808<I2c<I, Async>>,
    ) -> Result<TempReading, Error> {
        sensor
            .read_temperature()
            .map(|mcp_reading| TempReading {
                temp: mcp_reading.get_celsius(ResolutionVal::Deg_0_0625C),
                scale: TemperatureScale::C,
                sensor: "MCP9808",
            })
            .map_err(Error::from)
    }
}

pub fn new_mcp9808<I: embassy_rp::i2c::Instance>(
    i2c: I2c<I, Async>,
) -> Result<MCP9808<I2c<I, Async>>, Error> {
    let mut mcp9808 = MCP9808::new(i2c);
    mcp9808
        .read_configuration()
        .map(|mut c| {
            c.set_shutdown_mode(ShutdownMode::Continuous);
            mcp9808
        })
        .map_err(Error::from)
}

// Alternative approach with pattern matching:
// pub fn new_mcp9808<I: embassy_rp::i2c::Instance>(
//     i2c: I2c<I, Async>,
// ) -> Result<MCP9808<I2c<I, Async>>, Error> {
//     let mut mcp9808 = MCP9808::new(i2c);
//     match mcp9808.read_configuration() {
//         Ok(mut config) => {
//             config.set_shutdown_mode(ShutdownMode::Continuous);
//             Ok(mcp9808)
//         }
//         Err(e) => Err(Error::from(e)),
//     }
// }

pub fn format<const N: usize, T: IntoIterator<Item = Result<TempReading, Error>>>(
    s: &mut String<N>,
    readings: T,
) -> Result<(), Error> {
    s.clear();
    s.push('{').map_err(|_| Error::FormattingError)?;

    let fmt_success: bool = readings
        .into_iter()
        .flatten() // Only take the Ok<TempReading> values
        .map(|r| {
            write!(s, " {s} : {t:.*},", 2, s = r.sensor, t = r.get_fahrenheit()).is_ok()
            // if an error occurred, convert any errors into Option<E>
        }) // flat_map converts the iterator of readings into an iterator only of the errors.
        .all(identity); // each success = true, so this assumes all write ops were successes

    let _ = s.pop(); // Drop the trailing comma that we know is at the end, so we can replace with the final "}"
    let fmt_success = fmt_success && s.push_str(" }").is_ok();
    if fmt_success {
        Ok(())
    } else {
        Err(Error::FormattingError)
    }
}
