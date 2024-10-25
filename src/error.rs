use embassy_rp::adc::Error as ADCError;
use embassy_rp::i2c::Error as I2cError;
use mcp9808::error::Error as MCP9808Error;

#[derive(Copy, Clone)]
pub enum Error {
    MCP9808RegisterSizeMismatchError(u8),
    MCP9808I2cError,
    ADCSensorError(ADCError),
    FormattingError,
    NetworkError,
}

impl From<MCP9808Error<I2cError>> for Error {
    fn from(other: MCP9808Error<I2cError>) -> Self {
        match other {
            MCP9808Error::I2c(_) => Self::MCP9808I2cError,
            MCP9808Error::RegisterSizeMismatch(e) => Self::MCP9808RegisterSizeMismatchError(e),
        }
    }
}

impl From<ADCError> for Error {
    fn from(other: ADCError) -> Self {
        Self::ADCSensorError(other)
    }
}
