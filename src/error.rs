use embassy_rp::adc::Error as ADCError;
use mcp9808::error::Error as MCP9808Error;

pub enum Error<E> {
    MCPSensorError(MCP9808Error<E>),
    ADCSensorError(ADCError),
    NetworkError,
}

impl<E> From<MCP9808Error<E>> for Error<E> {
    fn from(other: MCP9808Error<E>) -> Self {
        Self::MCPSensorError(other)
    }
}

impl<E> From<ADCError> for Error<E> {
    fn from(other: ADCError) -> Self {
        Self::ADCSensorError(other)
    }
}
