use thiserrors::Error;

pub struct ParseError {
    InvalidChecksum,
    InvalidLength,
    InvalidHeader
};

