//! Validation of Nortek binary records.
//!
//! Reference: Nortek, Nucleus 1000 Integrator's Guide, N3015-033 (2025),
//! `_HeaderData` on p. 83 and the checksum procedure on p. 115.

use std::fmt;

/// Sync byte at the start of every Nortek record.
pub const NORTEK_SYNC_BYTE: u8 = 0xA5;
/// Header size of the records that this module accepts.
pub const NORTEK_HEADER_SIZE: usize = 10;
const CHECKSUM_SEED: u16 = 0xB58C;

/// One validated Nortek record. `data` excludes the 10-byte header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NortekFrame<'a> {
    pub data_series_id: u8,
    pub family_id: u8,
    pub data: &'a [u8],
}

/// Reasons why a byte slice is not a valid Nortek record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NortekFrameError {
    TooShort { length: usize },
    NoSyncByte { found: u8 },
    UnsupportedHeaderSize { header_size: u8 },
    HeaderChecksum { expected: u16, computed: u16 },
    Truncated { declared: usize, available: usize },
    DataChecksum { expected: u16, computed: u16 },
}

impl fmt::Display for NortekFrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort { length } => {
                write!(f, "{length} bytes is shorter than a Nortek header")
            }
            Self::NoSyncByte { found } => write!(f, "expected sync byte 0xA5, found {found:#04x}"),
            Self::UnsupportedHeaderSize { header_size } => {
                write!(f, "unsupported Nortek header size {header_size}")
            }
            Self::HeaderChecksum { expected, computed } => write!(
                f,
                "header checksum {computed:#06x} does not match {expected:#06x}"
            ),
            Self::Truncated {
                declared,
                available,
            } => write!(
                f,
                "record declares {declared} data bytes but only {available} are present"
            ),
            Self::DataChecksum { expected, computed } => write!(
                f,
                "data checksum {computed:#06x} does not match {expected:#06x}"
            ),
        }
    }
}

impl std::error::Error for NortekFrameError {}

/// Nortek checksum: start at 0xB58C and add every little-endian 16-bit
/// word. An odd final byte is added as `byte << 8`. The sum wraps at 16 bits.
pub fn nortek_checksum(bytes: &[u8]) -> u16 {
    let mut checksum = CHECKSUM_SEED;
    let mut index = 0;
    while index + 1 < bytes.len() {
        checksum = checksum.wrapping_add(u16::from_le_bytes([bytes[index], bytes[index + 1]]));
        index += 2;
    }
    if index < bytes.len() {
        checksum = checksum.wrapping_add(u16::from(bytes[index]) << 8);
    }
    checksum
}

/// Validate one Nortek record at the start of `bytes`.
///
/// Bytes after the declared data length are ignored. The function never
/// panics.
pub fn parse_nortek_frame(bytes: &[u8]) -> Result<NortekFrame<'_>, NortekFrameError> {
    if bytes.len() < NORTEK_HEADER_SIZE {
        return Err(NortekFrameError::TooShort {
            length: bytes.len(),
        });
    }
    if bytes[0] != NORTEK_SYNC_BYTE {
        return Err(NortekFrameError::NoSyncByte { found: bytes[0] });
    }
    if usize::from(bytes[1]) != NORTEK_HEADER_SIZE {
        return Err(NortekFrameError::UnsupportedHeaderSize {
            header_size: bytes[1],
        });
    }
    let expected_header = u16::from_le_bytes([bytes[8], bytes[9]]);
    let computed_header = nortek_checksum(&bytes[..8]);
    if expected_header != computed_header {
        return Err(NortekFrameError::HeaderChecksum {
            expected: expected_header,
            computed: computed_header,
        });
    }
    let declared = usize::from(u16::from_le_bytes([bytes[4], bytes[5]]));
    let available = bytes.len() - NORTEK_HEADER_SIZE;
    if declared > available {
        return Err(NortekFrameError::Truncated {
            declared,
            available,
        });
    }
    let data = &bytes[NORTEK_HEADER_SIZE..NORTEK_HEADER_SIZE + declared];
    let expected_data = u16::from_le_bytes([bytes[6], bytes[7]]);
    let computed_data = nortek_checksum(data);
    if expected_data != computed_data {
        return Err(NortekFrameError::DataChecksum {
            expected: expected_data,
            computed: computed_data,
        });
    }
    Ok(NortekFrame {
        data_series_id: bytes[2],
        family_id: bytes[3],
        data,
    })
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::{nortek_checksum, NORTEK_HEADER_SIZE, NORTEK_SYNC_BYTE};

    /// AHRS record from the worked example in the Integrator's Guide,
    /// p. 114 to 118: header plus 108 data bytes.
    pub(crate) const MANUAL_AHRS_FRAME_HEX: &str = concat!(
        "A50AD2206C008AE5F9C6",
        "022400000200000000350C0000000000040000000000000002000000CB82773E0000A040",
        "ACA025BFBC744ABF6BB68D4363EC48BF11AA0E3C4E7CFB3AF19E1E3F6DB96D3E7AFF783F",
        "7DE5FE3B19FB78BF9A936D3EF572833C6923623C7CFC38BC94F57F3F00000000FFFE2D3F",
    );

    pub(crate) fn hex_to_bytes(hex: &str) -> Vec<u8> {
        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("valid hex"))
            .collect()
    }

    pub(crate) fn manual_ahrs_frame() -> Vec<u8> {
        hex_to_bytes(MANUAL_AHRS_FRAME_HEX)
    }

    /// Build a valid Nucleus-family record around `data`.
    pub(crate) fn build_frame(data_series_id: u8, data: &[u8]) -> Vec<u8> {
        let mut frame = vec![
            NORTEK_SYNC_BYTE,
            NORTEK_HEADER_SIZE as u8,
            data_series_id,
            0x20,
        ];
        frame.extend_from_slice(&(data.len() as u16).to_le_bytes());
        frame.extend_from_slice(&nortek_checksum(data).to_le_bytes());
        let header_checksum = nortek_checksum(&frame[..8]);
        frame.extend_from_slice(&header_checksum.to_le_bytes());
        frame.extend_from_slice(data);
        frame
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::{build_frame, manual_ahrs_frame};
    use super::*;

    #[test]
    fn checksum_matches_manual_worked_example() {
        let frame = manual_ahrs_frame();
        assert_eq!(nortek_checksum(&frame[..8]), 0xC6F9);
        assert_eq!(nortek_checksum(&frame[10..]), 0xE58A);
    }

    #[test]
    fn checksum_adds_odd_final_byte_as_high_byte() {
        assert_eq!(nortek_checksum(&[]), 0xB58C);
        assert_eq!(nortek_checksum(&[0x01]), 0xB58C + 0x0100);
        assert_eq!(
            nortek_checksum(&[0x01, 0x02, 0x03]),
            0xB58C + 0x0201 + 0x0300
        );
    }

    #[test]
    fn manual_worked_example_is_a_valid_frame() {
        let bytes = manual_ahrs_frame();
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        assert_eq!(frame.data_series_id, 0xD2);
        assert_eq!(frame.family_id, 0x20);
        assert_eq!(frame.data.len(), 108);
    }

    #[test]
    fn trailing_bytes_after_the_record_are_ignored() {
        let mut bytes = manual_ahrs_frame();
        bytes.extend_from_slice(&[0xA5, 0x0A, 0xD2]);
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        assert_eq!(frame.data.len(), 108);
    }

    #[test]
    fn corrupted_header_checksum_is_rejected() {
        let mut bytes = manual_ahrs_frame();
        bytes[9] ^= 0xFF;
        assert!(matches!(
            parse_nortek_frame(&bytes),
            Err(NortekFrameError::HeaderChecksum { .. })
        ));
    }

    #[test]
    fn corrupted_data_is_rejected() {
        let mut bytes = manual_ahrs_frame();
        bytes[40] ^= 0x01;
        assert_eq!(
            parse_nortek_frame(&bytes),
            Err(NortekFrameError::DataChecksum {
                expected: 0xE58A,
                computed: nortek_checksum(&bytes[10..]),
            })
        );
    }

    #[test]
    fn ascii_reply_is_rejected_without_panicking() {
        for text in [
            &b"OK\r\n"[..],
            b"ERROR\r\n",
            b"Nortek Nucleus1000\r\nVersion 4.2.2\r\nOK\r\n",
        ] {
            assert!(parse_nortek_frame(text).is_err());
        }
    }

    #[test]
    fn short_input_is_rejected() {
        assert_eq!(
            parse_nortek_frame(&[0xA5, 0x0A, 0xB4]),
            Err(NortekFrameError::TooShort { length: 3 })
        );
        assert_eq!(
            parse_nortek_frame(&[]),
            Err(NortekFrameError::TooShort { length: 0 })
        );
    }

    #[test]
    fn missing_sync_byte_is_rejected() {
        let mut bytes = manual_ahrs_frame();
        bytes[0] = 0x00;
        assert_eq!(
            parse_nortek_frame(&bytes),
            Err(NortekFrameError::NoSyncByte { found: 0x00 })
        );
    }

    #[test]
    fn unsupported_header_size_is_rejected() {
        let mut bytes = manual_ahrs_frame();
        bytes[1] = 12;
        assert_eq!(
            parse_nortek_frame(&bytes),
            Err(NortekFrameError::UnsupportedHeaderSize { header_size: 12 })
        );
    }

    #[test]
    fn declared_length_beyond_input_is_rejected() {
        let bytes = build_frame(0xB4, &[0u8; 32]);
        assert_eq!(
            parse_nortek_frame(&bytes[..30]),
            Err(NortekFrameError::Truncated {
                declared: 32,
                available: 20,
            })
        );
    }

    #[test]
    fn built_frame_round_trips() {
        let data: Vec<u8> = (0..=50).collect();
        let bytes = build_frame(0xBE, &data);
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        assert_eq!(frame.data_series_id, 0xBE);
        assert_eq!(frame.data, &data[..]);
    }
}
