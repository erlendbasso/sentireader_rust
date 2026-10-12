//! Decoders for Nucleus 1000 records inside a validated Nortek frame.
//!
//! Reference: Nortek, Nucleus 1000 Integrator's Guide, N3015-033 (2025),
//! chapter 8. Field positions are byte offsets from the start of the record
//! data, after the 10-byte header. All values are little-endian.

use std::fmt;

use crate::nortek_frame::NortekFrame;

/// Family id of all Nucleus records (p. 83).
pub const NUCLEUS_FAMILY_ID: u8 = 0x20;
/// Bottom-track record id (p. 91).
pub const BOTTOM_TRACK_ID: u8 = 0xB4;
/// Water-track record id (p. 94). It has the bottom-track layout.
pub const WATER_TRACK_ID: u8 = 0xBE;
/// AHRS record id (p. 84).
pub const AHRS_ID: u8 = 0xD2;
/// The last track field, `deltaT XYZ`, ends at byte 124 (p. 93).
pub const TRACK_RECORD_MIN_LENGTH: usize = 124;
const COMMON_DATA_LENGTH: usize = 12;
/// The last AHRS field, depth, ends 72 bytes after the data offset.
const AHRS_FIELDS_LENGTH: usize = 72;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NucleusRecordError {
    WrongFamily { family_id: u8 },
    UnexpectedRecordId { found: u8 },
    TooShort { needed: usize, length: usize },
}

impl fmt::Display for NucleusRecordError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongFamily { family_id } => {
                write!(
                    f,
                    "family id {family_id:#04x} is not the Nucleus family 0x20"
                )
            }
            Self::UnexpectedRecordId { found } => {
                write!(f, "record id {found:#04x} is not expected here")
            }
            Self::TooShort { needed, length } => {
                write!(f, "record has {length} data bytes, {needed} are needed")
            }
        }
    }
}

impl std::error::Error for NucleusRecordError {}

/// `_CommonData` (p. 84).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonData {
    pub version: u8,
    pub data_offset: u8,
    /// True when `timestamp_s` is POSIX time, false when it counts seconds
    /// since the `START` command.
    pub posix_time: bool,
    pub timestamp_s: u32,
    pub microseconds: u32,
}

/// DVL status bit mask of a track record (p. 93 to 94).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrackStatus(pub u32);

impl TrackStatus {
    fn bit(self, index: usize) -> bool {
        (self.0 >> index) & 1 == 1
    }

    /// Bits 0 to 2. `beam` is 0, 1, or 2.
    pub fn beam_velocity_valid(self, beam: usize) -> bool {
        beam < 3 && self.bit(beam)
    }

    /// Bits 3 to 5.
    pub fn beam_distance_valid(self, beam: usize) -> bool {
        beam < 3 && self.bit(3 + beam)
    }

    /// Bits 6 to 8.
    pub fn beam_uncertainty_valid(self, beam: usize) -> bool {
        beam < 3 && self.bit(6 + beam)
    }

    /// Bits 9 to 11. `axis` 0, 1, 2 is X, Y, Z.
    pub fn xyz_velocity_valid(self, axis: usize) -> bool {
        axis < 3 && self.bit(9 + axis)
    }

    /// Bits 12 to 14.
    pub fn xyz_uncertainty_valid(self, axis: usize) -> bool {
        axis < 3 && self.bit(12 + axis)
    }

    pub fn all_beam_velocities_valid(self) -> bool {
        (0..3).all(|beam| self.beam_velocity_valid(beam))
    }

    pub fn all_xyz_velocities_valid(self) -> bool {
        (0..3).all(|axis| self.xyz_velocity_valid(axis))
    }
}

/// Bottom-track or water-track record (p. 91 to 94). Velocities are in the
/// instrument XYZ frame. Invalid estimates keep the sentinel values that the
/// instrument writes: -32.768 for velocities, 0.0 for distances, and 10.0 for
/// uncertainties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackRecord {
    pub water_track: bool,
    pub common: CommonData,
    pub status: TrackStatus,
    pub serial_number: u32,
    pub sound_velocity_m_s: f32,
    pub temperature_c: f32,
    /// Measured pressure minus the configured `SETMISSION,POFF` (p. 91).
    pub pressure_bar: f32,
    pub beam_velocity_m_s: [f32; 3],
    pub beam_distance_m: [f32; 3],
    pub beam_velocity_uncertainty_m_s: [f32; 3],
    /// Time from the center of the bottom echo of each beam to the record
    /// timestamp.
    pub beam_delta_t_s: [f32; 3],
    pub beam_pulse_length_s: [f32; 3],
    pub velocity_xyz_m_s: [f32; 3],
    pub velocity_uncertainty_xyz_m_s: [f32; 3],
    /// Time from the trigger to the center of the beam 1 bottom echo.
    pub delta_t_xyz_s: f32,
}

/// AHRS record (p. 84 to 86). Angles are in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AhrsRecord {
    pub common: CommonData,
    pub serial_number: u32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub heading_deg: f32,
    pub quaternion_wxyz: [f32; 4],
    pub declination_deg: f32,
    pub depth_m: f32,
}

fn too_short(needed: usize, data: &[u8]) -> NucleusRecordError {
    NucleusRecordError::TooShort {
        needed,
        length: data.len(),
    }
}

fn read_u32(data: &[u8], index: usize) -> Result<u32, NucleusRecordError> {
    data.get(index..index + 4)
        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .ok_or_else(|| too_short(index + 4, data))
}

fn read_f32(data: &[u8], index: usize) -> Result<f32, NucleusRecordError> {
    data.get(index..index + 4)
        .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .ok_or_else(|| too_short(index + 4, data))
}

fn read_f32x3(data: &[u8], first: usize) -> Result<[f32; 3], NucleusRecordError> {
    Ok([
        read_f32(data, first)?,
        read_f32(data, first + 4)?,
        read_f32(data, first + 8)?,
    ])
}

fn check_family(frame: &NortekFrame<'_>) -> Result<(), NucleusRecordError> {
    if frame.family_id == NUCLEUS_FAMILY_ID {
        Ok(())
    } else {
        Err(NucleusRecordError::WrongFamily {
            family_id: frame.family_id,
        })
    }
}

/// Decode `_CommonData` from the start of record data.
pub fn decode_common_data(data: &[u8]) -> Result<CommonData, NucleusRecordError> {
    if data.len() < COMMON_DATA_LENGTH {
        return Err(too_short(COMMON_DATA_LENGTH, data));
    }
    Ok(CommonData {
        version: data[0],
        data_offset: data[1],
        posix_time: data[2] & 1 == 1,
        timestamp_s: read_u32(data, 4)?,
        microseconds: read_u32(data, 8)?,
    })
}

/// Decode a bottom-track (`0xB4`) or water-track (`0xBE`) record.
pub fn decode_track_record(frame: &NortekFrame<'_>) -> Result<TrackRecord, NucleusRecordError> {
    check_family(frame)?;
    let water_track = match frame.data_series_id {
        BOTTOM_TRACK_ID => false,
        WATER_TRACK_ID => true,
        found => return Err(NucleusRecordError::UnexpectedRecordId { found }),
    };
    let data = frame.data;
    if data.len() < TRACK_RECORD_MIN_LENGTH {
        return Err(too_short(TRACK_RECORD_MIN_LENGTH, data));
    }
    Ok(TrackRecord {
        water_track,
        common: decode_common_data(data)?,
        status: TrackStatus(read_u32(data, 12)?),
        serial_number: read_u32(data, 16)?,
        sound_velocity_m_s: read_f32(data, 24)?,
        temperature_c: read_f32(data, 28)?,
        pressure_bar: read_f32(data, 32)?,
        beam_velocity_m_s: read_f32x3(data, 36)?,
        beam_distance_m: read_f32x3(data, 48)?,
        beam_velocity_uncertainty_m_s: read_f32x3(data, 60)?,
        beam_delta_t_s: read_f32x3(data, 72)?,
        beam_pulse_length_s: read_f32x3(data, 84)?,
        velocity_xyz_m_s: read_f32x3(data, 96)?,
        velocity_uncertainty_xyz_m_s: read_f32x3(data, 108)?,
        delta_t_xyz_s: read_f32(data, 120)?,
    })
}

/// Decode an AHRS (`0xD2`) record. The orientation fields start at the
/// common-data "offset of data" byte.
pub fn decode_ahrs_record(frame: &NortekFrame<'_>) -> Result<AhrsRecord, NucleusRecordError> {
    check_family(frame)?;
    if frame.data_series_id != AHRS_ID {
        return Err(NucleusRecordError::UnexpectedRecordId {
            found: frame.data_series_id,
        });
    }
    let data = frame.data;
    let common = decode_common_data(data)?;
    let offset = usize::from(common.data_offset);
    if data.len() < offset + AHRS_FIELDS_LENGTH {
        return Err(too_short(offset + AHRS_FIELDS_LENGTH, data));
    }
    Ok(AhrsRecord {
        common,
        serial_number: read_u32(data, 16)?,
        roll_deg: read_f32(data, offset)?,
        pitch_deg: read_f32(data, offset + 4)?,
        heading_deg: read_f32(data, offset + 8)?,
        quaternion_wxyz: [
            read_f32(data, offset + 12)?,
            read_f32(data, offset + 16)?,
            read_f32(data, offset + 20)?,
            read_f32(data, offset + 24)?,
        ],
        declination_deg: read_f32(data, offset + 64)?,
        depth_m: read_f32(data, offset + 68)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvl_nucleus1000_parser::{parse_track_data, DataID};
    use crate::nortek_frame::parse_nortek_frame;
    use crate::nortek_frame::test_support::{build_frame, manual_ahrs_frame};

    fn put_u32(data: &mut [u8], index: usize, value: u32) {
        data[index..index + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn put_f32(data: &mut [u8], index: usize, value: f32) {
        data[index..index + 4].copy_from_slice(&value.to_le_bytes());
    }

    const BEAM_VELOCITY: [f32; 3] = [0.5, 1.5, 2.5];
    const BEAM_DISTANCE: [f32; 3] = [10.0, 11.0, 12.0];
    const BEAM_UNCERTAINTY: [f32; 3] = [0.01, 0.011, 0.012];
    const BEAM_DELTA_T: [f32; 3] = [0.040, 0.025, 0.010];
    const BEAM_PULSE_LENGTH: [f32; 3] = [0.002, 0.0021, 0.0022];
    const VELOCITY_XYZ: [f32; 3] = [1.25, 0.75, 0.25];
    const UNCERTAINTY_XYZ: [f32; 3] = [0.003, 0.004, 0.005];

    fn put_f32x3(data: &mut [u8], first: usize, values: [f32; 3]) {
        for (i, value) in values.into_iter().enumerate() {
            put_f32(data, first + 4 * i, value);
        }
    }

    /// A 128-byte track record with a distinct value in every field.
    fn synthetic_track_data() -> Vec<u8> {
        let mut data = vec![0u8; 128];
        data[0] = 1; // version
        data[1] = 24; // offset of data
        data[2] = 1; // POSIX time flag
        put_u32(&mut data, 4, 1_760_000_000);
        put_u32(&mut data, 8, 250_000);
        put_u32(&mut data, 12, 0b0111_1111_1111_1111);
        put_u32(&mut data, 16, 300_136);
        put_f32(&mut data, 24, 1486.5);
        put_f32(&mut data, 28, 8.25);
        put_f32(&mut data, 32, 0.0625);
        put_f32x3(&mut data, 36, BEAM_VELOCITY);
        put_f32x3(&mut data, 48, BEAM_DISTANCE);
        put_f32x3(&mut data, 60, BEAM_UNCERTAINTY);
        put_f32x3(&mut data, 72, BEAM_DELTA_T);
        put_f32x3(&mut data, 84, BEAM_PULSE_LENGTH);
        put_f32x3(&mut data, 96, VELOCITY_XYZ);
        put_f32x3(&mut data, 108, UNCERTAINTY_XYZ);
        put_f32(&mut data, 120, 0.0135);
        data
    }

    #[test]
    fn common_data_matches_manual_worked_example() {
        let bytes = manual_ahrs_frame();
        let common = decode_common_data(&bytes[10..]).expect("common data");
        assert_eq!(
            common,
            CommonData {
                version: 2,
                data_offset: 36,
                posix_time: false,
                timestamp_s: 2,
                microseconds: 800_000,
            }
        );
    }

    #[test]
    fn ahrs_record_matches_manual_worked_example() {
        let bytes = manual_ahrs_frame();
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        let ahrs = decode_ahrs_record(&frame).expect("AHRS record");
        assert_eq!(ahrs.serial_number, 4);
        assert!((ahrs.roll_deg - (-0.646_982_9)).abs() < 1e-6);
        assert!((ahrs.pitch_deg - (-0.790_843_7)).abs() < 1e-6);
        assert!((ahrs.heading_deg - 283.4251).abs() < 1e-3);
        let expected_quaternion = [-0.784_857, 0.008_707_54, 0.001_918_683, 0.619_612_8];
        for (value, expected) in ahrs.quaternion_wxyz.iter().zip(expected_quaternion) {
            assert!((value - expected).abs() < 1e-6);
        }
        assert_eq!(ahrs.declination_deg, 0.0);
        assert!((ahrs.depth_m - 0.679_672_2).abs() < 1e-6);
    }

    #[test]
    fn track_record_decodes_every_field_at_its_manual_offset() {
        let bytes = build_frame(BOTTOM_TRACK_ID, &synthetic_track_data());
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        let record = decode_track_record(&frame).expect("track record");
        assert!(!record.water_track);
        assert_eq!(record.common.timestamp_s, 1_760_000_000);
        assert_eq!(record.common.microseconds, 250_000);
        assert!(record.common.posix_time);
        assert_eq!(record.serial_number, 300_136);
        assert_eq!(record.sound_velocity_m_s, 1486.5);
        assert_eq!(record.temperature_c, 8.25);
        assert_eq!(record.pressure_bar, 0.0625);
        assert_eq!(record.beam_velocity_m_s, BEAM_VELOCITY);
        assert_eq!(record.beam_distance_m, BEAM_DISTANCE);
        assert_eq!(record.beam_velocity_uncertainty_m_s, BEAM_UNCERTAINTY);
        assert_eq!(record.beam_delta_t_s, BEAM_DELTA_T);
        assert_eq!(record.beam_pulse_length_s, BEAM_PULSE_LENGTH);
        assert_eq!(record.velocity_xyz_m_s, VELOCITY_XYZ);
        assert_eq!(record.velocity_uncertainty_xyz_m_s, UNCERTAINTY_XYZ);
        assert_eq!(record.delta_t_xyz_s, 0.0135);
    }

    #[test]
    fn water_track_uses_the_same_layout() {
        let bytes = build_frame(WATER_TRACK_ID, &synthetic_track_data());
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        let record = decode_track_record(&frame).expect("track record");
        assert!(record.water_track);
        assert_eq!(record.velocity_xyz_m_s, VELOCITY_XYZ);
    }

    #[test]
    fn track_record_agrees_with_legacy_parser() {
        let bytes = build_frame(BOTTOM_TRACK_ID, &synthetic_track_data());
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        let record = decode_track_record(&frame).expect("track record");
        let legacy = parse_track_data(&bytes, DataID::BottomTrackData);
        assert_eq!(legacy.velocity, record.velocity_xyz_m_s);
        assert_eq!(legacy.vel_beams, record.beam_velocity_m_s);
        assert_eq!(
            legacy.uncertainty_beams,
            record.beam_velocity_uncertainty_m_s
        );
        assert_eq!(legacy.uncertainty_vel, record.velocity_uncertainty_xyz_m_s);
        assert_eq!(legacy.pressure, record.pressure_bar);
        assert_eq!(legacy.sound_speed, record.sound_velocity_m_s);
        assert_eq!(legacy.temperature, record.temperature_c);
        assert!(legacy.velocity_valid && legacy.beams_valid);
    }

    #[test]
    fn status_bits_follow_the_manual_table() {
        let status = TrackStatus(0b0101_0010_0100_1001);
        assert!(status.beam_velocity_valid(0));
        assert!(!status.beam_velocity_valid(1));
        assert!(status.beam_distance_valid(0));
        assert!(status.beam_uncertainty_valid(0));
        assert!(status.xyz_velocity_valid(0));
        assert!(!status.xyz_velocity_valid(2));
        assert!(status.xyz_uncertainty_valid(0));
        assert!(status.xyz_uncertainty_valid(2));
        assert!(!status.beam_velocity_valid(3));
        assert!(TrackStatus(0x7FFF).all_beam_velocities_valid());
        assert!(TrackStatus(0x7FFF).all_xyz_velocities_valid());
        assert!(!TrackStatus(0x7DFF).all_xyz_velocities_valid());
    }

    #[test]
    fn short_track_record_is_rejected_without_panicking() {
        let bytes = build_frame(BOTTOM_TRACK_ID, &synthetic_track_data()[..100]);
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        assert_eq!(
            decode_track_record(&frame),
            Err(NucleusRecordError::TooShort {
                needed: TRACK_RECORD_MIN_LENGTH,
                length: 100,
            })
        );
    }

    #[test]
    fn ahrs_with_offset_beyond_data_is_rejected() {
        let mut data = manual_ahrs_frame()[10..].to_vec();
        data[1] = 100;
        let bytes = build_frame(AHRS_ID, &data);
        let frame = parse_nortek_frame(&bytes).expect("valid frame");
        assert_eq!(
            decode_ahrs_record(&frame),
            Err(NucleusRecordError::TooShort {
                needed: 172,
                length: 108,
            })
        );
    }

    #[test]
    fn wrong_record_id_and_family_are_rejected() {
        let ahrs_bytes = manual_ahrs_frame();
        let ahrs_frame = parse_nortek_frame(&ahrs_bytes).expect("valid frame");
        assert_eq!(
            decode_track_record(&ahrs_frame),
            Err(NucleusRecordError::UnexpectedRecordId { found: AHRS_ID })
        );
        let mut foreign = ahrs_frame;
        foreign.family_id = 0x10;
        assert_eq!(
            decode_ahrs_record(&foreign),
            Err(NucleusRecordError::WrongFamily { family_id: 0x10 })
        );
    }
}
