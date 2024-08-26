use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::utils::get_f32_from_byte_array;

const HEADER_SIZE: usize = 10;

pub enum TrackMode {
    BottomTrack,
    WaterTrack,
}

pub enum DataID {
    AltimeterData,
    BottomTrackData,
    WaterTrackData,
    CurrentProfileData,
    ImuData,
    MagnetometerData,
    FieldCalibData,
    AHRSData,
    StringData,
}

pub struct DVLMessage {
    pub velocity: [f32; 3],
    pub valid: char,
    pub altitude: f32,
    pub figure_of_merit: f32,
    pub covariance: [f32; 9],
    pub time_of_validity: u64,
    pub time_of_transmission: u64,
    pub time: f32,
    pub status: i32,
}

pub struct AltimeterMessage {
    pub pressure: f32,
    pub temperature: f32,
    pub sound_speed: f32,
    pub altitude: f32,
    pub pressure_valid: bool,
    pub temperature_valid: bool,
    pub altimeter_dist_valid: bool,
    pub altimeter_quality_valid: bool,
}

pub struct ExtendedDVLMessage {
    pub velocity: [f32; 3],
    pub beams_valid: bool,
    pub velocity_valid: bool,
    pub altitude: f32,
    pub vel_beams: [f32; 3],
    pub uncertainty_beams: [f32; 3],
    pub uncertainty_vel: [f32; 3],
    pub pressure: f32,
    pub sound_speed: f32,
    pub temperature: f32,
    pub tracking_type: TrackMode,
}

pub struct AHRSMessage {
    pub roll: f32,
    pub pitch: f32,
    pub heading: f32,
    pub depth: f32,
    pub orientation: UnitQuaternion<f32>,
}

impl Default for AltimeterMessage {
    fn default() -> Self {
        AltimeterMessage {
            pressure: 0.0,
            temperature: 0.0,
            sound_speed: 0.0,
            altitude: 0.0,
            pressure_valid: false,
            temperature_valid: false,
            altimeter_dist_valid: false,
            altimeter_quality_valid: false,
        }
    }
}

impl Default for ExtendedDVLMessage {
    fn default() -> Self {
        ExtendedDVLMessage {
            velocity: [0.0, 0.0, 0.0],
            beams_valid: false,
            velocity_valid: false,
            altitude: 0.0,
            vel_beams: [0.0, 0.0, 0.0],
            uncertainty_beams: [0.0, 0.0, 0.0],
            uncertainty_vel: [0.0, 0.0, 0.0],
            pressure: 0.0,
            sound_speed: 0.0,
            temperature: 0.0,
            tracking_type: TrackMode::BottomTrack,
        }
    }
}

fn get_data_information(data_id: u8) -> DataID {
    match data_id {
        130 => DataID::ImuData,
        135 => DataID::MagnetometerData,
        139 => DataID::FieldCalibData,
        160 => DataID::StringData,
        170 => DataID::AltimeterData,
        180 => DataID::BottomTrackData,
        190 => DataID::WaterTrackData,
        192 => DataID::CurrentProfileData,
        210 => DataID::AHRSData,
        _ => panic!("Unknown data id: {}", data_id),
    }
}

pub fn get_data_id(data: &[u8]) -> DataID {
    let data_series_id = data[2];

    get_data_information(data_series_id)
}

fn remove_header_data(data: &[u8]) -> Vec<u8> {
    data[HEADER_SIZE..].to_vec()
}

// pub fn parse_nucleus_data(data: &Vec<u8>) -> (DataID, Option<ExtendedDVLMessage>, Option<AltimeterMessage>, Option<Vector3<f32>>) {
//     // println!("Size: {}", data.len());

//     // println!("Header size: {}", data[1]);
//     // println!("Data series id: {}", data[2]);

//     // altimeter data: 170
//     // bottom track data: 180
//     // water track data: 190
//     // current profile data: 192
//     // println!("Data size: {}", data[4]);
//     // println!("Data size: {}", get_u16_from_byte_array(data, 4));

//     let data_series_id = data[2];

//     let data_id = get_data_information(data_series_id);

//     match data_id {
//         DataID::BottomTrackData => {
//             (
//                 data_id,
//                 Some(parse_track_data(&data[HEADER_SIZE..].to_vec(), TrackMode::BottomTrack)),
//                 // AltimeterMessage {..Default::default() }
//                 None,
//                 None
//             )
//         },
//         DataID::WaterTrackData => {
//             (
//                 data_id,
//                 Some(parse_track_data(&data[HEADER_SIZE..].to_vec(), TrackMode::WaterTrack)),
//                 // AltimeterMessage {..Default::default() }
//                 None,
//                 None
//             )
//         },
//         DataID::AltimeterData => {
//             (
//                 data_id,
//                 // ExtendedDVLMessage { ..Default::default()},
//                 None,
//                 Some(parse_altimeter_data(&data[HEADER_SIZE..].to_vec())),
//                 None
//             )
//         },
//         DataID::MagnetometerData => {
//             (
//                 data_id,
//                 // ExtendedDVLMessage { ..Default::default()},
//                 None,
//                 None,
//                 Some(parse_magnetometer_data(&data[HEADER_SIZE..].to_vec()))
//             )
//         }
//         _ => {
//             // println!("Unknown data series id: {}", data_series_id);
//             (
//                 data_id,
//                 // ExtendedDVLMessage { ..Default::default()},
//                 // AltimeterMessage {..Default::default()}
//                 None,
//                 None,
//                 None
//             )
//         }
//     }
// }

pub fn parse_track_data(data: &[u8], data_id: DataID) -> ExtendedDVLMessage {
    let track_type = match data_id {
        DataID::BottomTrackData => TrackMode::BottomTrack,
        DataID::WaterTrackData => TrackMode::WaterTrack,
        _ => panic!("Unknown track type"),
    };

    let data = remove_header_data(data);

    let status = data[12..16].to_vec();

    let beams_valid = [status[0] & 1, (status[0] >> 1) & 1, (status[0] >> 2) & 1];

    let vel_valid = [
        (status[1] >> 1) & 1,
        (status[1] >> 2) & 1,
        (status[1] >> 3) & 1,
    ];

    ExtendedDVLMessage {
        velocity: [
            get_f32_from_byte_array(&data, 96),
            get_f32_from_byte_array(&data, 100),
            get_f32_from_byte_array(&data, 104),
        ],
        beams_valid: beams_valid[0] == 1 && beams_valid[1] == 1 && beams_valid[2] == 1,
        velocity_valid: vel_valid[0] == 1 && vel_valid[1] == 1 && vel_valid[2] == 1,
        altitude: 0.0,
        vel_beams: [
            get_f32_from_byte_array(&data, 36),
            get_f32_from_byte_array(&data, 40),
            get_f32_from_byte_array(&data, 44),
        ],
        uncertainty_beams: [
            get_f32_from_byte_array(&data, 60),
            get_f32_from_byte_array(&data, 64),
            get_f32_from_byte_array(&data, 68),
        ],
        uncertainty_vel: [
            get_f32_from_byte_array(&data, 108),
            get_f32_from_byte_array(&data, 112),
            get_f32_from_byte_array(&data, 116),
        ],
        pressure: get_f32_from_byte_array(&data, 32),
        sound_speed: get_f32_from_byte_array(&data, 24),
        temperature: get_f32_from_byte_array(&data, 28),
        tracking_type: track_type,
    }
}

pub fn parse_altimeter_data(data: &[u8]) -> AltimeterMessage {
    let data = remove_header_data(data);
    let status = data[12..16].to_vec();

    let altimeter_dist_valid = status[0] & 1;
    let altimeter_quality_valid = (status[0] >> 1) & 1;
    let pressure_valid = status[2] & 1;
    let temp_valid = (status[2] >> 1) & 1;

    AltimeterMessage {
        pressure: get_f32_from_byte_array(&data, 32),
        temperature: get_f32_from_byte_array(&data, 28),
        sound_speed: get_f32_from_byte_array(&data, 24),
        altitude: get_f32_from_byte_array(&data, 36),
        pressure_valid: matches!(pressure_valid, 1),
        temperature_valid: matches!(temp_valid, 1),
        altimeter_dist_valid: matches!(altimeter_dist_valid, 1),
        altimeter_quality_valid: matches!(altimeter_quality_valid, 1),
    }
}

pub fn parse_magnetometer_data(data: &[u8]) -> Vector3<f32> {
    let data = remove_header_data(data);
    let offset = data[1] as usize;

    let status = data[12..16].to_vec();

    let _comp_hard_iron = status[0];
    // println!("Comp hard iron: {}", _comp_hard_iron);

    Vector3::new(
        get_f32_from_byte_array(&data, offset),
        get_f32_from_byte_array(&data, offset + 4),
        get_f32_from_byte_array(&data, offset + 8),
    )
}

pub fn parse_ahrs_data(data: &[u8]) -> AHRSMessage {
    let data = remove_header_data(data);
    let offset = data[1] as usize;

    let orientation = UnitQuaternion::new_normalize(Quaternion::new(
        get_f32_from_byte_array(&data, offset + 12),
        get_f32_from_byte_array(&data, offset + 16),
        get_f32_from_byte_array(&data, offset + 20),
        get_f32_from_byte_array(&data, offset + 24),
    ));

    AHRSMessage {
        roll: get_f32_from_byte_array(&data, offset),
        pitch: get_f32_from_byte_array(&data, offset + 4),
        heading: get_f32_from_byte_array(&data, offset + 8),
        depth: get_f32_from_byte_array(&data, offset + 68),
        orientation,
    }
}
