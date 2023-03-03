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


pub fn parse_nucleus_data(data: &Vec<u8>) -> (DataID, ExtendedDVLMessage, AltimeterMessage) {
    // println!("Size: {}", data.len());

    // println!("Header size: {}", data[1]);
    // println!("Data series id: {}", data[2]);
    
    // altimeter data: 170
    // bottom track data: 180
    // water track data: 190
    // current profile data: 192
    // println!("Data size: {}", data[4]);
    // println!("Data size: {}", get_u16_from_byte_array(data, 4));

    let data_series_id = data[2];

    let data_id = get_data_information(data_series_id);

    match data_id {
        DataID::BottomTrackData => {
            (
                data_id,
                parse_track_data(&data[HEADER_SIZE..].to_vec(), TrackMode::BottomTrack), 
                AltimeterMessage {..Default::default() }
            )
        },
        DataID::WaterTrackData => {
            (
                data_id,
                parse_track_data(&data[HEADER_SIZE..].to_vec(), TrackMode::WaterTrack), 
                AltimeterMessage {..Default::default() }
            )
        },
        DataID::AltimeterData => {
            (
                data_id,
                ExtendedDVLMessage { ..Default::default()}, 
                parse_altimeter_data(&data[HEADER_SIZE..].to_vec())
            )
        },
        _ => {
            // println!("Unknown data series id: {}", data_series_id);
            (
                data_id,
                ExtendedDVLMessage { ..Default::default()}, 
                AltimeterMessage {..Default::default()}
            )
        }
    }
}

fn parse_track_data(data: &Vec<u8>, track_type: TrackMode) -> ExtendedDVLMessage {

    let status = data[12..16].to_vec();

    let beams_valid = [ 
        (status[0] >> 0) & 1,  
        (status[0] >> 1) & 1, 
        (status[0] >> 2) & 1
        ];

    let vel_valid = [
        (status[1] >> 1) & 1, 
        (status[1] >> 2) & 1, 
        (status[1] >> 3) & 1 
        ];


    ExtendedDVLMessage {
        velocity: [get_f32_from_byte_array(data, 96), get_f32_from_byte_array(data, 100), get_f32_from_byte_array(data, 104)],
        beams_valid: beams_valid[0] == 1 && beams_valid[1] == 1 && beams_valid[2] == 1,
        velocity_valid: vel_valid[0] == 1 && vel_valid[1] == 1 && vel_valid[2] == 1,
        altitude: 0.0,
        vel_beams: [get_f32_from_byte_array(data, 36), get_f32_from_byte_array(data, 40), get_f32_from_byte_array(data, 44)],
        uncertainty_beams: [get_f32_from_byte_array(data, 60), get_f32_from_byte_array(data, 64), get_f32_from_byte_array(data, 68)],
        uncertainty_vel: [get_f32_from_byte_array(data, 108), get_f32_from_byte_array(data, 112), get_f32_from_byte_array(data, 116)],
        pressure: get_f32_from_byte_array(data, 32),
        sound_speed: get_f32_from_byte_array(data, 24),
        temperature: get_f32_from_byte_array(data, 28),
        tracking_type: track_type,
    }
}

fn parse_altimeter_data(data: &Vec<u8>) -> AltimeterMessage {
    let status = data[12..16].to_vec();

    let altimeter_dist_valid = (status[0] >> 0) & 1;
    let altimeter_quality_valid = (status[0] >> 1) & 1;
    let pressure_valid = (status[2] >> 0) & 1;
    let temp_valid = (status[2] >> 1) & 1;

    AltimeterMessage {
        pressure: get_f32_from_byte_array(data, 32),
        temperature: get_f32_from_byte_array(data, 28),
        sound_speed: get_f32_from_byte_array(data, 24),
        altitude: get_f32_from_byte_array(data, 36),
        pressure_valid: match pressure_valid {
            1 => true,
            _ => false,
        },
        temperature_valid: match temp_valid {
            1 => true,
            _ => false,
        },
        altimeter_dist_valid: match altimeter_dist_valid {
            1 => true,
            _ => false,
        },
        altimeter_quality_valid: match altimeter_quality_valid {
            1 => true,
            _ => false,
        },
    }
}