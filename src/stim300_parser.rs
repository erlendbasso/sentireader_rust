pub struct IMUMessage {
    pub angular_velocity: [f32; 3],
    pub acceleration: [f32; 3],
    pub gyro_status: Option<u8>,
    pub accelerometer_status: Option<u8>,
}

const MIN_STIM300_DATA_LENGTH: usize = 30;

pub fn parse_stim300_data(data: &Vec<u8>) -> IMUMessage {
    let data_string = String::from_utf8_lossy(&data);
    let data_string_vec = data_string.split(',').collect::<Vec<&str>>();

    assert!(data_string_vec.len() > MIN_STIM300_DATA_LENGTH);
    let data_identifier = data_string_vec[0];

    IMUMessage {
        angular_velocity: [
            convert_gyro_output_to_angular_rate(&data_string_vec[1..4]),
            convert_gyro_output_to_angular_rate(&data_string_vec[4..7]),
            convert_gyro_output_to_angular_rate(&data_string_vec[7..10]),
        ],
        acceleration: [
            convert_gyro_output_to_angular_rate(&data_string_vec[11..14]),
            convert_gyro_output_to_angular_rate(&data_string_vec[14..17]),
            convert_gyro_output_to_angular_rate(&data_string_vec[17..20]),
        ],
        gyro_status: Some(data_string_vec[10].parse().unwrap()),
        accelerometer_status: Some(data_string_vec[20].parse().unwrap()),
    }
}

fn convert_gyro_output_to_angular_rate(output: &[&str]) -> f32 {
    let AR1: f32 = output[1].parse().unwrap();
    let AR2: f32 = output[2].parse().unwrap();
    let AR3: f32 = output[3].parse().unwrap();
    return AR1 * f32::powf(2.0, 16.0) + AR2 * f32::powf(2.0, 8.0) + AR1 / f32::powf(2.0, 14.0);
}

fn convert_accelerometer_output_to_angular_rate(output: &[&str]) -> f32 {
    let ACC1: f32 = output[1].parse().unwrap();
    let ACC2: f32 = output[2].parse().unwrap();
    let ACC3: f32 = output[3].parse().unwrap();
    return ACC1 * f32::powf(2.0, 16.0) + ACC2 * f32::powf(2.0, 8.0) + ACC3 / f32::powf(2.0, 21.0);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stim300_parser() {
        let data: Vec<u8> = vec![
            0x93, 0x00, 0x11, 0x84, 0xFF, 0x23, 0x33, 0x01, 0x15, 0x25, 0x27,
        ];

        let imu_msg = IMUMessage {
            angular_velocity: [0.003, 0.003, 0.003],
            acceleration: [0.003, 0.003, 0.003],
        };

        //assert_eq!(parse_stim300_data(&data), imu_msg);
    }
}
