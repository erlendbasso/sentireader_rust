use crc::{Crc, CRC_32_MPEG_2};
use std::error;

use crate::utils::get_u32_from_be_byte_array;
type Result<T> = std::result::Result<T, Box<dyn error::Error>>;

enum GyroOutput {
    AngularRate,
    IncrementalAngle,
    AverageAngularRate,
}

enum AccMeterOutput {
    Acceleration,
    IncrementalVelocity,
    AverageAcceleration,
    IntegratedVelocity,
}

enum InclMeterOutput {
    Acceleration,
    IncrementalVelocity,
    AverageAcceleration,
    IntegratedVelocity,
}

// R: Rate, A: Acceleration, I: Inclination, T: Temperature
#[derive(Debug, PartialEq, Eq, Default)]
pub enum IMUMode {
    #[default]
    R,
    RA,
    RI,
    RAI,
    RT,
    RAT,
    RIT,
    RAIT,
}

#[derive(Debug, Default)]
pub struct IMUMessage {
    pub mode: IMUMode,
    pub angular_velocity: Option<[f32; 3]>,
    pub gyro_status: Option<u8>,
    pub gyro_temp: Option<[f32; 3]>,
    pub gyro_temp_status: Option<u8>,
    pub acceleration: Option<[f32; 3]>,
    pub accmeter_status: Option<u8>,
    pub accmeter_temp: Option<[f32; 3]>,
    pub accmeter_temp_status: Option<u8>,
    pub inclination: Option<[f32; 3]>,
    pub inclmeter_status: Option<u8>,
    pub inclmeter_temp: Option<[f32; 3]>,
    pub inclmeter_temp_status: Option<u8>,
    pub count: Option<u8>,
    pub latency: Option<f32>,
}

const CHECKSUM_ALGORITHM: Crc<u32> = Crc::<u32>::new(&CRC_32_MPEG_2);

// "OUTPUT_START_POS", "OUTPUT_END_POS", "OUTPUT_LENGTH" denote the
// byte positions and length in the input data (byte array), respectively.
// Note that the const values defined below are for the normal mode diagram (0xAF)
// such that we subtract 10 bytes per measurement missing (for inclination and temperature position indices)
const MIN_DATA_LENGTH: usize = 18;
const G_RANGE: &str = "10g";
const GYRO_OUTPUT_TYPE: GyroOutput = GyroOutput::AngularRate;
const GYRO_OUTPUT_START_POS: usize = 1;
const GYRO_STATUS_POS: usize = 10;
const GYRO_TEMP_OUTPUT_START_POS: usize = 31;
const GYRO_TEMP_STATUS_POS: usize = 37;
const ACCMETER_OUTPUT_TYPE: AccMeterOutput = AccMeterOutput::Acceleration;
const ACCMETER_OUTPUT_START_POS: usize = 11;
const ACCMETER_STATUS_POS: usize = 20;
const ACCMETER_TEMP_OUTPUT_START_POS: usize = 38;
const ACCMETER_TEMP_STATUS_POS: usize = 44;
const INCLMETER_OUTPUT_TYPE: InclMeterOutput = InclMeterOutput::Acceleration;
const INCLMETER_OUTPUT_START_POS: usize = 21;
const INCLMETER_STATUS_POS: usize = 30;
const INCLMETER_TEMP_OUTPUT_START_POS: usize = 45;
const INCLMETER_TEMP_STATUS_POS: usize = 51;
const SENSOR_AXIS_OUTPUT_BYTE_LENGTH: usize = 3;
const TEMP_OUTPUT_BYTE_LENGTH: usize = 2;

#[doc = "parse_stim300_data"]
pub fn parse_stim300_data(data: &Vec<u8>) -> Result<IMUMessage> {
    assert!(data.len() >= MIN_DATA_LENGTH); // minimum number of bytes
    let (imu_mode, data_length, num_crc_dummy_bytes) = get_data_information(data[0])?;

    let computed_checksum = compute_checksum(data, data_length, num_crc_dummy_bytes);
    let received_checksum = get_received_checksum(data, data_length);
    compare_checksums(computed_checksum, received_checksum)?;

    match imu_mode {
        IMUMode::R => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: None,
            gyro_temp_status: None,
            acceleration: None,
            accmeter_status: None,
            accmeter_temp: None,
            accmeter_temp_status: None,
            inclination: None,
            inclmeter_status: None,
            inclmeter_temp: None,
            inclmeter_temp_status: None,
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
        IMUMode::RA => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: None,
            gyro_temp_status: None,
            acceleration: compute_acceleration_vector(data),
            accmeter_status: Some(data[ACCMETER_STATUS_POS]),
            accmeter_temp: None,
            accmeter_temp_status: None,
            inclination: None,
            inclmeter_status: None,
            inclmeter_temp: None,
            inclmeter_temp_status: None,
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
        IMUMode::RI => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: None,
            gyro_temp_status: None,
            acceleration: None,
            accmeter_status: None,
            accmeter_temp: None,
            accmeter_temp_status: None,
            inclination: compute_inclination_vector(data, INCLMETER_OUTPUT_START_POS - 10),
            inclmeter_status: Some(data[INCLMETER_STATUS_POS - 10]),
            inclmeter_temp: None,
            inclmeter_temp_status: None,
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
        IMUMode::RAI => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: None,
            gyro_temp_status: None,
            acceleration: compute_acceleration_vector(data),
            accmeter_status: Some(data[ACCMETER_STATUS_POS]),
            accmeter_temp: None,
            accmeter_temp_status: None,
            inclination: compute_inclination_vector(data, INCLMETER_OUTPUT_START_POS),
            inclmeter_status: Some(data[INCLMETER_STATUS_POS]),
            inclmeter_temp: None,
            inclmeter_temp_status: None,
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
        IMUMode::RT => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: compute_temperature(data, GYRO_TEMP_OUTPUT_START_POS - 20),
            gyro_temp_status: Some(data[GYRO_TEMP_STATUS_POS - 20]),
            acceleration: None,
            accmeter_status: None,
            accmeter_temp: None,
            accmeter_temp_status: None,
            inclination: None,
            inclmeter_status: None,
            inclmeter_temp: None,
            inclmeter_temp_status: None,
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
        IMUMode::RAT => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: compute_temperature(data, GYRO_TEMP_OUTPUT_START_POS - 10),
            gyro_temp_status: Some(data[GYRO_TEMP_STATUS_POS - 10]),
            acceleration: compute_acceleration_vector(data),
            accmeter_status: Some(data[ACCMETER_STATUS_POS]),
            accmeter_temp: compute_temperature(data, ACCMETER_TEMP_OUTPUT_START_POS - 10),
            accmeter_temp_status: Some(data[ACCMETER_TEMP_STATUS_POS - 10]),
            inclination: None,
            inclmeter_status: None,
            inclmeter_temp: None,
            inclmeter_temp_status: None,
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
        IMUMode::RIT => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: compute_temperature(data, GYRO_TEMP_OUTPUT_START_POS - 10),
            gyro_temp_status: Some(data[GYRO_TEMP_STATUS_POS]),
            acceleration: None,
            accmeter_status: None,
            accmeter_temp: None,
            accmeter_temp_status: None,
            inclination: compute_inclination_vector(data, INCLMETER_OUTPUT_START_POS - 10),
            inclmeter_status: Some(data[INCLMETER_STATUS_POS - 10]),
            inclmeter_temp: compute_temperature(data, INCLMETER_TEMP_OUTPUT_START_POS),
            inclmeter_temp_status: Some(data[INCLMETER_TEMP_STATUS_POS]),
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
        IMUMode::RAIT => Ok(IMUMessage {
            mode: imu_mode,
            angular_velocity: compute_angular_rate_vector(data),
            gyro_status: Some(data[GYRO_STATUS_POS]),
            gyro_temp: compute_temperature(data, GYRO_TEMP_OUTPUT_START_POS),
            gyro_temp_status: Some(data[GYRO_TEMP_STATUS_POS]),
            acceleration: compute_acceleration_vector(data),
            accmeter_status: Some(data[ACCMETER_STATUS_POS]),
            accmeter_temp: compute_temperature(data, ACCMETER_TEMP_OUTPUT_START_POS),
            accmeter_temp_status: Some(data[ACCMETER_TEMP_STATUS_POS]),
            inclination: compute_inclination_vector(data, INCLMETER_OUTPUT_START_POS),
            inclmeter_status: Some(data[INCLMETER_STATUS_POS]),
            inclmeter_temp: compute_temperature(data, INCLMETER_TEMP_OUTPUT_START_POS),
            inclmeter_temp_status: Some(data[INCLMETER_TEMP_STATUS_POS]),
            count: Some(data[data_length - 7]),
            latency: compute_latency(&data[data_length - 6..data_length - 4]),
        }),
    }
}

fn get_data_information(data_identifier: u8) -> Result<(IMUMode, usize, usize)> {
    //! INPUTS: A byte containing information on the data package
    //! OUTPUTS: a tuple of (IMUMode, data_length, num_crc_dummy_bytes)
    match data_identifier {
        0x90 => Ok((IMUMode::R, 18, 2)),
        0x91 => Ok((IMUMode::RA, 28, 0)),
        0x92 => Ok((IMUMode::RI, 28, 0)),
        0x93 => Ok((IMUMode::RAI, 38, 2)),
        0x94 => Ok((IMUMode::RT, 25, 3)),
        0xA5 => Ok((IMUMode::RAT, 42, 2)),
        0xA6 => Ok((IMUMode::RIT, 42, 2)),
        0xA7 => Ok((IMUMode::RAIT, 59, 1)),
        _ => Err("This IMU Mode is not supported.")?,
    }
}

fn compute_angular_rate_vector(data: &Vec<u8>) -> Option<[f32; 3]> {
    Some([
        convert_gyro_output_to_angular_rate(
            &data[GYRO_OUTPUT_START_POS..GYRO_OUTPUT_START_POS + SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
        convert_gyro_output_to_angular_rate(
            &data[GYRO_OUTPUT_START_POS + SENSOR_AXIS_OUTPUT_BYTE_LENGTH
                ..GYRO_OUTPUT_START_POS + 2 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
        convert_gyro_output_to_angular_rate(
            &data[GYRO_OUTPUT_START_POS + 2 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH
                ..GYRO_OUTPUT_START_POS + 3 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
    ])
}

fn convert_gyro_output_to_angular_rate(output: &[u8]) -> f32 {
    let ar1: f32 = output[0].into();
    let ar2: f32 = output[1].into();
    let ar3: f32 = output[2].into();
    let ar1_msb = ((output[0] >> 7_u8) & 1_u8) as f32;
    let div = get_gyro_output_divisor();
    let base: f32 = 2.0;
    return (ar1 * base.powf(16.0) + ar2 * base.powf(8.0) + ar3 - ar1_msb * base.powf(24.0)) / div;
}

fn convert_accmeter_output_to_acceleration(output: &[u8]) -> f32 {
    let acc1: f32 = output[0].into();
    let acc2: f32 = output[1].into();
    let acc3: f32 = output[2].into();
    let acc1_msb = ((output[0] >> 7_u8) & 1_u8) as f32;
    let div = get_accmeter_output_divisor();
    let base: f32 = 2.0;
    return (acc1 * base.powf(16.0) + acc2 * base.powf(8.0) + acc3 - acc1_msb * base.powf(24.0))
        / div;
}

fn get_gyro_output_divisor() -> f32 {
    let exponent = match GYRO_OUTPUT_TYPE {
        GyroOutput::AngularRate | GyroOutput::AverageAngularRate => 14.0,
        GyroOutput::IncrementalAngle => 21.0,
    };
    return f32::powf(2.0, exponent);
}

fn compute_acceleration_vector(data: &Vec<u8>) -> Option<[f32; 3]> {
    Some([
        convert_accmeter_output_to_acceleration(
            &data[ACCMETER_OUTPUT_START_POS
                ..ACCMETER_OUTPUT_START_POS + SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
        convert_accmeter_output_to_acceleration(
            &data[ACCMETER_OUTPUT_START_POS + SENSOR_AXIS_OUTPUT_BYTE_LENGTH
                ..ACCMETER_OUTPUT_START_POS + 2 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
        convert_accmeter_output_to_acceleration(
            &data[ACCMETER_OUTPUT_START_POS + 2 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH
                ..ACCMETER_OUTPUT_START_POS + 3 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
    ])
}

fn get_accmeter_output_divisor() -> f32 {
    let exponent: f32 = match ACCMETER_OUTPUT_TYPE {
        AccMeterOutput::Acceleration | AccMeterOutput::AverageAcceleration => match G_RANGE {
            "5g" => 20.0,
            "10g" => 19.0,
            "30g" => 18.0,
            "80g" => 16.0,
            &_ => 20.0,
        },
        AccMeterOutput::IncrementalVelocity | AccMeterOutput::IntegratedVelocity => match G_RANGE {
            "5g" => 23.0,
            "10g" => 22.0,
            "30g" => 21.0,
            "80g" => 19.0,
            &_ => 23.0,
        },
    };
    return f32::powf(2.0, exponent);
}

fn compute_inclination_vector(data: &Vec<u8>, start_index: usize) -> Option<[f32; 3]> {
    Some([
        convert_inclmeter_output_to_inclination(
            &data[start_index..start_index + SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
        convert_inclmeter_output_to_inclination(
            &data[start_index + SENSOR_AXIS_OUTPUT_BYTE_LENGTH
                ..start_index + 2 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
        convert_inclmeter_output_to_inclination(
            &data[start_index + 2 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH
                ..start_index + 3 * SENSOR_AXIS_OUTPUT_BYTE_LENGTH],
        ),
    ])
}

fn convert_inclmeter_output_to_inclination(output: &[u8]) -> f32 {
    let acc1: f32 = output[0].into();
    let acc2: f32 = output[1].into();
    let acc3: f32 = output[2].into();
    let acc1_msb = ((output[0] >> 7_u8) & 1_u8) as f32;
    let div = get_inclmeter_output_divisor();
    let base: f32 = 2.0;
    return (acc1 * base.powf(16.0) + acc2 * base.powf(8.0) + acc3 - acc1_msb * base.powf(24.0))
        / div;
}

fn get_inclmeter_output_divisor() -> f32 {
    let exponent = match INCLMETER_OUTPUT_TYPE {
        InclMeterOutput::Acceleration | InclMeterOutput::AverageAcceleration => 22.0,
        InclMeterOutput::IncrementalVelocity | InclMeterOutput::IntegratedVelocity => 25.0,
    };
    return f32::powf(2.0, exponent);
}

fn compute_temperature(data: &Vec<u8>, start_index: usize) -> Option<[f32; 3]> {
    Some([
        convert_temp_meas_output_to_temperature(
            &data[start_index..start_index + TEMP_OUTPUT_BYTE_LENGTH],
        ),
        convert_temp_meas_output_to_temperature(
            &data[start_index + TEMP_OUTPUT_BYTE_LENGTH..start_index + 2 * TEMP_OUTPUT_BYTE_LENGTH],
        ),
        convert_temp_meas_output_to_temperature(
            &data[start_index + 2 * TEMP_OUTPUT_BYTE_LENGTH
                ..start_index + 3 * TEMP_OUTPUT_BYTE_LENGTH],
        ),
    ])
}
fn convert_temp_meas_output_to_temperature(output: &[u8]) -> f32 {
    let t1: f32 = output[0].into();
    let t2: f32 = output[1].into();
    let t1_msb = ((output[0] >> 7_u8) & 1_u8) as f32;
    let base: f32 = 2.0;
    return (t1 * base.powf(8.0) + t2 - t1_msb * base.powf(16.0)) / base.powf(8.0);
}

fn compute_latency(data: &[u8]) -> Option<f32> {
    let lt1: f32 = data[0].into();
    let lt2: f32 = data[1].into();
    let base: f32 = 2.0;
    Some(lt1 * base.powf(8.0) + lt2)
}

fn compute_checksum(data: &Vec<u8>, data_length: usize, num_crc_dummy_bytes: usize) -> u32 {
    let mut crc_data: Vec<u8> = data[..data_length - 4].iter().cloned().collect();
    crc_data.resize(crc_data.len() + num_crc_dummy_bytes, 0);
    //println!("crc_data: {:?}", crc_data);
    CHECKSUM_ALGORITHM.checksum(&crc_data.as_slice())
}

fn get_received_checksum(data: &Vec<u8>, data_length: usize) -> u32 {
    //println!("relevant data: {:?}", &data[data_length - 4..data_length]);
    get_u32_from_be_byte_array(data, data_length - 4)
}

fn compare_checksums(computed_checksum: u32, received_checksum: u32) -> Result<()> {
    assert_eq!(computed_checksum, received_checksum);
    match computed_checksum == received_checksum {
        true => Ok(()),
        false => {
            println!(
                "Error: computed_checksum {} did not match received_checksum {}",
                computed_checksum, received_checksum
            );
            Err("Computed checksum did not match received checksum.")?
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stim300_parser() {
        let data: Vec<u8> = vec![
            147, 0, 2, 143, 255, 255, 27, 255, 251, 225, 0, 255, 231, 228, 0, 24, 145, 7, 246, 137,
            0, 255, 52, 121, 0, 39, 132, 64, 23, 20, 0, 217, 1, 244, 57, 44, 117, 39,
        ];

        let imu_msg = match parse_stim300_data(&data) {
            Ok(imu_msg) => imu_msg,
            Err(e) => IMUMessage {
                ..Default::default()
            },
        };

        println!("imu_msg = {:?}", imu_msg);

        println!(
            "accl: {:?}\n ar: {:?}\n",
            imu_msg.acceleration.unwrap(),
            imu_msg.angular_velocity.unwrap()
        );

        // assert_eq!(
        //     imu_msg.acceleration.unwrap(),
        //     [-0.01177216, 0.01199532, 0.99537849]
        // );

        // assert_eq!(
        //     imu_msg.angular_velocity.unwrap(),
        //     [0.03997803, -0.01397705, -0.06439209]
        // );

        // assert_eq!(imu_msg.latency.unwrap(), 62465.0);
        // should match
        // fps: 0.000000 0	697727900	accl:[-0.01177216  0.01199532  0.99537849]
        // ar:[ 0.03997803 -0.01397705 -0.06439209]
        // latency: 62465.0

        //assert_eq!(parse_stim300_data(&data), imu_msg);
    }
}
