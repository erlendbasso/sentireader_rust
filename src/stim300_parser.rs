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

const MIN_DATA_LENGTH: usize = 0;

const G_RANGE: &str = "2g";
const GYRO_OUTPUT_TYPE: GyroOutput = GyroOutput::AngularRate;
const GYRO_OUTPUT_START_POS: usize = 1;
const GYRO_OUTPUT_END_POS: usize = 9;
const GYRO_STATUS_POS: usize = 10;
const ACCMETER_OUTPUT_TYPE: AccMeterOutput = AccMeterOutput::Acceleration;
const ACCMETER_OUTPUT_START_POS: usize = 11;
const ACCMETER_OUTPUT_END_POS: usize = 19;
const ACCMETER_STATUS_POS: usize = 20;
const OUTPUT_LENGTH: usize = 3;

#[derive(Debug)]
pub struct IMUMessage {
    pub mode: Option<u8>,
    pub angular_velocity: Option<[f32; 3]>,
    pub acceleration: Option<[f32; 3]>,
    pub gyro_status: Option<u8>,
    pub accelerometer_status: Option<u8>,
}

pub fn parse_stim300_data(data: &Vec<u8>) -> IMUMessage {
    let data_string = String::from_utf8_lossy(&data);
    let data_string_vec = data_string.split(',').collect::<Vec<&str>>();

    println!("STIM300 data: {:?}", data);

    assert!(data.len() > MIN_DATA_LENGTH);
    let data_identifier = data[0];
    assert!(data_identifier.to_string() == "0x93");

    IMUMessage {
        mode: Some(data_identifier),
        angular_velocity: Some([
            convert_gyro_output_to_angular_rate(
                &data[GYRO_OUTPUT_START_POS..GYRO_OUTPUT_START_POS + OUTPUT_LENGTH],
            ),
            convert_gyro_output_to_angular_rate(
                &data[GYRO_OUTPUT_START_POS + OUTPUT_LENGTH
                    ..GYRO_OUTPUT_START_POS + 2 * OUTPUT_LENGTH],
            ),
            convert_gyro_output_to_angular_rate(
                &data[GYRO_OUTPUT_START_POS + 2 * OUTPUT_LENGTH..GYRO_OUTPUT_END_POS + 1],
            ),
        ]),
        acceleration: Some([
            convert_accelerometer_output_to_acceleration(
                &data[ACCMETER_OUTPUT_START_POS..ACCMETER_OUTPUT_START_POS + OUTPUT_LENGTH],
            ),
            convert_accelerometer_output_to_acceleration(
                &data[ACCMETER_OUTPUT_START_POS + OUTPUT_LENGTH
                    ..ACCMETER_OUTPUT_START_POS + 2 * OUTPUT_LENGTH],
            ),
            convert_accelerometer_output_to_acceleration(
                &data[ACCMETER_OUTPUT_START_POS + 2 * OUTPUT_LENGTH..(ACCMETER_OUTPUT_END_POS - 1)],
            ),
        ]),
        gyro_status: Some(data[GYRO_STATUS_POS]),
        accelerometer_status: Some(data[ACCMETER_STATUS_POS]),
    }
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

fn get_gyro_output_divisor() -> f32 {
    let exponent = match GYRO_OUTPUT_TYPE {
        GyroOutput::AngularRate | GyroOutput::AverageAngularRate => 14.0,
        GyroOutput::IncrementalAngle => 21.0,
    };
    return f32::powf(2.0, exponent);
}

fn convert_gyro_output_to_angular_rate(output: &[u8]) -> f32 {
    let ar1: f32 = output[1].into();
    let ar2: f32 = output[2].into();
    let ar3: f32 = output[3].into();
    let ar1_msb = ((output[1] >> 7_u8) & 1_u8) as f32;
    let div = get_gyro_output_divisor();
    let base: f32 = 2.0;
    return (ar1 * base.powf(16.0) + ar2 * base.powf(8.0) + ar3 - ar1_msb * base.powf(24.0)) / div;
}

fn convert_accelerometer_output_to_acceleration(output: &[u8]) -> f32 {
    let acc1: f32 = output[1].into();
    let acc2: f32 = output[2].into();
    let acc3: f32 = output[3].into();
    let acc1_msb = ((output[1] >> 7_u8) & 1_u8) as f32;
    let div = get_accmeter_output_divisor();
    let base: f32 = 2.0;
    return (acc1 * base.powf(16.0) + acc2 * base.powf(8.0) + acc3 - acc1_msb * base.powf(24.0))
        / div;
}

fn compute_stim_checksum(data: &Vec<u8>) -> u32 {
    return 0;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stim300_parser() {
        // let data: Vec<u8> = vec![
        //     0x93, 0x00, 0x11, 0x84, 0xFF, 0x23, 0x33, 0x01, 0x15, 0x25, 0x27,
        // ];

        // let imu_msg = IMUMessage {
        //     angular_velocity: [0.003, 0.003, 0.003],
        //     acceleration: [0.003, 0.003, 0.003],
        // };

        //assert_eq!(parse_stim300_data(&data), imu_msg);
    }
}
