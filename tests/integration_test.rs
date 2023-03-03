#[cfg(test)]
mod tests {
    use std::error;
    type Result<T> = std::result::Result<T, Box<dyn error::Error>>;

    use sentireader_rust::{
        dvl_a50_parser, sentireader,
        stim300_parser::{self, IMUMessage},
        dvl_nucleus1000_parser,
    };
    #[test]
    fn test_dvl_a50_parser() {
        let mut sentireader =
            sentireader::SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);

        const SENTIBOARD_MSG_ID_DVL: usize = 4; // UART1 port id: 4

        for _i in 0..10000 {
            let sentiboard_msg = sentireader.read_package().unwrap();
            println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
            println!(
                "sensor ID {:?}, tov {:?}, toa: {:?}",
                sentiboard_msg.sensor_id,
                sentiboard_msg.time_of_validity,
                sentiboard_msg.time_of_arrival
            );

            let sensor_data = sentiboard_msg.sensor_data.unwrap();

            let dvl_msg: dvl_a50_parser::DVLMessage;
            if sentiboard_msg.sensor_id.unwrap() == SENTIBOARD_MSG_ID_DVL as u8 {
                println!("data: {}", String::from_utf8_lossy(&sensor_data));
                dvl_msg = sentireader_rust::dvl_a50_parser::parse_a50_data(&sensor_data);
                println!("Vel: {:?}", dvl_msg.velocity);
            }
        }
    }

    #[test]
    fn test_dvl_nucleus_parser() {
        let mut sentireader =
            sentireader::SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);

        const SENTIBOARD_MSG_ID_NUCLEUS: usize = 1; // UART1 port id: 4

        for _i in 0..10000 {
            let sentiboard_msg = sentireader.read_package().unwrap();
            println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
            println!(
                "sensor ID {:?}, tov {:?}, toa: {:?}",
                sentiboard_msg.sensor_id,
                sentiboard_msg.time_of_validity,
                sentiboard_msg.time_of_arrival
            );

            let sensor_data = sentiboard_msg.sensor_data.unwrap();

            let dvl_msg: dvl_nucleus1000_parser::ExtendedDVLMessage;
            let altimeter_msg: dvl_nucleus1000_parser::AltimeterMessage;
            let data_id: dvl_nucleus1000_parser::DataID;

            if sentiboard_msg.sensor_id.unwrap() == SENTIBOARD_MSG_ID_NUCLEUS as u8 {
                // println!("data: {}", String::from_utf8_lossy(&sensor_data));
                (data_id, dvl_msg, altimeter_msg) = dvl_nucleus1000_parser::parse_nucleus_data(&sensor_data);
                // println!("Vel: {:?}", dvl_msg.velocity);
            }
        }
    }

    #[test]
    fn test_stim300_parser() -> Result<()> {
        let mut sentireader =
            sentireader::SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);

        //const SENTIBOARD_MSG_ID_DVL: usize = 4; // UART1 port id: 4
        let mut avg_acc: [f32; 3] = [0.0, 0.0, 0.0];
        let mut avg_ar: [f32; 3] = [0.0, 0.0, 0.0];
        let n_msgs = 1000;
        for _i in 0..n_msgs - 1 {
            let sentiboard_msg = sentireader.read_package().unwrap();
            println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
            println!(
                "sensor ID {:?}, tov {:?}, toa: {:?}",
                sentiboard_msg.sensor_id,
                sentiboard_msg.time_of_validity,
                sentiboard_msg.time_of_arrival
            );

            let imu_msg =
                match stim300_parser::parse_stim300_data(&sentiboard_msg.sensor_data.unwrap()) {
                    Ok(imu_msg) => imu_msg,
                    Err(e) => IMUMessage {
                        ..Default::default()
                    },
                };

            println!("imu_msg = {:?}", imu_msg);

            avg_acc[0] = avg_acc[0] + imu_msg.acceleration.unwrap()[0];
            avg_acc[1] = avg_acc[1] + imu_msg.acceleration.unwrap()[1];
            avg_acc[2] = avg_acc[2] + imu_msg.acceleration.unwrap()[2];

            avg_ar[0] = avg_ar[0] + imu_msg.angular_velocity.unwrap()[0];
            avg_ar[1] = avg_ar[1] + imu_msg.angular_velocity.unwrap()[1];
            avg_ar[2] = avg_ar[2] + imu_msg.angular_velocity.unwrap()[2];

            // let dvl_msg: dvl_a50_parser::DVLMessage;
            // if res && sentireader.sentiboard_msg.sensor_id == SENTIBOARD_MSG_ID_DVL as u8 {
            //     println!(
            //         "data: {}",
            //         String::from_utf8_lossy(&sentireader.sentiboard_msg.sensor_data)
            //     );
            //     dvl_msg = sentireader_rust::dvl_a50_parser::a50_parser(
            //         &sentireader.sentiboard_msg.sensor_data,
            //     );
            //     println!("Vel: {:?}", dvl_msg.velocity);
            // }
        }

        avg_acc[0] = avg_acc[0] / n_msgs as f32;
        avg_acc[1] = avg_acc[1] / n_msgs as f32;
        avg_acc[2] = avg_acc[2] / n_msgs as f32;

        avg_ar[0] = avg_ar[0] / n_msgs as f32;
        avg_ar[1] = avg_ar[1] / n_msgs as f32;
        avg_ar[2] = avg_ar[2] / n_msgs as f32;

        println!("avg_acc: {:?}, avg_ar: {:?}", avg_acc, avg_ar);
        Ok(())
    }
}
