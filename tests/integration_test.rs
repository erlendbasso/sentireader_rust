#[cfg(test)]
mod tests {
    use std::error;
    type Result<T> = std::result::Result<T, Box<dyn error::Error>>;

    use sentireader_rust::{
        dvl_a50_parser, dvl_nucleus1000_parser, sentireader,
        stim300_parser::{self, IMUMessage},
        ublox_f9p_parser,
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
                // (data_id, dvl_msg, altimeter_msg) = dvl_nucleus1000_parser::parse_nucleus_data(&sensor_data);
                let data_id = dvl_nucleus1000_parser::get_data_id(&sensor_data);

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

    #[test]
    fn test_ublox_parser() {
        let mut sentireader =
            sentireader::SentiReader::new("/dev/tty.usbmodem323103".to_string(), 115200);

        // sentireader.has_onboard_timestamp = true;

        const SENTIBOARD_MSG_ID_UBLOX_BASE: usize = 4; // UART1 port id: 4
        const SENTIBOARD_MSG_ID_UBLOX_ROVER: usize = 0; // UART1 port id: 4
        println!("test_ublox_parser");

        for _i in 0..100000 {
            let sentiboard_msg = sentireader.read_package().unwrap();
            // println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
            // println!(
            //     "sensor ID {:?}, tov {:?}, toa: {:?}",
            //     sentiboard_msg.sensor_id,
            //     sentiboard_msg.time_of_validity,
            //     sentiboard_msg.time_of_arrival
            // );

            let sensor_data = sentiboard_msg.sensor_data.unwrap();

            // let dvl_msg: dvl_nucleus1000_parser::ExtendedDVLMessage;
            // let altimeter_msg: dvl_nucleus1000_parser::AltimeterMessage;
            // let data_id: ublox_f9p_parser::DataID;

            if sentiboard_msg.sensor_id.unwrap() == SENTIBOARD_MSG_ID_UBLOX_BASE as u8
                || sentiboard_msg.sensor_id.unwrap() == SENTIBOARD_MSG_ID_UBLOX_ROVER as u8
            {
                // println!("data: {}", String::from_utf8_lossy(&sensor_data));
                // (data_id, dvl_msg, altimeter_msg) = dvl_nucleus1000_parser::parse_nucleus_data(&sensor_data);
                let msg_type = ublox_f9p_parser::get_message_type(&sensor_data);

                match msg_type {
                    ublox_f9p_parser::MessageType::NavPvt => {
                        let nav_pvt_msg = ublox_f9p_parser::decode_ubx_nav_pvt_msg(&sensor_data);
                        println!("nav_pvt_msg: {:?}", nav_pvt_msg);
                        println!("HEADING: {}", nav_pvt_msg.head_veh);
                        println!("HEADING VALID: {}", nav_pvt_msg.head_veh_valid);
                        println!("HEIGHT: {}", nav_pvt_msg.height);
                        println!("HEIGHT msl: {}", nav_pvt_msg.h_msl);
                    }
                    ublox_f9p_parser::MessageType::NavRelPosNed => {
                        let nav_rel_pos_ned_msg =
                            ublox_f9p_parser::decode_ubx_nav_relposned(&sensor_data);
                        println!("nav_rel_pos_ned_msg: {:?}", nav_rel_pos_ned_msg);
                    }
                    _ => {}
                }

                // println!("Vel: {:?}", dvl_msg.velocity);
            }
        }
    }
}
