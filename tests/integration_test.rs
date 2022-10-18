#[cfg(test)]
mod tests {

    use sentireader_rust::{dvl_a50_parser, sentireader, stim300_parser};
    #[test]
    fn test_dvl_a50_parser() {
        // let mut sentireader =
        //     sentireader::SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);

        //const SENTIBOARD_MSG_ID_DVL: usize = 4; // UART1 port id: 4

        // for _i in 0..10000 {
        //     let sentiboard_msg = sentireader.read_package().unwrap();
        //     println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
        //     println!(
        //         "sensor ID {:?}, tov {:?}, toa: {:?}",
        //         sentiboard_msg.sensor_id,
        //         sentiboard_msg.time_of_validity,
        //         sentiboard_msg.time_of_arrival
        //     );

        //     let dvl_msg: dvl_a50_parser::DVLMessage;
        //     if res && sentireader.sentiboard_msg.sensor_id == SENTIBOARD_MSG_ID_DVL as u8 {
        //         println!(
        //             "data: {}",
        //             String::from_utf8_lossy(&sentireader.sentiboard_msg.sensor_data)
        //         );
        //         dvl_msg = sentireader_rust::dvl_a50_parser::a50_parser(
        //             &sentireader.sentiboard_msg.sensor_data,
        //         );
        //         println!("Vel: {:?}", dvl_msg.velocity);
        //     }
        // }
    }

    #[test]
    fn test_stim300_parser() {
        let mut sentireader =
            sentireader::SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);

        //const SENTIBOARD_MSG_ID_DVL: usize = 4; // UART1 port id: 4

        for _i in 0..10000 {
            let sentiboard_msg = sentireader.read_package().unwrap();
            println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
            println!(
                "sensor ID {:?}, tov {:?}, toa: {:?}",
                sentiboard_msg.sensor_id,
                sentiboard_msg.time_of_validity,
                sentiboard_msg.time_of_arrival
            );

            let imu_msg = stim300_parser::parse_stim300_data(&sentiboard_msg.sensor_data.unwrap());

            println!("imu_msg = {:?}", imu_msg);
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
    }
}
