#[cfg(test)]
mod tests {
    #[test]
    fn test_dvl_a50_parser() {
        let mut sentireader = sentireader_rust::initialize_sentireader("/dev/tty.usbmodem23103".to_string(), 115200);

        const SENTIBOARD_MSG_ID_DVL : usize = 4; // UART1 port id: 4

        for _i in 0..10000 {
            let res = sentireader.read_package();
            // println!("{}: {}", i, sentireader.onboard_timestamp);
            // assert_eq!(res, true);
            println!("sensor id: {}", sentireader.sentiboard_msg.sensor_id);
            let dvl_msg: sentireader_rust::messages::DVLMessage;
            if res && sentireader.sentiboard_msg.sensor_id == SENTIBOARD_MSG_ID_DVL as u8 {
                println!("data: {}", String::from_utf8_lossy(&sentireader.sentiboard_msg.sensor_data));
                dvl_msg = sentireader_rust::dvl_a50_parser::a50_parser(&sentireader.sentiboard_msg.sensor_data);
                println!("Vel: {:?}", dvl_msg.velocity);

            }
            println!("tov {} toa: {}", sentireader.sentiboard_msg.time_of_validity, sentireader.sentiboard_msg.time_of_arrival);
        }
    }
}