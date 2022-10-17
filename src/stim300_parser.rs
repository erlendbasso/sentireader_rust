use crate::messages::IMUMessage;

pub fn stim300_parser(data: &Vec<u8>) -> IMUMessage {


    println!("{:?}", data);

    IMUMessage {
        angular_velocity: [0.0, 0.0, 0.0],
        acceleration: [0.0, 0.0, 0.0],
    }
}