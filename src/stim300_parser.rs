use crate::messages::IMUMessage;

// const SENTIBOARD_MSG_ID_STIM : usize = 2; // RS422 port id: 2


// pub struct IMUMessage {
//     pub angular_velocity: [f32; 3],
//     pub acceleration: [f32; 3],
// }

pub fn stim300_parser(data: &Vec<u8>) -> IMUMessage {


    println!("{:?}", data);

    IMUMessage {
        angular_velocity: [0.0, 0.0, 0.0],
        acceleration: [0.0, 0.0, 0.0],
    }
}