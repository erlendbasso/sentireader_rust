
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

pub struct IMUMessage {
    pub angular_velocity: [f32; 3],
    pub acceleration: [f32; 3],
}