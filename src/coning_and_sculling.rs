extern crate nalgebra as na;
use na::Vector3;
use std::time;

pub struct ConingAndSculling {
    pub decimation_factor: u32,
    pub sample: u32,
    pub time_prev: time::Instant,
    pub alpha: Vector3<f32>,
    pub delta_alpha: Vector3<f32>,
    pub nu: Vector3<f32>,
    pub delta_nu: Vector3<f32>,
    pub beta: Vector3<f32>,
    pub vel_scul: Vector3<f32>,
}

impl ConingAndSculling {
    pub fn new(decimation_factor: u32, time: time::Instant) -> ConingAndSculling {
        Self {
            decimation_factor: decimation_factor,
            sample: 1,
            time_prev: time,
            alpha: Vector3::new(0.0, 0.0, 0.0),
            delta_alpha: Vector3::new(0.0, 0.0, 0.0),
            nu: Vector3::new(0.0, 0.0, 0.0),
            delta_nu: Vector3::new(0.0, 0.0, 0.0),
            beta: Vector3::new(0.0, 0.0, 0.0),
            vel_scul: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn reset(&mut self, time: time::Instant) {
        self.time_prev = time;
        self.sample = 1;
        self.alpha = Vector3::new(0.0, 0.0, 0.0);
        self.delta_alpha = Vector3::new(0.0, 0.0, 0.0);
        self.nu = Vector3::new(0.0, 0.0, 0.0);
        self.delta_nu = Vector3::new(0.0, 0.0, 0.0);
        self.beta = Vector3::new(0.0, 0.0, 0.0);
    }

    pub fn update(
        &mut self,
        time: time::Instant,
        angular_velocity: [f32; 3],
        acceleration: [f32; 3],
    ) -> Option<(Vector3<f32>, Vector3<f32>)> {
        let delta_time: f32 = (time - self.time_prev).as_secs_f32();
        println!("delta_time: {} | sample: {} | ", delta_time, self.sample);

        if self.sample <= self.decimation_factor {
            self.time_prev = time;
            let alpha_prev: Vector3<f32> = self.alpha;
            let nu_prev: Vector3<f32> = self.nu;
            let delta_alpha_prev: Vector3<f32> = self.delta_alpha;
            let delta_nu_prev: Vector3<f32> = self.delta_nu;
            let beta_prev: Vector3<f32> = self.beta;

            let angle_incr: Vector3<f32> = Vector3::from(angular_velocity) * delta_time;
            let vel_incr: Vector3<f32> = Vector3::from(acceleration) * delta_time;

            self.alpha = alpha_prev + angle_incr;
            self.nu = nu_prev + vel_incr;

            let delta_beta: Vector3<f32> =
                0.5 * (alpha_prev + delta_alpha_prev / 6.0).cross_matrix() * self.delta_alpha;
            self.beta = beta_prev + delta_beta;

            let vel_scul_prev: Vector3<f32> = self.vel_scul;
            let mut first_factor: Vector3<f32> = alpha_prev.cross_matrix() * delta_alpha_prev / 6.0;
            first_factor = first_factor.cross_matrix() * self.delta_nu;

            let second_factor: Vector3<f32> =
                (nu_prev + delta_nu_prev / 6.0).cross_matrix() * self.delta_alpha;
            let delta_vel_scul: Vector3<f32> = 0.5 * (first_factor + second_factor);
            self.vel_scul = vel_scul_prev + delta_vel_scul;

            self.sample += 1;
            return None;
        } else {
            self.sample = 1;
            let vel_rot: Vector3<f32> = 0.5 * self.alpha.cross_matrix() * self.nu;
            let vel_imu: Vector3<f32> = self.nu + vel_rot + self.vel_scul;
            let rot_vec_imu: Vector3<f32> = self.alpha + self.beta;
            self.reset(self.time_prev);
            return Some((vel_imu, rot_vec_imu));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stim300_parser::IMUMessage;
    #[test]
    fn test_coning_and_sculling() {
        let t_0 = time::Instant::now();
        let decimation_factor: u32 = 4;

        let mut coning_and_sculling = ConingAndSculling::new(decimation_factor, t_0);
        let mut incr = 0.0;
        let mut t_now = t_0;
        while t_now - t_0 < time::Duration::from_millis(1000) {
            let mut imu_msg: IMUMessage = IMUMessage::new();
            imu_msg.angular_velocity = Some([0.0, 0.0, 0.0]);
            imu_msg.acceleration = Some([incr, 0.0, 1.0]);

            t_now = time::Instant::now();
            let (vel_imu, rot_vec_imu) = match coning_and_sculling.update(
                t_now,
                imu_msg.angular_velocity.unwrap(),
                imu_msg.acceleration.unwrap(),
            ) {
                Some((vel_imu, rot_vec_imu)) => (vel_imu, rot_vec_imu),
                None => continue,
            };

            println!("vel_imu: {:?} | rot_vec_imu: {:?}", vel_imu, rot_vec_imu);

            incr += 0.005;
        }
    }
}
