// use tokio::net::UdpSocket;
// use std::io;
use tokio::{net::UdpSocket, sync::mpsc, task};
use std::{io, net::SocketAddr, sync::Arc};
extern crate rmp_serde as rmps;

use serde::{Deserialize, Serialize};


use sentireader_rust::{
  sentireader,
  stim300_parser::{self, IMUMessage}, 
  dvl_nucleus1000_parser::{self, ExtendedDVLMessage, DataID, get_data_id},
//   coning_and_sculling::{self, ConingAndSculling}
};

use coning_and_sculling::{self, coning_and_sculling::ConingAndSculling};

extern crate nalgebra as na;
use na::{SMatrix, Vector3, UnitQuaternion, Quaternion};

const SAMPLE_RATE_IMU: f32 = 500.0;
const G_UNIT_SCALING: f32 = 9.80665;
// const ROT_IMU_TO_FRD: Quaternion<f32> = Quaternion::<f32>::new(1.0, 1.0, 0.0, 0.0); // 90 deg pos. rotation around x-axis
const ROT_IMU_TO_FRD: Quaternion<f32> = Quaternion::<f32>::new(0.0, 1.0, 0.0, 0.0); 

pub enum SensorID {
    STIM300,
    DVL_NUCLEUS,
    DVL_A50,
  }
  

#[derive(Serialize, Deserialize, Debug)]
struct IMUData {
    pub lin_accel: Vector3<f32>,
    pub ang_vel: Vector3<f32>,
}

#[derive(Serialize, Deserialize, Debug)]
struct SentiReaderConfig {
  pub remote_ip: String,
  pub remote_port: f64,
  pub port: f64,
}


#[tokio::main]
async fn main() -> io::Result<()> {
    let f = std::fs::File::open("config.yaml").expect("Could not open file.");
    let cfg: SentiReaderConfig =
        serde_yaml::from_reader(f).expect("Could not parse file.");

    let port_str = cfg.port.to_string();
    let mut addr = "0.0.0.0:".to_owned();
    addr.push_str(&port_str);
    
    // let socket = UdpSocket::bind("172.16.1.170:6004").await?;
    let socket = UdpSocket::bind(addr).await?;

    let mut remote_addr = cfg.remote_ip;
    let remote_port = cfg.remote_port.to_string();
    remote_addr.push_str(":");
    remote_addr.push_str(&remote_port);

    // let remote_addr = "127.0.0.1:6005";
    println!("Remote addr: {}", remote_addr);
    socket.connect(remote_addr).await?;
    println!("Connected to socket...");


  let mut sentireader =
  sentireader::SentiReader::new("/dev/tty.usbmodem212203".to_string(), 115200);

  let t_0 = std::time::Instant::now();
  let mut t_prev = t_0;
  let decimation_factor: u32 = 5;

  // let dt = Duration::from_secs_f64(dt);

  let mut coning_and_sculling = ConingAndSculling::new(decimation_factor, t_0);

  // let mut counter = 0;
  // let mut t_count = Instant::now();

  loop {
    let sentiboard_msg = sentireader.read_package().unwrap();

    let sensor_data = sentiboard_msg.sensor_data.unwrap();

    let sensor_id = get_sensor_id(sentiboard_msg.sensor_id.unwrap()).unwrap();

    
    
    // if sensor_id == SENTIBOARD_MSG_ID_NUCLEUS as u8 {
      // let dvl_msg: dvl_nucleus1000_parser::ExtendedDVLMessage;
      // let altimeter_msg: dvl_nucleus1000_parser::AltimeterMessage;

      match sensor_id {
        SensorID::DVL_NUCLEUS => {
          let data_id = get_data_id(&sensor_data);

          match data_id {
            DataID::BottomTrackData => {
              // let ros_dvl_msg = create_dvl_msg(&dvl_msg);
              // dvl_pub.publish(&ros_dvl_msg).unwrap();
            }
            _ => continue
          }
        }
        SensorID::STIM300 => {
          let imu_msg =
                    match stim300_parser::parse_stim300_data(&sensor_data) {
                        Ok(imu_msg) => imu_msg,
                        // Err(e) => IMUMessage {
                        //     ..Default::default()
                        // },
                        Err(e) => continue,
          };
          // println!("IMU message: {:?}", imu_msg.mode);

          let mut ang_vel = imu_msg.angular_velocity.unwrap(); // in deg/s
          let mut lin_accel = imu_msg.acceleration.unwrap(); // in units of g

          // ang_vel.iter_mut().for_each(|x| *x *= core::f32::consts::PI / 180.0);
          // lin_accel.iter_mut().for_each(|x| *x *= 500.0);
          ang_vel.iter_mut().for_each(|x| *x *= SAMPLE_RATE_IMU);
          lin_accel.iter_mut().for_each(|x| *x *= SAMPLE_RATE_IMU);

          let t_now = std::time::Instant::now();
          let (vel_imu, rot_vec_imu) = match coning_and_sculling.update(
              t_now,
              ang_vel,
              lin_accel,
          ) { 
              Some((vel_imu, rot_vec_imu)) => (vel_imu, rot_vec_imu),
              None => continue,
          };

          // counter += 1;

          // if (Instant::now() - t_count).as_secs_f32() >= 10.0 {
          //   println!("Counter: {}", counter);
          //   counter = 0;
          //   t_count = Instant::now();
          // }

          let dt = (t_now - t_prev).as_secs_f32();
          t_prev = std::time::Instant::now();

          let R_IMU_FRD: UnitQuaternion<f32> = UnitQuaternion::<f32>::from_axis_angle(
            &Vector3::z_axis(),
            core::f32::consts::PI / 180.0 * (90.0))
            * UnitQuaternion::<f32>::from_axis_angle(
              &Vector3::x_axis(),
              core::f32::consts::PI / 180.0 * (93.0));
    
          // let R_IMU_FRD = UnitQuaternion::new_normalize(ROT_IMU_TO_FRD);
          // let R_IMU_FRD = ROT_IMU_TO_FRD;

          let ang_vel = 1.0 / dt * R_IMU_FRD.to_rotation_matrix().matrix() * rot_vec_imu ;
          let lin_accel = 1.0 / dt * R_IMU_FRD.to_rotation_matrix().matrix() * vel_imu;
          // let ang_vel = R_IMU_FRD.to_rotation_matrix().matrix() * Vector3::new(ang_vel[0], ang_vel[1], ang_vel[2]);
          // let lin_accel = R_IMU_FRD.to_rotation_matrix().matrix() * Vector3::new(lin_accel[0], lin_accel[1], lin_accel[2]);

          let imu_data = IMUData {
            lin_accel: lin_accel,
            ang_vel: ang_vel,
          };

          let serialized = rmp_serde::to_vec(&imu_data).unwrap();
          let len = serialized.len();

          match socket.try_send(&serialized) {
            Ok(n) => {
                // break;
                // println!("SUCCESS {}", n);
            }
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                continue;
            }
            Err(e) => {
                return Err(e);
                // continue;
            }
        }
          }
        _ => continue
      }
    }
    Ok(())
}

fn get_sensor_id(id: u8) -> Result<SensorID, &'static str> {
  match id {
    1 => Ok(SensorID::DVL_NUCLEUS),
    2 => Ok(SensorID::STIM300),
    4 => Ok(SensorID::DVL_A50),
    _ => Err("This Sensor ID is not supported.")?,
  }
}
