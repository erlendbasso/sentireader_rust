// use tokio::net::UdpSocket;
// use std::io;
use tokio::{net::UdpSocket, sync::mpsc, task};
use std::{io, net::SocketAddr, sync::Arc};
extern crate rmp_serde as rmps;

use serde::{Deserialize, Serialize};


use sentireader_rust::{
  sentireader,
  stim300_parser::{self, IMUMessage}, 
  dvl_nucleus1000_parser::{self, ExtendedDVLMessage, DataID},
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


// #[tokio::main]
// async fn main() -> io::Result<()> {
//     // let socket = UdpSocket::bind("172.16.1.170:6004").await?;
//     let socket = UdpSocket::bind("0.0.0.0:6004").await?;

//     // let mut buf = Vec::new();
//     // asdf.serialize(&mut Serializer::new(&mut buf)).unwrap();

//     let asdf = IMUData {
//         lin_accel: Vector3::zeros(),
//         ang_vel: Vector3::zeros(),
//     };

//     let buff = rmp_serde::to_vec(&asdf).unwrap();
//     let len = buff.len();
    
//     // let serialized = serde::Serialize(&asdf);
//     // let serialized = serde_json::to_string(&asdf).unwrap();

//     // let remote_addr = "127.0.0.1:6005";
//     let remote_addr = "172.16.1.169:6005";
//     socket.connect(remote_addr).await?;
//     println!("Connected to socket...");
//     // let mut buf = [0; 1024];

//     // let mut buf = [0; 1024];
//     loop {
//         // let len = socket.send_to(&buff[..len], remote_addr).await?;
//         // let len = socket.send(&buff[..len]).await?;
//         // socket.writable().await?;
//         // let len = sock.send_to(&buff[..len], remote_addr).await?;
//         // println!("{:?} bytes sent", len);
//         match socket.try_send(&buff) {
//             Ok(n) => {
//                 // break;
//                 // println!("SUCCESS {}", n);
//             }
//             Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
//                 continue;
//             }
//             Err(e) => {
//                 return Err(e);
//                 // continue;
//             }
//         }

//         // tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        
//         // rmp_serde::from_read(rd)
//         // rmp_serde::Deserializer::from_read_ref(rd)
//         // rmp_serde::from_read(rd)
//         // let (len, addr) = sock.recv_from(&mut buf).await?;
        
//         // sock.recv(&mut buf).await?;
//         // println!("{:?} bytes received from {:?}", len, addr);
//         // rmp_serde::decode::from_slice(&buf);
//         // let deserialized: TestStruct = rmp_serde::from_slice(&buff[..len]).unwrap();
//         // let deserialized: TestStruct = rmp_serde::from_slice(&buff).unwrap();
//         // println!("Deserialized: {:?}", deserialized);
//     }
//     Ok(())
// }



#[tokio::main]
async fn main() -> io::Result<()> {
    // let socket = UdpSocket::bind("172.16.1.170:6004").await?;
    let socket = UdpSocket::bind("0.0.0.0:6004").await?;

    let remote_addr = "172.16.1.170:6005";
    socket.connect(remote_addr).await?;
    println!("Connected to socket...");


  let mut sentireader =
  sentireader::SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);

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
          let (data_id, dvl_msg, _) = sentireader_rust::dvl_nucleus1000_parser::parse_nucleus_data(&sensor_data);

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

          let mut ang_vel = imu_msg.angular_velocity.unwrap(); // in deg/s
          let mut lin_accel = imu_msg.acceleration.unwrap(); // in units of g

          // ang_vel.iter_mut().for_each(|x| *x *= core::f32::consts::PI / 180.0);
          // lin_accel.iter_mut().for_each(|x| *x *= 500.0);
          ang_vel.iter_mut().for_each(|x| *x *= SAMPLE_RATE_IMU);
          lin_accel.iter_mut().for_each(|x| *x *= SAMPLE_RATE_IMU);

          for val in ang_vel.iter() {
            if val.abs() > 30.0 {
              continue;
            }
          }
          if lin_accel[2].abs() < 0.3 {
            continue;
          }


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


          // let ang_vel = R_IMU_FRD.to_rotation_matrix().matrix().transpose() * Vector3::new(ang_vel[0], ang_vel[1], ang_vel[2]);
          // let lin_accel = R_IMU_FRD.to_rotation_matrix().matrix().transpose() * Vector3::new(lin_accel[0], lin_accel[1], lin_accel[2]);

          // let ros_imu_msg = create_imu_ros_msg(&ang_vel, &lin_accel);

          // imu_pub.publish(&ros_imu_msg).unwrap();
          }
        _ => continue
      }
    }
    Ok(())
}


// const SENTIBOARD_MSG_ID_NUCLEUS: usize = 1;
// const SENTIBOARD_MSG_ID_STIM: usize = 2;
fn get_sensor_id(id: u8) -> Result<SensorID, &'static str> {
  match id {
    1 => Ok(SensorID::DVL_NUCLEUS),
    2 => Ok(SensorID::STIM300),
    4 => Ok(SensorID::DVL_A50),
    _ => Err("This Sensor ID is not supported.")?,
  }
}


// async fn senti_reader() {
//     let mut sentireader =
//     sentireader::SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);
  
//     let t_0 = std::time::Instant::now();
//     let mut t_prev = t_0;
//     let decimation_factor: u32 = 10;
  
//     // let dt = Duration::from_secs_f64(dt);
  
//     let mut coning_and_sculling = ConingAndSculling::new(decimation_factor, t_0);
  
//     // let mut counter = 0;
//     // let mut t_count = Instant::now();
  
//     loop {
//       let sentiboard_msg = sentireader.read_package().unwrap();
  
//       let sensor_data = sentiboard_msg.sensor_data.unwrap();
  
//       let sensor_id = get_sensor_id(sentiboard_msg.sensor_id.unwrap()).unwrap();
  
      
      
//       // if sensor_id == SENTIBOARD_MSG_ID_NUCLEUS as u8 {
//         // let dvl_msg: dvl_nucleus1000_parser::ExtendedDVLMessage;
//         // let altimeter_msg: dvl_nucleus1000_parser::AltimeterMessage;
  
//         match sensor_id {
//           SensorID::DVL_NUCLEUS => {
//             let (data_id, dvl_msg, _) = sentireader_rust::dvl_nucleus1000_parser::parse_nucleus_data(&sensor_data);
  
//             match data_id {
//               DataID::BottomTrackData => {
//                 // let ros_dvl_msg = create_dvl_msg(&dvl_msg);
//                 // dvl_pub.publish(&ros_dvl_msg).unwrap();
//               }
//               _ => continue
//             }
//           }
//           SensorID::STIM300 => {
//             let imu_msg =
//                       match stim300_parser::parse_stim300_data(&sensor_data) {
//                           Ok(imu_msg) => imu_msg,
//                           // Err(e) => IMUMessage {
//                           //     ..Default::default()
//                           // },
//                           Err(e) => continue,
//             };
  
//             let mut ang_vel = imu_msg.angular_velocity.unwrap(); // in deg/s
//             let mut lin_accel = imu_msg.acceleration.unwrap(); // in units of g
  
//             ang_vel.iter_mut().for_each(|x| *x *= core::f32::consts::PI / 180.0);
//             lin_accel.iter_mut().for_each(|x| *x *= G_UNIT_SCALING);
  
  
//             let t_now = std::time::Instant::now();
//             let (vel_imu, rot_vec_imu) = match coning_and_sculling.update(
//                 t_now,
//                 ang_vel,
//                 lin_accel,
//             ) {
//                 Some((vel_imu, rot_vec_imu)) => (vel_imu, rot_vec_imu),
//                 None => continue,
//             };
  
//             // counter += 1;
  
//             // if (Instant::now() - t_count).as_secs_f32() >= 10.0 {
//             //   println!("Counter: {}", counter);
//             //   counter = 0;
//             //   t_count = Instant::now();
//             // }
  
//             let dt = (t_now - t_prev).as_secs_f32();
//             t_prev = std::time::Instant::now();
  
//             let R_IMU_FRD = UnitQuaternion::new_normalize(ROT_IMU_TO_FRD);
  
//             let ang_vel = 1.0 / dt * R_IMU_FRD.to_rotation_matrix().matrix().transpose() * rot_vec_imu ;
//             let lin_accel = 1.0 / dt * R_IMU_FRD.to_rotation_matrix().matrix().transpose() * vel_imu;
  
//             let imu_data = IMUData {
//               lin_accel: lin_accel,
//               ang_vel: ang_vel,
//             };


//             // let ang_vel = R_IMU_FRD.to_rotation_matrix().matrix().transpose() * Vector3::new(ang_vel[0], ang_vel[1], ang_vel[2]);
//             // let lin_accel = R_IMU_FRD.to_rotation_matrix().matrix().transpose() * Vector3::new(lin_accel[0], lin_accel[1], lin_accel[2]);
  
//             // let ros_imu_msg = create_imu_ros_msg(&ang_vel, &lin_accel);
  
//             // imu_pub.publish(&ros_imu_msg).unwrap();
//             }
//           _ => continue
//         }
//       }
//     }
  