use crate::utils::{
    get_f32_from_le_byte_array, get_i16_from_le_byte_array, get_i32_from_le_byte_array,
    get_u16_from_le_byte_array, get_u32_from_le_byte_array,
};

use anyhow::Result;

const HEADER_SIZE: usize = 6;

#[derive(Debug)]
pub struct UBXNavPvt {
    pub itow: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub valid_date: bool,
    pub valid_time: bool,
    pub fully_resolved: bool,
    pub valid_mag: bool,
    pub t_acc: u32,
    pub nano: i32,
    pub fix_type: u8,
    pub gnss_fix_ok: bool,
    pub diff_soln: bool,
    pub psm_state: u8,
    pub head_veh_valid: bool,
    pub carr_soln: u8,
    pub confirmed_avail: bool,
    pub confirmed_date: bool,
    pub confirmed_time: bool,
    pub num_sv: u8,
    pub lon: f32,
    pub lat: f32,
    pub height: f32,
    pub h_msl: f32,
    pub h_acc: f32,
    pub v_acc: f32,
    pub vel_n: f32,
    pub vel_e: f32,
    pub vel_d: f32,
    pub g_speed: i32,
    pub head_mot: f32,
    pub s_acc: u32,
    pub head_acc: u32,
    pub p_dop: u16,
    pub invalid_lat_lon_height: bool,
    pub last_correction_age: u8,
    pub head_veh: f32,
    pub mag_dec: f32,
    pub mag_acc: f32,
}

#[derive(Debug)]
pub struct UBXNavRelPosNed {
    pub version: u8,
    pub ref_station_id: u16,
    pub itow: u32,
    pub rel_pos_n: f32,        // unit [m]
    pub rel_pos_e: f32,        // unit [m]
    pub rel_pos_d: f32,        // unit [m]
    pub rel_pos_length: f32,   // unit [m]
    pub rel_pos_heading: f32,  // unit [deg]
    pub rel_pos_hpn: i8,       // unit [mm]
    pub rel_pos_hpe: i8,       // unit [mm]
    pub rel_pos_hpd: i8,       // unit [mm]
    pub rel_pos_hp_length: i8, // unit [mm]
    pub acc_n: u32,            // unit [mm]
    pub acc_e: u32,            // unit [mm]
    pub acc_d: u32,            // unit [mm]
    pub acc_length: u32,       // unit [mm]
    pub acc_heading: f32,      // unit [deg]
    pub gnss_fix_ok: bool,
    pub diff_soln: bool,
    pub rel_pos_valid: bool,
    pub carr_soln: u8,
    pub is_moving: bool,
    pub ref_pos_miss: bool,
    pub ref_obs_miss: bool,
    pub rel_pos_heading_valid: bool,
    pub rel_pos_normalized: bool,
}

#[derive(Debug)]
pub struct UBXNavHPPosECEF {
    pub version: u8,
    pub itow: u32,
    pub ecef_x: i32,
    pub ecef_y: i32,
    pub ecef_z: i32,
    pub ecef_x_hp: i8,
    pub ecef_y_hp: i8,
    pub ecef_z_hp: i8,
    pub invalid_ecef: u8,
    pub p_acc: u32,
}

#[derive(Debug)]
pub struct UBXNavHPPosLLH {
    pub version: u8,
    pub invalid_llh: u8,
    pub itow: u32,
    pub lon: i32,
    pub lat: i32,
    pub height: i32,
    pub h_msl: i32,
    pub lon_hp: i8,
    pub lat_hp: i8,
    pub height_hp: i8,
    pub h_msl_hp: i8,
    pub h_acc: u32,
    pub v_acc: u32,
}

// Covariance is in NED frame
#[derive(Debug)]
pub struct UBXNavCov {
    pub itow: u32,
    pub version: u8,
    pub pos_cov_valid: u8,
    pub vel_cov_valid: u8,
    pub pos_cov_nn: f32,
    pub pos_cov_ne: f32,
    pub pos_cov_nd: f32,
    pub pos_cov_ee: f32,
    pub pos_cov_ed: f32,
    pub pos_cov_dd: f32,
    pub vel_cov_nn: f32,
    pub vel_cov_ne: f32,
    pub vel_cov_nd: f32,
    pub vel_cov_ee: f32,
    pub vel_cov_ed: f32,
    pub vel_cov_dd: f32,
}

pub enum MessageType {
    NavPvt,
    NavRelPosNed,
    NavHPPosECEF,
    NavHPPosLLH,
    Unknown,
}

pub enum UBXMessageClass {
    Nav,
    Receiver,
    Unknown,
}

fn get_msg_class(msg_class: u8) -> UBXMessageClass {
    match msg_class {
        1 => UBXMessageClass::Nav,
        2 => UBXMessageClass::Receiver,
        _ => UBXMessageClass::Unknown,
    }
}

fn get_data_information(data_id: u8) -> MessageType {
    match data_id {
        7 => MessageType::NavPvt,
        19 => MessageType::NavHPPosECEF,
        20 => MessageType::NavHPPosLLH,
        60 => MessageType::NavRelPosNed,
        // _ => panic!("Unknown data id: {}", data_id),
        _ => MessageType::Unknown,
    }
}

pub fn get_ubx_message_class(data: &Vec<u8>) -> UBXMessageClass {
    let msg_class = data[2];
    get_msg_class(msg_class)
}

pub fn get_message_type(data: &Vec<u8>) -> MessageType {
    let msg_class = get_msg_class(data[2]);
    match msg_class {
        UBXMessageClass::Nav => {
            // println!("data_id: {}", data_id);
            get_data_information(data[3])
        }
        _ => MessageType::Unknown,
    }
}

fn compare_checksums(data: &Vec<u8>) -> Result<()> {
    let check_a = data[data.len() - 2];
    let check_b = data[data.len() - 1];

    let payload_length = get_u16_from_le_byte_array(&data, 4) as usize;

    let (ck_a, ck_b) = compute_checksum(&data[2..HEADER_SIZE + payload_length].to_vec());

    if check_a != ck_a || check_b != ck_b {
        println!("ublox checksum error");
        return Err(anyhow::anyhow!("ublox checksum error"));
    }
    Ok(())
}

pub fn decode_ubx_nav_hpposecef_msg(data: &Vec<u8>) -> Option<UBXNavHPPosECEF> {
    compare_checksums(&data).expect("ublox checksum error");

    let payload_length = get_u16_from_le_byte_array(&data, 4) as usize;
    // println!("Payload length: {}", payload_length);
    // if payload_length != 28 {
    //     println!("invalid payload length");
    //     return None; // Not enough data
    // }

    let payload = &data[6..6 + payload_length].to_vec();

    let version = payload[0];
    let itow = get_u32_from_le_byte_array(&payload, 4);
    let ecef_x = get_i32_from_le_byte_array(&payload, 8);
    let ecef_y = get_i32_from_le_byte_array(&payload, 12);
    let ecef_z = get_i32_from_le_byte_array(&payload, 16);
    let ecef_x_hp = payload[20] as i8;
    let ecef_y_hp = payload[21] as i8;
    let ecef_z_hp = payload[22] as i8;

    let invalid_ecef = payload[23];
    let p_acc = get_u32_from_le_byte_array(&payload, 24);

    Some(UBXNavHPPosECEF {
        version,
        itow,
        ecef_x,
        ecef_y,
        ecef_z,
        ecef_x_hp,
        ecef_y_hp,
        ecef_z_hp,
        invalid_ecef,
        p_acc,
    })
}

pub fn decode_ubx_nav_hpposllh_msg(data: &Vec<u8>) -> UBXNavHPPosLLH {
    compare_checksums(&data).expect("ublox checksum error");

    let payload_length = get_u16_from_le_byte_array(&data, 4) as usize;

    let payload = &data[6..6 + payload_length].to_vec();

    let version = payload[0];
    let invalid_llh = payload[1];
    let itow = get_u32_from_le_byte_array(&payload, 4);
    let lon = get_i32_from_le_byte_array(&payload, 8);
    let lat = get_i32_from_le_byte_array(&payload, 12);
    let height = get_i32_from_le_byte_array(&payload, 16);
    let h_msl = get_i32_from_le_byte_array(&payload, 20);
    let lon_hp = payload[24] as i8;
    let lat_hp = payload[25] as i8;
    let height_hp = payload[26] as i8;
    let h_msl_hp = payload[27] as i8;
    let h_acc = get_u32_from_le_byte_array(&payload, 28);
    let v_acc = get_u32_from_le_byte_array(&payload, 32);

    UBXNavHPPosLLH {
        version,
        invalid_llh,
        itow,
        lon,
        lat,
        height,
        h_msl,
        lon_hp,
        lat_hp,
        height_hp,
        h_msl_hp,
        h_acc,
        v_acc,
    }
}

pub fn decode_ubx_nav_pvt_msg(data: &Vec<u8>) -> UBXNavPvt {
    compare_checksums(&data).expect("ublox checksum error");

    let payload_length = get_u16_from_le_byte_array(&data, 4) as usize;

    let payload = &data[6..6 + payload_length].to_vec();

    let itow = get_u32_from_le_byte_array(&payload, 0);
    let year = get_u16_from_le_byte_array(&payload, 4);
    let month = payload[6];
    let day = payload[7];
    let hour = payload[8];
    let min = payload[9];
    let sec = payload[10];
    let valid = payload[11];
    let valid_date = (valid >> 0) & 1;
    let valid_time = (valid >> 1) & 1;
    let fully_resolved = (valid >> 2) & 1;
    let valid_mag = (valid >> 3) & 1;

    let t_acc = get_u32_from_le_byte_array(&payload, 12);
    let nano = get_i32_from_le_byte_array(&payload, 16);
    let fix_type = payload[20];
    let flags = payload[21];
    let gnss_fix_ok = (flags >> 0) & 1;
    let diff_soln = (flags >> 1) & 1;
    let psm_state = (flags >> 2) & 0b111;
    let head_veh_valid = (flags >> 5) & 1;
    let carr_soln = (flags >> 6) & 0b11;

    let flags2 = payload[22];
    let confirmed_avail = (flags2 >> 5) & 1;
    let confirmed_date = (flags2 >> 6) & 1;
    let confirmed_time = (flags2 >> 7) & 1;

    let num_sv = payload[23];
    let lon = (get_i32_from_le_byte_array(&payload, 24) as f32) * 1e-7;
    let lat = (get_i32_from_le_byte_array(&payload, 28) as f32) * 1e-7;
    let height = get_i32_from_le_byte_array(&payload, 32) as f32 * 1e-3;
    let h_msl = get_i32_from_le_byte_array(&payload, 36) as f32 * 1e-3;
    let h_acc = get_u32_from_le_byte_array(&payload, 40) as f32 * 1e-3;
    let v_acc = get_u32_from_le_byte_array(&payload, 44) as f32 * 1e-3;
    let vel_n = get_i32_from_le_byte_array(&payload, 48) as f32 * 1e-3;
    let vel_e = get_i32_from_le_byte_array(&payload, 52) as f32 * 1e-3;
    let vel_d = get_i32_from_le_byte_array(&payload, 56) as f32 * 1e-3;
    let g_speed = get_i32_from_le_byte_array(&payload, 60);
    let head_mot = get_i32_from_le_byte_array(&payload, 64) as f32 * 1e-5;
    let s_acc = get_u32_from_le_byte_array(&payload, 68);
    let head_acc = get_u32_from_le_byte_array(&payload, 72);
    let p_dop = get_u16_from_le_byte_array(&payload, 76);
    let flags3 = get_u16_from_le_byte_array(&payload, 78);
    let invalid_lat_lon_height = (flags3 >> 0) & 1;
    let last_correction_age = ((flags3 >> 1) & 0b1111) as u8;

    let head_veh = get_i32_from_le_byte_array(&payload, 84) as f32 * 1e-5;
    let mag_dec = get_i16_from_le_byte_array(&payload, 88) as f32 * 1e-2;
    let mag_acc = get_u16_from_le_byte_array(&payload, 90) as f32 * 1e-2;

    UBXNavPvt {
        itow,
        year,
        month,
        day,
        hour,
        min,
        sec,
        valid_date: valid_date == 1,
        valid_time: valid_time == 1,
        fully_resolved: fully_resolved == 1,
        valid_mag: valid_mag == 1,
        t_acc,
        nano,
        fix_type,
        gnss_fix_ok: gnss_fix_ok == 1,
        diff_soln: diff_soln == 1,
        psm_state,
        head_veh_valid: head_veh_valid == 1,
        carr_soln,
        confirmed_avail: confirmed_avail == 1,
        confirmed_date: confirmed_date == 1,
        confirmed_time: confirmed_time == 1,
        num_sv,
        lon,
        lat,
        height,
        h_msl,
        h_acc,
        v_acc,
        vel_n,
        vel_e,
        vel_d,
        g_speed,
        head_mot,
        s_acc,
        head_acc,
        p_dop,
        invalid_lat_lon_height: invalid_lat_lon_height == 1,
        last_correction_age,
        head_veh,
        mag_dec,
        mag_acc,
    }
}

pub fn decode_ubx_nav_relposned(data: &Vec<u8>) -> UBXNavRelPosNed {
    compare_checksums(&data).expect("ublox checksum error");

    let payload_length = get_u16_from_le_byte_array(&data, 4) as usize;

    let payload = &data[6..6 + payload_length].to_vec();

    let version = payload[0];
    let ref_station_id = get_u16_from_le_byte_array(&payload, 2);
    let itow = get_u32_from_le_byte_array(&payload, 4);
    let rel_pos_n = get_i32_from_le_byte_array(&payload, 8) as f32 * 1e-2; // unit [cm]
    let rel_pos_e = get_i32_from_le_byte_array(&payload, 12) as f32 * 1e-2; // unit [cm]
    let rel_pos_d = get_i32_from_le_byte_array(&payload, 16) as f32 * 1e-2; // unit [cm]
    let rel_pos_length = get_u32_from_le_byte_array(&payload, 20) as f32 * 1e-2; // unit [cm]
    let rel_pos_heading = (get_i32_from_le_byte_array(&payload, 24) as f32) * 1e-5; // unit [deg]
    let rel_pos_hpn = payload[32] as i8; // unit [mm]
    let rel_pos_hpe = payload[33] as i8; // unit [mm]
    let rel_pos_hpd = payload[34] as i8; // unit [mm]
    let rel_pos_hp_length = payload[35] as i8; // unit [mm]
    let acc_n = get_u32_from_le_byte_array(&payload, 36); // unit [mm]
    let acc_e = get_u32_from_le_byte_array(&payload, 40); // unit [mm]
    let acc_d = get_u32_from_le_byte_array(&payload, 44); // unit [mm]
    let acc_length = get_u32_from_le_byte_array(&payload, 48); // unit [mm]
    let acc_heading = get_u32_from_le_byte_array(&payload, 52) as f32 * 1e-5; // unit [deg]
    let flags = &payload[60..60 + 4];
    let gnss_fix_ok = (flags[0] >> 0) & 1;
    let diff_soln = (flags[0] >> 1) & 1;
    let rel_pos_valid = (flags[0] >> 2) & 1;
    let carr_soln = (flags[0] >> 3) & 0b11;
    let is_moving = (flags[0] >> 5) & 1;
    let ref_pos_miss = (flags[0] >> 6) & 1;
    let ref_obs_miss = (flags[0] >> 7) & 1;
    let rel_pos_heading_valid = (flags[1] >> 0) & 1;
    let rel_pos_normalized = (flags[1] >> 1) & 1;

    UBXNavRelPosNed {
        version,
        ref_station_id,
        itow,
        rel_pos_n,
        rel_pos_e,
        rel_pos_d,
        rel_pos_length,
        rel_pos_heading,
        rel_pos_hpn,
        rel_pos_hpe,
        rel_pos_hpd,
        rel_pos_hp_length,
        acc_n,
        acc_e,
        acc_d,
        acc_length,
        acc_heading,
        gnss_fix_ok: gnss_fix_ok == 1,
        diff_soln: diff_soln == 1,
        rel_pos_valid: rel_pos_valid == 1,
        carr_soln,
        is_moving: is_moving == 1,
        ref_pos_miss: ref_pos_miss == 1,
        ref_obs_miss: ref_obs_miss == 1,
        rel_pos_heading_valid: rel_pos_heading_valid == 1,
        rel_pos_normalized: rel_pos_normalized == 1,
    }
}

pub fn decode_ubx_nav_cov_msg(data: &Vec<u8>) -> Option<UBXNavCov> {
    compare_checksums(&data).expect("ublox checksum error");

    let payload_length = get_u16_from_le_byte_array(&data, 4) as usize;

    let payload = &data[6..6 + payload_length].to_vec();

    let itow = get_u32_from_le_byte_array(&payload, 0);
    let version = payload[4];
    let pos_cov_valid = payload[5];
    let vel_cov_valid = payload[6];
    let pos_cov_nn = get_f32_from_le_byte_array(&payload, 16) as f32;
    let pos_cov_ne = get_f32_from_le_byte_array(&payload, 20) as f32;
    let pos_cov_nd = get_f32_from_le_byte_array(&payload, 24) as f32;
    let pos_cov_ee = get_f32_from_le_byte_array(&payload, 28) as f32;
    let pos_cov_ed = get_f32_from_le_byte_array(&payload, 32) as f32;
    let pos_cov_dd = get_f32_from_le_byte_array(&payload, 36) as f32;
    let vel_cov_nn = get_f32_from_le_byte_array(&payload, 40) as f32;
    let vel_cov_ne = get_f32_from_le_byte_array(&payload, 44) as f32;
    let vel_cov_nd = get_f32_from_le_byte_array(&payload, 48) as f32;
    let vel_cov_ee = get_f32_from_le_byte_array(&payload, 52) as f32;
    let vel_cov_ed = get_f32_from_le_byte_array(&payload, 56) as f32;
    let vel_cov_dd = get_f32_from_le_byte_array(&payload, 60) as f32;

    Some(UBXNavCov {
        itow,
        version,
        pos_cov_valid,
        vel_cov_valid,
        pos_cov_nn,
        pos_cov_ne,
        pos_cov_nd,
        pos_cov_ee,
        pos_cov_ed,
        pos_cov_dd,
        vel_cov_nn,
        vel_cov_ne,
        vel_cov_nd,
        vel_cov_ee,
        vel_cov_ed,
        vel_cov_dd,
    })
}

/// 8 bit Fletcher checksum algorithm
fn compute_checksum(data: &Vec<u8>) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;

    for byte in data {
        ck_a = ck_a.wrapping_add(*byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    (ck_a, ck_b)
}

#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    // fn test_compute_checksum() {
    //     let data = vec![0x01, 0x02, 0x03, 0x04, 0x05];
    //     let (ck_a, ck_b) = compute_checksum(&data);
    //     assert_eq!(ck_a, 0x0f);
    //     assert_eq!(ck_b, 0x14);
    // }

    #[test]
    fn test_parse_ubx_nav_cov_message() {
        let message_bytes: Vec<u8> = vec![
            // UBX header
            0xB5, 0x62, // sync chars
            0x01, 0x36, // class, ID (NAV, COV)
            0x40, 0x00, // length (64 bytes)
            // UBX-NAV-COV message payload (with non-zero values)
            0x12, 0x34, 0x56, 0x78, // iTOW (305419896)
            // 0x01, 0x02, // cov (258)
            0x00, // version
            0x01, 0x01, // posCovValid, velCovValid
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved (9 bytes)
            0x00, 0x00, 0x00, 0x00, // posCovNN (0)
            0x00, 0x00, 0x00, 0x00, // posCovNE (0)
            0x00, 0x00, 0x00, 0x00, // posCovND (0)
            0x00, 0x00, 0x00, 0x00, // posCovEE (0)
            0x00, 0x00, 0x00, 0x00, // posCovED (0)
            0x00, 0x00, 0x00, 0x00, // posCovDD (0)
            0x00, 0x00, 0x00, 0x00, // velCovNN (0)
            0x00, 0x00, 0x00, 0x00, // velCovNE (0)
            0x00, 0x00, 0x00, 0x00, // velCovND (0)
            0x00, 0x00, 0x00, 0x00, // velCovEE (0)
            0x00, 0x00, 0x00, 0x00, // velCovED (0)
            0x00, 0x00, 0x00, 0x00, // velCovDD (0)
            // UBX checksum (placeholder - calculate the actual checksum)
            141, 19, // CK_A, CK_B
        ];
        println!("Length: {}", message_bytes.len());
        let payload_length = get_u16_from_le_byte_array(&message_bytes, 4) as usize;
        println!("Payload length: {}", payload_length);
        println!(
            "Checksums: {:?}",
            compute_checksum(&message_bytes[2..HEADER_SIZE + 64].to_vec())
        );

        let parsed_message = decode_ubx_nav_cov_msg(&message_bytes);

        match parsed_message {
            Some(nav_cov) => {
                println!("{:?}", nav_cov);
                // assert_eq!(nav_cov.itow, 305419896);
                assert_eq!(nav_cov.version, 0);
                // ... assert other fields as needed
            }
            _ => panic!("Unexpected message type"),
        }
    }
}
