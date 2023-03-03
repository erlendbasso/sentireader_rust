use crate::utils::*;
use std::error;
use std::io::{BufReader, Read};
use std::time::Duration;

type Result<T> = std::result::Result<T, Box<dyn error::Error>>;

const HEADER_SIZE: usize = 8;
const CHECKSUM_SIZE: usize = 2;
const SENTIBOARD_HEADER_CHECKSUM_POS: usize = HEADER_SIZE - CHECKSUM_SIZE;
const SENTIBOARD_TOV_POS: usize = HEADER_SIZE;
const SENTIBOARD_TOA_POS: usize = SENTIBOARD_TOV_POS + 4;
const SENTIBOARD_TOT_POS: usize = SENTIBOARD_TOA_POS + 4;
// const TIMESTAMP_LEN: usize = 4;
const TOV_LENGTH: usize = 4;
const TOA_LENGTH: usize = 4;
const TOT_LENGTH: usize = 4;
const BUF_SIZE: usize = 512;
const SENTIBOARD_MAX_SKIP: usize = 512;

#[derive(Clone)]
pub struct SentiboardMessage {
    pub sensor_id: Option<u8>,

    pub time_of_validity: Option<u32>,
    pub time_of_arrival: Option<u32>,
    pub time_of_transport: Option<u32>,

    pub onboard_timestamp: Option<f64>,

    pub sensor_data: Option<Vec<u8>>,

    pub initialized: Option<bool>,
}

pub struct SentiReader {
    // port: Box<dyn serialport::SerialPort>,
    reader: BufReader<Box<dyn serialport::SerialPort>>,
    serial_buf: Vec<u8>,
    data_length: u16,
    sentiboard_data: Vec<u8>,
    protocol_version: u8,
    has_onboard_timestamp: bool,
}

impl SentiReader {
    pub fn new(port_name: String, baud_rate: u32) -> SentiReader {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_secs_f32(1000.0))
            .open()
            .expect("Port should have opened.");

        let reader = BufReader::new(port);

        Self {
            reader: reader,
            serial_buf: vec![0; BUF_SIZE],
            data_length: 0,
            protocol_version: 0,
            has_onboard_timestamp: false,
            sentiboard_data: vec![0; BUF_SIZE],
        }
    }

    fn compare_header_checksum(&self) -> Result<()> {
        let header_checksum =
            get_u16_from_byte_array(&self.serial_buf, SENTIBOARD_HEADER_CHECKSUM_POS);

        compare_checksum(
            &self.serial_buf[0..SENTIBOARD_HEADER_CHECKSUM_POS],
            header_checksum,
        )
    }

    fn compare_data_checksum(&self) -> Result<()> {
        let data_checksum =
            get_u16_from_byte_array(&self.serial_buf, HEADER_SIZE + self.data_length as usize);

        compare_checksum(&self.sentiboard_data, data_checksum)
    }

    fn sync_package(&mut self) -> Result<()> {
        let mut max_skip = SENTIBOARD_MAX_SKIP;

        let mut buffer: Vec<u8> = vec![0; 2];

        self.reader
            .read_exact(buffer.as_mut_slice())
            .expect("Should have read two bytes here.");

        while buffer[0] as char != '^' || !(buffer[1] as char == 'B' || buffer[1] as char == 'C') {
            max_skip = max_skip - 1;
            if max_skip <= 0 {
                Err("Negative max_skip.")?;
            }

            buffer.remove(0);

            let mut byte: u8 = 0;
            self.reader
                .read_exact(std::slice::from_mut(&mut byte))
                .expect("Should have read a byte here.");
            buffer.push(byte);
        }

        if buffer[1] as char == 'C' {
            self.has_onboard_timestamp = true;
            buffer[1] = 'B' as u8;
        }

        self.serial_buf = buffer;

        Ok(())
    }

    pub fn read_package(&mut self) -> Result<SentiboardMessage> {
        self.sync_package()?;

        let mut sentiboard_msg: SentiboardMessage = SentiboardMessage {
            sensor_id: None,
            time_of_validity: None,
            time_of_arrival: None,
            time_of_transport: None,
            onboard_timestamp: None,
            sensor_data: None,
            initialized: None,
        };

        // read rest of the header (except the first two sync bytes)
        let mut header_buffer: Vec<u8> = vec![0; HEADER_SIZE - 2];
        self.reader
            .read_exact(header_buffer.as_mut_slice())
            .expect("Should have read a buffer of HEADER_SIZE - 2 in length here.");

        self.serial_buf.append(&mut header_buffer);

        self.compare_header_checksum()?;

        if self.has_onboard_timestamp {
            sentiboard_msg.onboard_timestamp = Some(get_f64_from_byte_array(&self.serial_buf, 8));
        }

        self.data_length = get_u16_from_byte_array(&self.serial_buf, 2);
        sentiboard_msg.sensor_id = Some(self.serial_buf[4]);
        self.protocol_version = self.serial_buf[5];

        let mut package_buffer: Vec<u8> = vec![0; self.data_length as usize + CHECKSUM_SIZE];

        // read the rest of the package and append it to serial buffer
        self.reader
            .read_exact(package_buffer.as_mut_slice())
            .expect("Should have read a buffer of data_length + CHECKSUM_SIZE here.");
        self.serial_buf.append(&mut package_buffer);

        sentiboard_msg.time_of_validity = Some(get_u32_from_byte_array(
            &self.serial_buf,
            SENTIBOARD_TOV_POS,
        ));
        sentiboard_msg.time_of_arrival = Some(get_u32_from_byte_array(
            &self.serial_buf,
            SENTIBOARD_TOA_POS,
        ));
        sentiboard_msg.time_of_transport = Some(get_u32_from_byte_array(
            &self.serial_buf,
            SENTIBOARD_TOT_POS,
        ));

        // self.sentiboard_data.resize(self.data_length as usize + HEADER_SIZE, 0);
        // sentiboard_msg.sensor_data.resize(self.data_length as usize - TOV_LENGTH - TOA_LENGTH - TOT_LENGTH , 0);

        self.sentiboard_data =
            self.serial_buf[HEADER_SIZE..(self.data_length as usize + HEADER_SIZE)].to_vec();

        sentiboard_msg.sensor_data =
            Some(self.sentiboard_data[TOV_LENGTH + TOA_LENGTH + TOT_LENGTH..].to_vec());

        self.compare_data_checksum()?;

        Ok(sentiboard_msg)
    }
}

fn compare_checksum(data: &[u8], received_checksum: u16) -> Result<()> {
    // let calc_checksum = fletcher::calc_fletcher16(data);
    let calc_checksum = fletcher16(data);
    // let received_checksum = get_u16_from_byte_array(&self.serial_buf, 6);

    if received_checksum != calc_checksum {
        // println!(
        //     "Checksums incorrect! Expected: {}, Received: {}",
        //     calc_checksum, received_checksum
        // );
        Err("Checksum was incorrect.")?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_sentireader() {
        let mut sentireader = SentiReader::new("/dev/tty.usbmodem223103".to_string(), 115200);

        for _i in 0..100 {
            let sentiboard_msg = sentireader.read_package().unwrap();
            println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
            println!(
                "tov {:?} toa: {:?}",
                sentiboard_msg.time_of_validity, sentiboard_msg.time_of_arrival
            );
            // println!("toa: {}", sentireader.sentiboard_msg.t\ime_of_arrival);
        }
    }
}
