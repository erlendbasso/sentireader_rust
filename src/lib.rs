use std::error;
use std::io::{BufReader, Read};
use std::time::Duration;

type Result<T> = std::result::Result<T, Box<dyn error::Error>>;

pub mod messages;

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

pub struct SentiboardMessage {
    pub sensor_id: u8,

    pub time_of_validity: u32,
    pub time_of_arrival: u32,
    pub time_of_transport: u32,

    pub onboard_timestamp: f64,

    pub sensor_data: Vec<u8>,

    pub initialized: bool,
}

pub struct SentiReader {
    // port: Box<dyn serialport::SerialPort>,
    reader: BufReader<Box<dyn serialport::SerialPort>>,
    serial_buf: Vec<u8>,
    data_length: u16,
    sentiboard_data: Vec<u8>,
    protocol_version: u8,
    has_onboard_timestamp: bool,
    pub sentiboard_msg: SentiboardMessage,
}

impl SentiReader {
    fn new(port_name: String, baud_rate: u32) -> Self {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(10000000))
            .open()
            .expect("Error: Port should have opened.");

        let reader = BufReader::new(port);

        Self {
            reader: reader,
            serial_buf: vec![0; BUF_SIZE],
            data_length: 0,
            protocol_version: 0,
            has_onboard_timestamp: false,
            sentiboard_data: vec![0; BUF_SIZE],
            sentiboard_msg: SentiboardMessage {
                sensor_id: 0,
                time_of_validity: 0,
                time_of_arrival: 0,
                time_of_transport: 0,
                onboard_timestamp: 0.0,
                sensor_data: vec![0; BUF_SIZE],
                initialized: false,
            },
        }
    }

    fn sync_package(&mut self) -> Result<bool> {
        let mut max_skip = SENTIBOARD_MAX_SKIP;
        let mut buffer: Vec<u8> = vec![0; 2];

        self.reader
            .read_exact(buffer.as_mut_slice())
            .expect("Found no data!");

        while buffer[0] as char != '^' || !(buffer[1] as char == 'B' || buffer[1] as char == 'C') {
            max_skip = max_skip - 1;
            if max_skip <= 0 {
                return Err("Negative max skip.")?;
            }

            buffer.remove(0);

            let mut byte: u8 = 0;
            self.reader
                .read_exact(std::slice::from_mut(&mut byte))
                .expect("Found no data!");
            buffer.push(byte);
        }

        if buffer[1] as char == 'C' {
            self.has_onboard_timestamp = true;
            buffer[1] = 'B' as u8;
        }

        self.serial_buf = buffer;

        return Ok(true);
    }

    pub fn read_package(&mut self) -> Result<()> {
        self.sync_package()?;

        // read rest of the header (except the first two sync bytes)
        let mut header_buffer: Vec<u8> = vec![0; HEADER_SIZE - 2];
        self.reader
            .read_exact(header_buffer.as_mut_slice())
            .expect("Found no data!");

        self.serial_buf.append(&mut header_buffer);

        self.compare_header_checksum()?;

        if self.has_onboard_timestamp {
            self.sentiboard_msg.onboard_timestamp = get_f64_from_byte_array(&self.serial_buf, 8);
        }

        self.data_length = get_u16_from_byte_array(&self.serial_buf, 2);
        self.sentiboard_msg.sensor_id = self.serial_buf[4];
        self.protocol_version = self.serial_buf[5];

        let mut package_buffer: Vec<u8> = vec![0; self.data_length as usize + CHECKSUM_SIZE];

        // read the rest of the package and append it to serial buffer
        self.reader
            .read_exact(package_buffer.as_mut_slice())
            .expect("Found no data!");
        self.serial_buf.append(&mut package_buffer);

        self.sentiboard_msg.time_of_validity =
            get_u32_from_byte_array(&self.serial_buf, SENTIBOARD_TOV_POS);
        self.sentiboard_msg.time_of_arrival =
            get_u32_from_byte_array(&self.serial_buf, SENTIBOARD_TOA_POS);
        self.sentiboard_msg.time_of_transport =
            get_u32_from_byte_array(&self.serial_buf, SENTIBOARD_TOT_POS);

        // self.sentiboard_data.resize(self.data_length as usize + HEADER_SIZE, 0);
        // self.sentiboard_msg.sensor_data.resize(self.data_length as usize - TOV_LENGTH - TOA_LENGTH - TOT_LENGTH , 0);

        self.sentiboard_data =
            self.serial_buf[HEADER_SIZE..(self.data_length as usize + HEADER_SIZE)].to_vec();

        self.sentiboard_msg.sensor_data =
            self.sentiboard_data[TOV_LENGTH + TOA_LENGTH + TOT_LENGTH..].to_vec();

        self.compare_data_checksum()?;

        self.sentiboard_msg.initialized = true;

        return Ok(());
    }

    fn compare_header_checksum(&self) -> Result<()> {
        let header_checksum =
            get_u16_from_byte_array(&self.serial_buf, SENTIBOARD_HEADER_CHECKSUM_POS);

        compare_checksum(&self.serial_buf[0..6], header_checksum)
    }

    fn compare_data_checksum(&self) -> Result<()> {
        let data_checksum =
            get_u16_from_byte_array(&self.serial_buf, HEADER_SIZE + self.data_length as usize);

        compare_checksum(&self.sentiboard_data, data_checksum)
    }
}

fn compare_checksum(data: &[u8], received_checksum: u16) -> Result<()> {
    let calc_checksum = fletcher16(data);
    if received_checksum != calc_checksum {
        Err("Checksum should be equal.")?;
    }
    return Ok(());
}

fn get_f64_from_byte_array(data: &Vec<u8>, index: usize) -> f64 {
    let buf: [u8; 8] = data[index..index + 8]
        .try_into()
        .expect("Slice should have length 8");
    return f64::from_ne_bytes(buf);
}

fn get_u16_from_byte_array(data: &Vec<u8>, index: usize) -> u16 {
    let buf: [u8; 2] = data[index..index + 2]
        .try_into()
        .expect("Slice should have length 2");
    u16::from_ne_bytes(buf)
}

fn get_u32_from_byte_array(data: &Vec<u8>, index: usize) -> u32 {
    let buf: [u8; 4] = data[index..index + 4]
        .try_into()
        .expect("Slice should have length 4");
    u32::from_ne_bytes(buf)
}

// Algorithm for computing a checksum (see https://en.wikipedia.org/wiki/Fletcher%27s_checksum)
fn fletcher16(data: &[u8]) -> u16 {
    let mut sum1: u16 = 0;
    let mut sum2: u16 = 0;
    for i in 0..data.len() {
        sum1 = (sum1 + data[i] as u16) % 256;
        sum2 = (sum2 + sum1) % 256;
    }
    (sum2 << 8) | sum1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_sentireader() {
        let mut sentireader = SentiReader::new("/dev/ttySentiboard02".to_string(), 115200);

        for _i in 0..10000 {
            //println!("{}: {}", _i, sentireader.onboard_timestamp);
            match sentireader.read_package() {
                Ok(_) => println!("Read success!"),
                Err(e) => panic!("Read failed with error: {}", e),
            }

            // println!(
            //     "tov {} toa: {}",
            //     sentireader.sentiboard_msg.time_of_validity,
            //     sentireader.sentiboard_msg.time_of_arrival
            // );
            // println!("toa: {}", sentireader.sentiboard_msg.time_of_arrival);
        }
    }
}
