use std::time::Duration;
use std::io::{BufReader, Read};

pub mod messages;
pub mod dvl_a50_parser;
pub mod stim300_parser;

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

    fn compare_header_checksum(&self) -> bool {
        let header_checksum = sentiboard_get_uint16(&self.serial_buf, SENTIBOARD_HEADER_CHECKSUM_POS);

        compare_checksum(&self.serial_buf[0..6], header_checksum)
    }

    fn compare_data_checksum(&self) -> bool {
        let data_checksum = sentiboard_get_uint16(&self.serial_buf, HEADER_SIZE + self.data_length as usize);

        compare_checksum(&self.sentiboard_data, data_checksum)
    }

    fn sync_package(&mut self) -> bool {
        let mut max_skip = SENTIBOARD_MAX_SKIP;

        let mut buffer: Vec<u8> = vec![0; 2];

        self.reader.read_exact(buffer.as_mut_slice()).expect("Found no data!");


        while buffer[0] as char != '^' || !(buffer[1] as char == 'B' || buffer[1] as char == 'C') {
            max_skip = max_skip - 1;
            if max_skip <= 0  {
                return false;
            }
    
            buffer.remove(0);

            let mut byte: u8 = 0;
            self.reader.read_exact(std::slice::from_mut(&mut byte)).expect("Found no data!");
            buffer.push(byte);
        }

        if buffer[1] as char == 'C' {
            self.has_onboard_timestamp = true;
            buffer[1] = 'B' as u8;
        }

        self.serial_buf = buffer;
    
        return true;
    }
    

    pub fn read_package(&mut self) -> bool {
        
        if !self.sync_package() {
            return false;
        }

        // read rest of the header (except the first two sync bytes)
        let mut header_buffer: Vec<u8> = vec![0; HEADER_SIZE - 2];
        self.reader.read_exact(header_buffer.as_mut_slice()).expect("Found no data!");
        
        self.serial_buf.append(&mut header_buffer);


        if !self.compare_header_checksum() {
            return false;
        }

        if self.has_onboard_timestamp {
            self.sentiboard_msg.onboard_timestamp = sentiboard_get_float64(&self.serial_buf, 8);
        }

        self.data_length = sentiboard_get_uint16(&self.serial_buf, 2);
        self.sentiboard_msg.sensor_id = self.serial_buf[4];
        self.protocol_version = self.serial_buf[5];

        let mut package_buffer: Vec<u8> = vec![0; self.data_length as usize + CHECKSUM_SIZE];

        // read the rest of the package and append it to serial buffer
        self.reader.read_exact(package_buffer.as_mut_slice()).expect("Found no data!");
        self.serial_buf.append(&mut package_buffer);

        self.sentiboard_msg.time_of_validity = sentiboard_get_uint32(&self.serial_buf, SENTIBOARD_TOV_POS);
        self.sentiboard_msg.time_of_arrival = sentiboard_get_uint32(&self.serial_buf, SENTIBOARD_TOA_POS);
        self.sentiboard_msg.time_of_transport = sentiboard_get_uint32(&self.serial_buf, SENTIBOARD_TOT_POS);

        // self.sentiboard_data.resize(self.data_length as usize + HEADER_SIZE, 0);
        // self.sentiboard_msg.sensor_data.resize(self.data_length as usize - TOV_LENGTH - TOA_LENGTH - TOT_LENGTH , 0);

        self.sentiboard_data = self.serial_buf[HEADER_SIZE..(self.data_length as usize + HEADER_SIZE)].to_vec(); 

        self.sentiboard_msg.sensor_data = self.sentiboard_data[TOV_LENGTH+TOA_LENGTH+TOT_LENGTH..].to_vec();
        
        if !self.compare_data_checksum() {
            return false;
        }
        
        self.sentiboard_msg.initialized = true;

        return true;
    }

}

pub fn initialize_sentireader(port_name: String, baud_rate: u32) -> SentiReader {
    let port = serialport::new(port_name, baud_rate)
    .timeout(Duration::from_millis(10000000))
    .open()
    .expect("Error: ");

    let reader = BufReader::new(port);

    SentiReader {
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


fn compare_checksum(data: &[u8], received_checksum: u16) -> bool {
    // let calc_checksum = fletcher::calc_fletcher16(data);
    let calc_checksum = fletcher(data);
    // let received_checksum = sentiboard_get_uint16(&self.serial_buf, 6);

    if received_checksum != calc_checksum {
        println!("Checksums incorrect! Expected: {}, Received: {}", calc_checksum, received_checksum);
        return false;
    }
    return true;
}

fn sentiboard_get_float64(data: &Vec<u8>, index: usize) -> f64 {
    let buf: [u8;8] = data[index..index+8].try_into().expect("slice with incorrect length");
    return f64::from_ne_bytes(buf);
}

fn sentiboard_get_uint16(data: &Vec<u8>, index: usize) -> u16 {
    let buf: [u8;2] = data[index..index+2].try_into().expect("slice with incorrect length");
    u16::from_ne_bytes(buf)
}

fn sentiboard_get_uint32(data: &Vec<u8>, index: usize) -> u32 {
    let buf: [u8;4] = data[index..index+4].try_into().expect("slice with incorrect length");
    u32::from_ne_bytes(buf)
}

fn fletcher(data: &[u8]) -> u16 {
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
        let mut sentireader = initialize_sentireader("/dev/tty.usbmodem23103".to_string(), 115200);

        for _i in 0..10000 {
            let res = sentireader.read_package();
            // println!("{}: {}", i, sentireader.onboard_timestamp);
            assert_eq!(res, true);
            println!("tov {} toa: {}", sentireader.sentiboard_msg.time_of_validity, sentireader.sentiboard_msg.time_of_arrival);
            // println!("toa: {}", sentireader.sentiboard_msg.t\ime_of_arrival);
        }
    }
}
