use std::time::Duration;

mod dvl_a50_parser;
pub use crate::dvl_a50_parser::a50_parser;

// static PORT_NAME : &str = "/dev/tty.usbmodem23103";
const HEADER_SIZE: usize = 8;
const CHECKSUM_SIZE: usize = 2;
const SENTIBOARD_HEADER_CHECKSUM_POS: usize = HEADER_SIZE - CHECKSUM_SIZE;
const SENTIBOARD_TOV_POS: usize = HEADER_SIZE;
const SENTIBOARD_TOA_POS: usize = SENTIBOARD_TOV_POS + 4;
const SENTIBOARD_TOT_POS: usize = SENTIBOARD_TOA_POS + 4;
// const TIMESTAMP_LEN: usize = 4;
// const TOV_LEN: usize = 4;
// const TOA_LEN: usize = 4;
// const BUF_SIZE: usize = 512;
const BUF_SIZE: usize = 20480;

// const SENTIBOARD_MSG_ID_STIM : usize = 2; // RS422 port id: 2
// const SENTIBOARD_MSG_ID_DVL : usize = 4; // UART1 port id: 4

pub struct SentiboardMessage {
    pub sensor_id: u8,
    
    pub time_of_validity: u32,
    pub time_of_arrival: u32,
    pub time_of_transport: u32,

    pub onboard_timestamp: f64,

    pub initialized: bool,
}

pub struct SentiReader {
    port: Box<dyn serialport::SerialPort>,
    serial_buf: Vec<u8>,
    data_length: u16,
    pub sentiboard_data: Vec<u8>,
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
        let mut max_skip = BUF_SIZE;
    
        while self.serial_buf[0] as char != '^' || !(self.serial_buf[1] as char == 'B' || self.serial_buf[1] as char == 'C') {
            max_skip = max_skip - 1;
            if max_skip <= 0  {
                return false;
            }
    
            self.serial_buf.remove(0);
        }

        if self.serial_buf[1] as char == 'C' {
            self.has_onboard_timestamp = true;
            self.serial_buf[1] = 'B' as u8;
        }
    
        return true;
    }
    

    pub fn read_package(&mut self) -> bool {

        // let mut serial_buf: Vec<u8> = vec![0; BUF_SIZE];
        
        self.port.read(self.serial_buf.as_mut_slice()).expect("Found no data!");


        if !self.sync_package() {
            return false;
        }

        if !self.compare_header_checksum() {
            return false;
        }

        if self.has_onboard_timestamp {
            self.sentiboard_msg.onboard_timestamp = sentiboard_get_float64(&self.serial_buf, 8);
        }

        self.data_length = sentiboard_get_uint16(&self.serial_buf, 2);
        self.sentiboard_msg.sensor_id = self.serial_buf[4];
        self.protocol_version = self.serial_buf[5];

        self.sentiboard_msg.time_of_validity = sentiboard_get_uint32(&self.serial_buf, SENTIBOARD_TOV_POS);
        self.sentiboard_msg.time_of_arrival = sentiboard_get_uint32(&self.serial_buf, SENTIBOARD_TOA_POS);
        self.sentiboard_msg.time_of_transport = sentiboard_get_uint32(&self.serial_buf, SENTIBOARD_TOT_POS);

        self.sentiboard_data = self.serial_buf[HEADER_SIZE..(self.data_length as usize + HEADER_SIZE)].to_vec(); 


        if !self.compare_data_checksum() {
            return false;
        }

        self.sentiboard_msg.initialized = true;

        return true;
    }

}

pub fn initialize_sentireader(port_name: String, baud_rate: u32) -> SentiReader {
    let port = serialport::new(port_name, baud_rate)
    .timeout(Duration::from_millis(1000))
    .open()
    .expect("Error: ");

    let serial_buf = vec![0; BUF_SIZE];

    SentiReader {
        port: port,
        serial_buf: serial_buf,
        data_length: 0,
        // sentiboard_msg.sensor_id: 0,
        protocol_version: 0,
        has_onboard_timestamp: false,
        sentiboard_data: vec![0; BUF_SIZE],
        sentiboard_msg: SentiboardMessage {
            sensor_id: 0,
            time_of_validity: 0,
            time_of_arrival: 0,
            time_of_transport: 0,
            onboard_timestamp: 0.0,
            initialized: false,
        },
    }
}


fn compare_checksum(data: &[u8], received_checksum: u16) -> bool {
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

        for _i in 0..100 {
            let res = sentireader.read_package();
            // println!("{}: {}", i, sentireader.onboard_timestamp);
            assert_eq!(res, true);
            // println!("tov {} toa: {}", sentireader.time_of_validity, sentireader.time_of_arrival);
            println!("sensor id: {}", sentireader.sentiboard_msg.sensor_id);
        }
        // let res = sentireader.read_package();

        // println!("res: {}", res);
        // assert_eq!(res, true);

        // assert_eq!(sentireader.port_name, ""/dev/tty.usbmodem23103"");
        // assert_eq!(sentireader.baud_rate, 115200);
    }
    #[test]
    fn test_dvl_a50_parser() {
        let mut sentireader = initialize_sentireader("/dev/tty.usbmodem23103".to_string(), 115200);

        for _i in 0..100 {
            let res = sentireader.read_package();
            // println!("{}: {}", i, sentireader.onboard_timestamp);
            // assert_eq!(res, true);
            if res {
                dvl_a50_parser::a50_parser(&sentireader.sentiboard_data);
            }
            println!("Has onboard timestamp: {}", sentireader.has_onboard_timestamp);
            println!("tov {} toa: {}", sentireader.sentiboard_msg.time_of_validity, sentireader.sentiboard_msg.time_of_arrival);
            println!("sensor id: {}", sentireader.sentiboard_msg.sensor_id);
        }
    }
}
