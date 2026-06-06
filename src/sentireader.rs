use crate::utils::*;
use std::error;
use std::io::Read;
use std::time::{Duration, SystemTime};

type Result<T> = std::result::Result<T, Box<dyn error::Error>>;

const HEADER_SIZE: usize = 8;
const CHECKSUM_SIZE: usize = 2;
const SENTIBOARD_HEADER_CHECKSUM_POS: usize = HEADER_SIZE - CHECKSUM_SIZE;
const ONBOARD_TIMESTAMP_LENGTH: usize = 8;
const TOV_LENGTH: usize = 4;
const TOA_LENGTH: usize = 4;
const TOT_LENGTH: usize = 4;
const SENTIBOARD_TIMING_LENGTH: usize = TOV_LENGTH + TOA_LENGTH + TOT_LENGTH;
const BUF_SIZE: usize = 512;
const SENTIBOARD_MAX_SKIP: usize = 512;

#[derive(Clone)]
pub struct SentiboardMessage {
    pub sensor_id: Option<u8>,

    pub time_of_validity: Option<u32>,
    pub time_of_arrival: Option<u32>,
    pub time_of_transport: Option<u32>,

    pub onboard_timestamp: Option<f64>,
    pub host_receive_time: Option<SystemTime>,

    pub sensor_data: Option<Vec<u8>>,

    pub initialized: Option<bool>,
}

pub struct SentiReader {
    reader: Box<dyn serialport::SerialPort>,
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

        Self {
            reader: port,
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

    fn sync_package(&mut self) -> Result<SystemTime> {
        let mut max_skip = SENTIBOARD_MAX_SKIP;
        let mut buffer: Vec<u8> = vec![0; 2];

        self.reader.read_exact(buffer.as_mut_slice())?;

        while buffer[0] as char != '^' || !(buffer[1] as char == 'B' || buffer[1] as char == 'C') {
            max_skip -= 1;
            if max_skip == 0 {
                Err("Zero max_skip.")?;
            }

            buffer.remove(0);

            let mut byte: u8 = 0;
            self.reader.read_exact(std::slice::from_mut(&mut byte))?;
            buffer.push(byte);
        }

        // We just observed the sync bytes; timestamp as close as we can
        let sync_time = SystemTime::now();

        self.has_onboard_timestamp = buffer[1] as char == 'C';
        if self.has_onboard_timestamp {
            buffer[1] = b'B'
        }

        self.serial_buf = buffer;

        Ok(sync_time)
    }

    pub fn read_package(&mut self) -> Result<SentiboardMessage> {
        let sync_time = self.sync_package()?;

        let mut sentiboard_msg: SentiboardMessage = SentiboardMessage {
            sensor_id: None,
            time_of_validity: None,
            time_of_arrival: None,
            time_of_transport: None,
            onboard_timestamp: None,
            host_receive_time: Some(sync_time),
            sensor_data: None,
            initialized: None,
        };

        // read rest of the header (except the first two sync bytes)
        let mut header_buffer: Vec<u8> = vec![0; HEADER_SIZE - 2];
        self.reader.read_exact(header_buffer.as_mut_slice())?;

        self.serial_buf.append(&mut header_buffer);

        self.compare_header_checksum()?;

        self.data_length = get_u16_from_byte_array(&self.serial_buf, 2);
        sentiboard_msg.sensor_id = Some(self.serial_buf[4]);
        self.protocol_version = self.serial_buf[5];

        let mut package_buffer: Vec<u8> = vec![0; self.data_length as usize + CHECKSUM_SIZE];

        // read the rest of the package and append it to serial buffer
        self.reader.read_exact(package_buffer.as_mut_slice())?;
        self.serial_buf.append(&mut package_buffer);

        let payload = parse_sentiboard_payload(
            &self.serial_buf,
            self.data_length as usize,
            self.has_onboard_timestamp,
        )?;
        sentiboard_msg.onboard_timestamp = payload.onboard_timestamp;
        sentiboard_msg.time_of_validity = Some(payload.time_of_validity);
        sentiboard_msg.time_of_arrival = Some(payload.time_of_arrival);
        sentiboard_msg.time_of_transport = Some(payload.time_of_transport);
        sentiboard_msg.sensor_data = Some(payload.sensor_data);

        self.sentiboard_data =
            self.serial_buf[HEADER_SIZE..(self.data_length as usize + HEADER_SIZE)].to_vec();

        self.compare_data_checksum()?;

        Ok(sentiboard_msg)
    }
}

struct ParsedSentiboardPayload {
    onboard_timestamp: Option<f64>,
    time_of_validity: u32,
    time_of_arrival: u32,
    time_of_transport: u32,
    sensor_data: Vec<u8>,
}

fn parse_sentiboard_payload(
    serial_buf: &[u8],
    data_length: usize,
    has_onboard_timestamp: bool,
) -> Result<ParsedSentiboardPayload> {
    let package_end = HEADER_SIZE
        .checked_add(data_length)
        .ok_or("Sentiboard data length overflow.")?;
    if serial_buf.len() < package_end {
        return Err("Sentiboard packet is shorter than its payload length.".into());
    }

    let timing_offset = if has_onboard_timestamp {
        ONBOARD_TIMESTAMP_LENGTH
    } else {
        0
    };
    let minimum_payload_length = timing_offset + SENTIBOARD_TIMING_LENGTH;
    if data_length < minimum_payload_length {
        return Err("Sentiboard payload is too short for timing fields.".into());
    }

    let onboard_timestamp =
        has_onboard_timestamp.then(|| get_f64_from_byte_array(serial_buf, HEADER_SIZE));
    let timing_start = HEADER_SIZE + timing_offset;
    let sensor_data_start = timing_start + SENTIBOARD_TIMING_LENGTH;

    Ok(ParsedSentiboardPayload {
        onboard_timestamp,
        time_of_validity: get_u32_from_byte_array(serial_buf, timing_start),
        time_of_arrival: get_u32_from_byte_array(serial_buf, timing_start + TOV_LENGTH),
        time_of_transport: get_u32_from_byte_array(
            serial_buf,
            timing_start + TOV_LENGTH + TOA_LENGTH,
        ),
        sensor_data: serial_buf[sensor_data_start..package_end].to_vec(),
    })
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

    fn test_packet_payload(has_onboard_timestamp: bool) -> (Vec<u8>, usize) {
        let mut serial_buf = vec![0; HEADER_SIZE];

        if has_onboard_timestamp {
            serial_buf.extend_from_slice(&123.5_f64.to_ne_bytes());
        }

        serial_buf.extend_from_slice(&11_u32.to_ne_bytes());
        serial_buf.extend_from_slice(&22_u32.to_ne_bytes());
        serial_buf.extend_from_slice(&33_u32.to_ne_bytes());
        serial_buf.extend_from_slice(b"sensor");

        let data_length = serial_buf.len() - HEADER_SIZE;
        (serial_buf, data_length)
    }

    #[test]
    fn parse_payload_without_onboard_timestamp_reads_timing_then_sensor_data() {
        let (serial_buf, data_length) = test_packet_payload(false);

        let payload = parse_sentiboard_payload(&serial_buf, data_length, false).unwrap();

        assert_eq!(payload.onboard_timestamp, None);
        assert_eq!(payload.time_of_validity, 11);
        assert_eq!(payload.time_of_arrival, 22);
        assert_eq!(payload.time_of_transport, 33);
        assert_eq!(payload.sensor_data, b"sensor");
    }

    #[test]
    fn parse_payload_with_onboard_timestamp_shifts_timing_and_sensor_data() {
        let (serial_buf, data_length) = test_packet_payload(true);

        let payload = parse_sentiboard_payload(&serial_buf, data_length, true).unwrap();

        assert_eq!(payload.onboard_timestamp, Some(123.5));
        assert_eq!(payload.time_of_validity, 11);
        assert_eq!(payload.time_of_arrival, 22);
        assert_eq!(payload.time_of_transport, 33);
        assert_eq!(payload.sensor_data, b"sensor");
    }

    #[test]
    fn parse_payload_rejects_truncated_onboard_timestamp_packet() {
        let mut serial_buf = vec![0; HEADER_SIZE];
        serial_buf.extend_from_slice(&123.5_f64.to_ne_bytes());

        let payload = parse_sentiboard_payload(&serial_buf, ONBOARD_TIMESTAMP_LENGTH, true);

        assert!(payload.is_err());
    }

    // #[test]
    // fn init_sentireader() {
    //     let mut sentireader = SentiReader::new("/dev/tty.usbmodem223103".to_string(), 115200);

    //     for _i in 0..100 {
    //         let sentiboard_msg = sentireader.read_package().unwrap();
    //         println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
    //         println!(
    //             "tov {:?} toa: {:?}",
    //             sentiboard_msg.time_of_validity, sentiboard_msg.time_of_arrival
    //         );
    //         // println!("toa: {}", sentireader.sentiboard_msg.t\ime_of_arrival);
    //     }
    // }
}
