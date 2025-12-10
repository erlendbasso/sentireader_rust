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
const MIN_DATA_LENGTH: usize = TOV_LENGTH + TOA_LENGTH + TOT_LENGTH;

fn slice_to_array<const N: usize>(data: &[u8], index: usize) -> Result<[u8; N]> {
    let end = index
        .checked_add(N)
        .ok_or("Index overflow when slicing bytes.")?;
    let slice = data
        .get(index..end)
        .ok_or("Not enough bytes available for requested slice.")?;

    let arr: [u8; N] = slice
        .try_into()
        .map_err(|_| "Slice length mismatch when converting array.")?;
    Ok(arr)
}

fn get_u16_checked(data: &[u8], index: usize) -> Result<u16> {
    Ok(u16::from_ne_bytes(slice_to_array::<2>(data, index)?))
}

fn get_u32_checked(data: &[u8], index: usize) -> Result<u32> {
    Ok(u32::from_ne_bytes(slice_to_array::<4>(data, index)?))
}

fn get_f64_checked(data: &[u8], index: usize) -> Result<f64> {
    Ok(f64::from_ne_bytes(slice_to_array::<8>(data, index)?))
}

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
    pub fn new(port_name: String, baud_rate: u32) -> Result<SentiReader> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_secs_f32(1000.0))
            .open()?;

        let reader = BufReader::new(port);

        Ok(Self {
            reader: reader,
            serial_buf: vec![0; BUF_SIZE],
            data_length: 0,
            protocol_version: 0,
            has_onboard_timestamp: false,
            sentiboard_data: vec![0; BUF_SIZE],
        })
    }

    fn compare_header_checksum(&self) -> Result<()> {
        let header_checksum = get_u16_checked(&self.serial_buf, SENTIBOARD_HEADER_CHECKSUM_POS)?;
        let header_slice = self
            .serial_buf
            .get(0..SENTIBOARD_HEADER_CHECKSUM_POS)
            .ok_or("Header shorter than checksum position.")?;

        compare_checksum(header_slice, header_checksum)
    }

    fn compare_data_checksum(&self) -> Result<()> {
        if self.serial_buf.len()
            < HEADER_SIZE + self.data_length as usize + CHECKSUM_SIZE
        {
            Err("Not enough bytes to validate data checksum.")?;
        }

        let data_checksum =
            get_u16_checked(&self.serial_buf, HEADER_SIZE + self.data_length as usize)?;

        compare_checksum(&self.sentiboard_data, data_checksum)
    }

    fn sync_package(&mut self) -> Result<()> {
        let mut max_skip = SENTIBOARD_MAX_SKIP;

        let mut buffer: [u8; 2] = [0; 2];

        self.reader.read_exact(&mut buffer)?;

        while buffer[0] as char != '^' || !(buffer[1] as char == 'B' || buffer[1] as char == 'C')
        {
            if max_skip == 0 {
                Err("Failed to resync after skipping bytes.")?;
            }

            max_skip -= 1;

            buffer[0] = buffer[1];

            let mut byte: u8 = 0;
            self.reader.read_exact(std::slice::from_mut(&mut byte))?;
            buffer[1] = byte;
        }

        if buffer[1] as char == 'C' {
            self.has_onboard_timestamp = true;
            buffer[1] = 'B' as u8;
        }

        self.serial_buf.clear();
        self.serial_buf.extend_from_slice(&buffer);

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
        let mut header_buffer: [u8; HEADER_SIZE - 2] = [0; HEADER_SIZE - 2];
        self.reader.read_exact(header_buffer.as_mut_slice())?;

        self.serial_buf.extend_from_slice(&header_buffer);

        self.compare_header_checksum()?;

        self.data_length = get_u16_checked(&self.serial_buf, 2)?;
        sentiboard_msg.sensor_id = self.serial_buf.get(4).copied();
        self.protocol_version = *self.serial_buf.get(5).ok_or("Missing protocol version.")?;

        if (self.data_length as usize) < MIN_DATA_LENGTH {
            Err("Data length shorter than required metadata.")?;
        }

        if (HEADER_SIZE + self.data_length as usize + CHECKSUM_SIZE) > BUF_SIZE {
            Err("Data length exceeds internal buffer capacity.")?;
        }

        let mut package_buffer: Vec<u8> = vec![0; self.data_length as usize + CHECKSUM_SIZE];

        // read the rest of the package and append it to serial buffer
        self.reader.read_exact(package_buffer.as_mut_slice())?;
        self.serial_buf.extend_from_slice(&package_buffer);

        let payload_end = HEADER_SIZE + self.data_length as usize;

        if payload_end > self.serial_buf.len() {
            Err("Incomplete payload received.")?;
        }

        sentiboard_msg.time_of_validity =
            Some(get_u32_checked(&self.serial_buf, SENTIBOARD_TOV_POS)?);
        sentiboard_msg.time_of_arrival =
            Some(get_u32_checked(&self.serial_buf, SENTIBOARD_TOA_POS)?);
        sentiboard_msg.time_of_transport =
            Some(get_u32_checked(&self.serial_buf, SENTIBOARD_TOT_POS)?);

        // self.sentiboard_data.resize(self.data_length as usize + HEADER_SIZE, 0);
        // sentiboard_msg.sensor_data.resize(self.data_length as usize - TOV_LENGTH - TOA_LENGTH - TOT_LENGTH , 0);

        self.sentiboard_data = self
            .serial_buf
            .get(HEADER_SIZE..payload_end)
            .ok_or("Payload slice is out of bounds.")?
            .to_vec();

        let sensor_data_start = SENTIBOARD_TOT_POS + TOT_LENGTH;
        sentiboard_msg.sensor_data = Some(
            self.serial_buf[sensor_data_start..payload_end]
                .to_vec(),
        );

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
    fn init_sentireader() -> Result<()> {
        let mut sentireader = SentiReader::new("/dev/tty.usbmodem223103".to_string(), 115200)?;

        for _i in 0..100 {
            let sentiboard_msg = sentireader.read_package()?;
            println!("{}, msg: {:?}", _i, sentiboard_msg.onboard_timestamp);
            println!(
                "tov {:?} toa: {:?}",
                sentiboard_msg.time_of_validity, sentiboard_msg.time_of_arrival
            );
            // println!("toa: {}", sentireader.sentiboard_msg.t\ime_of_arrival);
        }

        Ok(())
    }
}
