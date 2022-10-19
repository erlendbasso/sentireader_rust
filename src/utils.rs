pub fn get_f64_from_byte_array(data: &Vec<u8>, index: usize) -> f64 {
    let buf: [u8; 8] = data[index..index + 8]
        .try_into()
        .expect("Slice should be of length 8.");
    return f64::from_ne_bytes(buf);
}

pub fn get_u16_from_byte_array(data: &Vec<u8>, index: usize) -> u16 {
    let buf: [u8; 2] = data[index..index + 2]
        .try_into()
        .expect("Slice should be of length 2.");
    u16::from_ne_bytes(buf)
}

pub fn get_u32_from_byte_array(data: &Vec<u8>, index: usize) -> u32 {
    println!("data: {:?}", &data[index..index + 4]);
    let buf: [u8; 4] = data[index..index + 4]
        .try_into()
        .expect("Slice should be of length 4.");
    u32::from_ne_bytes(buf)
}

pub fn get_u32_from_be_byte_array(data: &Vec<u8>, index: usize) -> u32 {
    println!("data: {:?}", &data[index..index + 4]);
    let buf: [u8; 4] = data[index..index + 4]
        .try_into()
        .expect("Slice should be of length 4.");
    u32::from_be_bytes(buf)
}

pub fn fletcher16(data: &[u8]) -> u16 {
    let mut sum1: u16 = 0;
    let mut sum2: u16 = 0;
    for i in 0..data.len() {
        sum1 = (sum1 + data[i] as u16) % 256;
        sum2 = (sum2 + sum1) % 256;
    }
    (sum2 << 8) | sum1
}
