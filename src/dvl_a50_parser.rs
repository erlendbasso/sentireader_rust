

// const SENTIBOARD_MSG_ID_DVL : usize = 4; // UART1 port id: 4


pub fn a50_parser(data: &Vec<u8>) {
    let string_data = String::from_utf8_lossy(&data);
    let string_data_vec: Vec<&str> = string_data.split(',').collect();
    println!("A50 data: {}", string_data);
    println!("A50 data_str_1: {}", string_data_vec[2]);


}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse() {
        assert_eq!(1,1);
    }
}
