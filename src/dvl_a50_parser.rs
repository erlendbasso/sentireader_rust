// const SENTIBOARD_MSG_ID_DVL : usize = 4; // UART1 port id: 4

pub struct DVLMessage {
    pub velocity: [f32; 3],
    pub valid: char,
    pub altitude: f32,
    pub figure_of_merit: f32,
    pub covariance: [f32; 9],
    pub time_of_validity: u64,
    pub time_of_transmission: u64,
    pub time: f32,
    pub status: i32,
}

pub fn parse_a50_data(data: &[u8]) -> DVLMessage {
    let string_data = String::from_utf8_lossy(data);

    // let regex = Regex::new(r"(wrz.*)").unwrap();
    // let captures = regex.captures(&string_data).unwrap();

    // let string_data_vec = captures.get(1).unwrap().as_str().split(',').collect::<Vec<&str>>();

    let string_data_vec = string_data.split(',').collect::<Vec<&str>>();

    // println!("{:?}", string_data_vec);

    let covariance_string_vec: Vec<&str> = string_data_vec[7].split(';').collect();
    let status_string_vec: Vec<&str> = string_data_vec[11].split('*').collect();

    let _crc = &data[59..63];

    DVLMessage {
        velocity: [
            string_data_vec[1].parse().unwrap(),
            string_data_vec[2].parse().unwrap(),
            string_data_vec[3].parse().unwrap(),
        ],
        valid: string_data_vec[4].as_bytes()[0] as char,
        altitude: string_data_vec[5].parse().unwrap(),
        figure_of_merit: string_data_vec[6].parse().unwrap(),
        covariance: [
            covariance_string_vec[0].parse().unwrap(),
            covariance_string_vec[1].parse().unwrap(),
            covariance_string_vec[2].parse().unwrap(),
            covariance_string_vec[3].parse().unwrap(),
            covariance_string_vec[4].parse().unwrap(),
            covariance_string_vec[5].parse().unwrap(),
            covariance_string_vec[6].parse().unwrap(),
            covariance_string_vec[7].parse().unwrap(),
            covariance_string_vec[8].parse().unwrap(),
        ],
        time_of_validity: string_data_vec[8].parse().unwrap(),
        time_of_transmission: string_data_vec[9].parse().unwrap(),
        time: string_data_vec[10].parse().unwrap(),
        status: status_string_vec[0].parse().unwrap(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    // fn test_regex() {
    //     let s: String = "��L�f�wrz,0.000,-0.001,-0.000,y,0.21,0.001,7.618684207955084e-07;-2.821287807819317e-07;-5.334814900948004e-08;-2.821287807819317e-07;7.512716706514766e-07;4.705292511175685e-08;-5.334814900948004e-08;4.705292511175685e-08;5.9216439751708094e-08,1550144842797701,1550144842962146,80.86,0*15".to_string();

    //     let re = Regex::new(r"(wrz.*)").unwrap();
    //     let captures = re.captures(&s).unwrap();

    //     let data_str = captures.get(1).unwrap().as_str().split(',').collect::<Vec<&str>>();
    //     // println!("data_str: {:?}", data_str);
    //     // println!("re: {}", captures.get(1).unwrap().as_str());

    //     assert_eq!("wrz",data_str[0]);
    // }
}
