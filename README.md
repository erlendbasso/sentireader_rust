# sentireader_rust

Rust library for the Sentiboard. Contains parsers for the Waterlinked A50 DVL, the Nortek Nucleus1000 DVL the STIM300 IMU.

The sentireader provides a struct SentiReader for parsing the data stream from the Sentiboard connected to an input serial port (typically /dev/ttySentiboard02).

Using the SentiReader, one can extract data from each sensor connected to the sentiboard, and parse the raw sensor data package using e.g. the stim300 parser or dvl parser modules. The parsers take a variable length raw byte array from the sensor as input, and return a custom message for the sensor.
