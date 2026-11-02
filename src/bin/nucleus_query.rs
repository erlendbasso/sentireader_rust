//! Bench tool: read the Nucleus 1000 configuration over a direct serial port.
//!
//! The tool stops Nucleus measurement for a few seconds and always sends
//! `START` before it exits. Do not run it during a mission.
//!
//! Usage: nucleus_query --port /dev/ttyS0 [--baud 115200] [--timeout-s 2.0] [--settle-ms 500]

use std::process::ExitCode;
use std::time::Duration;

use sentireader_rust::nucleus_command::{query_configuration, QueryOptions};

/// Upper bounds that keep a typing error from leaving the unit stopped
/// for a long time.
const MAX_TIMEOUT_S: f64 = 60.0;
const MAX_SETTLE_MS: u64 = 10_000;

const USAGE: &str =
    "usage: nucleus_query --port <device> [--baud 115200] [--timeout-s 2.0] [--settle-ms 500]";

#[derive(Debug, Clone, PartialEq)]
struct Args {
    port: String,
    baud: u32,
    timeout_s: f64,
    settle_ms: u64,
}

fn parse_args(args: &[String]) -> Result<Args, String> {
    let mut port = None;
    let mut baud = 115_200;
    let mut timeout_s: f64 = 2.0;
    let mut settle_ms = 500;
    let mut iter = args.iter();
    while let Some(flag) = iter.next() {
        let value = iter
            .next()
            .ok_or_else(|| format!("missing value for {flag}"))?;
        match flag.as_str() {
            "--port" => port = Some(value.clone()),
            "--baud" => baud = value.parse().map_err(|_| format!("bad --baud {value}"))?,
            "--timeout-s" => {
                timeout_s = value
                    .parse()
                    .map_err(|_| format!("bad --timeout-s {value}"))?
            }
            "--settle-ms" => {
                settle_ms = value
                    .parse()
                    .map_err(|_| format!("bad --settle-ms {value}"))?
            }
            other => return Err(format!("unknown option {other}")),
        }
    }
    if !(timeout_s > 0.0 && timeout_s <= MAX_TIMEOUT_S) {
        return Err(format!(
            "--timeout-s must be above 0 and at most {MAX_TIMEOUT_S}, got {timeout_s}"
        ));
    }
    if settle_ms > MAX_SETTLE_MS {
        return Err(format!(
            "--settle-ms must be at most {MAX_SETTLE_MS}, got {settle_ms}"
        ));
    }
    Ok(Args {
        port: port.ok_or("--port is required")?,
        baud,
        timeout_s,
        settle_ms,
    })
}

fn main() -> ExitCode {
    let raw: Vec<String> = std::env::args().skip(1).collect();
    let args = match parse_args(&raw) {
        Ok(args) => args,
        Err(message) => {
            eprintln!("{message}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let mut port = match serialport::new(&args.port, args.baud)
        .timeout(Duration::from_millis(50))
        .open()
    {
        Ok(port) => port,
        Err(error) => {
            eprintln!("cannot open {}: {error}", args.port);
            return ExitCode::from(1);
        }
    };
    let options = QueryOptions {
        command_timeout: Duration::from_secs_f64(args.timeout_s),
        settle_time: Duration::from_millis(args.settle_ms),
    };
    match query_configuration(&mut port, &options) {
        Ok(config) => match serde_yaml::to_string(&config) {
            Ok(yaml) => {
                print!("{yaml}");
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("cannot format the result: {error}");
                ExitCode::from(1)
            }
        },
        Err(error) => {
            eprintln!("query failed: {error}");
            ExitCode::from(1)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn strings(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| value.to_string()).collect()
    }

    #[test]
    fn defaults_apply_when_only_the_port_is_given() {
        assert_eq!(
            parse_args(&strings(&["--port", "/dev/ttyS0"])),
            Ok(Args {
                port: "/dev/ttyS0".to_string(),
                baud: 115_200,
                timeout_s: 2.0,
                settle_ms: 500,
            })
        );
    }

    #[test]
    fn every_option_is_parsed() {
        let args = parse_args(&strings(&[
            "--baud",
            "921600",
            "--port",
            "/dev/ttyUSB1",
            "--timeout-s",
            "0.5",
            "--settle-ms",
            "100",
        ]))
        .expect("valid arguments");
        assert_eq!(args.baud, 921_600);
        assert_eq!(args.port, "/dev/ttyUSB1");
        assert_eq!(args.timeout_s, 0.5);
        assert_eq!(args.settle_ms, 100);
    }

    #[test]
    fn oversized_options_are_rejected() {
        let base = ["--port", "/dev/ttyS0"];
        let with = |extra: &[&str]| {
            let mut all: Vec<&str> = base.to_vec();
            all.extend_from_slice(extra);
            parse_args(&strings(&all))
        };
        assert!(with(&["--timeout-s", "1e19"]).is_err());
        assert!(with(&["--timeout-s", "60.5"]).is_err());
        assert!(with(&["--settle-ms", "10001"]).is_err());
        assert!(with(&["--timeout-s", "60", "--settle-ms", "10000"]).is_ok());
    }

    #[test]
    fn invalid_arguments_are_rejected() {
        assert!(parse_args(&[]).is_err());
        assert!(parse_args(&strings(&["--port"])).is_err());
        assert!(parse_args(&strings(&["--port", "/dev/ttyS0", "--baud", "fast"])).is_err());
        assert!(parse_args(&strings(&["--port", "/dev/ttyS0", "--timeout-s", "0"])).is_err());
        assert!(parse_args(&strings(&["--port", "/dev/ttyS0", "--verbose", "1"])).is_err());
    }
}
