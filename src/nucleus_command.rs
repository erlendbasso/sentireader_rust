//! Read-only configuration query over the Nucleus ASCII command interface.
//!
//! Reference: Nortek, Nucleus 1000 Integrator's Guide, N3015-033 (2025),
//! chapters 6 and 7. `GET` commands work only in command mode, so the query
//! sends `STOP` first and always sends `START` at the end. The client never
//! sends a `SET` command.

use std::fmt;
use std::io::{self, Read, Write};
use std::str::FromStr;
use std::time::{Duration, Instant};

use serde::Serialize;

/// Nucleus settings that affect navigation and calibration.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NucleusConfig {
    pub instrument_name: String,
    pub serial_number: u32,
    pub firmware: String,
    /// `SETMISSION,POFF`, subtracted from the reported pressure (p. 56).
    pub pressure_offset_dbar: f32,
    /// `SETMISSION,SV`. 0.0 means the unit uses measured sound velocity.
    pub sound_velocity_setting_m_s: f32,
    /// `SETMISSION,SA`.
    pub salinity_ppt: f32,
    /// `SETBT,MODE`: `FAST_ACQ`, `AUTO`, or `CRAWLER`.
    pub bottom_track_mode: String,
    pub rotation_xy_deg: f32,
    pub rotation_yz_deg: f32,
    pub rotation_xz_deg: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueryOptions {
    /// Maximum wait for the reply to one command.
    pub command_timeout: Duration,
    /// Time to discard input after `STOP`, so that a binary record in
    /// flight does not reach the reply parser.
    pub settle_time: Duration,
}

impl Default for QueryOptions {
    fn default() -> Self {
        QueryOptions {
            command_timeout: Duration::from_secs(2),
            settle_time: Duration::from_millis(500),
        }
    }
}

#[derive(Debug)]
pub enum NucleusCommandError {
    Io(io::Error),
    Timeout {
        command: String,
    },
    Rejected {
        command: String,
    },
    MalformedReply {
        command: String,
        reply: String,
    },
    Forbidden {
        command: String,
    },
    /// `START` failed. The Nucleus can still be stopped.
    NotRestarted {
        start_error: Box<NucleusCommandError>,
        query_error: Option<Box<NucleusCommandError>>,
    },
}

impl fmt::Display for NucleusCommandError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "serial I/O error: {error}"),
            Self::Timeout { command } => write!(f, "no reply to {command}"),
            Self::Rejected { command } => write!(f, "the Nucleus rejected {command}"),
            Self::MalformedReply { command, reply } => {
                write!(f, "unexpected reply to {command}: {reply:?}")
            }
            Self::Forbidden { command } => write!(f, "refused to send {command}"),
            Self::NotRestarted {
                start_error,
                query_error,
            } => {
                write!(
                    f,
                    "START failed, the Nucleus may still be stopped: {start_error}"
                )?;
                if let Some(query_error) = query_error {
                    write!(f, " (query error: {query_error})")?;
                }
                Ok(())
            }
        }
    }
}

impl std::error::Error for NucleusCommandError {}

impl From<io::Error> for NucleusCommandError {
    fn from(error: io::Error) -> Self {
        NucleusCommandError::Io(error)
    }
}

/// How a reply line ends the reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Terminator {
    /// The whole line must be `OK` or `ERROR`.
    Exact,
    /// The line must end with `OK` or `ERROR`. Used after `STOP`, where the
    /// reply can follow binary bytes without a line break.
    Suffix,
}

/// Stop the Nucleus, read its configuration, and start it again.
///
/// `START` is sent even when a query step fails. If `START` fails, the
/// result is [`NucleusCommandError::NotRestarted`]. The transport should
/// return `ErrorKind::TimedOut` or `ErrorKind::WouldBlock` from `read` when
/// no byte arrives within a short read timeout.
pub fn query_configuration<T: Read + Write>(
    port: &mut T,
    options: &QueryOptions,
) -> Result<NucleusConfig, NucleusCommandError> {
    let query = stop_and_query(port, options);
    let start = send_command(port, "START", options.command_timeout, Terminator::Exact);
    match (query, start) {
        (Ok(config), Ok(_)) => Ok(config),
        (Err(query_error), Ok(_)) => Err(query_error),
        (query, Err(start_error)) => Err(NucleusCommandError::NotRestarted {
            start_error: Box::new(start_error),
            query_error: query.err().map(Box::new),
        }),
    }
}

fn stop_and_query<T: Read + Write>(
    port: &mut T,
    options: &QueryOptions,
) -> Result<NucleusConfig, NucleusCommandError> {
    match send_command(port, "STOP", options.command_timeout, Terminator::Suffix) {
        // ERROR means that the unit is already in command mode.
        Ok(_) | Err(NucleusCommandError::Rejected { .. }) => {}
        Err(error) => return Err(error),
    }
    drain(port, options.settle_time)?;
    let mission = query_values(port, "GETMISSION,POFF,SV,SA", 3, options)?;
    let bottom_track = query_values(port, "GETBT,MODE", 1, options)?;
    let instrument = query_values(port, "GETINST,ROTXY,ROTYZ,ROTXZ", 3, options)?;
    let firmware = query_values(port, "GETFW,STR", 1, options)?;
    let identity = query_values(port, "ID,STR,SN", 2, options)?;
    Ok(NucleusConfig {
        instrument_name: identity[0].clone(),
        serial_number: parse_value(&identity[1], "ID,STR,SN")?,
        firmware: firmware[0].clone(),
        pressure_offset_dbar: parse_value(&mission[0], "GETMISSION,POFF,SV,SA")?,
        sound_velocity_setting_m_s: parse_value(&mission[1], "GETMISSION,POFF,SV,SA")?,
        salinity_ppt: parse_value(&mission[2], "GETMISSION,POFF,SV,SA")?,
        bottom_track_mode: bottom_track[0].clone(),
        rotation_xy_deg: parse_value(&instrument[0], "GETINST,ROTXY,ROTYZ,ROTXZ")?,
        rotation_yz_deg: parse_value(&instrument[1], "GETINST,ROTXY,ROTYZ,ROTXZ")?,
        rotation_xz_deg: parse_value(&instrument[2], "GETINST,ROTXY,ROTYZ,ROTXZ")?,
    })
}

fn is_quiet(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
    )
}

fn send_command<T: Read + Write>(
    port: &mut T,
    command: &str,
    timeout: Duration,
    terminator: Terminator,
) -> Result<Vec<String>, NucleusCommandError> {
    if command.to_ascii_uppercase().starts_with("SET") {
        return Err(NucleusCommandError::Forbidden {
            command: command.to_string(),
        });
    }
    port.write_all(command.as_bytes())?;
    port.write_all(b"\r\n")?;
    port.flush()?;
    read_reply(port, command, timeout, terminator)
}

/// Collect printable reply lines until `OK` or `ERROR`. A non-printable
/// byte clears the current line, so binary data does not join a reply line.
fn read_reply<T: Read>(
    port: &mut T,
    command: &str,
    timeout: Duration,
    terminator: Terminator,
) -> Result<Vec<String>, NucleusCommandError> {
    let deadline = Instant::now() + timeout;
    let mut lines = Vec::new();
    let mut current = String::new();
    let mut buffer = [0u8; 256];
    while Instant::now() < deadline {
        let count = match port.read(&mut buffer) {
            Ok(0) => {
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            Ok(count) => count,
            Err(error) if is_quiet(&error) => continue,
            Err(error) => return Err(error.into()),
        };
        for &byte in &buffer[..count] {
            match byte {
                b'\r' | b'\n' => {
                    if current.is_empty() {
                        continue;
                    }
                    let line = std::mem::take(&mut current);
                    let (ok, error) = match terminator {
                        Terminator::Exact => (line == "OK", line == "ERROR"),
                        Terminator::Suffix => (line.ends_with("OK"), line.ends_with("ERROR")),
                    };
                    if ok {
                        return Ok(lines);
                    }
                    if error {
                        return Err(NucleusCommandError::Rejected {
                            command: command.to_string(),
                        });
                    }
                    lines.push(line);
                }
                0x20..=0x7E => current.push(char::from(byte)),
                _ => current.clear(),
            }
        }
    }
    Err(NucleusCommandError::Timeout {
        command: command.to_string(),
    })
}

/// Read and discard input for `settle`.
fn drain<T: Read>(port: &mut T, settle: Duration) -> Result<(), NucleusCommandError> {
    let deadline = Instant::now() + settle;
    let mut buffer = [0u8; 256];
    while Instant::now() < deadline {
        match port.read(&mut buffer) {
            Ok(0) => std::thread::sleep(Duration::from_millis(1)),
            Ok(_) => {}
            Err(error) if is_quiet(&error) => {}
            Err(error) => return Err(error.into()),
        }
    }
    Ok(())
}

/// Send a `GET` command and split the last reply line into `count` values.
fn query_values<T: Read + Write>(
    port: &mut T,
    command: &str,
    count: usize,
    options: &QueryOptions,
) -> Result<Vec<String>, NucleusCommandError> {
    let lines = send_command(port, command, options.command_timeout, Terminator::Exact)?;
    let reply = lines.last().cloned().unwrap_or_default();
    let values = split_values(&reply);
    if values.len() != count {
        return Err(NucleusCommandError::MalformedReply {
            command: command.to_string(),
            reply,
        });
    }
    Ok(values)
}

/// Split a reply such as `"Nucleus1000",900002` on commas outside quotes and
/// remove the quotes.
fn split_values(reply: &str) -> Vec<String> {
    let mut values = Vec::new();
    let mut current = String::new();
    let mut quoted = false;
    for character in reply.chars() {
        match character {
            '"' => quoted = !quoted,
            ',' if !quoted => values.push(std::mem::take(&mut current).trim().to_string()),
            _ => current.push(character),
        }
    }
    if !reply.is_empty() {
        values.push(current.trim().to_string());
    }
    values
}

fn parse_value<V: FromStr>(value: &str, command: &str) -> Result<V, NucleusCommandError> {
    value
        .parse()
        .map_err(|_| NucleusCommandError::MalformedReply {
            command: command.to_string(),
            reply: value.to_string(),
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// In-memory serial port. A complete written line queues the scripted
    /// reply for that command. `read` returns `TimedOut` when nothing waits.
    struct ScriptedPort {
        replies: Vec<(&'static str, Vec<u8>)>,
        pending: VecDeque<u8>,
        partial_line: Vec<u8>,
        commands: Vec<String>,
    }

    impl ScriptedPort {
        fn new(preamble: &[u8], replies: Vec<(&'static str, Vec<u8>)>) -> Self {
            ScriptedPort {
                replies,
                pending: preamble.iter().copied().collect(),
                partial_line: Vec::new(),
                commands: Vec::new(),
            }
        }
    }

    impl Write for ScriptedPort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            for &byte in buf {
                if byte == b'\n' {
                    let line = String::from_utf8_lossy(&self.partial_line)
                        .trim_end_matches('\r')
                        .to_string();
                    self.partial_line.clear();
                    if let Some((_, reply)) =
                        self.replies.iter().find(|(command, _)| *command == line)
                    {
                        self.pending.extend(reply.iter().copied());
                    }
                    self.commands.push(line);
                } else {
                    self.partial_line.push(byte);
                }
            }
            Ok(buf.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Read for ScriptedPort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if self.pending.is_empty() {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "no data"));
            }
            let count = buf.len().min(self.pending.len());
            for slot in buf.iter_mut().take(count) {
                *slot = self.pending.pop_front().expect("pending byte");
            }
            Ok(count)
        }
    }

    fn fast_options() -> QueryOptions {
        QueryOptions {
            command_timeout: Duration::from_millis(50),
            settle_time: Duration::from_millis(5),
        }
    }

    fn healthy_replies(stop_reply: &[u8]) -> Vec<(&'static str, Vec<u8>)> {
        vec![
            ("STOP", stop_reply.to_vec()),
            (
                "GETMISSION,POFF,SV,SA",
                b"9.50,1500.00,35.00\r\nOK\r\n".to_vec(),
            ),
            ("GETBT,MODE", b"\"AUTO\"\r\nOK\r\n".to_vec()),
            (
                "GETINST,ROTXY,ROTYZ,ROTXZ",
                b"0.00,0.00,180.00\r\nOK\r\n".to_vec(),
            ),
            ("GETFW,STR", b"\"4.2.2\"\r\nOK\r\n".to_vec()),
            ("ID,STR,SN", b"\"Nucleus1000\",300136\r\nOK\r\n".to_vec()),
            ("START", b"OK\r\n".to_vec()),
        ]
    }

    fn expected_config() -> NucleusConfig {
        NucleusConfig {
            instrument_name: "Nucleus1000".to_string(),
            serial_number: 300_136,
            firmware: "4.2.2".to_string(),
            pressure_offset_dbar: 9.5,
            sound_velocity_setting_m_s: 1500.0,
            salinity_ppt: 35.0,
            bottom_track_mode: "AUTO".to_string(),
            rotation_xy_deg: 0.0,
            rotation_yz_deg: 0.0,
            rotation_xz_deg: 180.0,
        }
    }

    #[test]
    fn query_reads_every_setting_and_restarts() {
        let mut port = ScriptedPort::new(&[], healthy_replies(b"OK\r\n"));
        let config = query_configuration(&mut port, &fast_options()).expect("config");
        assert_eq!(config, expected_config());
        assert_eq!(port.commands.first().map(String::as_str), Some("STOP"));
        assert_eq!(port.commands.last().map(String::as_str), Some("START"));
    }

    #[test]
    fn binary_stream_before_the_stop_reply_is_ignored() {
        let mut stop_reply = vec![0xA5, 0x0A, 0xB4, 0x20, 0x41, 0x7F, 0x00, b'Z'];
        stop_reply.extend_from_slice(b"OK\r\n");
        stop_reply.extend_from_slice(&[0xA5, 0x0A, 0xB4, 0x20, 0x80, 0x00]);
        let mut port = ScriptedPort::new(&[0x13, 0xA5, 0x0A], healthy_replies(&stop_reply));
        let config = query_configuration(&mut port, &fast_options()).expect("config");
        assert_eq!(config, expected_config());
    }

    #[test]
    fn stop_rejected_because_already_stopped_still_queries() {
        let mut port = ScriptedPort::new(&[], healthy_replies(b"ERROR\r\n"));
        let config = query_configuration(&mut port, &fast_options()).expect("config");
        assert_eq!(config, expected_config());
    }

    #[test]
    fn rejected_get_returns_error_and_still_starts() {
        let mut replies = healthy_replies(b"OK\r\n");
        replies[2] = ("GETBT,MODE", b"ERROR\r\n".to_vec());
        let mut port = ScriptedPort::new(&[], replies);
        let error = query_configuration(&mut port, &fast_options()).expect_err("rejected");
        assert!(
            matches!(error, NucleusCommandError::Rejected { ref command } if command == "GETBT,MODE")
        );
        assert_eq!(port.commands.last().map(String::as_str), Some("START"));
    }

    #[test]
    fn missing_reply_times_out_and_still_starts() {
        let mut replies = healthy_replies(b"OK\r\n");
        replies.retain(|(command, _)| *command != "GETFW,STR");
        let mut port = ScriptedPort::new(&[], replies);
        let error = query_configuration(&mut port, &fast_options()).expect_err("timeout");
        assert!(
            matches!(error, NucleusCommandError::Timeout { ref command } if command == "GETFW,STR")
        );
        assert_eq!(port.commands.last().map(String::as_str), Some("START"));
    }

    #[test]
    fn failed_start_is_reported_as_not_restarted() {
        let mut replies = healthy_replies(b"OK\r\n");
        replies.retain(|(command, _)| *command != "START");
        let mut port = ScriptedPort::new(&[], replies);
        let error = query_configuration(&mut port, &fast_options()).expect_err("not restarted");
        assert!(matches!(
            error,
            NucleusCommandError::NotRestarted {
                query_error: None,
                ..
            }
        ));
    }

    #[test]
    fn malformed_value_is_reported() {
        let mut replies = healthy_replies(b"OK\r\n");
        replies[1] = (
            "GETMISSION,POFF,SV,SA",
            b"9.50,abc,35.00\r\nOK\r\n".to_vec(),
        );
        let mut port = ScriptedPort::new(&[], replies);
        let error = query_configuration(&mut port, &fast_options()).expect_err("malformed");
        assert!(
            matches!(error, NucleusCommandError::MalformedReply { ref reply, .. } if reply == "abc")
        );
    }

    #[test]
    fn wrong_value_count_is_reported() {
        let mut replies = healthy_replies(b"OK\r\n");
        replies[1] = ("GETMISSION,POFF,SV,SA", b"9.50,1500.00\r\nOK\r\n".to_vec());
        let mut port = ScriptedPort::new(&[], replies);
        let error = query_configuration(&mut port, &fast_options()).expect_err("malformed");
        assert!(matches!(error, NucleusCommandError::MalformedReply { .. }));
    }

    #[test]
    fn never_sends_a_set_command() {
        let mut port = ScriptedPort::new(&[], healthy_replies(b"OK\r\n"));
        query_configuration(&mut port, &fast_options()).expect("config");
        assert!(port
            .commands
            .iter()
            .all(|command| !command.to_ascii_uppercase().starts_with("SET")));
        let error = send_command(
            &mut port,
            "SETMISSION,POFF=0",
            Duration::ZERO,
            Terminator::Exact,
        )
        .expect_err("forbidden");
        assert!(matches!(error, NucleusCommandError::Forbidden { .. }));
    }

    #[test]
    fn split_values_handles_quotes_and_commas() {
        assert_eq!(
            split_values("\"Nucleus1000\",900002"),
            vec!["Nucleus1000", "900002"]
        );
        assert_eq!(split_values("\"a,b\",1"), vec!["a,b", "1"]);
        assert_eq!(
            split_values("9.50, 1500.00 ,35.00"),
            vec!["9.50", "1500.00", "35.00"]
        );
        assert!(split_values("").is_empty());
    }
}
