# sentireader_rust

Rust library for reading and parsing the Sentiboard serial stream. It includes
parsers for the STIM300 IMU, u-blox ZED-F9P GNSS receivers, and the Waterlinked
A50 and Nortek Nucleus1000 DVLs.

The ROS nodes, published topics, LiDAR integration, GNSS conversion services,
and complete timing architecture are documented in
[`blueboat_sentinode/impl_doc.md`](../blueboat_sentinode/impl_doc.md).

## Serial reader

`SentiReader` normally reads `/dev/ttySentiboard02`. It scans for the `^B` or
`^C` synchronization marker, validates the header and payload Fletcher-16
checksums, and returns a `SentiboardMessage` containing:

- the sensor ID;
- the raw 32-bit time of validity (`TOV`), time of arrival (`TOA`), and time of
  transport (`TOT`) counters;
- the sensor payload;
- an optional onboard floating-point timestamp; and
- `host_receive_time`, sampled with `SystemTime::now()` immediately after the
  synchronization bytes are observed.

`^C` frames contain the optional eight-byte onboard timestamp before the three
counter fields. The reader normalizes the marker internally before validating
the header. `host_receive_time` describes host-side serial arrival and can be
used for transport and stale-frame checks; it does not define or discipline the
Sentiboard measurement clock.

The reader exposes cumulative valid-frame and resynchronization-byte counters.
When session logging is enabled, `LoggingReader` also exposes logging drop and
error counters.

## Sensor parsers

After `SentiReader` validates and separates a frame, the parser modules decode
the payload for the selected sensor:

- `stim300_parser` decodes STIM300 inertial data and its sample counter;
- `ublox_f9p_parser` decodes the supported UBX navigation, receiver, monitor,
  security, raw-observation, and relative-position messages;
- `dvl_a50_parser` and `dvl_nucleus1000_parser` decode the supported DVL wire
  formats.

These modules return Rust data structures. ROS message construction and topic
selection belong to `blueboat_sentinode`.

## Nucleus protocol modules

- `nortek_frame` validates one Nortek record: sync byte `0xA5`, header
  size 10, and the header and data checksums (Nucleus Integrator's Guide
  N3015-033, p. 83 and p. 115). It never panics. Bytes after the declared
  data length are ignored.
- `nucleus_record` decodes `_CommonData`, bottom-track (`0xB4`) and
  water-track (`0xBE`) records with every field of p. 91 to 94, and AHRS
  (`0xD2`) records. All values are little-endian.
- `nucleus_command` runs a read-only configuration query over any
  `Read + Write` transport: `STOP`, `GETMISSION,POFF,SV,SA`, `GETBT,MODE`,
  `GETINST,ROTXY,ROTYZ,ROTXZ`, `GETFW,STR`, `ID,STR,SN`, then `START`. It
  sends `START` also after a failed step. It never sends a `SET` command.
- `dvl_nucleus1000_parser::try_get_data_id` is the non-panicking
  replacement for `get_data_id`.

### Bench tool

`cargo run --release --bin nucleus_query -- --port /dev/ttyS0` prints the
Nucleus configuration as YAML. The query stops Nucleus measurement for a
few seconds. Do not run it during a mission.

### Tests

Run `cargo test --lib` and `cargo test --bin nucleus_query`. The tests in
`tests/integration_test.rs` need a SentiBoard on `/dev/ttySentiboard02`.

## Sentiboard counter mapping

The library's `sentiboard_clock::SentiboardClock` maps the free-running 100 MHz
Sentiboard counter into a synthetic time domain:

- one counter tick is 10 ns;
- two consecutive OC7/PPS rising edges acquire the clock, and the second
  accepted edge becomes synthetic epoch `2000-01-01T00:00:00`;
- the raw 32-bit counter is unwrapped into an internal 64-bit counter;
- natural rollover is accepted, and recent negative deltas do not move the
  unwrap cursor backward;
- a forward movement greater than 1,000,000,000 ticks (10 seconds) enters a
  sticky `CounterFault`; and
- later PPS edges update availability, interval, frequency, sequence, and
  jitter diagnostics but never re-anchor the clock.

The externally visible states are:

```mermaid
stateDiagram-v2
    [*] --> Unanchored: Process startup
    Unanchored --> Acquiring: First OC7 rising edge
    Acquiring --> Running: Second edge within tolerance
    Acquiring --> Acquiring: Replace invalid second-edge candidate
    Running --> Running: Valid observations and PPS edges
    Running --> CounterFault: Forward counter jump over 10 seconds
    CounterFault --> Unanchored: Process restart
```

Until the second accepted OC7 edge, `counter_to_time()` returns `None`. In
`Running`, it returns the synthetic epoch plus the unwrapped tick offset. A
process restart creates a new synthetic epoch, so timestamps from different
runs are not directly comparable without an additional run or epoch identifier.

GNSS, host realtime, PTP, and NTP are intentionally not inputs to
`SentiboardClock`.
The separate GNSS relationship, timestamp publication rules, chronology
limitations, and external-computer synchronization procedure are canonical in
the [Sentiboard timing implementation document](../blueboat_sentinode/impl_doc.md).
