use chrono::{Duration, NaiveDate, NaiveDateTime};
use std::time::{Duration as StdDuration, Instant};

const UNIT_NS: i64 = 10;
const PPS_PERIOD_TICKS: u32 = 100_000_000;
const PPS_STALE_AFTER: StdDuration = StdDuration::from_millis(2500);
const PPS_JITTER_WINDOW: usize = 16;
const MAX_FORWARD_DELTA_TICKS: u32 = 1_000_000_000; // 10 s at 100 MHz

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeSyncState {
    Unanchored = 0,
    Running = 1,
    CounterFault = 2,
}

impl TimeSyncState {
    pub fn as_u8(self) -> u8 {
        self as u8
    }
    pub fn source(self) -> &'static str {
        match self {
            Self::Unanchored => "none",
            Self::Running => "sentiboard_clock",
            Self::CounterFault => "sentiboard_clock_fault",
        }
    }
}

#[derive(Debug, Clone)]
pub struct TimeSyncStatusSnapshot {
    pub state: TimeSyncState,
    pub synchronized: bool,
    pub pps_available: bool,
    // Compatibility fields. GNSS is intentionally not part of this clock.
    pub nav_pvt_available: bool,
    pub nav_pvt_trusted: bool,
    pub source: String,
    pub message: String,
    pub nav_pvt_offset_ms: f64,
    pub last_pps_toa: u32,
    pub last_observed_toa: u32,
    pub pps_missing_count: u32,
    pub nav_pvt_missing_count: u32,
    pub reference_time: Option<NaiveDateTime>,
}

#[derive(Debug, Clone)]
pub struct SentiBoardTimingStatusSnapshot {
    pub time_sync: TimeSyncStatusSnapshot,
    pub pps_sequence_count: u32,
    pub last_pps_interval_ticks: u32,
    pub last_pps_interval_error_ticks: i32,
    pub estimated_board_frequency_hz: f64,
    pub pps_interval_jitter_ticks: f64,
    pub last_unwrapped_ticks: u64,
}

#[derive(Debug)]
pub struct TimeSync {
    state: TimeSyncState,
    anchor_counter: Option<u64>,
    last_raw_counter: Option<u32>,
    last_unwrapped_counter: Option<u64>,
    latest_pps_toa: Option<u32>,
    last_pps_seen: Option<Instant>,
    pps_missing_count: u32,
    pps_sequence_count: u32,
    last_pps_interval_ticks: Option<u32>,
    pps_interval_errors: Vec<i32>,
    message: String,
}

impl Default for TimeSync {
    fn default() -> Self {
        Self {
            state: TimeSyncState::Unanchored,
            anchor_counter: None,
            last_raw_counter: None,
            last_unwrapped_counter: None,
            latest_pps_toa: None,
            last_pps_seen: None,
            pps_missing_count: 0,
            pps_sequence_count: 0,
            last_pps_interval_ticks: None,
            pps_interval_errors: Vec::with_capacity(PPS_JITTER_WINDOW),
            message: "Waiting for first independent OC7 rising edge".into(),
        }
    }
}

impl TimeSync {
    pub fn new() -> Self {
        Self::default()
    }

    /// Observe counters in stream order so the 32-bit hardware counter can be
    /// unwrapped. Host time and GNSS time are deliberately not inputs.
    pub fn update_observation(&mut self, raw: u32) {
        if self.state == TimeSyncState::CounterFault {
            return;
        }
        match (self.last_raw_counter, self.last_unwrapped_counter) {
            (Some(previous), Some(unwrapped)) => {
                let delta = counter_delta(raw, previous);
                // Captured events can be serialized after a newer event. A
                // recent negative delta is therefore valid and must not move
                // the unwrap cursor backwards. Counter wrap naturally appears
                // as a small positive signed delta.
                if delta < 0 {
                    return;
                }
                let delta = delta as u32;
                if delta > MAX_FORWARD_DELTA_TICKS {
                    self.state = TimeSyncState::CounterFault;
                    self.message =
                        format!("Implausible Sentiboard counter movement: {delta} ticks");
                    return;
                }
                self.last_unwrapped_counter = Some(unwrapped + u64::from(delta));
            }
            _ => self.last_unwrapped_counter = Some(u64::from(raw)),
        }
        self.last_raw_counter = Some(raw);
    }

    /// The first OC7 edge defines synthetic Unix epoch. Later edges are
    /// diagnostics only and never alter the active mapping.
    pub fn update_pps(&mut self, raw: u32) {
        if self.anchor_counter.is_none() {
            // Startup must be atomic: buffered sensor packets may carry event
            // counters on either side of this edge, but none of them is
            // allowed to establish or fault the epoch before OC7 does.
            self.last_raw_counter = Some(raw);
            self.last_unwrapped_counter = Some(u64::from(raw));
            self.anchor_counter = Some(u64::from(raw));
            self.state = TimeSyncState::Running;
            self.latest_pps_toa = Some(raw);
            self.last_pps_seen = Some(Instant::now());
            self.pps_missing_count = 0;
            self.pps_sequence_count = 1;
            self.message = "Sentiboard clock anchored at first OC7 edge".into();
            return;
        }

        self.update_observation(raw);
        if self.state == TimeSyncState::CounterFault {
            return;
        }
        if let Some(previous) = self.latest_pps_toa {
            let interval = raw.wrapping_sub(previous);
            self.last_pps_interval_ticks = Some(interval);
            self.push_pps_interval_error(i64::from(interval) - i64::from(PPS_PERIOD_TICKS));
        }
        self.latest_pps_toa = Some(raw);
        self.last_pps_seen = Some(Instant::now());
        self.pps_missing_count = 0;
        self.pps_sequence_count = self.pps_sequence_count.saturating_add(1);
    }

    pub fn absolute_time(&self, raw: u32) -> Option<NaiveDateTime> {
        if self.state != TimeSyncState::Running {
            return None;
        }
        let anchor = self.anchor_counter?;
        let counter = self.counter_value(raw)?;
        let ticks = i128::from(counter) - i128::from(anchor);
        let ns = ticks.checked_mul(i128::from(UNIT_NS))?;
        let ns = i64::try_from(ns).ok()?;
        Some(unix_epoch() + Duration::nanoseconds(ns))
    }

    pub fn is_synchronized(&self) -> bool {
        self.state == TimeSyncState::Running
    }
    pub fn state(&self) -> TimeSyncState {
        self.state
    }

    pub fn status_snapshot(&mut self) -> TimeSyncStatusSnapshot {
        let pps_available = self
            .last_pps_seen
            .is_some_and(|seen| seen.elapsed() <= PPS_STALE_AFTER);
        if !pps_available && self.latest_pps_toa.is_some() {
            self.pps_missing_count = self.pps_missing_count.saturating_add(1);
        }
        TimeSyncStatusSnapshot {
            state: self.state,
            synchronized: self.is_synchronized(),
            pps_available,
            nav_pvt_available: false,
            nav_pvt_trusted: false,
            source: self.state.source().into(),
            message: self.message.clone(),
            nav_pvt_offset_ms: 0.0,
            last_pps_toa: self.latest_pps_toa.unwrap_or_default(),
            last_observed_toa: self.last_raw_counter.unwrap_or_default(),
            pps_missing_count: self.pps_missing_count,
            nav_pvt_missing_count: 0,
            reference_time: self.anchor_counter.map(|_| unix_epoch()),
        }
    }

    pub fn sentiboard_timing_status_snapshot(&mut self) -> SentiBoardTimingStatusSnapshot {
        let time_sync = self.status_snapshot();
        let interval = self.last_pps_interval_ticks.unwrap_or_default();
        SentiBoardTimingStatusSnapshot {
            time_sync,
            pps_sequence_count: self.pps_sequence_count,
            last_pps_interval_ticks: interval,
            last_pps_interval_error_ticks: self
                .last_pps_interval_ticks
                .map(|v| (i64::from(v) - i64::from(PPS_PERIOD_TICKS)) as i32)
                .unwrap_or(0),
            estimated_board_frequency_hz: self
                .last_pps_interval_ticks
                .map(f64::from)
                .unwrap_or(f64::from(PPS_PERIOD_TICKS)),
            pps_interval_jitter_ticks: self.pps_interval_jitter_ticks(),
            last_unwrapped_ticks: self.last_unwrapped_counter.unwrap_or_default(),
        }
    }

    fn counter_value(&self, raw: u32) -> Option<u64> {
        let last_raw = self.last_raw_counter?;
        let last = self.last_unwrapped_counter?;
        let signed_delta = i64::from(counter_delta(raw, last_raw));
        if signed_delta >= 0 {
            last.checked_add(signed_delta as u64)
        } else {
            last.checked_sub((-signed_delta) as u64)
        }
    }

    fn push_pps_interval_error(&mut self, error: i64) {
        self.pps_interval_errors
            .push(error.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32);
        if self.pps_interval_errors.len() > PPS_JITTER_WINDOW {
            self.pps_interval_errors.remove(0);
        }
    }

    fn pps_interval_jitter_ticks(&self) -> f64 {
        if self.pps_interval_errors.len() < 2 {
            return 0.0;
        }
        let mean = self
            .pps_interval_errors
            .iter()
            .map(|v| f64::from(*v))
            .sum::<f64>()
            / self.pps_interval_errors.len() as f64;
        let variance = self
            .pps_interval_errors
            .iter()
            .map(|v| (f64::from(*v) - mean).powi(2))
            .sum::<f64>()
            / self.pps_interval_errors.len() as f64;
        variance.sqrt()
    }
}

fn counter_delta(value: u32, reference: u32) -> i32 {
    value.wrapping_sub(reference) as i32
}

fn unix_epoch() -> NaiveDateTime {
    NaiveDate::from_ymd_opt(1970, 1, 1)
        .unwrap()
        .and_hms_opt(0, 0, 0)
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unavailable_before_first_oc7() {
        let mut clock = TimeSync::new();
        clock.update_observation(42);
        assert!(clock.absolute_time(42).is_none());
    }

    #[test]
    fn first_oc7_is_unix_epoch_and_never_reanchors() {
        let mut clock = TimeSync::new();
        clock.update_pps(100);
        assert_eq!(clock.absolute_time(100), Some(unix_epoch()));
        clock.update_pps(100 + PPS_PERIOD_TICKS);
        assert_eq!(
            clock.absolute_time(100 + PPS_PERIOD_TICKS),
            Some(unix_epoch() + Duration::seconds(1))
        );
    }

    #[test]
    fn unwraps_counter_rollover() {
        let mut clock = TimeSync::new();
        clock.update_pps(u32::MAX - 4);
        clock.update_observation(5);
        assert_eq!(
            clock.absolute_time(5),
            Some(unix_epoch() + Duration::nanoseconds(100))
        );
    }

    #[test]
    fn reports_implausible_counter_jump_without_reanchoring() {
        let mut clock = TimeSync::new();
        clock.update_pps(10);
        clock.update_observation(10 + MAX_FORWARD_DELTA_TICKS + 1);
        assert_eq!(clock.state(), TimeSyncState::CounterFault);
        assert!(clock.absolute_time(20).is_none());
    }

    #[test]
    fn tolerates_recent_out_of_order_capture() {
        let mut clock = TimeSync::new();
        clock.update_observation(1_000);
        clock.update_pps(900);
        assert_eq!(clock.state(), TimeSyncState::Running);
        assert_eq!(clock.absolute_time(900), Some(unix_epoch()));
        assert_eq!(
            clock.absolute_time(1_000),
            Some(unix_epoch() + Duration::nanoseconds(1_000))
        );
    }
}
