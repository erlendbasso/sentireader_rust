use chrono::{Duration, NaiveDate, NaiveDateTime};
use std::time::{Duration as StdDuration, Instant};

const UNIT_NS: i64 = 10;
const PPS_PERIOD_TICKS: u32 = 100_000_000;
const PPS_STALE_AFTER: StdDuration = StdDuration::from_millis(2500);
const PPS_JITTER_WINDOW: usize = 16;
const MAX_FORWARD_DELTA_TICKS: u32 = 1_000_000_000; // 10 s at 100 MHz
const PPS_ACQUISITION_TOLERANCE_TICKS: u32 = 1_000_000; // 1 000 000 * 10ns = 10 ms = 1% of 1 s

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SentiboardClockState {
    Unanchored = 0,
    Acquiring = 1,
    Running = 2,
    CounterFault = 3,
}

impl SentiboardClockState {
    pub fn as_u8(self) -> u8 {
        self as u8
    }
}

#[derive(Debug, Clone)]
pub struct SentiboardTimingStatusSnapshot {
    pub state: SentiboardClockState,
    pub pps_available: bool,
    pub message: String,
    pub last_pps_interval_error_ticks: i32,
    pub pps_interval_jitter_ticks: f64,
}

#[derive(Debug)]
pub struct SentiboardClock {
    state: SentiboardClockState,
    anchor_counter: Option<u64>,
    last_raw_counter: Option<u32>,
    last_unwrapped_counter: Option<u64>,
    latest_pps_toa: Option<u32>,
    last_pps_seen: Option<Instant>,
    last_pps_interval_ticks: Option<u32>,
    pps_interval_errors: Vec<i32>,
    message: String,
    candidate_pps: Option<u32>,
}

impl Default for SentiboardClock {
    fn default() -> Self {
        Self {
            state: SentiboardClockState::Unanchored,
            anchor_counter: None,
            last_raw_counter: None,
            last_unwrapped_counter: None,
            latest_pps_toa: None,
            last_pps_seen: None,
            last_pps_interval_ticks: None,
            pps_interval_errors: Vec::with_capacity(PPS_JITTER_WINDOW),
            message: "Waiting for first independent OC7 rising edge".into(),
            candidate_pps: None,
        }
    }
}

impl SentiboardClock {
    pub fn new() -> Self {
        Self::default()
    }

    /// Observe counters in stream order so the 32-bit hardware counter can be
    /// unwrapped. Host time and GNSS time are deliberately not inputs.
    pub fn observe_counter(&mut self, raw: u32) {
        if self.state == SentiboardClockState::CounterFault {
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
                    self.state = SentiboardClockState::CounterFault;
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
    pub fn observe_pps(&mut self, raw: u32) {
        if self.state == SentiboardClockState::CounterFault {
            return;
        }

        if self.state != SentiboardClockState::Running {
            match self.candidate_pps {
                None => {
                    self.candidate_pps = Some(raw);
                    self.state = SentiboardClockState::Acquiring;
                    self.message = "Waiting for a second independent OC7 rising edge".into();
                }

                Some(previous) => {
                    let interval = raw.wrapping_sub(previous);
                    let error = i64::from(interval) - i64::from(PPS_PERIOD_TICKS);
                    
                    if error.unsigned_abs() <= u64::from(PPS_ACQUISITION_TOLERANCE_TICKS) {
                        self.anchor_at(raw);
                    } else {
                        self.candidate_pps = Some(raw);
                        self.message = format!(
                            "Second OC7 edge is not within tolerance: {error} ticks error"
                        );
                    }
                }
            }
            return;
        }

        self.observe_counter(raw);
        if self.state == SentiboardClockState::CounterFault {
            return;
        }
        if let Some(previous) = self.latest_pps_toa {
            let interval = raw.wrapping_sub(previous);
            self.last_pps_interval_ticks = Some(interval);
            self.push_pps_interval_error(i64::from(interval) - i64::from(PPS_PERIOD_TICKS));
        }
        self.latest_pps_toa = Some(raw);
        self.last_pps_seen = Some(Instant::now());
    }

    pub fn counter_to_time(&self, raw: u32) -> Option<NaiveDateTime> {
        if self.state != SentiboardClockState::Running {
            return None;
        }
        let anchor = self.anchor_counter?;
        let counter = self.counter_value(raw)?;
        let ticks = i128::from(counter) - i128::from(anchor);
        let ns = ticks.checked_mul(i128::from(UNIT_NS))?;
        let ns = i64::try_from(ns).ok()?;
        Some(unix_epoch() + Duration::nanoseconds(ns))
    }

    pub fn is_anchored(&self) -> bool {
        self.state == SentiboardClockState::Running
    }
    pub fn state(&self) -> SentiboardClockState {
        self.state
    }

    pub fn message(&self) -> &str {
        &self.message
    }

    pub fn timing_status_snapshot(&self) -> SentiboardTimingStatusSnapshot {
        let pps_available = self
            .last_pps_seen
            .is_some_and(|seen| seen.elapsed() <= PPS_STALE_AFTER);
        SentiboardTimingStatusSnapshot {
            state: self.state,
            pps_available,
            message: self.message.clone(),
            last_pps_interval_error_ticks: self
                .last_pps_interval_ticks
                .map(|v| (i64::from(v) - i64::from(PPS_PERIOD_TICKS)) as i32)
                .unwrap_or(0),
            pps_interval_jitter_ticks: self.pps_interval_jitter_ticks(),
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

    fn anchor_at(&mut self, raw: u32) {
        self.anchor_counter = Some(u64::from(raw));
        self.last_raw_counter = Some(raw);
        self.last_unwrapped_counter = Some(u64::from(raw));
        self.latest_pps_toa = Some(raw);
        self.last_pps_seen = Some(Instant::now());
        self.candidate_pps = None;
        self.state = SentiboardClockState::Running;
        self.message = "Sentiboard clock anchored at second OC7 edge".into(); 
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
        let mut clock = SentiboardClock::new();
        clock.observe_counter(42);
        assert!(clock.counter_to_time(42).is_none());
    }

    #[test]
    fn first_oc7_is_unix_epoch_and_never_reanchors() {
        let mut clock = SentiboardClock::new();
        clock.observe_pps(100);
        assert_eq!(clock.counter_to_time(100), Some(unix_epoch()));
        clock.observe_pps(100 + PPS_PERIOD_TICKS);
        assert_eq!(
            clock.counter_to_time(100 + PPS_PERIOD_TICKS),
            Some(unix_epoch() + Duration::seconds(1))
        );
    }

    #[test]
    fn unwraps_counter_rollover() {
        let mut clock = SentiboardClock::new();
        clock.observe_pps(u32::MAX - 4);
        clock.observe_counter(5);
        assert_eq!(
            clock.counter_to_time(5),
            Some(unix_epoch() + Duration::nanoseconds(100))
        );
    }

    #[test]
    fn reports_implausible_counter_jump_without_reanchoring() {
        let mut clock = SentiboardClock::new();
        clock.observe_pps(10);
        clock.observe_counter(10 + MAX_FORWARD_DELTA_TICKS + 1);
        assert_eq!(clock.state(), SentiboardClockState::CounterFault);
        assert!(clock.counter_to_time(20).is_none());
    }

    #[test]
    fn tolerates_recent_out_of_order_capture() {
        let mut clock = SentiboardClock::new();
        clock.observe_counter(1_000);
        clock.observe_pps(900);
        assert_eq!(clock.state(), SentiboardClockState::Running);
        assert_eq!(clock.counter_to_time(900), Some(unix_epoch()));
        assert_eq!(
            clock.counter_to_time(1_000),
            Some(unix_epoch() + Duration::nanoseconds(1_000))
        );
    }
}
