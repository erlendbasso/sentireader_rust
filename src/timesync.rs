use crate::ublox_f9p_parser::UBXNavPvt;
use chrono::{DateTime, Duration, NaiveDate, NaiveDateTime, Utc};
use std::time::{Duration as StdDuration, Instant, SystemTime};

const MAX_COUNTER: u64 = 0x1_0000_0000;
const UNIT_NS: i64 = 10;
const PPS_STALE_AFTER: StdDuration = StdDuration::from_millis(2500);
const NAV_PVT_HOLDOVER_AFTER_PPS: u32 = 5;
const MAX_NAV_PVT_OFFSET_MS: f64 = 250.0;
const MAX_NAV_PVT_OFFSET_TICKS: i64 = 25_000_000;
const MAX_NAV_PVT_T_ACC_NS: u32 = 100_000_000;
const REQUIRED_CONSECUTIVE_NAV_PVT: u8 = 3;
const WHOLE_SECOND_NANO_TOLERANCE_NS: i32 = 1_000_000;
const PPS_PERIOD_TICKS: u32 = 100_000_000;
const PPS_JITTER_WINDOW: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeSyncState {
    Unsynced = 0,
    HostFallback = 1,
    GnssLocked = 2,
    GnssHoldover = 3,
    PpsLostHostClock = 4,
}

impl TimeSyncState {
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    pub fn source(self) -> &'static str {
        match self {
            Self::Unsynced => "none",
            Self::HostFallback => "host_clock",
            Self::GnssLocked => "gnss_nav_pvt_pps",
            Self::GnssHoldover => "pps_holdover",
            Self::PpsLostHostClock => "host_clock_pps_lost",
        }
    }
}

#[derive(Debug, Clone)]
pub enum NavPvtUpdate {
    Accepted,
    PendingConsecutiveValidation { accepted_count: u8 },
    PendingPpsMatch,
    Rejected(String),
}

impl NavPvtUpdate {
    pub fn trusted(&self) -> bool {
        matches!(
            self,
            Self::Accepted | Self::PendingConsecutiveValidation { .. }
        )
    }
}

#[derive(Debug, Clone)]
pub struct TimeSyncStatusSnapshot {
    pub state: TimeSyncState,
    pub synchronized: bool,
    pub pps_available: bool,
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
}

#[derive(Debug, Clone, Copy)]
struct PendingNavPvt {
    reference_time: NaiveDateTime,
    nav_pvt_toa: u32,
}

#[derive(Debug)]
pub struct TimeSync {
    pub latest_pps_toa: Option<u32>,
    pub second_reference: Option<(NaiveDateTime, u32)>,
    pub synchronized: bool,
    state: TimeSyncState,
    last_observed_toa: Option<u32>,
    last_pps_seen: Option<Instant>,
    last_nav_pvt_seen: Option<Instant>,
    pps_missing_count: u32,
    nav_pvt_missing_count: u32,
    pps_sequence_count: u32,
    last_pps_interval_ticks: Option<u32>,
    pps_interval_errors: Vec<i32>,
    pending_nav_pvt: Option<PendingNavPvt>,
    consecutive_nav_pvt_valid: u8,
    nav_pvt_trusted: bool,
    last_nav_pvt_offset_ms: f64,
    message: String,
}

impl Default for TimeSync {
    fn default() -> Self {
        Self {
            latest_pps_toa: None,
            second_reference: None,
            synchronized: false,
            state: TimeSyncState::Unsynced,
            last_observed_toa: None,
            last_pps_seen: None,
            last_nav_pvt_seen: None,
            pps_missing_count: 0,
            nav_pvt_missing_count: 0,
            pps_sequence_count: 0,
            last_pps_interval_ticks: None,
            pps_interval_errors: Vec::with_capacity(PPS_JITTER_WINDOW),
            pending_nav_pvt: None,
            consecutive_nav_pvt_valid: 0,
            nav_pvt_trusted: false,
            last_nav_pvt_offset_ms: 0.0,
            message: "No time reference available".to_string(),
        }
    }
}

impl TimeSync {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update_observation(&mut self, time_of_arrival: u32) {
        self.last_observed_toa = Some(time_of_arrival);

        if self.pps_is_stale() {
            self.pps_missing_count = self.pps_missing_count.saturating_add(1);
            self.anchor_to_host_clock(time_of_arrival, TimeSyncState::PpsLostHostClock);
            self.message = "PPS missing; using host system clock".to_string();
        } else if self.second_reference.is_none() {
            self.anchor_to_host_clock(time_of_arrival, TimeSyncState::HostFallback);
            self.message = "Using host system clock until GNSS/PPS time is trusted".to_string();
        }
    }

    pub fn update_pps(&mut self, time_of_arrival: u32) {
        if let Some(previous_pps_toa) = self.latest_pps_toa {
            let interval_ticks = counter_delta(time_of_arrival, previous_pps_toa);
            if interval_ticks >= 0 {
                let interval_ticks = interval_ticks as u32;
                self.last_pps_interval_ticks = Some(interval_ticks);
                self.push_pps_interval_error(interval_ticks as i64 - PPS_PERIOD_TICKS as i64);
            }
        }

        self.pps_sequence_count = self.pps_sequence_count.saturating_add(1);
        self.latest_pps_toa = Some(time_of_arrival);
        self.last_pps_seen = Some(Instant::now());
        self.pps_missing_count = 0;

        let pps_time = if self.nav_pvt_trusted
            || matches!(
                self.state,
                TimeSyncState::GnssLocked | TimeSyncState::GnssHoldover
            ) {
            self.absolute_time_from_reference(time_of_arrival)
                .map(snap_to_nearest_utc_second)
                .unwrap_or_else(current_utc_second)
        } else {
            current_utc_second()
        };

        self.second_reference = Some((pps_time, time_of_arrival));
        self.synchronized = true;

        match self.state {
            TimeSyncState::GnssLocked => {
                self.message = "GNSS/PPS time synchronized".to_string();
            }
            TimeSyncState::GnssHoldover => {
                self.message = "NAV-PVT missing; holding time with PPS".to_string();
            }
            TimeSyncState::PpsLostHostClock
            | TimeSyncState::Unsynced
            | TimeSyncState::HostFallback => {
                let nav_pvt_locked = self.nav_pvt_trusted
                    && self.consecutive_nav_pvt_valid >= REQUIRED_CONSECUTIVE_NAV_PVT;
                self.state = if nav_pvt_locked {
                    TimeSyncState::GnssLocked
                } else {
                    TimeSyncState::HostFallback
                };
                self.message = if nav_pvt_locked {
                    "PPS restored; GNSS/PPS time synchronized".to_string()
                } else {
                    "PPS restored; using host fallback until NAV-PVT is trusted".to_string()
                };
            }
        }

        if self.nav_pvt_is_stale() && self.state == TimeSyncState::GnssLocked {
            self.state = TimeSyncState::GnssHoldover;
            self.nav_pvt_trusted = false;
            self.message = "NAV-PVT missing; holding time with PPS".to_string();
        }

        self.match_pending_nav_pvt(time_of_arrival);
    }

    pub fn update_nav_pvt(&mut self, nav_pvt: &UBXNavPvt, nav_pvt_toa: u32) -> NavPvtUpdate {
        self.last_nav_pvt_seen = Some(Instant::now());
        self.nav_pvt_missing_count = 0;

        let candidate = match nav_pvt_datetime(nav_pvt) {
            Ok(time) => time,
            Err(e) => return self.reject_nav_pvt(e),
        };

        if !nav_pvt.valid_date
            || !nav_pvt.valid_time
            || !nav_pvt.fully_resolved
            || !nav_pvt.confirmed_date
            || !nav_pvt.confirmed_time
        {
            return self
                .reject_nav_pvt("NAV-PVT time flags are not fully valid/confirmed".to_string());
        }

        if nav_pvt.t_acc > MAX_NAV_PVT_T_ACC_NS {
            return self.reject_nav_pvt(format!(
                "NAV-PVT time accuracy {} ns exceeds {} ns",
                nav_pvt.t_acc, MAX_NAV_PVT_T_ACC_NS
            ));
        }

        if !is_whole_second_nav_pvt(nav_pvt) {
            self.message =
                "NAV-PVT observed between PPS seconds; keeping current PPS reference".to_string();
            return NavPvtUpdate::PendingConsecutiveValidation {
                accepted_count: self.consecutive_nav_pvt_valid,
            };
        }

        let reference_time = snap_to_nearest_utc_second(candidate);

        if let Some(pps_toa) = self.latest_pps_toa.filter(|_| !self.pps_is_stale()) {
            let offset_ticks = counter_delta(nav_pvt_toa, pps_toa);
            self.last_nav_pvt_offset_ms = ticks_to_ms(offset_ticks);

            if valid_nav_pvt_pps_offset_ticks(offset_ticks) {
                self.pending_nav_pvt = None;
                return self.accept_nav_pvt_pps_match(reference_time, pps_toa);
            }
        }

        self.pending_nav_pvt = Some(PendingNavPvt {
            reference_time,
            nav_pvt_toa,
        });
        self.message = format!(
            "NAV-PVT waiting for matching PPS trigger within {:.3} ms",
            MAX_NAV_PVT_OFFSET_MS
        );
        NavPvtUpdate::PendingPpsMatch
    }

    fn accept_nav_pvt_pps_match(
        &mut self,
        reference_time: NaiveDateTime,
        pps_toa: u32,
    ) -> NavPvtUpdate {
        self.consecutive_nav_pvt_valid = self.consecutive_nav_pvt_valid.saturating_add(1);
        self.nav_pvt_trusted = true;
        self.second_reference = Some((reference_time, pps_toa));
        self.synchronized = true;

        if self.consecutive_nav_pvt_valid >= REQUIRED_CONSECUTIVE_NAV_PVT {
            self.state = if self.pps_is_stale() {
                TimeSyncState::PpsLostHostClock
            } else {
                TimeSyncState::GnssLocked
            };
            self.message = "NAV-PVT accepted and GNSS time trusted".to_string();
            NavPvtUpdate::Accepted
        } else {
            if !self.pps_is_stale() && self.state == TimeSyncState::Unsynced {
                self.state = TimeSyncState::HostFallback;
            }
            self.message = format!(
                "NAV-PVT accepted; waiting for {}/{} consecutive samples",
                self.consecutive_nav_pvt_valid, REQUIRED_CONSECUTIVE_NAV_PVT
            );
            NavPvtUpdate::PendingConsecutiveValidation {
                accepted_count: self.consecutive_nav_pvt_valid,
            }
        }
    }

    fn match_pending_nav_pvt(&mut self, pps_toa: u32) {
        let Some(pending_nav_pvt) = self.pending_nav_pvt.take() else {
            return;
        };

        let offset_ticks = counter_delta(pending_nav_pvt.nav_pvt_toa, pps_toa);
        self.last_nav_pvt_offset_ms = ticks_to_ms(offset_ticks);

        if valid_nav_pvt_pps_offset_ticks(offset_ticks) {
            let _ = self.accept_nav_pvt_pps_match(pending_nav_pvt.reference_time, pps_toa);
        } else {
            let reason = if offset_ticks < 0 {
                format!(
                    "Pending NAV-PVT/PPS TOA mismatch rejected: PPS TOA is after NAV-PVT \
                     (offset {:.3} ms, pps_toa={}, nav_pvt_toa={})",
                    self.last_nav_pvt_offset_ms, pps_toa, pending_nav_pvt.nav_pvt_toa
                )
            } else {
                format!(
                    "Pending NAV-PVT/PPS TOA mismatch rejected: offset {:.3} ms exceeds {:.3} ms \
                     (pps_toa={}, nav_pvt_toa={})",
                    self.last_nav_pvt_offset_ms,
                    MAX_NAV_PVT_OFFSET_MS,
                    pps_toa,
                    pending_nav_pvt.nav_pvt_toa
                )
            };
            let _ = self.reject_nav_pvt(reason);
        }
    }

    pub fn absolute_time(&self, sensor_validity: u32) -> Option<NaiveDateTime> {
        self.absolute_time_from_reference(sensor_validity)
    }

    pub fn absolute_utc_time(&self, sensor_validity: u32) -> Option<DateTime<Utc>> {
        self.absolute_time(sensor_validity)
            .map(|time| DateTime::from_naive_utc_and_offset(time, Utc))
    }

    pub fn gnss_pps_reference_time(&self) -> Option<NaiveDateTime> {
        if !matches!(
            self.state,
            TimeSyncState::GnssLocked | TimeSyncState::GnssHoldover
        ) {
            return None;
        }

        self.second_reference
            .map(|(reference_time, _)| reference_time)
    }

    pub fn status_snapshot(&mut self) -> TimeSyncStatusSnapshot {
        let pps_available = !self.pps_is_stale();
        if !pps_available {
            self.pps_missing_count = self.pps_missing_count.saturating_add(1);
            if let Some(toa) = self.last_observed_toa {
                self.anchor_to_host_clock(toa, TimeSyncState::PpsLostHostClock);
            }
        }

        let nav_pvt_available = !self.nav_pvt_is_stale();
        if !nav_pvt_available {
            self.nav_pvt_trusted = false;
            if self.state == TimeSyncState::GnssLocked {
                self.state = TimeSyncState::GnssHoldover;
                self.message = "NAV-PVT missing; holding time with PPS".to_string();
            }
        }

        let reference_time = self.second_reference.map(|(time, _)| time);

        TimeSyncStatusSnapshot {
            state: self.state,
            synchronized: self.synchronized,
            pps_available,
            nav_pvt_available,
            nav_pvt_trusted: self.nav_pvt_trusted,
            source: self.state.source().to_string(),
            message: self.message.clone(),
            nav_pvt_offset_ms: self.last_nav_pvt_offset_ms,
            last_pps_toa: self.latest_pps_toa.unwrap_or_default(),
            last_observed_toa: self.last_observed_toa.unwrap_or_default(),
            pps_missing_count: self.pps_missing_count,
            nav_pvt_missing_count: self.nav_pvt_missing_count,
            reference_time,
        }
    }

    pub fn sentiboard_timing_status_snapshot(&mut self) -> SentiBoardTimingStatusSnapshot {
        let time_sync = self.status_snapshot();
        let last_pps_interval_ticks = self.last_pps_interval_ticks.unwrap_or_default();
        let last_pps_interval_error_ticks =
            last_pps_interval_ticks as i64 - PPS_PERIOD_TICKS as i64;

        SentiBoardTimingStatusSnapshot {
            time_sync,
            pps_sequence_count: self.pps_sequence_count,
            last_pps_interval_ticks,
            last_pps_interval_error_ticks: if self.last_pps_interval_ticks.is_some() {
                last_pps_interval_error_ticks as i32
            } else {
                0
            },
            estimated_board_frequency_hz: self.estimated_board_frequency_hz(),
            pps_interval_jitter_ticks: self.pps_interval_jitter_ticks(),
        }
    }

    pub fn is_synchronized(&self) -> bool {
        self.synchronized
    }

    pub fn state(&self) -> TimeSyncState {
        self.state
    }

    fn absolute_time_from_reference(&self, sensor_validity: u32) -> Option<NaiveDateTime> {
        let (ref_time, ref_toa) = self.second_reference?;
        let delta_ticks = counter_delta(sensor_validity, ref_toa);
        let delta_ns = delta_ticks * UNIT_NS;
        Some(ref_time + Duration::nanoseconds(delta_ns))
    }

    fn anchor_to_host_clock(&mut self, time_of_arrival: u32, state: TimeSyncState) {
        self.second_reference = Some((current_utc_naive(), time_of_arrival));
        self.synchronized = true;
        self.state = state;
    }

    fn reject_nav_pvt(&mut self, reason: String) -> NavPvtUpdate {
        self.pending_nav_pvt = None;
        self.consecutive_nav_pvt_valid = 0;
        self.nav_pvt_trusted = false;
        if self.state == TimeSyncState::GnssLocked {
            self.state = TimeSyncState::GnssHoldover;
        }
        self.message = reason.clone();
        NavPvtUpdate::Rejected(reason)
    }

    fn pps_is_stale(&self) -> bool {
        self.last_pps_seen
            .map(|seen| seen.elapsed() > PPS_STALE_AFTER)
            .unwrap_or(true)
    }

    fn nav_pvt_is_stale(&mut self) -> bool {
        match self.last_nav_pvt_seen {
            Some(seen) if seen.elapsed() <= PPS_STALE_AFTER * NAV_PVT_HOLDOVER_AFTER_PPS => false,
            Some(_) => {
                self.nav_pvt_missing_count = self.nav_pvt_missing_count.saturating_add(1);
                true
            }
            None => true,
        }
    }

    fn push_pps_interval_error(&mut self, error_ticks: i64) {
        let clamped_error = error_ticks.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        self.pps_interval_errors.push(clamped_error);
        if self.pps_interval_errors.len() > PPS_JITTER_WINDOW {
            self.pps_interval_errors.remove(0);
        }
    }

    fn estimated_board_frequency_hz(&self) -> f64 {
        self.last_pps_interval_ticks
            .map(|ticks| ticks as f64)
            .unwrap_or(PPS_PERIOD_TICKS as f64)
    }

    fn pps_interval_jitter_ticks(&self) -> f64 {
        if self.pps_interval_errors.len() < 2 {
            return 0.0;
        }

        let mean = self
            .pps_interval_errors
            .iter()
            .map(|error| *error as f64)
            .sum::<f64>()
            / self.pps_interval_errors.len() as f64;

        let variance = self
            .pps_interval_errors
            .iter()
            .map(|error| {
                let delta = *error as f64 - mean;
                delta * delta
            })
            .sum::<f64>()
            / self.pps_interval_errors.len() as f64;

        variance.sqrt()
    }
}

fn nav_pvt_datetime(nav_pvt: &UBXNavPvt) -> Result<NaiveDateTime, String> {
    let date = NaiveDate::from_ymd_opt(
        nav_pvt.year.into(),
        nav_pvt.month.into(),
        nav_pvt.day.into(),
    )
    .ok_or("Invalid NAV-PVT date")?;

    let whole_second = date
        .and_hms_opt(nav_pvt.hour.into(), nav_pvt.min.into(), nav_pvt.sec.into())
        .ok_or_else(|| "Invalid NAV-PVT time".to_string())?;

    Ok(whole_second + Duration::nanoseconds(nav_pvt.nano.into()))
}

fn is_whole_second_nav_pvt(nav_pvt: &UBXNavPvt) -> bool {
    nav_pvt.itow.is_multiple_of(1000) && nav_pvt.nano.abs() <= WHOLE_SECOND_NANO_TOLERANCE_NS
}

fn snap_to_nearest_utc_second(abs_time: NaiveDateTime) -> NaiveDateTime {
    let utc_time = abs_time.and_utc();
    let round_up = utc_time.timestamp_subsec_nanos() >= 500_000_000;
    let seconds = utc_time.timestamp().saturating_add(i64::from(round_up));

    DateTime::<Utc>::from_timestamp(seconds, 0)
        .unwrap_or(utc_time)
        .naive_utc()
}

fn current_utc_naive() -> NaiveDateTime {
    DateTime::<Utc>::from(SystemTime::now()).naive_utc()
}

fn current_utc_second() -> NaiveDateTime {
    let now = DateTime::<Utc>::from(SystemTime::now());
    DateTime::<Utc>::from_timestamp(now.timestamp(), 0)
        .unwrap_or(now)
        .naive_utc()
}

fn counter_delta(sensor_validity: u32, reference: u32) -> i64 {
    let mut delta_ticks = sensor_validity as i64 - reference as i64;

    if delta_ticks < -(MAX_COUNTER as i64 / 2) {
        delta_ticks += MAX_COUNTER as i64;
    } else if delta_ticks > (MAX_COUNTER as i64 / 2) {
        delta_ticks -= MAX_COUNTER as i64;
    }

    delta_ticks
}

fn ticks_to_ms(ticks: i64) -> f64 {
    ticks as f64 * UNIT_NS as f64 / 1_000_000.0
}

fn valid_nav_pvt_pps_offset_ticks(offset_ticks: i64) -> bool {
    (0..=MAX_NAV_PVT_OFFSET_TICKS).contains(&offset_ticks)
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::{Datelike, Timelike};

    fn pvt(time: NaiveDateTime) -> UBXNavPvt {
        UBXNavPvt {
            itow: time.and_utc().timestamp_subsec_millis(),
            year: time.year() as u16,
            month: time.month() as u8,
            day: time.day() as u8,
            hour: time.hour() as u8,
            min: time.minute() as u8,
            sec: time.second() as u8,
            valid_date: true,
            valid_time: true,
            fully_resolved: true,
            valid_mag: false,
            t_acc: 1_000,
            nano: time.and_utc().timestamp_subsec_nanos() as i32,
            fix_type: 3,
            gnss_fix_ok: true,
            diff_soln: true,
            psm_state: 0,
            head_veh_valid: false,
            carr_soln: 0,
            confirmed_avail: true,
            confirmed_date: true,
            confirmed_time: true,
            num_sv: 10,
            lon: 0.0,
            lat: 0.0,
            height: 0.0,
            h_msl: 0.0,
            h_acc: 0.0,
            v_acc: 0.0,
            vel_n: 0.0,
            vel_e: 0.0,
            vel_d: 0.0,
            g_speed: 0,
            head_mot: 0.0,
            s_acc: 0,
            head_acc: 0,
            p_dop: 0,
            invalid_lat_lon_height: false,
            last_correction_age: 0,
            head_veh: 0.0,
            mag_dec: 0.0,
            mag_acc: 0.0,
        }
    }

    fn pvt_with_itow(time: NaiveDateTime, itow: u32) -> UBXNavPvt {
        UBXNavPvt { itow, ..pvt(time) }
    }

    fn pvt_with_nano(time: NaiveDateTime, nano: i32) -> UBXNavPvt {
        UBXNavPvt { nano, ..pvt(time) }
    }

    fn locked_sync(reference_time: NaiveDateTime, reference_toa: u32) -> TimeSync {
        let mut sync = TimeSync::new();
        sync.second_reference = Some((reference_time, reference_toa));
        sync.latest_pps_toa = Some(reference_toa);
        sync.last_pps_seen = Some(Instant::now());
        sync.synchronized = true;
        sync.nav_pvt_trusted = true;
        sync.state = TimeSyncState::GnssLocked;
        sync
    }

    #[test]
    fn observation_without_nav_pvt_uses_host_fallback() {
        let mut sync = TimeSync::new();
        sync.update_observation(100);

        assert!(sync.is_synchronized());
        assert_eq!(sync.state(), TimeSyncState::PpsLostHostClock);
        assert!(sync.absolute_time(100).is_some());
    }

    #[test]
    fn accepted_nav_pvt_locks_after_consecutive_samples() {
        let mut sync = TimeSync::new();
        sync.update_observation(100);
        sync.update_pps(100);
        let candidate = sync.absolute_time(100).unwrap();

        assert!(matches!(
            sync.update_nav_pvt(&pvt(candidate), 100),
            NavPvtUpdate::PendingConsecutiveValidation { .. }
        ));
        assert!(matches!(
            sync.update_nav_pvt(&pvt(candidate), 100),
            NavPvtUpdate::PendingConsecutiveValidation { .. }
        ));
        assert!(matches!(
            sync.update_nav_pvt(&pvt(candidate), 100),
            NavPvtUpdate::Accepted
        ));
        assert_eq!(sync.state(), TimeSyncState::GnssLocked);
    }

    #[test]
    fn pps_does_not_lock_before_required_nav_pvt_samples() {
        let mut sync = TimeSync::new();
        sync.update_observation(100);
        sync.update_pps(100);
        let candidate = sync.absolute_time(100).unwrap();

        assert!(matches!(
            sync.update_nav_pvt(&pvt(candidate), 100),
            NavPvtUpdate::PendingConsecutiveValidation { accepted_count: 1 }
        ));

        sync.update_pps(100 + PPS_PERIOD_TICKS);

        assert_eq!(sync.state(), TimeSyncState::HostFallback);
    }

    #[test]
    fn pending_nav_pvt_is_rejected_when_pps_is_too_far_earlier() {
        let mut sync = TimeSync::new();
        let nav_pvt_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let nav_pvt_toa = 25_000_001;

        let update = sync.update_nav_pvt(&pvt(nav_pvt_time), nav_pvt_toa);

        assert!(matches!(update, NavPvtUpdate::PendingPpsMatch));
        assert!(!update.trusted());
        assert_eq!(sync.consecutive_nav_pvt_valid, 0);
        assert!(sync.pending_nav_pvt.is_some());

        sync.update_pps(0);

        assert!(sync.pending_nav_pvt.is_none());
        assert!((sync.last_nav_pvt_offset_ms - 250.00001).abs() < 1e-9);
        assert_eq!(sync.consecutive_nav_pvt_valid, 0);
        assert!(!sync.nav_pvt_trusted);
    }

    #[test]
    fn subsecond_nav_pvt_samples_do_not_reject_or_reset_reference() {
        let mut sync = TimeSync::new();
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        sync.second_reference = Some((reference_time, 100));
        sync.latest_pps_toa = Some(100);
        sync.last_pps_seen = Some(Instant::now());
        sync.synchronized = true;
        sync.state = TimeSyncState::HostFallback;

        let subsecond = reference_time + Duration::milliseconds(200);
        assert!(matches!(
            sync.update_nav_pvt(&pvt_with_itow(subsecond, 200), 100),
            NavPvtUpdate::PendingConsecutiveValidation { accepted_count: 0 }
        ));
        assert_eq!(sync.second_reference, Some((reference_time, 100)));

        assert!(matches!(
            sync.update_nav_pvt(&pvt_with_itow(reference_time, 1000), 100),
            NavPvtUpdate::PendingConsecutiveValidation { accepted_count: 1 }
        ));
    }

    #[test]
    fn direct_toa_match_uses_nav_pvt_second_and_latest_pps() {
        let mut sync = TimeSync::new();
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        sync.second_reference = Some((reference_time, 100));
        sync.latest_pps_toa = Some(100);
        sync.last_pps_seen = Some(Instant::now());
        sync.synchronized = true;
        sync.state = TimeSyncState::HostFallback;

        let adjacent_second = reference_time + Duration::seconds(1);
        assert!(matches!(
            sync.update_nav_pvt(&pvt_with_itow(adjacent_second, 1000), 100),
            NavPvtUpdate::PendingConsecutiveValidation { accepted_count: 1 }
        ));
        assert_eq!(sync.last_nav_pvt_offset_ms, 0.0);
        assert_eq!(sync.second_reference, Some((adjacent_second, 100)));
    }

    #[test]
    fn nav_pvt_before_trigger_is_stored_then_matched_on_pps() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let mut sync = locked_sync(reference_time, 100);

        let next_pps_toa = 100u32.wrapping_add(PPS_PERIOD_TICKS);
        let nav_pvt_toa = next_pps_toa + 10_000_000;
        let nav_pvt_time = reference_time + Duration::seconds(1);

        let update = sync.update_nav_pvt(&pvt_with_itow(nav_pvt_time, 1000), nav_pvt_toa);

        assert!(matches!(update, NavPvtUpdate::PendingPpsMatch));
        assert!(!update.trusted());
        assert_eq!(sync.consecutive_nav_pvt_valid, 0);
        assert_eq!(sync.second_reference, Some((reference_time, 100)));

        sync.update_pps(next_pps_toa);

        assert_eq!(sync.last_nav_pvt_offset_ms, 100.0);
        assert_eq!(sync.consecutive_nav_pvt_valid, 1);
        assert_eq!(sync.second_reference, Some((nav_pvt_time, next_pps_toa)));
    }

    #[test]
    fn latest_pps_is_used_when_it_matches_nav_pvt() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let latest_pps_toa = 100u32.wrapping_add(PPS_PERIOD_TICKS);
        let mut sync = locked_sync(reference_time, 100);
        sync.update_pps(latest_pps_toa);

        let nav_pvt_time = reference_time + Duration::seconds(1);
        let nav_pvt_toa = latest_pps_toa + 10_000_000;

        assert!(matches!(
            sync.update_nav_pvt(&pvt_with_itow(nav_pvt_time, 1000), nav_pvt_toa),
            NavPvtUpdate::PendingConsecutiveValidation { accepted_count: 1 }
        ));
        assert_eq!(sync.last_nav_pvt_offset_ms, 100.0);
        assert_eq!(sync.second_reference, Some((nav_pvt_time, latest_pps_toa)));
    }

    #[test]
    fn close_nav_pvt_toa_matches_latest_pps_across_counter_rollover() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let pps_toa = u32::MAX - 10_000;
        let nav_pvt_toa = pps_toa.wrapping_add(10_000_000);
        let mut sync = locked_sync(reference_time, pps_toa);

        let update = sync.update_nav_pvt(&pvt_with_itow(reference_time, 0), nav_pvt_toa);

        assert!(matches!(
            update,
            NavPvtUpdate::PendingConsecutiveValidation { accepted_count: 1 }
        ));
        assert_eq!(sync.last_nav_pvt_offset_ms, 100.0);
        assert_eq!(sync.second_reference, Some((reference_time, pps_toa)));
    }

    #[test]
    fn pending_nav_pvt_is_rejected_when_pps_toa_is_after_nav_pvt() {
        let mut sync = TimeSync::new();
        let nav_pvt_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();

        let update = sync.update_nav_pvt(&pvt(nav_pvt_time), 100);

        assert!(matches!(update, NavPvtUpdate::PendingPpsMatch));

        sync.update_pps(200);

        assert!(sync.pending_nav_pvt.is_none());
        assert!((sync.last_nav_pvt_offset_ms + 0.001).abs() < 1e-9);
        assert_eq!(sync.consecutive_nav_pvt_valid, 0);
        assert!(!sync.nav_pvt_trusted);
    }

    #[test]
    fn three_pending_nav_pvt_matches_lock_gnss() {
        let mut sync = TimeSync::new();
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();

        for sample in 0..REQUIRED_CONSECUTIVE_NAV_PVT {
            let pps_toa = 100u32.wrapping_add(PPS_PERIOD_TICKS * sample as u32);
            let nav_pvt_toa = pps_toa.wrapping_add(10_000_000);
            let nav_pvt_time = reference_time + Duration::seconds(sample.into());
            let nav_pvt_itow = sample as u32 * 1000;

            assert!(matches!(
                sync.update_nav_pvt(&pvt_with_itow(nav_pvt_time, nav_pvt_itow), nav_pvt_toa),
                NavPvtUpdate::PendingPpsMatch
            ));
            sync.update_pps(pps_toa);
        }

        assert_eq!(sync.consecutive_nav_pvt_valid, REQUIRED_CONSECUTIVE_NAV_PVT);
        assert_eq!(sync.state(), TimeSyncState::GnssLocked);
        assert_eq!(
            sync.second_reference,
            Some((
                reference_time + Duration::seconds((REQUIRED_CONSECUTIVE_NAV_PVT - 1).into()),
                100u32.wrapping_add(PPS_PERIOD_TICKS * (REQUIRED_CONSECUTIVE_NAV_PVT - 1) as u32)
            ))
        );
    }

    #[test]
    fn pending_nav_pvt_matches_pps_across_counter_rollover() {
        let mut sync = TimeSync::new();
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let pps_toa = u32::MAX - 10_000;
        let nav_pvt_toa = pps_toa.wrapping_add(10_000_000);

        assert!(matches!(
            sync.update_nav_pvt(&pvt_with_itow(reference_time, 0), nav_pvt_toa),
            NavPvtUpdate::PendingPpsMatch
        ));

        sync.update_pps(pps_toa);

        assert_eq!(sync.last_nav_pvt_offset_ms, 100.0);
        assert_eq!(sync.second_reference, Some((reference_time, pps_toa)));
        assert_eq!(sync.consecutive_nav_pvt_valid, 1);
    }

    #[test]
    fn nav_pvt_datetime_includes_nano() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_milli_opt(12, 0, 0, 200)
            .unwrap();

        assert_eq!(
            nav_pvt_datetime(&pvt(reference_time)).unwrap(),
            reference_time
        );
    }

    #[test]
    fn nav_pvt_datetime_handles_negative_nano() {
        let whole_second = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 1)
            .unwrap();

        assert_eq!(
            nav_pvt_datetime(&pvt_with_nano(whole_second, -200_000_000)).unwrap(),
            whole_second - Duration::milliseconds(200)
        );
    }

    #[test]
    fn counter_rollover_stays_continuous() {
        let mut sync = TimeSync::new();
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        sync.second_reference = Some((reference_time, u32::MAX - 5));
        sync.synchronized = true;

        let abs_time = sync.absolute_time(4).unwrap();
        assert_eq!(abs_time, reference_time + Duration::nanoseconds(100));
    }

    #[test]
    fn gnss_pps_reference_returns_locked_reference_second() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let sync = locked_sync(reference_time, 100);

        let trigger_time = sync.gnss_pps_reference_time().unwrap();

        assert_eq!(trigger_time, reference_time);
    }

    #[test]
    fn gnss_pps_reference_returns_holdover_reference_second() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let mut sync = locked_sync(reference_time, 100);
        sync.state = TimeSyncState::GnssHoldover;

        let trigger_time = sync.gnss_pps_reference_time().unwrap();

        assert_eq!(trigger_time, reference_time);
    }

    #[test]
    fn gnss_pps_reference_is_unavailable_in_host_fallback() {
        let mut sync = TimeSync::new();
        sync.update_observation(100);
        sync.update_pps(100);

        assert_eq!(sync.state(), TimeSyncState::HostFallback);
        assert!(sync.gnss_pps_reference_time().is_none());
    }

    #[test]
    fn gnss_pps_reference_stays_on_integer_second_when_time_is_interpolated() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let sync = locked_sync(reference_time, 100);

        let interpolated = sync.absolute_time(100 + 12_345).unwrap();
        let trigger_time = sync.gnss_pps_reference_time().unwrap();

        assert_eq!(trigger_time, reference_time);
        assert_eq!(
            interpolated.and_utc().timestamp(),
            trigger_time.and_utc().timestamp()
        );
        assert_eq!(interpolated.and_utc().timestamp_subsec_nanos(), 123_450);
    }

    #[test]
    fn update_pps_snaps_jittered_gnss_reference_to_integer_second() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let mut sync = locked_sync(reference_time, 100);
        let jittered_next_pps_toa = 100u32.wrapping_add(PPS_PERIOD_TICKS + 12);

        sync.update_pps(jittered_next_pps_toa);

        let trigger_time = sync.gnss_pps_reference_time().unwrap();
        assert_eq!(trigger_time, reference_time + Duration::seconds(1));
        assert_eq!(
            sync.second_reference,
            Some((reference_time + Duration::seconds(1), jittered_next_pps_toa))
        );
        assert_eq!(
            sync.absolute_time(jittered_next_pps_toa + 12_345)
                .unwrap()
                .and_utc()
                .timestamp_subsec_nanos(),
            123_450
        );
    }

    #[test]
    fn update_pps_preserves_direct_nav_pvt_integer_second() {
        let reference_time = NaiveDate::from_ymd_opt(2026, 5, 22)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let mut sync = locked_sync(reference_time, 100);
        let next_pps_toa = 100u32.wrapping_add(PPS_PERIOD_TICKS);
        let nav_pvt_toa = next_pps_toa + 10_000_000;
        let nav_pvt_time = reference_time + Duration::seconds(1);

        sync.update_pps(next_pps_toa);
        assert!(matches!(
            sync.update_nav_pvt(&pvt_with_itow(nav_pvt_time, 1000), nav_pvt_toa),
            NavPvtUpdate::PendingConsecutiveValidation { accepted_count: 1 }
        ));
        assert_eq!(sync.second_reference, Some((nav_pvt_time, next_pps_toa)));

        sync.update_pps(next_pps_toa);

        let trigger_time = sync.gnss_pps_reference_time().unwrap();
        assert_eq!(sync.second_reference, Some((nav_pvt_time, next_pps_toa)));
        assert_eq!(trigger_time, nav_pvt_time);
    }

    #[test]
    fn nominal_pps_interval_has_zero_error() {
        let mut sync = TimeSync::new();

        sync.update_pps(100);
        sync.update_pps(100 + PPS_PERIOD_TICKS);

        let status = sync.sentiboard_timing_status_snapshot();
        assert_eq!(status.pps_sequence_count, 2);
        assert_eq!(status.last_pps_interval_ticks, PPS_PERIOD_TICKS);
        assert_eq!(status.last_pps_interval_error_ticks, 0);
        assert_eq!(status.estimated_board_frequency_hz, PPS_PERIOD_TICKS as f64);
    }

    #[test]
    fn pps_interval_handles_counter_wrap() {
        let mut sync = TimeSync::new();
        let first_pps = u32::MAX - 10;
        let second_pps = first_pps.wrapping_add(PPS_PERIOD_TICKS);

        sync.update_pps(first_pps);
        sync.update_pps(second_pps);

        let status = sync.sentiboard_timing_status_snapshot();
        assert_eq!(status.last_pps_interval_ticks, PPS_PERIOD_TICKS);
        assert_eq!(status.last_pps_interval_error_ticks, 0);
    }

    #[test]
    fn missing_pps_does_not_corrupt_last_interval() {
        let mut sync = TimeSync::new();

        sync.update_pps(100);
        sync.update_pps(100 + PPS_PERIOD_TICKS + 12);
        sync.last_pps_seen = Some(Instant::now() - PPS_STALE_AFTER * 2);

        let status = sync.sentiboard_timing_status_snapshot();
        assert!(!status.time_sync.pps_available);
        assert_eq!(status.last_pps_interval_ticks, PPS_PERIOD_TICKS + 12);
        assert_eq!(status.last_pps_interval_error_ticks, 12);
    }
}
