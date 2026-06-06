use crate::ublox_f9p_parser::UBXNavPvt;
use chrono::{DateTime, Duration, NaiveDate, Utc};
use std::error;
use std::fmt;

const SENTIBOARD_COUNTER_MODULUS: i64 = 0x1_0000_0000;
const SENTIBOARD_COUNTER_HALF_RANGE: i64 = SENTIBOARD_COUNTER_MODULUS / 2;
const SENTIBOARD_TICK_NS: i64 = 10;

#[derive(Debug, Default, Clone)]
pub struct SentiboardTimeSync {
    latest_pps_toa: Option<u32>,
    reference: Option<TimeReference>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TimeReference {
    pub utc_time: DateTime<Utc>,
    pub sentiboard_counter: u32,
    pub source: TimeSource,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeSource {
    Ntp,
    Gnss,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TimeSyncError {
    MissingPps,
    InvalidNavPvtTime,
}

impl fmt::Display for TimeSyncError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingPps => write!(f, "received NAV-PVT before any PPS arrival"),
            Self::InvalidNavPvtTime => write!(f, "NAV-PVT did not contain a valid UTC date/time"),
        }
    }
}

impl error::Error for TimeSyncError {}

impl SentiboardTimeSync {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update_pps(&mut self, time_of_arrival: u32) {
        self.latest_pps_toa = Some(time_of_arrival);
    }

    pub fn set_reference(
        &mut self,
        utc_time: DateTime<Utc>,
        sentiboard_counter: u32,
        source: TimeSource,
    ) {
        self.reference = Some(TimeReference {
            utc_time,
            sentiboard_counter,
            source,
        });
    }

    pub fn reference(&self) -> Option<&TimeReference> {
        self.reference.as_ref()
    }

    pub fn update_nav_pvt(&mut self, nav_pvt: &UBXNavPvt) -> Result<(), TimeSyncError> {
        let pps_toa = self.latest_pps_toa.ok_or(TimeSyncError::MissingPps)?;

        if !nav_pvt.valid_date || !nav_pvt.valid_time {
            return Err(TimeSyncError::InvalidNavPvtTime);
        }

        let date = NaiveDate::from_ymd_opt(
            i32::from(nav_pvt.year),
            u32::from(nav_pvt.month),
            u32::from(nav_pvt.day),
        )
        .ok_or(TimeSyncError::InvalidNavPvtTime)?;
        let utc_second = date
            .and_hms_opt(
                u32::from(nav_pvt.hour),
                u32::from(nav_pvt.min),
                u32::from(nav_pvt.sec),
            )
            .ok_or(TimeSyncError::InvalidNavPvtTime)?;

        self.set_reference(
            DateTime::from_naive_utc_and_offset(utc_second, Utc),
            pps_toa,
            TimeSource::Gnss,
        );
        Ok(())
    }

    pub fn absolute_time(&self, sentiboard_counter: u32) -> Option<DateTime<Utc>> {
        let reference = self.reference.as_ref()?;
        let delta_ticks = counter_delta_ticks(sentiboard_counter, reference.sentiboard_counter);
        let delta_ns = delta_ticks * SENTIBOARD_TICK_NS;

        Some(reference.utc_time + Duration::nanoseconds(delta_ns))
    }

    pub fn is_synchronized(&self) -> bool {
        self.reference.is_some()
    }
}

fn counter_delta_ticks(counter: u32, reference_counter: u32) -> i64 {
    let mut delta = i64::from(counter) - i64::from(reference_counter);

    if delta < -SENTIBOARD_COUNTER_HALF_RANGE {
        delta += SENTIBOARD_COUNTER_MODULUS;
    } else if delta > SENTIBOARD_COUNTER_HALF_RANGE {
        delta -= SENTIBOARD_COUNTER_MODULUS;
    }

    delta
}

#[cfg(test)]
mod tests {
    use super::*;

    fn utc_time(year: i32, month: u32, day: u32, hour: u32, min: u32, sec: u32) -> DateTime<Utc> {
        DateTime::from_naive_utc_and_offset(
            NaiveDate::from_ymd_opt(year, month, day)
                .unwrap()
                .and_hms_opt(hour, min, sec)
                .unwrap(),
            Utc,
        )
    }

    fn nav_pvt(year: u16, month: u8, day: u8, hour: u8, min: u8, sec: u8) -> UBXNavPvt {
        UBXNavPvt {
            itow: 0,
            year,
            month,
            day,
            hour,
            min,
            sec,
            valid_date: true,
            valid_time: true,
            fully_resolved: true,
            valid_mag: false,
            t_acc: 0,
            nano: 0,
            fix_type: 0,
            gnss_fix_ok: false,
            diff_soln: false,
            psm_state: 0,
            head_veh_valid: false,
            carr_soln: 0,
            confirmed_avail: false,
            confirmed_date: false,
            confirmed_time: false,
            num_sv: 0,
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

    #[test]
    fn nav_pvt_before_pps_returns_error() {
        let mut time_sync = SentiboardTimeSync::new();

        let err = time_sync
            .update_nav_pvt(&nav_pvt(2026, 6, 6, 12, 0, 0))
            .unwrap_err();

        assert_eq!(err, TimeSyncError::MissingPps);
        assert!(!time_sync.is_synchronized());
    }

    #[test]
    fn pps_and_valid_nav_pvt_establishes_sync() {
        let mut time_sync = SentiboardTimeSync::new();
        time_sync.update_pps(100);

        time_sync
            .update_nav_pvt(&nav_pvt(2026, 6, 6, 12, 0, 0))
            .unwrap();

        let absolute_time = time_sync.absolute_time(100).unwrap();
        assert!(time_sync.is_synchronized());
        assert_eq!(time_sync.reference().unwrap().source, TimeSource::Gnss);
        assert_eq!(absolute_time.timestamp(), 1_780_747_200);
        assert_eq!(absolute_time.timestamp_subsec_nanos(), 0);
    }

    #[test]
    fn explicit_ntp_reference_bootstraps_absolute_time() {
        let mut time_sync = SentiboardTimeSync::new();
        time_sync.set_reference(utc_time(2026, 6, 6, 12, 0, 0), 1_000, TimeSource::Ntp);

        assert_eq!(
            time_sync
                .absolute_time(1_025)
                .unwrap()
                .timestamp_subsec_nanos(),
            250
        );
        assert_eq!(
            time_sync
                .absolute_time(975)
                .unwrap()
                .timestamp_subsec_nanos(),
            999_999_750
        );
        assert_eq!(time_sync.reference().unwrap().source, TimeSource::Ntp);
    }

    #[test]
    fn gnss_reference_replaces_ntp_reference() {
        let mut time_sync = SentiboardTimeSync::new();
        time_sync.set_reference(utc_time(2026, 6, 6, 11, 59, 59), 50, TimeSource::Ntp);

        time_sync.update_pps(100);
        time_sync
            .update_nav_pvt(&nav_pvt(2026, 6, 6, 12, 0, 0))
            .unwrap();

        let reference = time_sync.reference().unwrap();
        assert_eq!(reference.sentiboard_counter, 100);
        assert_eq!(reference.source, TimeSource::Gnss);
        assert_eq!(
            time_sync.absolute_time(100).unwrap().timestamp(),
            1_780_747_200
        );
    }

    #[test]
    fn counter_rollover_is_handled() {
        let mut time_sync = SentiboardTimeSync::new();
        time_sync.set_reference(
            utc_time(2026, 6, 6, 12, 0, 0),
            u32::MAX - 4,
            TimeSource::Ntp,
        );

        assert_eq!(
            time_sync.absolute_time(5).unwrap().timestamp_subsec_nanos(),
            100
        );
    }

    #[test]
    fn invalid_nav_pvt_time_is_rejected() {
        let mut message = nav_pvt(2026, 2, 30, 12, 0, 0);
        let mut time_sync = SentiboardTimeSync::new();
        time_sync.update_pps(100);

        let err = time_sync.update_nav_pvt(&message).unwrap_err();
        assert_eq!(err, TimeSyncError::InvalidNavPvtTime);

        message = nav_pvt(2026, 6, 6, 12, 0, 0);
        message.valid_time = false;
        let err = time_sync.update_nav_pvt(&message).unwrap_err();
        assert_eq!(err, TimeSyncError::InvalidNavPvtTime);
    }
}
