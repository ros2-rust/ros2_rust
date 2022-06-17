use crate::rcl_bindings::*;
//use crate::RclReturnCode;
use crate::RclrsError;
use num::{abs, signum};
use std::cmp::Ordering;
use std::ops::{Add, Mul, Sub};
//use std::os::raw::{c_uint, c_void};
use std::time;

/// Enum for Duration constructor arguments
#[allow(missing_docs)]
pub enum DurationFrom {
    /// Create Duration instance using seconds
    Secs { s: i32 },

    /// Create Duration instance using nanoseconds
    NanoSecs { ns: rcl_duration_value_t },

    /// Create Duration instance using seconds and ns
    SecsAndNanoSecs { s: i32, ns: rcl_duration_value_t },

    /// Create Duration instance using std::time::Duration
    Duration { duration: time::Duration },

    /// Create Duration from rmw_time_t
    RMWTime { time: rmw_time_t },
}

/// The Duration struct
pub struct Duration {
    _duration_handle: rcl_duration_t,
}

impl Add for Duration {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        let sum = self._duration_handle.nanoseconds + rhs._duration_handle.nanoseconds;
        if (sum as u128) > (rcl_duration_value_t::MAX as u128) {
            panic!(
                "Addition leads to {} overflow",
                std::any::type_name::<rcl_duration_value_t>()
            );
        }

        Self::new(DurationFrom::NanoSecs {
            ns: sum as rcl_duration_value_t,
        })
        .unwrap()
    }
}

impl Clone for Duration {
    fn clone(&self) -> Self {
        Self::new(DurationFrom::NanoSecs {
            ns: self._duration_handle.nanoseconds,
        })
        .unwrap()
    }
}

impl Eq for Duration {}

impl Mul<f32> for Duration {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self {
        let prod = (self._duration_handle.nanoseconds as f32) * rhs;

        if (prod as i128) > (rcl_duration_value_t::MAX as i128) {
            panic!(
                "Scaling leads to {} overflow",
                std::any::type_name::<rcl_duration_value_t>()
            );
        }

        if (prod as i128) < (rcl_duration_value_t::MIN as i128) {
            panic!(
                "Scaling leads to {} underflow",
                std::any::type_name::<rcl_duration_value_t>()
            );
        }

        Self::new(DurationFrom::NanoSecs {
            ns: prod as rcl_duration_value_t,
        })
        .unwrap()
    }
}

impl Ord for Duration {
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.nanoseconds().cmp(&rhs.nanoseconds())
    }
}

impl PartialEq for Duration {
    fn eq(&self, rhs: &Self) -> bool {
        self.nanoseconds() == rhs.nanoseconds()
    }
}

impl PartialOrd for Duration {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}

impl Sub for Duration {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        let diff = self._duration_handle.nanoseconds - rhs._duration_handle.nanoseconds;
        if (diff as i128) < (rcl_duration_value_t::MIN as i128) {
            panic!(
                "Subtraction leads to {} underflow",
                std::any::type_name::<rcl_duration_value_t>()
            );
        }

        Self::new(DurationFrom::NanoSecs {
            ns: diff as rcl_duration_value_t,
        })
        .unwrap()
    }
}

#[allow(dead_code)]
impl Duration {
    /// Function to instantiate a Duration object
    ///
    /// # Example
    /// ## initialize using Seconds
    /// `Duration::new(DurationFrom::Secs{s: <seconds>})`
    ///
    /// ## initialize using Nanoseconds
    /// `Duration::new(DurationFrom::NanoSecs{ns: <nanoseconds>})`
    ///
    /// ## initialize using Seconds and Nanoseconds
    /// `Duration::new(DurationFrom::SecsAndNanoSecs{s: <seconds>, ns: <nanoseconds>})`
    ///
    /// ## initialize using `std::time::Duration`
    /// `Duration::new(DurationFrom::Duration{duration: std::time::Duration::from...})`
    ///
    /// ## initialize using `rmw_time_t`
    /// `Duration::new(DurationFrom::Duration{time: rmw_time_t { ... }})`
    pub fn new(duration: DurationFrom) -> Result<Self, RclrsError> {
        match duration {
            DurationFrom::Secs { s } => Ok(Self {
                _duration_handle: rcl_duration_t {
                    nanoseconds: ((signum(s) as i64)
                        * (time::Duration::from_secs(abs(s) as u64).as_nanos()) as i64)
                        as rcl_duration_value_t,
                },
            }),
            DurationFrom::NanoSecs { ns } => Ok(Self {
                _duration_handle: rcl_duration_t { nanoseconds: ns },
            }),
            DurationFrom::SecsAndNanoSecs { s, ns } => Ok(Self {
                _duration_handle: rcl_duration_t {
                    nanoseconds: ((signum(s) as i64)
                        * (time::Duration::from_secs(abs(s) as u64).as_nanos() as i64)
                            as rcl_duration_value_t)
                        + ns,
                },
            }),
            DurationFrom::Duration { duration } => Ok(Self {
                _duration_handle: rcl_duration_t {
                    nanoseconds: duration.as_nanos() as rcl_duration_value_t,
                },
            }),
            DurationFrom::RMWTime { time } => Ok(Self {
                _duration_handle: rcl_duration_t {
                    nanoseconds: ((time::Duration::from_secs(time.sec).as_nanos() as u64)
                        + time.nsec) as rcl_duration_value_t,
                },
            }),
        }
    }

    /// Function to get the count of nanoseconds in the Duration object
    pub fn nanoseconds(&self) -> rcl_duration_value_t {
        self._duration_handle.nanoseconds
    }

    /// Function to get the count of seconds in the Duration object
    pub fn seconds(&self) -> i32 {
        let ns = self._duration_handle.nanoseconds;
        (signum(ns) as i32) * (time::Duration::from_nanos(abs(ns) as u64).as_secs() as i32)
    }

    /// Function to get a `std::time::Duration` object
    pub fn to_duration(&self) -> time::Duration {
        time::Duration::from_nanos(self._duration_handle.nanoseconds as u64)
    }

    /// Function to get the maximum value that can be held in duration
    pub fn max(&self) -> Self {
        Self {
            _duration_handle: rcl_duration_t {
                nanoseconds: rcl_duration_value_t::MAX,
            },
        }
    }
}
