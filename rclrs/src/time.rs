use crate::error::RclrsError;
use crate::rcl_bindings::*;
//use crate::RclReturnCode;
use crate::duration::Duration;
use parking_lot::{Mutex, MutexGuard};
use std::ops::{Add, Sub};
use std::time;

/// Enum to provide different ways to construct the Time struct
#[allow(missing_docs)]
pub enum TimeFrom {
    /// Create a Time object using seconds
    Secs { s: u64 },

    /// Create a Time object using nanoseconds
    NanoSecs { ns: u64 },

    /// Create a Time object using `std::time::Duration`
    Duration { d: time::Duration },
}

/// The Time struct
pub struct Time {
    rcl_time_: Mutex<rcl_time_point_t>,
    // wrapped in mutex to allow the Clock struct to access it
}

impl Add<Duration> for Time {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self {
        let lock = self.get_lock();
        let sum = (*lock).nanoseconds + (rhs.nanoseconds() as rcl_time_point_value_t);
        if (sum as u128) > (rcl_time_point_value_t::MAX as u128) {
            panic!(
                "Addition causes a {} overlflow",
                std::any::type_name::<rcl_time_point_value_t>()
            )
        }
        Self {
            rcl_time_: Mutex::new(rcl_time_point_t {
                nanoseconds: sum,
                clock_type: (*lock).clock_type,
            }),
        }
    }
}

impl Sub for Time {
    type Output = Self;

    fn sub(self, rhs: Time) -> Self {
        let lock = self.get_lock();
        let rhs_lock = rhs.get_lock();
        let diff = lock.nanoseconds - rhs_lock.nanoseconds;
        if lock.clock_type != rhs_lock.clock_type {
            panic!("Can not subtract times with different time sources");
        } else if diff < 0 {
            panic!("Time subtraction leads to negative time");
        }
        Self {
            rcl_time_: Mutex::new(rcl_time_point_t {
                nanoseconds: diff,
                clock_type: lock.clock_type,
            }),
        }
    }
}

impl Sub<Duration> for Time {
    type Output = Self;

    fn sub(self, rhs: Duration) -> Self {
        let lock = self.get_lock();
        let diff = lock.nanoseconds - (rhs.nanoseconds() as rcl_time_point_value_t);
        if diff < 0 {
            panic!("Subtraction leads to negative time");
        }

        Self {
            rcl_time_: Mutex::new(rcl_time_point_t {
                nanoseconds: diff,
                clock_type: lock.clock_type,
            }),
        }
    }
}

impl Clone for Time {
    fn clone(&self) -> Self {
        let lock = self.get_lock();
        Self {
            rcl_time_: Mutex::new(rcl_time_point_t {
                nanoseconds: lock.nanoseconds,
                clock_type: lock.clock_type,
            }),
        }
    }
}

#[allow(dead_code)]
impl Time {
    /// Function to create a new instance of Time
    ///
    /// # Example
    ///
    /// ## Create a Time object using seconds
    /// `Time::new(TimeFrom::Secs{s: <seconds>})`
    ///
    /// ## Create a Time object using nanoseconds
    /// `Time::new(TimeFrom::NanoSecs{ns: <nanoseconds>})`
    ///
    /// ## Create a Time object using `std::time::Duration`
    /// `Time::new(TimeFrom::Duration{d: std::time::Duration...})`
    pub fn new(arg: TimeFrom, clock_type: rcl_clock_type_t) -> Result<Self, RclrsError> {
        match arg {
            TimeFrom::Secs { s } => Ok(Self {
                rcl_time_: Mutex::new(rcl_time_point_t {
                    nanoseconds: time::Duration::from_secs(s).as_nanos() as rcl_time_point_value_t,
                    clock_type,
                }),
            }),
            TimeFrom::NanoSecs { ns } => Ok(Self {
                rcl_time_: Mutex::new(rcl_time_point_t {
                    nanoseconds: ns as rcl_time_point_value_t,
                    clock_type,
                }),
            }),
            TimeFrom::Duration { d } => Ok(Self {
                rcl_time_: Mutex::new(rcl_time_point_t {
                    nanoseconds: d.as_nanos() as rcl_time_point_value_t,
                    clock_type,
                }),
            }),
        }
    }

    /// Function to return time in nanoseconds
    pub fn nanoseconds(&self) -> rcl_time_point_value_t {
        (*self.get_lock()).nanoseconds
    }

    /// Function to return time in seconds
    pub fn seconds(&self) -> u64 {
        time::Duration::from_nanos((*self.rcl_time_.lock()).nanoseconds.try_into().unwrap())
            .as_secs()
    }

    /// Function to get a lock on `rcl_time_`
    pub fn get_lock(&self) -> MutexGuard<'_, rcl_time_point_t> {
        self.rcl_time_.lock()
    }

    /// Function to return clock type
    pub fn get_clock_type(&self) -> rcl_clock_type_t {
        self.get_lock().clock_type
    }

    /// Function to return the maximum possible value that can be held in the given instance
    pub fn max(&self) -> Self {
        Self {
            rcl_time_: Mutex::new(rcl_time_point_t {
                nanoseconds: rcl_time_point_value_t::MAX,
                clock_type: self.get_lock().clock_type,
            }),
        }
    }
}
