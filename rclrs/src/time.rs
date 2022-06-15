use crate::error::RclrsError;
use crate::rcl_bindings::*;
//use crate::RclReturnCode;
//use crate::duration;
use parking_lot::Mutex;
use std::time;


/// Enum to provide different ways to construct the Time struct
#[allow(missing_docs)]
pub enum TimeFrom {
    /// Create a Time object using seconds
    Secs{s: u64},

    /// Create a Time object using nanoseconds
    NanoSecs{ns: u64},

    /// Create a Time object using `std::time::Duration`
    Duration{d: time::Duration},
}

/// The Time struct
pub struct Time {
    rcl_time_: Mutex<rcl_time_point_t>,
    // wrapped in mutex to allow the Clock struct to access it
}

impl Clone for Time {
    fn clone(&self) -> Self {
        let lock = self.rcl_time_.lock();
        Self {
            rcl_time_: Mutex::new(rcl_time_point_t {
                nanoseconds: (*lock).nanoseconds,
                clock_type: (*lock).clock_type,
            })
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
            TimeFrom::Secs{s} => Ok(Self {
                rcl_time_: Mutex::new(rcl_time_point_t {
                    nanoseconds: time::Duration::from_secs(s).as_nanos() as rcl_time_point_value_t,
                    clock_type,
                })
            }),
            TimeFrom::NanoSecs{ns} => Ok(Self {
                rcl_time_: Mutex::new(rcl_time_point_t {
                    nanoseconds: ns as rcl_time_point_value_t,
                    clock_type,
                }),
            }),
            TimeFrom::Duration{d} => Ok(Self {
                rcl_time_: Mutex::new(rcl_time_point_t {
                    nanoseconds: d.as_nanos() as rcl_time_point_value_t,
                    clock_type,
                }),
            }),
        }
    }
}
