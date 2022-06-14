use crate::rcl_bindings::*;
use crate::RclReturnCode;
use crate::RclrsError;
use std::os::raw::{c_uint, c_void};
use std::time;

/// Enum for Duration constructor arguments
pub enum DurationFrom {
    // Create Duration instance using seconds
    SECONDS { s: u32 },

    // Create Duration instance using seconds and ns
    SECONDS_AND_NANOS { s: u32, ns: u64 },

    // Create Duration instance using std::time::Duration
    DURATION { duration: time::Duration },
}

/// The Duration struct
pub struct Duration {
    _duration_handle: rcl_duration_t,
}

impl Duration {
    /// Function to instantiate a Duration object
    pub fn new(duration: DurationFrom) -> Result<Self, RclrsError> {
        match duration {
            DurationFrom::SECONDS { s } => Ok(Self {
                _duration_handle: rcl_duration_t {
                    nanoseconds: time::Duration::from_secs(s.into()).as_nanos()
                        as rcl_duration_value_t,
                },
            }),
            DurationFrom::SECONDS_AND_NANOS { s, ns } => Ok(Self {
                _duration_handle: rcl_duration_t {
                    nanoseconds: (time::Duration::from_secs(s.into()).as_nanos()
                        as rcl_duration_value_t)
                        + (ns as rcl_duration_value_t),
                },
            }),
            DurationFrom::DURATION { duration } => Ok(Self {
                _duration_handle: rcl_duration_t {
                    nanoseconds: duration.as_nanos() as rcl_duration_value_t,
                },
            }),
        }
    }
}
