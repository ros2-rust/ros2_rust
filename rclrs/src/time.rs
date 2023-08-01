use crate::clock::ClockType;
use crate::rcl_bindings::*;

/// Struct that represents time.
#[derive(Debug)]
pub struct Time {
    /// Timestamp in nanoseconds.
    pub nsec: i64,
    /// Clock type.
    pub clock_type: ClockType,
}

impl From<Time> for rcl_time_point_t {
    fn from(time: Time) -> Self {
        Self {
            nanoseconds: time.nsec,
            clock_type: time.clock_type.into(),
        }
    }
}
