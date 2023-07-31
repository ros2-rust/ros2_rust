use crate::clock::ClockType;
use crate::rcl_bindings::*;

// TODO(luca) this is currently unused, maybe remove?
#[derive(Debug)]
pub struct Time {
    pub nsec: i64,
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
