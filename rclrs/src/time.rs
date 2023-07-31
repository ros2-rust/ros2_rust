use crate::clock::ClockType;
use crate::rcl_bindings::*;

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

/*
impl Default for Time {
    fn default() -> Self {
        Self {
            nsec: 0,
            clock_type: ClockType::SystemTime,
        }
    }
}

impl From<crate::vendor::builtin_interfaces::msg::Time> for Time {
    fn from(time_msg: crate::vendor::builtin_interfaces::msg::Time) -> Self {
        Self {
            nsec: (time_msg.sec as i64 * 1_000_000_000) + time_msg.nanosec as i64,
            clock_type: ClockType::RosTime,
        }
    }
}
*/
