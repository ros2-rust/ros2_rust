use crate::{rcl_bindings::*,};
use std::{
    num::TryFromIntError,
    ops::{Add, Sub},
    sync::{Mutex, Weak},
    time::Duration,
};
/// Struct that represents time.
#[derive(Clone, Debug)]
pub struct Time {
    /// Timestamp in nanoseconds.
    pub nsec: i64,
    /// Weak reference to the clock that generated this time
    pub clock: Weak<Mutex<rcl_clock_t>>,
}

impl Time {
    /// Compares self to rhs, if they can be compared (originated from the same clock) calls f with
    /// the values of the timestamps.
    pub fn compare_with<U, F>(&self, rhs: &Time, f: F) -> Option<U>
    where
        F: FnOnce(i64, i64) -> U,
    {
        self.clock
            .ptr_eq(&rhs.clock)
            .then(|| f(self.nsec, rhs.nsec))
    }
    pub fn to_ros_msg<T::FromSecNano>(&self) -> Result<T, TryFromIntError> {
        Ok(T::time_stamp {
            nanosec: (self.nsec %10_i64.pow(9)) as u32,
            sec: (self.nsec /10_i64.pow(9)) as i32,
        })
    }
}
trait FromSecNano {
    fn time_stamp(nsec:u32,sec:i32) -> Result<T, TryFromIntError>;
}
#[cfg(feature="builtin_interfaces")]
impl FromSecNano for builtin_interfaces::msg::Time {
    fn time_stamp(nsec:u32,sec:i32) -> Result<Self, TryFromIntError> {
        Ok(Self {
            nanosec: nsec,
            sec: sec,
        })
    }
}
#[cfg(feature="builtin_interfaces")]
impl FromSecNano for builtin_interfaces::msg::rmw::Time {
    fn time_stamp(nsec:u32,sec:i32) -> Result<Self, TryFromIntError> {
        Ok(Self {
            nanosec: nsec,
            sec: sec,
        })
    }
}

impl Add<Duration> for Time {
    type Output = Self;

    fn add(self, other: Duration) -> Self {
        let dur_ns = i64::try_from(other.as_nanos()).unwrap();
        Time {
            nsec: self.nsec.checked_add(dur_ns).unwrap(),
            clock: self.clock.clone(),
        }
    }
}

impl Sub<Duration> for Time {
    type Output = Self;

    fn sub(self, other: Duration) -> Self {
        let dur_ns = i64::try_from(other.as_nanos()).unwrap();
        Time {
            nsec: self.nsec.checked_sub(dur_ns).unwrap(),
            clock: self.clock.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Clock;

    #[test]
    fn compare_times_from_same_clock() {
        let clock = Clock::system();
        let t1 = clock.now();
        std::thread::sleep(Duration::from_micros(1));
        let t2 = clock.now();
        assert_eq!(t1.compare_with(&t2, |t1, t2| t1 > t2), Some(false));
        assert_eq!(t1.compare_with(&t2, |t1, t2| t2 > t1), Some(true));
    }

    #[test]
    fn compare_times_from_different_clocks() {
        // Times from different clocks, even if of the same type, can't be compared
        let c1 = Clock::system();
        let c2 = Clock::system();
        let t1 = c1.now();
        let t2 = c2.now();
        assert!(t2.compare_with(&t1, |_, _| ()).is_none());
        assert!(t1.compare_with(&t2, |_, _| ()).is_none());
    }

    #[test]
    fn add_duration_to_time() {
        let (clock, _) = Clock::with_source();
        let t = clock.now();
        let t2 = t.clone() + Duration::from_secs(1);
        assert_eq!(t2.nsec - t.nsec, 1_000_000_000i64);
        let t3 = t2 - Duration::from_secs(1);
        assert_eq!(t3.nsec, t.nsec);
    }

    #[test]
    fn test_conversion() {
        let clock = Clock::system();
        let t1 = clock.now();
        let time = Time {
            nsec: 1_000_000_100,
            clock: t1.clock.clone(),
        };
        let msg = time.to_ros_msg().unwrap();
        assert_eq!(msg.nanosec, 100);
        assert_eq!(msg.sec, 1);
    }
}
