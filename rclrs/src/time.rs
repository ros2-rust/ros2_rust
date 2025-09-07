use crate::{rcl_bindings::*, vendor::builtin_interfaces};
use std::{
    fmt,
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

    /// Convenience function for converting time to ROS message
    pub fn to_ros_msg(&self) -> Result<builtin_interfaces::msg::Time, TryFromIntError> {
        let nanosec = self.nsec % 1_000_000_000;
        let sec = self.nsec / 1_000_000_000;

        Ok(builtin_interfaces::msg::Time {
            nanosec: nanosec.try_into()?,
            sec: sec.try_into()?,
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

/// An error which can be returned when converting a ROS [`Time`] into a [`Duration`].
///
/// # Example
/// ```
/// use std::time::Duration;
/// use rclrs::Time;
/// # use std::sync::Weak;
///
/// let ros_time = Time{nsec: -1, clock: Weak::new()};
///
/// if let Err(e) = TryInto::<Duration>::try_into(ros_time) {
///     println!("Failed conversion to Duration: {e}");
/// }
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TryFromTimeError {
    kind: TryFromTimeErrorKind,
}

impl fmt::Display for TryFromTimeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.kind {
            TryFromTimeErrorKind::Negative => {
                "cannot convert nanoseconds to Duration: value is negative"
            }
        }
        .fmt(f)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum TryFromTimeErrorKind {
    // Value is negative.
    Negative,
}

impl TryInto<Duration> for Time {
    type Error = TryFromTimeError;

    fn try_into(self) -> Result<Duration, Self::Error> {
        if self.nsec < 0 {
            return Err(TryFromTimeError {
                kind: TryFromTimeErrorKind::Negative,
            });
        }

        Ok(Duration::from_nanos(u64::try_from(self.nsec).unwrap()))
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

    #[test]
    fn test_duration_conversion() {
        let clock = Clock::system();
        let t1 = clock.now();
        let bad_time = Time {
            nsec: -1,
            clock: t1.clock.clone(),
        };

        assert_eq!(
            TryInto::<Duration>::try_into(bad_time).err(),
            Some(TryFromTimeError {
                kind: TryFromTimeErrorKind::Negative
            })
        );

        let good_time = Time {
            nsec: 1_000_000_100,
            clock: t1.clock.clone(),
        };

        assert_eq!(
            TryInto::<Duration>::try_into(good_time).unwrap(),
            Duration::from_nanos(1_000_000_100)
        );
    }
}
