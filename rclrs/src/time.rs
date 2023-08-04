use crate::rcl_bindings::*;
use std::sync::{Mutex, Weak};

/// Struct that represents time.
#[derive(Debug)]
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
}

#[cfg(test)]
mod tests {
    use crate::clock::Clock;

    #[test]
    fn compare_times_from_same_clock() {
        let clock = Clock::system().unwrap();
        let t1 = clock.now();
        std::thread::sleep(std::time::Duration::from_micros(1));
        let t2 = clock.now();
        assert_eq!(t1.compare_with(&t2, |t1, t2| t1 > t2), Some(false));
        assert_eq!(t1.compare_with(&t2, |t1, t2| t2 > t1), Some(true));
    }

    #[test]
    fn compare_times_from_different_clocks() {
        // Times from different clocks, even if of the same type, can't be compared
        let c1 = Clock::system().unwrap();
        let c2 = Clock::system().unwrap();
        let t1 = c1.now();
        let t2 = c2.now();
        assert!(t2.compare_with(&t1, |_, _| ()).is_none());
        assert!(t1.compare_with(&t2, |_, _| ()).is_none());
    }
}
