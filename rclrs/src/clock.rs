use crate::{error::ToResult, rcl_bindings::*, time::Time, to_rclrs_result};
use std::sync::{Arc, Mutex};

/// Enum to describe clock type. Redefined for readability and to eliminate the uninitialized case
/// from the `rcl_clock_type_t` enum in the binding.
#[derive(Clone, Debug, Copy)]
pub enum ClockType {
    /// Time with behavior dependent on the `set_ros_time(bool)` function. If called with `true`
    /// it will be driven by a manual value override, otherwise it will be System Time
    RosTime = 1,
    /// Wall time depending on the current system
    SystemTime = 2,
    /// Steady time, monotonically increasing but not necessarily equal to wall time.
    SteadyTime = 3,
}

impl From<ClockType> for rcl_clock_type_t {
    fn from(clock_type: ClockType) -> Self {
        match clock_type {
            ClockType::RosTime => rcl_clock_type_t::RCL_ROS_TIME,
            ClockType::SystemTime => rcl_clock_type_t::RCL_SYSTEM_TIME,
            ClockType::SteadyTime => rcl_clock_type_t::RCL_STEADY_TIME,
        }
    }
}

/// Struct that implements a Clock and wraps `rcl_clock_t`.
#[derive(Clone, Debug)]
pub struct Clock {
    kind: ClockType,
    rcl_clock: Arc<Mutex<rcl_clock_t>>,
    // TODO(luca) Implement jump callbacks
}

/// A clock source that can be used to drive the contained clock. Created when a clock of type
/// `ClockType::RosTime` is constructed
pub struct ClockSource {
    rcl_clock: Arc<Mutex<rcl_clock_t>>,
}

impl Clock {
    /// Creates a new Clock with `ClockType::SystemTime`
    pub fn system() -> Self {
        Self::make(ClockType::SystemTime)
    }

    /// Creates a new Clock with `ClockType::SteadyTime`
    pub fn steady() -> Self {
        Self::make(ClockType::SteadyTime)
    }

    /// Creates a new Clock with `ClockType::RosTime` and a matching `ClockSource` that can be used
    /// to update it
    pub fn with_source() -> (Self, ClockSource) {
        let clock = Self::make(ClockType::RosTime);
        let clock_source = ClockSource::new(clock.rcl_clock.clone());
        (clock, clock_source)
    }

    /// Creates a new clock of the given `ClockType`.
    pub fn new(kind: ClockType) -> (Self, Option<ClockSource>) {
        let clock = Self::make(kind);
        let clock_source =
            matches!(kind, ClockType::RosTime).then(|| ClockSource::new(clock.rcl_clock.clone()));
        (clock, clock_source)
    }

    fn make(kind: ClockType) -> Self {
        let mut rcl_clock;
        unsafe {
            // SAFETY: Getting a default value is always safe.
            rcl_clock = Self::init_generic_clock();
            let mut allocator = rcutils_get_default_allocator();
            // Function will return Err(_) only if there isn't enough memory to allocate a clock
            // object.
            rcl_clock_init(kind.into(), &mut rcl_clock, &mut allocator)
                .ok()
                .unwrap();
        }
        Self {
            kind,
            rcl_clock: Arc::new(Mutex::new(rcl_clock)),
        }
    }

    /// Returns the clock's `ClockType`.
    pub fn clock_type(&self) -> ClockType {
        self.kind
    }

    /// Returns the current clock's timestamp.
    pub fn now(&self) -> Time {
        let mut clock = self.rcl_clock.lock().unwrap();
        let mut time_point: i64 = 0;
        unsafe {
            // SAFETY: No preconditions for this function
            rcl_clock_get_now(&mut *clock, &mut time_point);
        }
        Time {
            nsec: time_point,
            clock: Arc::downgrade(&self.rcl_clock),
        }
    }

    /// Helper function to privately initialize a default clock, with the same behavior as
    /// `rcl_init_generic_clock`. By defining a private function instead of implementing
    /// `Default`,  we avoid exposing a public API to create an invalid clock.
    // SAFETY: Getting a default value is always safe.
    unsafe fn init_generic_clock() -> rcl_clock_t {
        let allocator = rcutils_get_default_allocator();
        rcl_clock_t {
            type_: rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED,
            jump_callbacks: std::ptr::null_mut::<rcl_jump_callback_info_t>(),
            num_jump_callbacks: 0,
            get_now: None,
            data: std::ptr::null_mut::<std::os::raw::c_void>(),
            allocator,
        }
    }
}

impl Drop for ClockSource {
    fn drop(&mut self) {
        self.set_ros_time_enable(false);
    }
}

impl PartialEq for ClockSource {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.rcl_clock, &other.rcl_clock)
    }
}

impl ClockSource {
    /// Sets the value of the current ROS time.
    pub fn set_ros_time_override(&self, nanoseconds: i64) {
        let mut clock = self.rcl_clock.lock().unwrap();
        // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
        // by the mutex
        unsafe {
            // Function will only fail if timer was uninitialized or not RosTime, which should
            // not happen
            rcl_set_ros_time_override(&mut *clock, nanoseconds)
                .ok()
                .unwrap();
        }
    }

    fn new(rcl_clock: Arc<Mutex<rcl_clock_t>>) -> Self {
        let source = Self { rcl_clock };
        source.set_ros_time_enable(true);
        source
    }

    /// Sets the clock to use ROS Time, if enabled the clock will report the last value set through
    /// `Clock::set_ros_time_override(nanoseconds: i64)`.
    fn set_ros_time_enable(&self, enable: bool) {
        let mut clock = self.rcl_clock.lock().unwrap();
        if enable {
            // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
            // by the mutex
            unsafe {
                // Function will only fail if timer was uninitialized or not RosTime, which should
                // not happen
                rcl_enable_ros_time_override(&mut *clock).ok().unwrap();
            }
        } else {
            // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
            // by the mutex
            unsafe {
                // Function will only fail if timer was uninitialized or not RosTime, which should
                // not happen
                rcl_disable_ros_time_override(&mut *clock).ok().unwrap();
            }
        }
    }
}

impl Drop for rcl_clock_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        let rc = unsafe { rcl_clock_fini(&mut *self) };
        if let Err(e) = to_rclrs_result(rc) {
            panic!("Unable to release Clock. {:?}", e)
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_clock_t {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<Clock>();
        assert_sync::<Clock>();
    }

    #[test]
    fn clock_system_time_now() {
        let clock = Clock::system();
        assert!(clock.now().nsec > 0);
    }

    #[test]
    fn clock_ros_time_with_override() {
        let (clock, source) = Clock::with_source();
        // No manual time set, it should default to 0
        assert!(clock.now().nsec == 0);
        let set_time = 1234i64;
        source.set_ros_time_override(set_time);
        // Ros time is set, should return the value that was set
        assert_eq!(clock.now().nsec, set_time);
    }
}
