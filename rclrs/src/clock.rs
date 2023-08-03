use crate::rcl_bindings::*;
use crate::{error::ToResult, time::Time, to_rclrs_result, RclrsError};
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
pub struct Clock {
    _type: ClockType,
    _rcl_clock: Arc<Mutex<rcl_clock_t>>,
    // TODO(luca) Implement jump callbacks
}

/// A clock source that can be used to drive the contained clock. Created when a clock of type
/// `ClockType::RosTime` is constructed
pub struct ClockSource {
    _rcl_clock: Arc<Mutex<rcl_clock_t>>,
}

impl Clock {
    /// Creates a new Clock with `ClockType::SystemTime`
    pub fn system() -> Result<Self, RclrsError> {
        Self::make(ClockType::SystemTime)
    }

    /// Creates a new Clock with `ClockType::SteadyTime`
    pub fn steady() -> Result<Self, RclrsError> {
        Self::make(ClockType::SteadyTime)
    }

    /// Creates a new Clock with `ClockType::RosTime` and a matching `ClockSource` that can be used
    /// to update it
    pub fn with_source() -> Result<(Self, ClockSource), RclrsError> {
        let clock = Self::make(ClockType::RosTime)?;
        let clock_source = ClockSource {
            _rcl_clock: clock._rcl_clock.clone(),
        };
        Ok((clock, clock_source))
    }

    /// Creates a new clock of the given `ClockType`.
    pub fn new(type_: ClockType) -> Result<(Self, Option<ClockSource>), RclrsError> {
        let clock = Self::make(type_)?;
        let clock_source = if matches!(type_, ClockType::RosTime) {
            Some(ClockSource {
                _rcl_clock: clock._rcl_clock.clone(),
            })
        } else {
            None
        };
        Ok((clock, clock_source))
    }

    fn make(type_: ClockType) -> Result<Self, RclrsError> {
        let mut rcl_clock;
        unsafe {
            // SAFETY: Getting a default value is always safe.
            rcl_clock = Self::init_generic_clock();
            let mut allocator = rcutils_get_default_allocator();
            rcl_clock_init(type_.into(), &mut rcl_clock, &mut allocator).ok()?;
        }
        Ok(Self {
            _type: type_,
            _rcl_clock: Arc::new(Mutex::new(rcl_clock)),
        })
    }

    /// Returns the clock's `ClockType`.
    pub fn clock_type(&self) -> ClockType {
        self._type
    }

    /// Returns the current clock's timestamp.
    pub fn now(&self) -> Time {
        let mut clock = self._rcl_clock.lock().unwrap();
        let mut time_point: i64 = 0;
        unsafe {
            // SAFETY: No preconditions for his function
            rcl_clock_get_now(&mut *clock, &mut time_point);
        }
        Time {
            nsec: time_point,
            clock_type: self._type,
        }
    }

    /// Helper function to initialize a default clock, same behavior as `rcl_init_generic_clock`.
    /// Needed because functions that initialize a clock take as an input a mutable reference
    /// to a clock and don't actually return one, so we need a function to generate one. Doing this
    /// instead of a `Default` implementation allows the function to be private and avoids
    /// exposing a public API to create an invalid clock
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

impl PartialEq for ClockSource {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self._rcl_clock, &other._rcl_clock)
    }
}

impl ClockSource {
    /// Sets the clock to use ROS Time, if enabled the clock will report the last value set through
    /// `Clock::set_ros_time_override(nanoseconds: i64)`.
    pub fn set_ros_time_enable(&self, enable: bool) {
        let mut clock = self._rcl_clock.lock().unwrap();
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

    /// Sets the value of the current ROS time.
    pub fn set_ros_time_override(&self, nanoseconds: i64) {
        let mut clock = self._rcl_clock.lock().unwrap();
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

    fn assert_send<T: Send>() {}
    fn assert_sync<T: Sync>() {}

    #[test]
    fn clock_is_send_and_sync() {
        assert_send::<Clock>();
        assert_sync::<Clock>();
    }

    #[test]
    fn clock_system_time_now() {
        let clock = Clock::system().unwrap();
        assert!(clock.now().nsec > 0);
    }

    #[test]
    fn clock_ros_time_with_override() {
        let (clock, source) = Clock::with_source().unwrap();
        let start = clock.now();
        // Ros time is not set, should return wall time
        assert!(start.nsec > 0);
        source.set_ros_time_enable(true);
        // No manual time set, it should default to 0
        assert!(clock.now().nsec == 0);
        let set_time = 1234i64;
        source.set_ros_time_override(set_time);
        // Ros time is set, should return the value that was set
        assert_eq!(clock.now().nsec, set_time);
        // Back to normal time, should be greater than before
        source.set_ros_time_enable(false);
        assert!(clock.now().nsec != set_time);
        assert!(clock.now().nsec > start.nsec);
    }
}
