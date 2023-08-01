use crate::rcl_bindings::*;
use crate::{error::ToResult, time::Time, to_rclrs_result, RclrsError};
use std::sync::{Arc, Mutex};

/// Enum to describe clock type. Redefined for readability and to eliminate the uninitialized case
/// from the `rcl_clock_type_t` enum in the binding.
#[derive(Clone, Debug, Copy)]
pub enum ClockType {
    RosTime = 1,
    SystemTime = 2,
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

pub struct Clock {
    _type: ClockType,
    _rcl_clock: Arc<Mutex<rcl_clock_t>>,
    // TODO(luca) Implement jump callbacks
}

impl Clock {
    // TODO(luca) proper error handling
    pub fn new(type_: ClockType) -> Result<Self, RclrsError> {
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

    pub fn clock_type(&self) -> ClockType {
        self._type
    }

    pub fn set_ros_time(&mut self, enable: bool) {
        let mut clock = self._rcl_clock.lock().unwrap();
        if enable {
            // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
            // by the mutex
            unsafe {
                rcl_enable_ros_time_override(&mut *clock);
            }
        } else {
            // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
            // by the mutex
            unsafe {
                rcl_disable_ros_time_override(&mut *clock);
            }
        }
    }

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

    pub fn set_ros_time_override(&self, nanoseconds: i64) {
        let mut clock = self._rcl_clock.lock().unwrap();
        // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
        // by the mutex
        unsafe {
            rcl_set_ros_time_override(&mut *clock, nanoseconds);
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

impl Drop for Clock {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        let rc = unsafe { rcl_clock_fini(&mut *self._rcl_clock.lock().unwrap()) };
        if let Err(e) = to_rclrs_result(rc) {
            panic!("Unable to release Clock. {:?}", e)
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_clock_t {}
