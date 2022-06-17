use crate::context;
use crate::duration;
use crate::error::RclrsError;
use crate::rcl_bindings::*;
use crate::time;
use crate::RclReturnCode;
use parking_lot::{Condvar, Mutex};
use std::os::raw::{c_int, c_void};
use std::sync::Arc;

impl std::default::Default for rcl_allocator_t {
    fn default() -> Self {
        let empty: c_int = 0;
        rcl_allocator_t {
            allocate: None,
            deallocate: None,
            reallocate: None,
            zero_allocate: None,
            state: empty as *mut c_void,
        }
    }
}

impl std::default::Default for rcl_clock_t {
    fn default() -> Self {
        let mut threshold_ = rcl_jump_callback_info_t {
            callback: None,
            threshold: rcl_jump_threshold_t {
                on_clock_change: false,
                min_forward: rcl_duration_t {
                    nanoseconds: 1 as rcl_duration_value_t,
                },
                min_backward: rcl_duration_t {
                    nanoseconds: -1 as rcl_duration_value_t,
                },
            },
            user_data: 0 as *mut c_void,
        };

        rcl_clock_t {
            type_: rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED,
            jump_callbacks: &mut threshold_,
            num_jump_callbacks: usize::default(),
            get_now: None,
            data: 0 as *mut c_void,
            allocator: rcl_allocator_t::default(),
        }
    }
}

struct JumpHandler {
    pre_callback: &'static dyn Fn(),
    post_callback: &'static dyn Fn() -> Mutex<rcl_time_jump_t>,
    threshold: rcl_jump_threshold_t,
}

#[allow(dead_code)]
impl JumpHandler {
    fn new(
        pre_callback: &dyn Fn(),
        post_callback: &dyn Fn() -> Mutex<rcl_time_jump_t>,
        threshold: rcl_jump_threshold_t,
    ) -> Self {
        todo!("implement it");
    }
}

struct Impl {
    rcl_clock_: Mutex<rcl_clock_t>,
    allocator_: Mutex<rcl_allocator_t>,
    thread_handler_: Arc<(Mutex<bool>, Condvar)>,
}

/// The Clock struct
pub struct Clock {
    impl_: Impl,
}

#[allow(dead_code)]
impl Clock {
    /// Function to create a new Clock instance
    pub fn new(clock_type: rcl_clock_type_t) -> Result<Self, RclrsError> {
        let mut impl_ = Impl {
            rcl_clock_: Mutex::new(rcl_clock_t::default()),
            allocator_: Mutex::new(rcl_allocator_t::default()),
            thread_handler_: Arc::new((Mutex::new(bool::default()), Condvar::new())),
        };
        // Safety: variables are wrapped in Mutex
        // raw pointer get converted back to safe types once `get_mut` goes out of scope
        let ret: rcl_ret_t = unsafe {
            rcl_clock_init(
                clock_type,
                impl_.rcl_clock_.get_mut() as *mut rcl_clock_t,
                impl_.allocator_.get_mut() as *mut rcl_allocator_t,
            )
        };

        if ret != 0 {
            return Err(RclrsError::RclError {
                code: RclReturnCode::Error,
                msg: None,
            });
        }

        Ok(Self { impl_ })
    }

    /// Function to get clock type of Clock object
    pub fn get_clock_type(&self) -> rcl_clock_type_t {
        (*self.impl_.rcl_clock_.lock()).type_
    }

    /// Function to get the time from the source at a given instant
    pub fn now(&self) -> Result<time::Time, RclrsError> {
        let now =
            time::Time::new(time::TimeFrom::NanoSecs { ns: 0u64 }, self.get_clock_type()).unwrap();

        // Safety: Variables are wrapped in mutex, to ensure type safety
        // Unsafe variables are converted back to safe types
        let ret = unsafe {
            rcl_clock_get_now(
                &mut *self.impl_.rcl_clock_.lock(),
                &mut (now.get_lock()).nanoseconds,
            )
        };

        if ret != 0 {
            return Err(RclrsError::RclError {
                code: RclReturnCode::Error,
                msg: None,
            });
        }

        Ok(now)
    }

    /// Function to check if ros clock is valid or not
    fn ros_time_is_active(&self) -> bool {
        // Safety: No preconditions for this function
        if unsafe { !rcl_clock_valid(&mut *self.impl_.rcl_clock_.lock()) } {
            return false;
        }

        let mut is_enabled: bool = bool::default();

        // Safety: No preconditions for this function
        let ret = unsafe {
            rcl_is_enabled_ros_time_override(&mut *self.impl_.rcl_clock_.lock(), &mut is_enabled)
        };
        if ret != 0 {
            panic!("Failed to check ros time status")
        }
        is_enabled
    }

    /// Function to return clock handle
    pub fn get_clock_handle(&mut self) -> &mut rcl_clock_t {
        self.impl_.rcl_clock_.get_mut()
    }

    /// Function to sleep until a given time stamp
    pub fn sleep_until(
        &self,
        until: time::Time,
        context: &context::Context,
    ) -> Result<bool, RclrsError> {
        let context_mtx = Arc::clone(&context.rcl_context_mtx);
        // Safety: No preconditions for this function
        if unsafe { !rcl_context_is_valid(&mut *context_mtx.lock()) } {
            return Err(RclrsError::RclError {
                code: RclReturnCode::Error,
                msg: None,
            });
        } else {
            let mut time_source_changed: bool = false;
            match self.get_clock_type() {
                rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED => {
                    return Err(RclrsError::RclError {
                        code: RclReturnCode::Error,
                        msg: None,
                    });
                }
                rcl_clock_type_t::RCL_ROS_TIME => {
                    todo!("implement it for RCL_ROS_TIME");
                }
                rcl_clock_type_t::RCL_SYSTEM_TIME => {
                    let &(ref lock, ref cvar) = &*(Arc::clone(&self.impl_.thread_handler_));

                    let delta = (until.clone() - self.now().unwrap()).to_duration();
                    // Safety: No preconditions for this function
                    while (self.now().unwrap() < until)
                        && unsafe { rcl_context_is_valid(&mut *(*context.rcl_context_mtx).lock()) }
                    {
                        cvar.wait_for(&mut lock.lock(), delta);
                    }
                }
                rcl_clock_type_t::RCL_STEADY_TIME => {
                    todo!("implement it for RCL_STEADY_TIME");
                }
            }
        }
        Ok(true)
    }

    /// Function to sleep for a given duration
    pub fn sleep_for(
        &self,
        duration: duration::Duration,
        context: &context::Context,
    ) -> Result<bool, RclrsError> {
        self.sleep_until(self.now().unwrap() + duration, context)
    }
}
/*
    todo!("add function sleep_until");
    todo!("add function get_clock_mutex");
    todo!("add function on_time_jump");
    todo!("add function create_jump_callback");
*/
