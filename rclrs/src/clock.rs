use crate::error::RclrsError;
use crate::rcl_bindings::*;
use crate::RclReturnCode;
use crate::duration;
use crate::time;
use parking_lot::{Mutex, MutexGuard};
use std::os::raw::{c_void, c_int};

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
                    nanoseconds: 1 as rcl_duration_value_t
                },
                min_backward: rcl_duration_t {
                    nanoseconds: -1 as rcl_duration_value_t,
                },
            } ,
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
    pre_callback: &'static dyn Fn() -> (),
    post_callback: &'static dyn Fn() -> Mutex<rcl_time_jump_t>,
    threshold: rcl_jump_threshold_t,
}

impl JumpHandler {
    fn new(pre_callback: &dyn Fn() -> (),
           post_callback: &dyn Fn() -> Mutex<rcl_time_jump_t>,
           threshold: rcl_jump_threshold_t) -> Self {
        todo!("implement it");
    }
}

struct Impl {
    rcl_clock_: Mutex<rcl_clock_t>,
    allocator_: Mutex<rcl_allocator_t>,
    //clock_mutex_: Mutex<_>,
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
            allocator_: Mutex::new(rcl_allocator_t::default())
        };
        // Safety: variables are wrapped in Mutex
        // raw pointer get converted back to safe types once `get_mut` goes out of scope
        let ret: rcl_ret_t = unsafe {
            rcl_clock_init(clock_type,
                           impl_.rcl_clock_.get_mut() as *mut rcl_clock_t,
                           impl_.allocator_.get_mut() as *mut rcl_allocator_t)
        };

        if ret != 0 {
            return Err(RclrsError::RclError {
                code: RclReturnCode::Error,
                msg: None,
            });
        }

        Ok(Self{
            impl_,
        })
    }

    pub fn get_clock_type(&self) -> rcl_clock_type_t {
        (*self.impl_.rcl_clock_.lock()).type_
    }

    pub fn now(&self) -> time::Time {

        let now = time::Time::new(
            time::TimeFrom::NanoSecs{ns: 0u64},
            self.get_clock_type(),
        ).unwrap();

        let ret = rcl_clock_get_now(&mut *self.impl_.rcl_clock_.lock(), &mut now.get_lock().nanoseconds)

    }

    todo!("add function sleep_until");
    todo!("add function sleep_for");
    todo!("add function ros_time_is_active");
    todo!("add function get_clock_handle");
    todo!("add function get_clock_type");
    todo!("add function get_clock_mutex");
    todo!("add function on_time_jump");
    todo!("add function create_jump_callback");
}
