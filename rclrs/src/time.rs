use crate::error::RclrsError;
use crate::RclReturnCode;
use crate::rcl_bindings::*;
use std::os::raw::{c_void, c_uint};
use std::time::Duration;


/// Function to get the current steady time
fn rcl_get_steady_time(current_time: *mut rcl_time_point_value_t) -> rcl_ret_t {
    // SAFETY: No preconditions for this function
    unsafe {
        rcutils_steady_time_now(current_time)
    }
}

/// Function to return the current system time
fn rcl_get_system_time(current_time: *mut rcl_time_point_value_t) -> rcl_ret_t {
    // SAFETY: No preconditions for this function
    unsafe {
        rcutils_system_time_now(current_time)
    }
}

impl rcl_clock_t {
    /*
    /// Function to create a new rcl_clock_t instance
    pub fn new(type_: rcl_clock_type_t, allocator: rcl_allocator_t) -> Self {
        unsafe {
        let empty_data = c_void::null; 
        Self {
            type_: type_,
            jump_callbacks: None,
            num_jump_callbacks: 0 as usize,
            get_now: None,
            data: empty_data,
            allocator: allocator,
        }
        }
    }*/
    /// Function to check validity of the clock
    fn rcl_clock_valid(&self) -> bool {
        let mut _flag: bool = false;
        match self.type_ {
            rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED => _flag = false,
            _ => _flag = true,
        }

        if !_flag {
            match self.get_now {
                Option::None => false,
                _ => true,
            }
        } else {
            _flag
        }
    }
}

pub struct Time {
    seconds: i32,
    nanoseconds: u32,
    clock_type: rcl_clock_type_t,
}

impl Time {
    /// Function to create a new instance of Time
    pub fn new(seconds: i32, 
    nanoseconds: u32, 
    clock_type: rcl_clock_type_t) -> Result<Self, RclrsError> {
        if seconds < 0 {
            return Err(RclrsError::RclError{
                code: RclReturnCode::InvalidArgument,
                msg: None
            });
        }

        Ok(Self {
            seconds,
            nanoseconds,
            clock_type,
        })
    }
}
