use crate::{
    clock::Clock, context::Context, error::RclrsError, rcl_bindings::*, to_rclrs_result
};
use std::sync::{Arc, Mutex};


#[derive(Debug)]
pub struct Timer {
    rcl_timer: Arc<Mutex<rcl_timer_t>>,
}

unsafe extern "C" fn timer_callback(_: *mut rcl_timer_t, time_since_last_callback_ns: i64) {
    println!("timer_callback, time_since_last_callback_ns {0}", time_since_last_callback_ns);
}

impl Timer {
    pub fn new(clock: &Clock, context: &Context, period: i64) -> Result<Timer, RclrsError> {
        let mut rcl_timer;
        let timer_init_result;
        unsafe {
            // SAFETY: Getting a default value is always safe.
            rcl_timer = rcl_get_zero_initialized_timer();
            let allocator = rcutils_get_default_allocator();
            let mut rcl_clock = clock.rcl_clock.lock().unwrap();
            let mut rcl_context = context.handle.rcl_context.lock().unwrap();
            let callback: rcl_timer_callback_t = Some(timer_callback);
            // Function will return Err(_) only if there isn't enough memory to allocate a clock
            // object.
            timer_init_result = rcl_timer_init(
                &mut rcl_timer,
                &mut *rcl_clock,
                &mut *rcl_context,
                period,
                callback,
                allocator,
            );
        }        
        to_rclrs_result(timer_init_result).map(|_| {
            Timer {
                rcl_timer: Arc::new(Mutex::new(rcl_timer))
            }
        })
    }

    // handle() -> RCLC Timer Type

    // destroy() -> None 

    // clock() -> Clock ?

    // timer_period_ns -> i64 ?

    // is_ready() -> bool

    // is_cancelled() -> bool

    // cancel() -> None

    // reset() -> None

    // time_since_last_call() -> i64

    // time_until_next_call() -> Option<i64>

}

impl Drop for rcl_timer_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        let rc = unsafe { rcl_timer_fini(&mut *self) };
        if let Err(e) = to_rclrs_result(rc) {
            panic!("Unable to release Timer. {:?}", e)
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_timer_t {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<Timer>();
        assert_sync::<Timer>();
    }
}
