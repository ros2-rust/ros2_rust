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
        let timer_init_result = unsafe {
            // SAFETY: Getting a default value is always safe.
            rcl_timer = rcl_get_zero_initialized_timer();
            let allocator = rcutils_get_default_allocator();
            let mut rcl_clock = clock.rcl_clock.lock().unwrap();
            let mut rcl_context = context.handle.rcl_context.lock().unwrap();
            let callback: rcl_timer_callback_t = Some(timer_callback);
            // Function will return Err(_) only if there isn't enough memory to allocate a clock
            // object.
            rcl_timer_init(
                &mut rcl_timer,
                &mut *rcl_clock,
                &mut *rcl_context,
                period,
                callback,
                allocator,
            )
        };
        to_rclrs_result(timer_init_result).map(|_| {
            Timer {
                rcl_timer: Arc::new(Mutex::new(rcl_timer))
            }
        })
    }

    pub fn timer_period_ns(&self) -> Result<i64, RclrsError> {
        let mut timer_period_ns = 0;
        let get_period_result = unsafe {
            let rcl_timer = self.rcl_timer.lock().unwrap();
            rcl_timer_get_period(
                &* rcl_timer,
                &mut timer_period_ns
            )
        };
        to_rclrs_result(get_period_result).map(|_| {
            timer_period_ns
        })
    }

    pub fn cancel(&self) -> Result<(), RclrsError> {
        let mut rcl_timer = self.rcl_timer.lock().unwrap();
        let cancel_result = unsafe { rcl_timer_cancel(&mut *rcl_timer) };
        to_rclrs_result(cancel_result)
    }

    pub fn is_canceled(&self) -> Result<bool, RclrsError> {
        let mut is_canceled = false;
        let is_canceled_result = unsafe {
            let rcl_timer = self.rcl_timer.lock().unwrap();
            rcl_timer_is_canceled(
                &* rcl_timer,
                &mut is_canceled
            )
        };
        to_rclrs_result(is_canceled_result).map(|_| {
            is_canceled
        })
    }

    pub fn time_since_last_call(&self) -> Result<i64, RclrsError> {
        let mut time_value_ns: i64 = 0;
        let time_since_last_call_result = unsafe {
            let rcl_timer = self.rcl_timer.lock().unwrap();
            rcl_timer_get_time_since_last_call(
                &* rcl_timer,
                &mut time_value_ns
            )
        };
        to_rclrs_result(time_since_last_call_result).map(|_| {
            time_value_ns
        })
    }

    pub fn time_until_next_call(&self) -> Result<i64, RclrsError> {
        let mut time_value_ns: i64 = 0;
        let time_until_next_call_result = unsafe {
            let rcl_timer = self.rcl_timer.lock().unwrap();
            rcl_timer_get_time_until_next_call(
                &* rcl_timer,
                &mut time_value_ns
            )
        };
        to_rclrs_result(time_until_next_call_result).map(|_| {
            time_value_ns
        })
    }

    pub fn reset(&mut self) -> Result<(), RclrsError>
    {
        let mut rcl_timer = self.rcl_timer.lock().unwrap();
        to_rclrs_result(unsafe {rcl_timer_reset(&mut *rcl_timer)})
    }

    pub fn call(&mut self) -> Result<(), RclrsError>
    {
        let mut rcl_timer = self.rcl_timer.lock().unwrap();
        to_rclrs_result(unsafe {rcl_timer_call(&mut *rcl_timer)})
    }

    // handle() -> RCLC Timer Type

    // clock() -> Clock ?

    // is_ready() -> bool

    // reset() -> None

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
    use std::{thread, time};

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<Timer>();
        assert_sync::<Timer>();
    }

    #[test]
    fn test_new_with_system_clock() {
        let clock = Clock::system();
        let context = Context::new(vec![]).unwrap();
        let period: i64 = 1e6 as i64;  // 1 milliseconds.

        let dut = Timer::new(&clock, &context, period);
        assert!(dut.is_ok());
    }

    #[test]
    fn test_new_with_steady_clock() {
        let clock = Clock::steady();
        let context = Context::new(vec![]).unwrap();
        let period: i64 = 1e6 as i64;  // 1 milliseconds.

        let dut = Timer::new(&clock, &context, period);
        assert!(dut.is_ok());
    }

    #[ignore = "SIGSEGV when creating the timer with Clock::with_source()."]
    #[test]
    fn test_new_with_source_clock() {
        let (clock, source) = Clock::with_source();
        // No manual time set, it should default to 0
        assert!(clock.now().nsec == 0);
        let set_time = 1234i64;
        source.set_ros_time_override(set_time);
        // Ros time is set, should return the value that was set
        assert_eq!(clock.now().nsec, set_time);

        let context = Context::new(vec![]).unwrap();
        let period: i64 = 1e6 as i64;  // 1 milliseconds..

        let dut = Timer::new(&clock, &context, period);
        assert!(dut.is_ok());
    }

    #[test]
    fn test_get_period() {
        let clock = Clock::steady();
        let context = Context::new(vec![]).unwrap();
        let period: i64 = 1e6 as i64;  // 1 milliseconds.

        let dut = Timer::new(&clock, &context, period);
        assert!(dut.is_ok());
        let dut = dut.unwrap();
        let period_result = dut.timer_period_ns();
        assert!(period_result.is_ok());
        let period_result = period_result.unwrap();
        assert_eq!(period_result, 1e6 as i64);
    }

    #[test]
    fn test_cancel() {
        let clock = Clock::steady();
        let context = Context::new(vec![]).unwrap();
        let period: i64 = 1e6 as i64;  // 1 milliseconds.

        let dut = Timer::new(&clock, &context, period);
        assert!(dut.is_ok());
        let dut = dut.unwrap();
        assert!(dut.is_canceled().is_ok());
        assert!(!dut.is_canceled().unwrap());
        let cancel_result = dut.cancel();
        assert!(cancel_result.is_ok());
        assert!(dut.is_canceled().is_ok());
        assert!(dut.is_canceled().unwrap());
    }

    #[test]
    fn test_time_since_last_call_before_first_event() {
        let clock = Clock::steady();
        let context = Context::new(vec![]).unwrap();
        let period_ns: i64 = 2e6 as i64;  // 2 milliseconds.
        let sleep_period_ms = time::Duration::from_millis(1);

        let dut = Timer::new(&clock, &context, period_ns);
        assert!(dut.is_ok());
        let dut = dut.unwrap();
        thread::sleep(sleep_period_ms);
        let time_since_last_call = dut.time_since_last_call();
        assert!(time_since_last_call.is_ok());
        let time_since_last_call = time_since_last_call.unwrap();
        assert!(time_since_last_call > 9e5 as i64, "time_since_last_call: {}", time_since_last_call);
    }

    #[test]
    fn test_time_until_next_call_before_first_event() {
        let clock = Clock::steady();
        let context = Context::new(vec![]).unwrap();
        let period_ns: i64 = 2e6 as i64;  // 2 milliseconds.
        let dut = Timer::new(&clock, &context, period_ns);
        assert!(dut.is_ok());
        let dut = dut.unwrap();
        let time_until_next_call = dut.time_until_next_call();
        assert!(time_until_next_call.is_ok());
        let time_until_next_call = time_until_next_call.unwrap();
        assert!(time_until_next_call < period_ns, "time_until_next_call: {}", time_until_next_call);
    }

    #[test]
    fn test_reset() {
        let tolerance = 20e4 as i64;
        let clock = Clock::steady();
        let context = Context::new(vec![]).unwrap();
        let period_ns: i64 = 2e6 as i64;  // 2 milliseconds.
        let mut dut = Timer::new(&clock, &context, period_ns).unwrap();
        let elapsed = period_ns - dut.time_until_next_call().unwrap();
        assert!(elapsed < tolerance , "elapsed before reset: {}", elapsed);
        thread::sleep(time::Duration::from_millis(1));
        assert!(dut.reset().is_ok());
        let elapsed = period_ns - dut.time_until_next_call().unwrap();
        assert!(elapsed < tolerance , "elapsed after reset: {}", elapsed);
    }

    #[test]
    fn test_call() {
        let tolerance = 20e4 as i64;
        let clock = Clock::steady();
        let context = Context::new(vec![]).unwrap();
        let period_ns: i64 = 1e6 as i64;  // 1 millisecond.
        let mut dut = Timer::new(&clock, &context, period_ns).unwrap();
        let elapsed = period_ns - dut.time_until_next_call().unwrap();
        assert!(elapsed < tolerance , "elapsed before reset: {}", elapsed);
        
        thread::sleep(time::Duration::from_micros(1500));

        let elapsed = period_ns - dut.time_until_next_call().unwrap();
        assert!(elapsed > 1500000i64, "time_until_next_call before call: {}", elapsed);
        
        assert!(dut.call().is_ok());
        
        let elapsed = dut.time_until_next_call().unwrap();
        assert!(elapsed < 500000i64, "time_until_next_call after call: {}", elapsed);
    }
}
