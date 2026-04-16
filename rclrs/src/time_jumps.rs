use crate::error::ToResult;
use crate::rcl_bindings::{rcl_clock_change_e, rcl_time_jump_t};
use crate::{rcl_bindings::rcl_jump_threshold_t, ENTITY_LIFECYCLE_MUTEX};
use std::time::Duration;
use std::{
    ffi::c_void,
    sync::{Arc, Weak},
};

use crate::{rcl_bindings, Clock, RclrsError};

type ClockTimeJumpCallback = dyn Fn(&rcl_bindings::rcl_time_jump_t) + Send + 'static + Sync;
unsafe extern "C" fn on_post_time_jump(
    time_jump: *const rcl_bindings::rcl_time_jump_t,
    before_jump: bool,
    user_data: *mut c_void,
) {
    if before_jump {
        return;
    }
    let weak = &*(user_data as *const Weak<ClockTimeJumpCallback>);
    let jump = &*time_jump;
    if let Some(cb) = weak.upgrade() {
        cb(jump);
    }
}

/// Holding a reference to this handle keeps the jump callback alive
pub struct ClockTimeJumpCallbackHandle {
    clock: Clock,
    user_data: *mut c_void,
    _cb: Arc<ClockTimeJumpCallback>,
}
unsafe impl Send for ClockTimeJumpCallbackHandle {}
unsafe impl Sync for ClockTimeJumpCallbackHandle {}

/// Condition for typed clock time jump callbacks.
/// ```
/// # use rclrs::*;
///
/// # ClockTimeJumpConditions::on_min_forward(Duration::as_secs(1))
/// # ClockTimeJumpConditions::on_min_backward(Duration::as_secs(1))
/// # ClockTimeJumpConditions::on_min_forward_or_backward(Duration::as_secs(1), Duration::as_secs(1))
/// # ClockTimeJumpConditions::on_clock_change()
/// ```
/// See [`crate::ClockTimeCondition`]
pub trait ClockTimeJumpCondition: Send + Sync + 'static + std::fmt::Debug {
    /// The jump payload depending on the jump event.
    type CallbackParameter: Send + 'static + std::fmt::Debug;

    /// Converts jump condition into rcl data type
    fn into_rcl_jump_threshold(&self) -> rcl_jump_threshold_t;

    /// Converts rcl change payload into callback parameter
    fn into_callback_param(data: &rcl_time_jump_t) -> Self::CallbackParameter;
}

#[derive(Debug)]
struct OnMinForwardJump(Duration);
#[derive(Debug)]
struct OnMinBackwardJump(Duration);
#[derive(Debug)]
struct OnMinForwardBackwardJump(Duration, Duration);
#[derive(Debug)]
struct OnClockChange;

impl ClockTimeJumpCondition for OnMinBackwardJump {
    type CallbackParameter = i64;

    fn into_rcl_jump_threshold(&self) -> rcl_jump_threshold_t {
        rcl_jump_threshold_t {
            min_backward: rcl_bindings::rcl_duration_s {
                nanoseconds: -(self.0.as_nanos() as i64),
            },
            min_forward: rcl_bindings::rcl_duration_s { nanoseconds: 0 },
            on_clock_change: false,
        }
    }
    fn into_callback_param(data: &rcl_time_jump_t) -> i64 {
        data.delta.nanoseconds
    }
}
impl ClockTimeJumpCondition for OnMinForwardJump {
    type CallbackParameter = i64;

    fn into_rcl_jump_threshold(&self) -> rcl_jump_threshold_t {
        rcl_jump_threshold_t {
            min_backward: rcl_bindings::rcl_duration_s { nanoseconds: 0 },
            min_forward: rcl_bindings::rcl_duration_s {
                nanoseconds: self.0.as_nanos() as i64,
            },
            on_clock_change: false,
        }
    }
    fn into_callback_param(data: &rcl_time_jump_t) -> i64 {
        data.delta.nanoseconds
    }
}
impl ClockTimeJumpCondition for OnMinForwardBackwardJump {
    type CallbackParameter = i64;

    fn into_rcl_jump_threshold(&self) -> rcl_jump_threshold_t {
        rcl_jump_threshold_t {
            min_backward: rcl_bindings::rcl_duration_s {
                nanoseconds: -(self.0.as_nanos() as i64),
            },
            min_forward: rcl_bindings::rcl_duration_s {
                nanoseconds: self.1.as_nanos() as i64,
            },
            on_clock_change: false,
        }
    }
    fn into_callback_param(data: &rcl_time_jump_t) -> i64 {
        data.delta.nanoseconds
    }
}
impl ClockTimeJumpCondition for OnClockChange {
    type CallbackParameter = ClockChange;

    fn into_rcl_jump_threshold(&self) -> rcl_jump_threshold_t {
        rcl_jump_threshold_t {
            min_backward: rcl_bindings::rcl_duration_s { nanoseconds: 0 },
            min_forward: rcl_bindings::rcl_duration_s { nanoseconds: 0 },
            on_clock_change: true,
        }
    }
    fn into_callback_param(data: &rcl_time_jump_t) -> ClockChange {
        match data.clock_change {
            rcl_clock_change_e::RCL_ROS_TIME_ACTIVATED => ClockChange::RclRosTimeActivated,
            rcl_clock_change_e::RCL_ROS_TIME_DEACTIVATED => ClockChange::RclRosTimeDeactivated,
            _ => {
                unreachable!()
            }
        }
    }
}

/// Provides constructors for [`crate::ClockTimeJumpCondition`]s.
pub struct ClockTimeJumpConditions {}
impl ClockTimeJumpConditions {
    /// Returns [`crate::ClockTimeJumpCondition`] for time jumps further forwards than given duration.
    /// The callback accepts the time jump delta.
    pub fn on_min_forward(
        min_forward: Duration,
    ) -> impl ClockTimeJumpCondition<CallbackParameter = i64> {
        OnMinForwardJump(min_forward)
    }
    /// Returns [`crate::ClockTimeJumpCondition`] for time jumps further backwards than given duration.
    /// The callback accepts the time jump delta.
    pub fn on_min_backward(
        min_backward: Duration,
    ) -> impl ClockTimeJumpCondition<CallbackParameter = i64> {
        OnMinBackwardJump(min_backward)
    }
    /// Returns [`crate::ClockTimeJumpCondition`] for time jumps further forwards/backwards than given durations.
    /// The callback accepts the time jump delta.
    pub fn on_min_forward_or_backward(
        min_backward: Duration,
        min_forward: Duration,
    ) -> impl ClockTimeJumpCondition<CallbackParameter = i64> {
        OnMinForwardBackwardJump(min_backward, min_forward)
    }

    /// Returns [`crate::ClockTimeJumpCondition`] for ros time activation/deactivation.
    pub fn on_clock_change() -> impl ClockTimeJumpCondition<CallbackParameter = ClockChange> {
        OnClockChange {}
    }
}

/// Represents a clock type change.
#[derive(Debug)]
pub enum ClockChange {
    /// Triggered when RclRosTime is activated.
    RclRosTimeActivated,
    /// Triggered when RclRosTime is deactivated.
    RclRosTimeDeactivated,
}

impl ClockTimeJumpCallbackHandle {
    pub(crate) fn new<T: ClockTimeJumpCondition>(
        clock: Clock,
        jump_condition: T,
        cb: impl Fn(T::CallbackParameter) + Send + Sync + 'static,
    ) -> Result<Self, RclrsError> {
        let rcl_jump_condition = jump_condition.into_rcl_jump_threshold();
        let cb: Arc<ClockTimeJumpCallback> = Arc::new(move |jump: &rcl_time_jump_t| {
            cb(T::into_callback_param(jump));
        });

        let user_data = Box::into_raw(Box::new(Arc::downgrade(&cb))) as *mut c_void;
        {
            // register
            let mut rcl_clock = clock.get_rcl_clock().lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            let rcl_ret = unsafe {
                rcl_bindings::rcl_clock_add_jump_callback(
                    &mut *rcl_clock,
                    rcl_jump_condition,
                    Some(on_post_time_jump),
                    user_data,
                )
                .ok()
            };
            if let Err(err) = rcl_ret {
                // Reclaim box
                unsafe {
                    let _ = Box::from_raw(user_data as *mut Weak<ClockTimeJumpCallback>);
                }
                return Err(err);
            }
        }
        let handle = ClockTimeJumpCallbackHandle {
            clock,
            user_data,
            _cb: cb,
        };
        Ok(handle)
    }
}

impl Drop for ClockTimeJumpCallbackHandle {
    fn drop(&mut self) {
        let mut rcl_clock = self.clock.get_rcl_clock().lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // No clue what to do with this here. Logging in Drop is discouraged
        // print error to stderr?
        let _rcl_ret = unsafe {
            let rcl_ret = rcl_bindings::rcl_clock_remove_jump_callback(
                &mut *rcl_clock,
                Some(on_post_time_jump),
                self.user_data,
            );
            // Reclaim
            let _ = Box::from_raw(self.user_data as *mut Weak<ClockTimeJumpCallback>);
            rcl_ret.ok()
        };
    }
}
