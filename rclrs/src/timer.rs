use crate::{
    clock::Clock, context::ContextHandle, error::RclrsError, log_error, rcl_bindings::*, Node,
    RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, ReadyKind, ToLogParams, ToResult, Waitable,
    WaitableLifecycle, WorkScope, Worker, WorkerCommands, ENTITY_LIFECYCLE_MUTEX,
};
// TODO: fix me when the callback type is properly defined.
// use std::fmt::Debug;
use std::{
    any::Any,
    sync::{Arc, Mutex, Weak},
    time::Duration,
};

mod any_timer_callback;
pub use any_timer_callback::*;

mod timer_options;
pub use timer_options::*;

mod into_node_timer_callback;
pub use into_node_timer_callback::*;

mod into_worker_timer_callback;
pub use into_worker_timer_callback::*;

/// Struct for executing periodic events.
///
/// The executor needs to be [spinning][1] for a timer's callback to be triggered.
///
/// Timers can be created by a [`Node`] using one of these methods:
/// - [`NodeState::create_timer_repeating`][2]
/// - [`NodeState::create_timer_oneshot`][3]
/// - [`NodeState::create_timer_inert`][4]
///
/// Timers can also be created by a [`Worker`], in which case they can access the worker's payload:
/// - [`WorkerState::create_timer_repeating`][5]
/// - [`WorkerState::create_timer_oneshot`][6]
/// - [`WorkerState::create_timer_inert`][7]
///
/// The API of timers is given by [`TimerState`].
///
/// [1]: crate::Executor::spin
/// [2]: crate::NodeState::create_timer_repeating
/// [3]: crate::NodeState::create_timer_oneshot
/// [4]: crate::NodeState::create_timer_inert
/// [5]: crate::WorkerState::create_timer_repeating
/// [6]: crate::WorkerState::create_timer_oneshot
/// [7]: crate::WorkerState::create_timer_inert
pub type Timer = Arc<TimerState<Node>>;

/// A [`Timer`] that runs on a [`Worker`].
///
/// Create a worker timer using [`create_timer_repeating`][1],
/// [`create_timer_oneshot`][2], or [`create_timer_inert`][3].
///
/// [1]: crate::WorkerState::create_timer_repeating
/// [2]: crate::WorkerState::create_timer_oneshot
/// [3]: crate::WorkerState::create_timer_inert
pub type WorkerTimer<Payload> = Arc<TimerState<Worker<Payload>>>;

/// The inner state of a [`Timer`].
///
/// This is public so that you can choose to create a [`Weak`] reference to it
/// if you want to be able to refer to a [`Timer`] in a non-owning way. It is
/// generally recommended to manage the `TimerState` inside of an [`Arc`], and
/// [`Timer`] is provided as a convenience alias for that.
///
/// The public API of [`Timer`] is implemented via `TimerState`.
///
/// Timers that run inside of a [`Worker`] are represented by [`WorkerTimer`].
pub struct TimerState<Scope: WorkScope> {
    pub(crate) handle: Arc<TimerHandle>,
    /// The callback function that runs when the timer is due.
    callback: Mutex<Option<AnyTimerCallback<Scope>>>,
    /// What was the last time lapse between calls to this timer
    last_elapse: Mutex<Duration>,
    /// We use Mutex<Option<>> here because we need to construct the TimerState object
    /// before we can get the lifecycle handle.
    #[allow(unused)]
    lifecycle: Mutex<Option<WaitableLifecycle>>,
    /// We optionally hold onto a live node if the timer is depending on node time.
    #[allow(unused)]
    node: Option<Node>,
    _ignore: std::marker::PhantomData<Scope>,
}

impl<Scope: WorkScope> TimerState<Scope> {
    /// Gets the period of the timer
    pub fn get_timer_period(&self) -> Result<Duration, RclrsError> {
        let mut timer_period_ns = 0;
        unsafe {
            // SAFETY: The unwrap is safe here since we never use the rcl_timer
            // in a way that could panic while the mutex is locked.
            let rcl_timer = self.handle.rcl_timer.lock().unwrap();

            // SAFETY: The rcl_timer is kept valid by the TimerState. This C
            // function call is thread-safe and only requires a valid rcl_timer
            // to be passed in.
            rcl_timer_get_period(&*rcl_timer, &mut timer_period_ns)
        }
        .ok()?;

        rcl_duration(timer_period_ns)
    }

    /// Cancels the timer, stopping the execution of the callback.
    ///
    /// [`TimerState::is_ready`] will always return false while the timer is in
    /// a cancelled state. [`TimerState::reset`] can be used to revert the timer
    /// out of the cancelled state.
    pub fn cancel(&self) -> Result<(), RclrsError> {
        let cancel_result = unsafe {
            // SAFETY: The unwrap is safe here since we never use the rcl_timer
            // in a way that could panic while the mutex is locked.
            let mut rcl_timer = self.handle.rcl_timer.lock().unwrap();

            // SAFETY: The rcl_timer is kept valid by the TimerState. This C
            // function call is thread-safe and only requires a valid rcl_timer
            // to be passed in.
            rcl_timer_cancel(&mut *rcl_timer)
        }
        .ok()?;
        Ok(cancel_result)
    }

    /// Checks whether the timer is canceled or not
    pub fn is_canceled(&self) -> Result<bool, RclrsError> {
        let mut is_canceled = false;
        unsafe {
            // SAFETY: The unwrap is safe here since we never use the rcl_timer
            // in a way that could panic while the mutex is locked.
            let rcl_timer = self.handle.rcl_timer.lock().unwrap();

            // SAFETY: The rcl_timer is kept valid by the TimerState. This C
            // function call is thread-safe and only requires a valid rcl_timer
            // to be passed in.
            rcl_timer_is_canceled(&*rcl_timer, &mut is_canceled)
        }
        .ok()?;
        Ok(is_canceled)
    }

    /// Get the last time lapse between calls to the timer.
    ///
    /// This is different from [`Self::time_since_last_call`] because it remains
    /// constant between calls to the Timer.
    ///
    /// It keeps track of the what the value of [`Self::time_since_last_call`]
    /// was immediately before the most recent call to the callback. This will
    /// be [`Duration::ZERO`] if the `Timer` has never been triggered.
    pub fn last_elapse(&self) -> Duration {
        *self.last_elapse.lock().unwrap()
    }

    /// Retrieves the time since the last call to the callback
    pub fn time_since_last_call(&self) -> Result<Duration, RclrsError> {
        let mut time_value_ns: i64 = 0;
        unsafe {
            // SAFETY: The unwrap is safe here since we never use the rcl_timer
            // in a way that could panic while the mutex is locked.
            let rcl_timer = self.handle.rcl_timer.lock().unwrap();

            // SAFETY: The rcl_timer is kept valid by the TimerState. This C
            // function call is thread-safe and only requires a valid rcl_timer
            // to be passed in.
            rcl_timer_get_time_since_last_call(&*rcl_timer, &mut time_value_ns)
        }
        .ok()?;

        rcl_duration(time_value_ns)
    }

    /// Retrieves the time until the next call of the callback
    pub fn time_until_next_call(&self) -> Result<Duration, RclrsError> {
        let mut time_value_ns: i64 = 0;
        unsafe {
            // SAFETY: The unwrap is safe here since we never use the rcl_timer
            // in a way that could panic while the mutex is locked.
            let rcl_timer = self.handle.rcl_timer.lock().unwrap();

            // SAFETY: The rcl_timer is kept valid by the TimerState. This C
            // function call is thread-safe and only requires a valid rcl_timer
            // to be passed in.
            rcl_timer_get_time_until_next_call(&*rcl_timer, &mut time_value_ns)
        }
        .ok()?;

        rcl_duration(time_value_ns)
    }

    /// Resets the timer.
    ///
    /// For all timers it will reset the last call time to now. For cancelled
    /// timers it will revert the timer to no longer being cancelled.
    pub fn reset(&self) -> Result<(), RclrsError> {
        // SAFETY: The unwrap is safe here since we never use the rcl_timer
        // in a way that could panic while the mutex is locked.
        let mut rcl_timer = self.handle.rcl_timer.lock().unwrap();

        unsafe {
            // SAFETY: The rcl_timer is kept valid by the TimerState. This C
            // function call is thread-safe and only requires a valid rcl_timer
            // to be passed in.
            rcl_timer_reset(&mut *rcl_timer)
        }
        .ok()
    }

    /// Checks if the timer is ready (not canceled)
    pub fn is_ready(&self) -> Result<bool, RclrsError> {
        let is_ready = unsafe {
            // SAFETY: The timer is valid because its lifecycle is managed by
            // this struct. There are no other preconditions.
            let mut is_ready: bool = false;
            let rcl_timer = self.handle.rcl_timer.lock().unwrap();
            rcl_timer_is_ready(&*rcl_timer, &mut is_ready).ok()?;
            is_ready
        };

        Ok(is_ready)
    }

    /// Get the clock that this timer runs on.
    pub fn clock(&self) -> &Clock {
        &self.handle.clock
    }

    /// Set a new callback for the timer. This will return whatever callback
    /// was already present unless you are calling the function from inside of
    /// the timer's callback, in which case you will receive [`None`].
    ///
    /// See also:
    /// * [`Self::set_repeating`]
    /// * [`Self::set_oneshot`]
    /// * [`Self::set_inert`].
    pub fn set_callback(
        &self,
        callback: AnyTimerCallback<Scope>,
    ) -> Option<AnyTimerCallback<Scope>> {
        self.callback.lock().unwrap().replace(callback)
    }

    /// Remove the callback from the timer.
    ///
    /// This does not cancel the timer; it will continue to wake up and be
    /// triggered at its regular period. However, nothing will happen when the
    /// timer is triggered until you give a new callback to the timer.
    ///
    /// You can give the timer a new callback at any time by calling:
    /// * [`Self::set_repeating`]
    /// * [`Self::set_oneshot`]
    pub fn set_inert(&self) -> Option<AnyTimerCallback<Scope>> {
        self.set_callback(AnyTimerCallback::Inert)
    }

    /// Creates a new timer. Users should call one of [`Node::create_timer`],
    /// [`Node::create_timer_repeating`], [`Node::create_timer_oneshot`], or
    /// [`Node::create_timer_inert`].
    pub(crate) fn create<'a>(
        period: Duration,
        clock: Clock,
        callback: AnyTimerCallback<Scope>,
        commands: &Arc<WorkerCommands>,
        context: &ContextHandle,
        node: Option<Node>,
    ) -> Result<Arc<Self>, RclrsError> {
        let period = period.as_nanos() as i64;

        // Callbacks will be handled at the rclrs layer.
        let rcl_timer_callback: rcl_timer_callback_t = None;

        let rcl_timer = Arc::new(Mutex::new(unsafe {
            // SAFETY: Zero-initializing a timer is always safe
            rcl_get_zero_initialized_timer()
        }));

        unsafe {
            let mut rcl_clock = clock.get_rcl_clock().lock().unwrap();
            let mut rcl_context = context.rcl_context.lock().unwrap();

            // SAFETY: Getting a default value is always safe.
            let allocator = rcutils_get_default_allocator();

            let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();

            // The API for initializing timers changed with the kilted releaase.
            #[cfg(any(ros_distro = "humble", ros_distro = "jazzy"))]
            {
                // SAFETY: We lock the lifecycle mutex since rcl_timer_init is not
                // thread-safe.
                rcl_timer_init(
                    &mut *rcl_timer.lock().unwrap(),
                    &mut *rcl_clock,
                    &mut *rcl_context,
                    period,
                    rcl_timer_callback,
                    allocator,
                )
            }

            // The API for initializing timers changed with the kilted releaase.
            // This new API allows you to opt out of automatically starting the
            // timer as soon as it is created. We could consider exposing this
            // capability to the user, but for now we are just telling it to
            // immediately start the timer.
            #[cfg(not(any(ros_distro = "humble", ros_distro = "jazzy")))]
            {
                // SAFETY: We lock the lifecycle mutex since rcl_timer_init is not
                // thread-safe.
                rcl_timer_init2(
                    &mut *rcl_timer.lock().unwrap(),
                    &mut *rcl_clock,
                    &mut *rcl_context,
                    period,
                    rcl_timer_callback,
                    allocator,
                    true,
                )
            }
        }
        .ok()?;

        let timer = Arc::new(TimerState {
            handle: Arc::new(TimerHandle { rcl_timer, clock }),
            callback: Mutex::new(Some(callback)),
            last_elapse: Mutex::new(Duration::ZERO),
            lifecycle: Mutex::default(),
            node,
            _ignore: Default::default(),
        });

        let (waitable, lifecycle) = Waitable::new(
            Box::new(TimerExecutable::<Scope> {
                timer: Arc::downgrade(&timer),
                handle: Arc::clone(&timer.handle),
            }),
            Some(Arc::clone(commands.get_guard_condition())),
        );

        *timer.lifecycle.lock().unwrap() = Some(lifecycle);

        commands.add_to_wait_set(waitable);

        Ok(timer)
    }

    /// Force the timer to be called, even if it is not ready to be triggered yet.
    /// We could consider making this public, but the behavior may confuse users.
    fn call(self: &Arc<Self>, any_payload: &mut dyn Any) -> Result<(), RclrsError> {
        // Keep track of the time elapsed since the last call. We need to run
        // this before we trigger rcl_call.
        let last_elapse = self.time_since_last_call().unwrap_or(Duration::ZERO);
        *self.last_elapse.lock().unwrap() = last_elapse;

        if let Err(err) = self.rcl_call() {
            log_error!("timer", "Unable to call timer: {err:?}",);
        }

        let Some(callback) = self.callback.lock().unwrap().take() else {
            log_error!(
                "timer".once(),
                "Timer is missing its callback information. This should not \
                be possible, please report it to the maintainers of rclrs.",
            );
            return Ok(());
        };

        let Some(payload) = any_payload.downcast_mut::<Scope::Payload>() else {
            return Err(RclrsError::InvalidPayload {
                expected: std::any::TypeId::of::<Scope::Payload>(),
                received: (*any_payload).type_id(),
            });
        };

        match callback {
            AnyTimerCallback::Repeating(mut callback) => {
                callback(payload, self);
                self.restore_callback(AnyTimerCallback::Repeating(callback).into());
            }
            AnyTimerCallback::OneShot(callback) => {
                callback(payload, self);
                self.restore_callback(AnyTimerCallback::Inert);
            }
            AnyTimerCallback::Inert => {
                self.restore_callback(AnyTimerCallback::Inert);
            }
        }

        Ok(())
    }

    /// Updates the state of the rcl_timer to know that it has been called. This
    /// should only be called by [`Self::call`].
    ///
    /// The callback held by the rcl_timer is null because we store the callback
    /// in the [`Timer`] struct. This means there are no side-effects to this
    /// except to keep track of when the timer has been called.
    fn rcl_call(&self) -> Result<(), RclrsError> {
        // SAFETY: The unwrap is safe here since we never use the rcl_timer
        // in a way that could panic while the mutex is locked.
        let mut rcl_timer = self.handle.rcl_timer.lock().unwrap();

        unsafe {
            // SAFETY: The rcl_timer is kept valid by the TimerState. This C
            // function call is thread-safe and only requires a valid rcl_timer
            // to be passed in.
            rcl_timer_call(&mut *rcl_timer)
        }
        .ok()
    }

    /// Used by [`Timer::execute`] to restore the state of the callback if and
    /// only if the user has not already set a new callback.
    fn restore_callback(&self, callback: AnyTimerCallback<Scope>) {
        let mut self_callback = self.callback.lock().unwrap();
        if self_callback.is_none() {
            *self_callback = Some(callback);
        }
    }
}

impl TimerState<Node> {
    /// Set a repeating callback for this timer.
    ///
    /// See also:
    /// * [`Self::set_oneshot`]
    /// * [`Self::set_inert`]
    pub fn set_repeating<Args>(
        &self,
        f: impl IntoNodeTimerRepeatingCallback<Args>,
    ) -> Option<AnyTimerCallback<Node>> {
        self.set_callback(f.into_node_timer_repeating_callback())
    }

    /// Set a one-shot callback for the timer.
    ///
    /// The next time the timer is triggered, the callback will be set to
    /// [`AnyTimerCallback::Inert`] after this callback is triggered. To keep the
    /// timer useful, you can reset the Timer callback at any time, including
    /// inside the one-shot callback itself.
    ///
    /// See also:
    /// * [`Self::set_repeating`]
    /// * [`Self::set_inert`]
    pub fn set_oneshot<Args>(
        &self,
        f: impl IntoNodeTimerOneshotCallback<Args>,
    ) -> Option<AnyTimerCallback<Node>> {
        self.set_callback(f.into_node_timer_oneshot_callback())
    }
}

impl<Payload: 'static + Send + Sync> TimerState<Worker<Payload>> {
    /// Set a repeating callback for this worker timer.
    ///
    /// See also:
    /// * [`Self::set_worker_oneshot`]
    /// * [`Self::set_inert`]
    pub fn set_worker_repeating<Args>(
        &self,
        f: impl IntoWorkerTimerRepeatingCallback<Worker<Payload>, Args>,
    ) -> Option<AnyTimerCallback<Worker<Payload>>> {
        self.set_callback(f.into_worker_timer_repeating_callback())
    }

    /// Set a one-shot callback for the worker timer.
    ///
    /// The next time the timer is triggered, the callback will be set to
    /// [`AnyTimerCallback::Inert`] after this callback is triggered. To keep the
    /// timer useful, you can reset the Timer callback at any time, including
    /// inside the one-shot callback itself.
    ///
    /// See also:
    /// * [`Self::set_worker_repeating`]
    /// * [`Self::set_inert`]
    pub fn set_worker_oneshot<Args>(
        &self,
        f: impl IntoWorkerTimerOneshotCallback<Worker<Payload>, Args>,
    ) -> Option<AnyTimerCallback<Worker<Payload>>> {
        self.set_callback(f.into_worker_timer_oneshot_callback())
    }
}

struct TimerExecutable<Scope: WorkScope> {
    timer: Weak<TimerState<Scope>>,
    handle: Arc<TimerHandle>,
}

impl<Scope: WorkScope> RclPrimitive for TimerExecutable<Scope> {
    unsafe fn execute(
        &mut self,
        ready: ReadyKind,
        payload: &mut dyn Any,
    ) -> Result<(), RclrsError> {
        ready.for_basic()?;
        if let Some(timer) = self.timer.upgrade() {
            if timer.is_ready()? {
                timer.call(payload)?;
            }
        }

        Ok(())
    }

    fn kind(&self) -> RclPrimitiveKind {
        RclPrimitiveKind::Timer
    }

    fn handle(&self) -> RclPrimitiveHandle {
        RclPrimitiveHandle::Timer(self.handle.rcl_timer.lock().unwrap())
    }
}

impl<Scope: WorkScope> PartialEq for TimerState<Scope> {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.handle.rcl_timer, &other.handle.rcl_timer)
    }
}

fn rcl_duration(duration_value_ns: i64) -> Result<Duration, RclrsError> {
    if duration_value_ns < 0 {
        Err(RclrsError::NegativeDuration(duration_value_ns))
    } else {
        Ok(Duration::from_nanos(duration_value_ns as u64))
    }
}

/// Manage the lifecycle of an `rcl_timer_t`, including managing its dependency
/// on `rcl_clock_t` by ensuring that this dependency are [dropped after][1]
/// the `rcl_timer_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub(crate) struct TimerHandle {
    pub(crate) rcl_timer: Arc<Mutex<rcl_timer_t>>,
    clock: Clock,
}

/// 'Drop' trait implementation to be able to release the resources
impl Drop for TimerHandle {
    fn drop(&mut self) {
        let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        unsafe {
            // SAFETY: The lifecycle mutex is locked and the clock for the timer
            // must still be valid because TimerHandle keeps it alive.
            rcl_timer_fini(&mut *self.rcl_timer.lock().unwrap())
        };
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_timer_t {}

#[cfg(test)]
mod tests {
    use super::TimerExecutable;
    use crate::*;
    use std::{
        sync::{
            atomic::{AtomicBool, Ordering},
            Arc,
        },
        thread,
        time::Duration,
    };

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<TimerState<Node>>();
        assert_sync::<TimerState<Node>>();
    }

    #[test]
    fn test_new_with_system_clock() {
        let executor = Context::default().create_basic_executor();
        let result = TimerState::<Node>::create(
            Duration::from_millis(1),
            Clock::system(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        );
        assert!(result.is_ok());
    }

    #[test]
    fn test_new_with_steady_clock() {
        let executor = Context::default().create_basic_executor();
        let result = TimerState::<Node>::create(
            Duration::from_millis(1),
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        );
        assert!(result.is_ok());
    }

    #[test]
    fn test_new_with_source_clock() {
        let (clock, source) = Clock::with_source();
        // No manual time set, it should default to 0
        assert_eq!(clock.now().nsec, 0);
        let set_time = 1234i64;
        source.set_ros_time_override(set_time);

        // ROS time is set, should return the value that was set
        assert_eq!(clock.now().nsec, set_time);

        let executor = Context::default().create_basic_executor();
        let result = TimerState::<Node>::create(
            Duration::from_millis(1),
            clock,
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        );
        assert!(result.is_ok());
    }

    #[test]
    fn test_get_period() {
        let period = Duration::from_millis(1);

        let executor = Context::default().create_basic_executor();

        let result = TimerState::<Node>::create(
            period,
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        );

        let timer = result.unwrap();
        let timer_period = timer.get_timer_period().unwrap();
        assert_eq!(timer_period, period);
    }

    #[test]
    fn test_cancel() {
        let executor = Context::default().create_basic_executor();

        let result = TimerState::<Node>::create(
            Duration::from_millis(1),
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        );

        let timer = result.unwrap();
        assert!(!timer.is_canceled().unwrap());
        timer.cancel().unwrap();
        assert!(timer.is_canceled().unwrap());
    }

    #[test]
    fn test_time_since_last_call_before_first_event() {
        let executor = Context::default().create_basic_executor();

        let result = TimerState::<Node>::create(
            Duration::from_millis(2),
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        );
        let timer = result.unwrap();

        let sleep_period = Duration::from_millis(1);
        thread::sleep(sleep_period);

        let time_since_last_call = timer.time_since_last_call().unwrap();
        assert!(
            time_since_last_call >= sleep_period,
            "time_since_last_call: {:?} vs sleep period: {:?}",
            time_since_last_call,
            sleep_period,
        );
    }

    #[test]
    fn test_time_until_next_call_before_first_event() {
        let executor = Context::default().create_basic_executor();
        let period = Duration::from_millis(2);

        let result = TimerState::<Node>::create(
            period,
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        );
        let timer = result.unwrap();

        let time_until_next_call = timer.time_until_next_call().unwrap();
        assert!(
            time_until_next_call <= period,
            "time_until_next_call: {:?} vs period: {:?}",
            time_until_next_call,
            period,
        );
    }

    #[test]
    fn test_reset() {
        let executor = Context::default().create_basic_executor();
        let period = Duration::from_millis(2);
        let timer = TimerState::<Node>::create(
            period,
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        )
        .unwrap();

        // The unwrap will panic if the remaining time is negative
        timer.time_until_next_call().unwrap();

        // Sleep until we're past the timer period
        thread::sleep(Duration::from_millis(3));

        // Now the time until next call should give an error
        assert!(matches!(
            timer.time_until_next_call(),
            Err(RclrsError::NegativeDuration(_))
        ));

        // Reset the timer so its interval begins again
        assert!(timer.reset().is_ok());

        // The unwrap will panic if the remaining time is negative
        timer.time_until_next_call().unwrap();
    }

    #[test]
    fn test_call() {
        let executor = Context::default().create_basic_executor();
        let timer = TimerState::<Node>::create(
            Duration::from_millis(1),
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        )
        .unwrap();

        // The unwrap will panic if the remaining time is negative
        timer.time_until_next_call().unwrap();

        // Sleep until we're past the timer period
        thread::sleep(Duration::from_micros(1500));

        // Now the time until the next call should give an error
        assert!(matches!(
            timer.time_until_next_call(),
            Err(RclrsError::NegativeDuration(_))
        ));

        // The unwrap will panic if anything went wrong with the call
        timer.call(&mut ()).unwrap();

        // The unwrap will panic if the remaining time is negative
        timer.time_until_next_call().unwrap();
    }

    #[test]
    fn test_is_ready() {
        let executor = Context::default().create_basic_executor();
        let timer = TimerState::<Node>::create(
            Duration::from_millis(1),
            Clock::steady(),
            (|| {}).into_node_timer_repeating_callback(),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        )
        .unwrap();

        assert!(!timer.is_ready().unwrap());

        // Sleep until the period has elapsed
        thread::sleep(Duration::from_micros(1100));

        assert!(timer.is_ready().unwrap());
    }

    #[test]
    fn test_callback() {
        let clock = Clock::steady();
        let initial_time = clock.now();

        let executor = Context::default().create_basic_executor();
        let executed = Arc::new(AtomicBool::new(false));

        let timer = TimerState::<Node>::create(
            Duration::from_millis(1),
            clock,
            create_timer_callback_for_testing(initial_time, Arc::clone(&executed)),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        )
        .unwrap();

        timer.call(&mut ()).unwrap();
        assert!(executed.load(Ordering::Acquire));
    }

    #[test]
    fn test_execute_when_is_not_ready() {
        let clock = Clock::steady();
        let initial_time = clock.now();

        let executor = Context::default().create_basic_executor();
        let executed = Arc::new(AtomicBool::new(false));

        let timer = TimerState::<Node>::create(
            Duration::from_millis(1),
            clock,
            create_timer_callback_for_testing(initial_time, Arc::clone(&executed)),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        )
        .unwrap();

        let mut executable = TimerExecutable {
            timer: Arc::downgrade(&timer),
            handle: Arc::clone(&timer.handle),
        };

        unsafe {
            // SAFETY: Node timers expect a payload of ()
            executable.execute(ReadyKind::Basic, &mut ()).unwrap();
        }
        assert!(!executed.load(Ordering::Acquire));
    }

    #[test]
    fn test_execute_when_is_ready() {
        let clock = Clock::steady();
        let initial_time = clock.now();

        let executor = Context::default().create_basic_executor();
        let executed = Arc::new(AtomicBool::new(false));

        let timer = TimerState::<Node>::create(
            Duration::from_millis(1),
            clock,
            create_timer_callback_for_testing(initial_time, Arc::clone(&executed)),
            executor.commands().async_worker_commands(),
            &executor.commands().context().handle,
            None,
        )
        .unwrap();

        let mut executable = TimerExecutable {
            timer: Arc::downgrade(&timer),
            handle: Arc::clone(&timer.handle),
        };

        thread::sleep(Duration::from_millis(2));

        unsafe {
            // SAFETY: Node timers expect a payload of ()
            executable.execute(ReadyKind::Basic, &mut ()).unwrap();
        }
        assert!(executed.load(Ordering::Acquire));
    }

    fn create_timer_callback_for_testing(
        initial_time: Time,
        executed: Arc<AtomicBool>,
    ) -> AnyTimerCallback<Node> {
        (move |t: Time| {
            assert!(t
                .compare_with(&initial_time, |t, initial| t >= initial)
                .unwrap());
            executed.store(true, Ordering::Release);
        })
        .into_node_timer_oneshot_callback()
    }
}
