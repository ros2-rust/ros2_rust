use std::sync::{Arc, Mutex};

use crate::{
    rcl_bindings::*,
    ContextHandle, RclrsError, ToResult, WaiterLifecycle, Executable,
    Waitable, ExecutableKind, ExecutableHandle, ExecutorCommands,
};

/// A waitable entity used for waking up a wait set manually.
///
/// If a wait set that is currently waiting on events should be interrupted from
/// a separate thread, trigger a `GuardCondition`.
///
/// A guard condition may be triggered any number of times, but can only be
/// associated with one wait set.
///
/// # Example
/// ```
/// # use rclrs::{Context, GuardCondition, WaitSet, RclrsError};
/// # use std::sync::{Arc, atomic::Ordering};
///
/// let context = Context::new([])?;
///
/// let atomic_bool = Arc::new(std::sync::atomic::AtomicBool::new(false));
/// let atomic_bool_for_closure = Arc::clone(&atomic_bool);
///
/// let gc = Arc::new(GuardCondition::new_with_callback(
///     &context,
///     move || {
///         atomic_bool_for_closure.store(true, Ordering::Relaxed);
///     },
/// ));
///
/// let mut ws = WaitSet::new(0, 1, 0, 0, 0, 0, &context)?;
/// ws.add_guard_condition(Arc::clone(&gc))?;
///
/// // Trigger the guard condition, firing the callback and waking the wait set being waited on, if any.
/// gc.trigger()?;
///
/// // The provided callback has now been called.
/// assert_eq!(atomic_bool.load(Ordering::Relaxed), true);
///
/// // The wait call will now immediately return.
/// ws.wait(Some(std::time::Duration::from_millis(10)))?;
///
/// # Ok::<(), RclrsError>(())
/// ```
pub struct GuardCondition {
    /// The rcl_guard_condition_t that this struct encapsulates. Holding onto
    /// this keeps the rcl_guard_condition alive and allows us to trigger it.
    handle: Arc<GuardConditionHandle>,
    /// This manages the lifecycle of this guard condition's waiter. Dropping
    /// this will remove the guard condition from its wait set.
    lifecycle: WaiterLifecycle,
}

// SAFETY: rcl_guard_condition is the only member that doesn't implement Send, and it is designed to be accessed from other threads
unsafe impl Send for rcl_guard_condition_t {}

impl GuardCondition {
    /// Creates a new guard condition with no callback.
    pub(crate) fn new(commands: &ExecutorCommands) -> Self {
        let context = commands.context();
        let rcl_guard_condition = {
            // SAFETY: Getting a zero initialized value is always safe
            let mut guard_condition = unsafe { rcl_get_zero_initialized_guard_condition() };
            let mut rcl_context = context.handle.rcl_context.lock().unwrap();
            unsafe {
                // SAFETY: The context must be valid, and the guard condition must be zero-initialized
                rcl_guard_condition_init(
                    &mut guard_condition,
                    &mut *rcl_context,
                    rcl_guard_condition_get_default_options(),
                );
            }

            Mutex::new(guard_condition)
        };

        let handle = Arc::new(GuardConditionHandle {
            rcl_guard_condition,
            context_handle: Arc::clone(&context.handle),
        });

        let (waiter, lifecycle) = Waitable::new(
            Box::new(GuardConditionWaitable { handle: Arc::clone(&handle) }),
            None,
        );

        commands.add_to_wait_set(waiter);

        Self { handle, lifecycle }
    }

    /// Triggers this guard condition, activating the wait set, and calling the optionally assigned callback.
    pub fn trigger(&self) -> Result<(), RclrsError> {
        unsafe {
            // SAFETY: The rcl_guard_condition_t is valid.
            rcl_trigger_guard_condition(&mut *self.handle.rcl_guard_condition.lock().unwrap())
                .ok()?;
        }
        Ok(())
    }
}

/// Manage the lifecycle of an `rcl_guard_condition_t`, including managing its dependency
/// on `rcl_context_t` by ensuring that this dependency is [dropped after][1] the
/// `rcl_guard_condition_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
struct GuardConditionHandle {
    rcl_guard_condition: Mutex<rcl_guard_condition_t>,
    /// Keep the context alive for the whole lifecycle of the guard condition
    #[allow(dead_code)]
    context_handle: Arc<ContextHandle>,
}

impl Drop for GuardConditionHandle {
    fn drop(&mut self) {
        unsafe {
            // SAFETY: No precondition for this function (besides passing in a valid guard condition)
            rcl_guard_condition_fini(&mut *self.rcl_guard_condition.lock().unwrap());
        }
    }
}

struct GuardConditionWaitable {
    handle: Arc<GuardConditionHandle>,
}

impl Executable for GuardConditionWaitable {
    fn execute(&mut self) -> Result<(), RclrsError> {
        // Do nothing
        Ok(())
    }

    fn kind(&self) -> ExecutableKind {
        ExecutableKind::GuardCondition
    }

    fn handle(&self) -> super::ExecutableHandle {
        ExecutableHandle::GuardCondition(
            self.handle.rcl_guard_condition.lock().unwrap()
        )
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::Ordering;

    use super::*;
    use crate::WaitSet;

    #[test]
    fn test_guard_condition() -> Result<(), RclrsError> {
        let context = Context::new([])?;

        let atomic_bool = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let atomic_bool_for_closure = Arc::clone(&atomic_bool);

        let guard_condition = GuardCondition::new_with_callback(&context, move || {
            atomic_bool_for_closure.store(true, Ordering::Relaxed);
        });

        guard_condition.trigger()?;

        assert!(atomic_bool.load(Ordering::Relaxed));

        Ok(())
    }

    #[test]
    fn test_guard_condition_wait() -> Result<(), RclrsError> {
        let context = Context::new([])?;

        let atomic_bool = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let atomic_bool_for_closure = Arc::clone(&atomic_bool);

        let guard_condition = Arc::new(GuardCondition::new_with_callback(&context, move || {
            atomic_bool_for_closure.store(true, Ordering::Relaxed);
        }));

        let mut wait_set = WaitSet::new(0, 1, 0, 0, 0, 0, &context)?;
        wait_set.add_guard_condition(Arc::clone(&guard_condition))?;
        guard_condition.trigger()?;

        assert!(atomic_bool.load(Ordering::Relaxed));
        wait_set.wait(Some(std::time::Duration::from_millis(10)))?;

        Ok(())
    }

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<GuardCondition>();
        assert_sync::<GuardCondition>();
    }
}
