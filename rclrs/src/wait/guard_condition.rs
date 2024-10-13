use std::sync::{Arc, Mutex};

use crate::{
    rcl_bindings::*,
    ContextHandle, RclrsError, ToResult, WaitableLifecycle, Executable,
    Waitable, ExecutableKind, ExecutableHandle,
};

/// A waitable entity used for waking up a wait set manually.
///
/// This is used internally to wake up the executor's wait set to check if its
/// spin conditions are still valid or to perform cleanup (releasing waitable
/// entries that have been dropped by the user).
///
/// Users of rclrs have no reason to use this struct unless you are
/// implementing a custom executor. If you want to trigger a function to run on
/// the executor you should use [`ExecutorCommands::run`] or
/// [`ExecutorCommands::run_detached`].
pub struct GuardCondition {
    /// The rcl_guard_condition_t that this struct encapsulates. Holding onto
    /// this keeps the rcl_guard_condition alive and allows us to trigger it.
    handle: Arc<GuardConditionHandle>,
    /// This manages the lifecycle of this guard condition's waiter. Dropping
    /// this will remove the guard condition from its wait set.
    lifecycle: WaitableLifecycle,
}

// SAFETY: rcl_guard_condition is the only member that doesn't implement Send, and it is designed to be accessed from other threads
unsafe impl Send for rcl_guard_condition_t {}

impl GuardCondition {
    /// Triggers this guard condition, activating the wait set, and calling the optionally assigned callback.
    pub fn trigger(&self) -> Result<(), RclrsError> {
        unsafe {
            // SAFETY: The rcl_guard_condition_t is valid.
            rcl_trigger_guard_condition(&mut *self.handle.rcl_guard_condition.lock().unwrap())
                .ok()?;
        }
        Ok(())
    }

    /// Creates a new guard condition. This is only for internal use. Ordinary
    /// users of rclrs do not need to access guard conditions.
    pub(crate) fn new(context: &Arc<ContextHandle>) -> (Self, Waitable) {
        let rcl_guard_condition = {
            // SAFETY: Getting a zero initialized value is always safe
            let mut guard_condition = unsafe { rcl_get_zero_initialized_guard_condition() };
            let mut rcl_context = context.rcl_context.lock().unwrap();
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
            context_handle: Arc::clone(&context),
        });

        let (waitable, lifecycle) = Waitable::new(
            Box::new(GuardConditionExecutable { handle: Arc::clone(&handle) }),
            None,
        );

        (Self { handle, lifecycle }, waitable)
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

struct GuardConditionExecutable {
    handle: Arc<GuardConditionHandle>,
}

impl Executable for GuardConditionExecutable {
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
    use super::*;

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<GuardCondition>();
        assert_sync::<GuardCondition>();
    }
}
