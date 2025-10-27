use std::{
    any::Any,
    sync::{Arc, Mutex},
};

use crate::{
    rcl_bindings::*, ContextHandle, RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, RclrsError,
    ReadyKind, ToResult, Waitable, WaitableLifecycle,
};

/// A waitable entity used for waking up a wait set manually.
///
/// This is used internally to wake up the executor's wait set to check if its
/// spin conditions are still valid or to perform cleanup (releasing waitable
/// entries that have been dropped by the user).
///
/// Users of rclrs have no reason to use this struct unless you are
/// implementing a custom executor. If you want to trigger a function to run on
/// the executor you should use [`ExecutorCommands::run`][1] or
/// [`ExecutorCommands::query`][2].
///
/// [1]: crate::ExecutorCommands::run
/// [2]: crate::ExecutorCommands::query
pub struct GuardCondition {
    /// The rcl_guard_condition_t that this struct encapsulates. Holding onto
    /// this keeps the rcl_guard_condition alive and allows us to trigger it.
    handle: Arc<GuardConditionHandle>,
    /// This manages the lifecycle of this guard condition's waiter. Dropping
    /// this will remove the guard condition from its wait set.
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
}

// SAFETY: rcl_guard_condition is the only member that doesn't implement Send, and it is designed to be accessed from other threads
unsafe impl Send for rcl_guard_condition_t {}
// SAFETY: We make sure internally to defend all invariants of the unowned
// pointer.
unsafe impl Send for InnerGuardConditionHandle {}

impl GuardCondition {
    /// Triggers this guard condition, activating the wait set, and calling the optionally assigned callback.
    pub fn trigger(&self) -> Result<(), RclrsError> {
        // SAFETY: The rcl_guard_condition_t is valid.
        if let Some(handle) = self.handle.rcl_guard_condition.lock().unwrap().owned_mut() {
            unsafe {
                rcl_trigger_guard_condition(handle).ok()?;
            }
        } else {
            return Err(RclrsError::UnownedGuardCondition);
        }
        Ok(())
    }

    /// Creates a new guard condition. This is only for internal use. Ordinary
    /// users of rclrs do not need to access guard conditions.
    pub(crate) fn new(
        context: &Arc<ContextHandle>,
        callback: Option<Box<dyn FnMut() + Send + Sync>>,
    ) -> (Arc<Self>, Waitable) {
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

            Mutex::new(InnerGuardConditionHandle::Owned(guard_condition))
        };

        let handle = Arc::new(GuardConditionHandle {
            rcl_guard_condition,
            context_handle: Arc::clone(&context),
        });

        let (waitable, lifecycle) = Waitable::new(
            Box::new(GuardConditionExecutable {
                handle: Arc::clone(&handle),
                callback,
            }),
            None,
        );

        (Arc::new(Self { handle, lifecycle }), waitable)
    }

    /// SAFETY: The caller is responsible for ensuring that the pointer being
    /// passed in is valid and non-null, and also that as long as `owner` is
    /// held, the pointer will remain valid.
    pub(crate) unsafe fn from_rcl(
        context: &Arc<ContextHandle>,
        rcl_guard_condition: *const rcl_guard_condition_t,
        owner: Box<dyn Any>,
        callback: Option<Box<dyn FnMut() + Send + Sync>>,
    ) -> (Self, Waitable) {
        let rcl_guard_condition = Mutex::new(InnerGuardConditionHandle::Unowned {
            handle: rcl_guard_condition,
            owner,
        });

        let handle = Arc::new(GuardConditionHandle {
            rcl_guard_condition,
            context_handle: Arc::clone(&context),
        });

        let (waitable, lifecycle) = Waitable::new(
            Box::new(GuardConditionExecutable {
                handle: Arc::clone(&handle),
                callback,
            }),
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
    rcl_guard_condition: Mutex<InnerGuardConditionHandle>,
    /// Keep the context alive for the whole lifecycle of the guard condition
    #[allow(dead_code)]
    context_handle: Arc<ContextHandle>,
}

/// We need to wrap the underlying condition variable with this because some
/// condition variables are owned at the rclrs layer while others were obtained
/// from rcl and either have static lifetimes or lifetimes that depend on
/// something else.
#[derive(Debug)]
pub enum InnerGuardConditionHandle {
    /// This variant means the guard condition was created and owned by rclrs.
    /// Its memory is managed by us.
    Owned(rcl_guard_condition_t),
    /// This variant means the guard condition was created and owned by rcl.
    /// The owner object represents something that the lifecycle of the guard
    /// condition depends on, such as the rcl_node that created it.
    Unowned {
        /// This is the unowned guard condition pointer. We must not deallocate
        /// it.
        handle: *const rcl_guard_condition_t,
        /// This somehow holds a shared reference to the owner of the guard
        /// condition. We need to hold onto this to ensure the guard condition
        /// remains valid.
        owner: Box<dyn Any>,
    },
}

impl InnerGuardConditionHandle {
    /// Get the handle if it is owned by rclrs
    pub fn owned(&self) -> Option<&rcl_guard_condition_t> {
        match self {
            Self::Owned(handle) => Some(handle),
            _ => None,
        }
    }

    /// Get the handle if it is owned by rclrs
    pub fn owned_mut(&mut self) -> Option<&mut rcl_guard_condition_t> {
        match self {
            Self::Owned(handle) => Some(handle),
            _ => None,
        }
    }

    /// Apply a function to the handle
    pub fn use_handle<Out>(&self, f: impl FnOnce(&rcl_guard_condition_t) -> Out) -> Out {
        match self {
            Self::Owned(handle) => f(handle),
            Self::Unowned { handle, .. } => f(unsafe {
                // SAFETY: The enum ensures that the pointer remains valid
                handle.as_ref().unwrap()
            }),
        }
    }
}

impl Drop for GuardConditionHandle {
    fn drop(&mut self) {
        if let InnerGuardConditionHandle::Owned(rcl_guard_condition) =
            &mut *self.rcl_guard_condition.lock().unwrap()
        {
            unsafe {
                // SAFETY: No precondition for this function (besides passing in a valid guard condition)
                rcl_guard_condition_fini(rcl_guard_condition);
            }
        }
    }
}

struct GuardConditionExecutable {
    handle: Arc<GuardConditionHandle>,
    /// The callback that will be triggered when execute is called. Not all
    /// guard conditions need to have a callback associated, so this could be
    /// [`None`].
    callback: Option<Box<dyn FnMut() + Send + Sync>>,
}

impl RclPrimitive for GuardConditionExecutable {
    unsafe fn execute(&mut self, ready: ReadyKind, _: &mut dyn Any) -> Result<(), RclrsError> {
        ready.for_basic()?;
        if let Some(callback) = &mut self.callback {
            callback();
        }
        Ok(())
    }

    fn kind(&self) -> RclPrimitiveKind {
        RclPrimitiveKind::GuardCondition
    }

    fn handle(&self) -> RclPrimitiveHandle {
        RclPrimitiveHandle::GuardCondition(self.handle.rcl_guard_condition.lock().unwrap())
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
