use std::sync::{atomic::AtomicBool, Arc, Mutex};

use crate::{rcl_bindings::*, Context, ContextHandle, RclrsError, ToResult};

/// A waitable entity used for waking up a wait set manually.
///
/// If a wait set that is currently waiting on events should be interrupted from a separate thread, this can be done
/// by adding an `Arc<GuardCondition>` to the wait set, and calling `trigger()` on the same `GuardCondition` while
/// the wait set is waiting.
///
/// The guard condition may be reused multiple times, but like other waitable entities, can not be used in
/// multiple wait sets concurrently.
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
    /// The rcl_guard_condition_t that this struct encapsulates.
    pub(crate) handle: GuardConditionHandle,
    /// An optional callback to call when this guard condition is triggered.
    callback: Option<Box<dyn Fn() + Send + Sync>>,
    /// A flag to indicate if this guard condition has already been assigned to a wait set.
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

/// Manage the lifecycle of an `rcl_guard_condition_t`, including managing its dependency
/// on `rcl_context_t` by ensuring that this dependency is [dropped after][1] the
/// `rcl_guard_condition_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub(crate) struct GuardConditionHandle {
    pub(crate) rcl_guard_condition: Mutex<rcl_guard_condition_t>,
    /// Keep the context alive for the whole lifecycle of the guard condition
    #[allow(dead_code)]
    pub(crate) context_handle: Arc<ContextHandle>,
}

impl Drop for GuardCondition {
    fn drop(&mut self) {
        unsafe {
            // SAFETY: No precondition for this function (besides passing in a valid guard condition)
            rcl_guard_condition_fini(&mut *self.handle.rcl_guard_condition.lock().unwrap());
        }
    }
}

impl PartialEq for GuardCondition {
    fn eq(&self, other: &Self) -> bool {
        // Because GuardCondition controls the creation of the rcl_guard_condition, each unique GuardCondition should have a unique
        // rcl_guard_condition. Thus comparing equality of this member should be enough.
        std::ptr::eq(
            &self.handle.rcl_guard_condition.lock().unwrap().impl_,
            &other.handle.rcl_guard_condition.lock().unwrap().impl_,
        )
    }
}

impl Eq for GuardCondition {}

// SAFETY: rcl_guard_condition is the only member that doesn't implement Send, and it is designed to be accessed from other threads
unsafe impl Send for rcl_guard_condition_t {}

impl GuardCondition {
    /// Creates a new guard condition with no callback.
    pub fn new(context: &Context) -> Self {
        Self::new_with_context_handle(Arc::clone(&context.handle), None)
    }

    /// Creates a new guard condition with a callback.
    pub fn new_with_callback<F>(context: &Context, callback: F) -> Self
    where
        F: Fn() + Send + Sync + 'static,
    {
        Self::new_with_context_handle(
            Arc::clone(&context.handle),
            Some(Box::new(callback) as Box<dyn Fn() + Send + Sync>),
        )
    }

    /// Creates a new guard condition by providing the rcl_context_t and an optional callback.
    /// Note this function enables calling `Node::create_guard_condition`[1] without providing the Context separately
    ///
    /// [1]: Node::create_guard_condition
    pub(crate) fn new_with_context_handle(
        context_handle: Arc<ContextHandle>,
        callback: Option<Box<dyn Fn() + Send + Sync>>,
    ) -> Self {
        let rcl_guard_condition = {
            // SAFETY: Getting a zero initialized value is always safe
            let mut guard_condition = unsafe { rcl_get_zero_initialized_guard_condition() };
            let mut rcl_context = context_handle.rcl_context.lock().unwrap();
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

        Self {
            handle: GuardConditionHandle {
                rcl_guard_condition,
                context_handle,
            },
            callback,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Triggers this guard condition, activating the wait set, and calling the optionally assigned callback.
    pub fn trigger(&self) -> Result<(), RclrsError> {
        unsafe {
            // SAFETY: The rcl_guard_condition_t is valid.
            rcl_trigger_guard_condition(&mut *self.handle.rcl_guard_condition.lock().unwrap())
                .ok()?;
        }
        if let Some(callback) = &self.callback {
            callback();
        }
        Ok(())
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
