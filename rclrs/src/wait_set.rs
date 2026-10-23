use std::{collections::HashMap, sync::Arc, time::Duration, vec::Vec};

use crate::{
    error::{to_rclrs_result, RclReturnCode, RclrsError, ToResult},
    log_error,
    rcl_bindings::*,
    Context, ContextHandle,
};

mod guard_condition;
pub use guard_condition::*;

mod rcl_primitive;
pub use rcl_primitive::*;

mod waitable;
pub use waitable::*;

mod wait_set_runner;
pub use wait_set_runner::*;

/// A struct for waiting on subscriptions and other waitable entities to become ready.
pub struct WaitSet {
    primitives: HashMap<RclPrimitiveKind, Vec<Waitable>>,
    handle: WaitSetHandle,
}

// SAFETY: While the rcl_wait_set_t does have some interior mutability (because it has
// members of non-const pointer type), this interior mutability is hidden/not used by
// the WaitSet type. Therefore, sharing &WaitSet between threads does not risk data races.
unsafe impl Sync for WaitSet {}

impl WaitSet {
    /// Creates a new empty wait set.
    pub fn new(context: &Context) -> Result<Self, RclrsError> {
        let count = WaitableCount::new();
        let rcl_wait_set =
            unsafe { count.initialize(&mut context.handle.rcl_context.lock().unwrap())? };

        let handle = WaitSetHandle {
            rcl_wait_set,
            context_handle: Arc::clone(&context.handle),
        };

        let mut wait_set = Self {
            primitives: HashMap::new(),
            handle,
        };
        wait_set.register_rcl_primitives()?;
        Ok(wait_set)
    }

    /// Take all the items out of `entities` and move them into this wait set.
    pub fn add(&mut self, entities: impl IntoIterator<Item = Waitable>) -> Result<(), RclrsError> {
        for entity in entities {
            if entity.in_wait_set() {
                return Err(RclrsError::AlreadyAddedToWaitSet);
            }
            let kind = entity.primitive.kind();
            self.primitives.entry(kind).or_default().push(entity);
        }
        self.resize_rcl_containers()?;
        self.register_rcl_primitives()?;
        Ok(())
    }

    /// Removes all entities from the wait set.
    ///
    /// This effectively resets the wait set to the state it was in after being created by
    /// [`WaitSet::new`].
    pub fn clear(&mut self) {
        self.primitives.clear();
        self.rcl_clear();
    }

    /// Blocks until the wait set is ready, or until the timeout has been exceeded.
    ///
    /// If the timeout is `None` then this function will block indefinitely until
    /// something in the wait set is valid or it is interrupted.
    ///
    /// If the timeout is [`Duration::ZERO`][1] then this function will be non-blocking; checking what's
    /// ready now, but not waiting if nothing is ready yet.
    ///
    /// If the timeout is greater than [`Duration::ZERO`][1] then this function will return after
    /// that period of time has elapsed or the wait set becomes ready, which ever
    /// comes first.
    ///
    /// Once one or more items in the wait set are ready, `f` will be triggered
    /// for each ready item.
    ///
    /// This function does not change the entities registered in the wait set.
    ///
    /// # Errors
    ///
    /// - Passing a wait set with no wait-able items in it will return an error.
    /// - The timeout must not be so large so as to overflow an `i64` with its nanosecond
    ///   representation, or an error will occur.
    ///
    /// This list is not comprehensive, since further errors may occur in the `rmw` or `rcl` layers.
    ///
    /// [1]: std::time::Duration::ZERO
    pub fn wait(
        &mut self,
        timeout: Option<Duration>,
        mut f: impl FnMut(ReadyKind, &mut dyn RclPrimitive) -> Result<(), RclrsError>,
    ) -> Result<(), RclrsError> {
        let timeout_ns = match timeout.map(|d| d.as_nanos()) {
            None => -1,
            Some(ns) if ns <= i64::MAX as u128 => ns as i64,
            _ => {
                return Err(RclrsError::RclError {
                    code: RclReturnCode::InvalidArgument,
                    msg: None,
                })
            }
        };

        // SAFETY: The comments in rcl mention "This function cannot operate on the same wait set
        // in multiple threads, and the wait sets may not share content."
        // * The we have exclusive access to rcl_wait_set because this is a
        //   mutable borrow of WaitSet, which houses rcl_wait_set.
        // * We guarantee that the wait sets do not share content by funneling
        //   the waitable of each primitive to one (and only one) WaitSet when
        //   the primitive gets constructed. The waitables are never allowed to
        //   move between wait sets.
        let r = match unsafe { rcl_wait(&mut self.handle.rcl_wait_set, timeout_ns) }.ok() {
            Ok(_) => Ok(()),
            Err(error) => match error {
                RclrsError::RclError { code, msg } => match code {
                    RclReturnCode::WaitSetEmpty => Ok(()),
                    _ => Err(RclrsError::RclError { code, msg }),
                },
                _ => Err(error),
            },
        };

        // Remove any waitables that are no longer being used
        for waitable in self.primitives.values_mut() {
            waitable.retain(|w| w.in_use());
        }

        // Do not check the readiness if an error was reported.
        if r.is_ok() {
            // For the remaining entities, check if they were activated and then run
            // the callback for those that were.
            for waiter in self.primitives.values_mut().flatten() {
                if let Some(ready) = waiter.is_ready(&self.handle.rcl_wait_set) {
                    f(ready, &mut *waiter.primitive)?;
                }
            }
        }

        // Each time we call rcl_wait, the rcl_wait_set_t handle will have some
        // of its entities set to null, so we need to put them back in. We do
        // not need to resize the rcl_wait_set_t because no new entities could
        // have been added while we had the mutable borrow of the WaitSet. Some
        // entities could have been removed, but that does not require a resizing.

        // Note that self.clear() will not change the allocated size of each rcl
        // entity container, so we do not need to resize before re-registering
        // the rcl entities.
        self.rcl_clear();
        if let Err(err) = self.register_rcl_primitives() {
            log_error!(
                "rclrs.WaitSet.wait",
                "Error while registering rcl primitives: {err}",
            );
        }

        r
    }

    /// Get a count of the different kinds of entities in the wait set.
    pub fn count(&self) -> WaitableCount {
        let mut c = WaitableCount::new();
        for (kind, collection) in &self.primitives {
            c.add_group(kind, collection);
        }
        c
    }

    fn resize_rcl_containers(&mut self) -> Result<(), RclrsError> {
        let count = self.count();
        unsafe {
            count.resize(&mut self.handle.rcl_wait_set)?;
        }
        Ok(())
    }

    /// Clear only the rcl_wait_set. This is done so that we can safely repopulate
    /// it to perform another wait. This does not effect the entities that we
    /// consider to still be in the wait set.
    fn rcl_clear(&mut self) {
        // This cannot fail – the rcl_wait_set_clear function only checks that the input handle is
        // valid, which it always is in our case. Hence, only debug_assert instead of returning
        // Result.
        // SAFETY: No preconditions for this function (besides passing in a valid wait set).
        let ret = unsafe { rcl_wait_set_clear(&mut self.handle.rcl_wait_set) };
        debug_assert_eq!(ret, 0);
    }

    /// Registers all the waitable entities with the rcl wait set.
    ///
    /// # Errors
    /// - If the number of subscriptions in the wait set is larger than the
    ///   allocated size [`WaitSetFull`][1] will be returned. If this happens
    ///   then there is a bug in rclrs.
    ///
    /// [1]: crate::RclReturnCode
    fn register_rcl_primitives(&mut self) -> Result<(), RclrsError> {
        for entity in self.primitives.values_mut().flatten() {
            entity.add_to_wait_set(&mut self.handle.rcl_wait_set)?;
        }
        Ok(())
    }
}

impl Drop for rcl_wait_set_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function (besides passing in a valid wait set).
        let rc = unsafe { rcl_wait_set_fini(self) };
        if let Err(e) = to_rclrs_result(rc) {
            panic!("Unable to release WaitSet. {:?}", e)
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_wait_set_t {}

/// Manage the lifecycle of an `rcl_wait_set_t`, including managing its dependency
/// on `rcl_context_t` by ensuring that this dependency is [dropped after][1] the
/// `rcl_wait_set_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
struct WaitSetHandle {
    pub(crate) rcl_wait_set: rcl_wait_set_t,
    // Used to ensure the context is alive while the wait set is alive.
    #[allow(dead_code)]
    context_handle: Arc<ContextHandle>,
}

#[cfg(test)]
mod tests {
    use crate::*;
    use std::time::Duration;

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<WaitSet>();
        assert_sync::<WaitSet>();
    }

    #[test]
    fn guard_condition_in_wait_set_readies() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_basic_executor();

        // After spinning has started, wait a moment and then wake up the wait sets.
        // TODO(@mxgrey): When we have timers, change this to use a one-shot timer instead.
        let commands = executor.commands().clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(1));
            commands.wake_all_wait_sets();
        });

        let start = std::time::Instant::now();
        // This should stop spinning right away because the guard condition was
        // already triggered.
        executor
            .spin(SpinOptions::spin_once().timeout(Duration::from_secs(10)))
            .first_error()?;

        // If it took more than a second to finish spinning then something is
        // probably wrong.
        //
        // Note that this test could theoretically be flaky if it runs on a
        // machine with very strange CPU scheduling behaviors. To have a test
        // that is guaranteed to be stable we could write a custom executor for
        // testing that will give us more introspection.
        assert!(std::time::Instant::now() - start < Duration::from_secs(1));

        Ok(())
    }
}
