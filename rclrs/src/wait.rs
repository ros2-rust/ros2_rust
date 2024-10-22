// Copyright 2020 DCS Corporation, All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.

use std::{sync::Arc, time::Duration, vec::Vec};

use crate::{
    error::{to_rclrs_result, RclReturnCode, RclrsError, ToResult},
    rcl_bindings::*,
    ClientBase, Context, ContextHandle, Node, ServiceBase, SubscriptionBase,
};

mod exclusivity_guard;
mod guard_condition;
use exclusivity_guard::*;
pub use guard_condition::*;

/// Manage the lifecycle of an `rcl_wait_set_t`, including managing its dependency
/// on `rcl_context_t` by ensuring that this dependency is [dropped after][1] the
/// `rcl_wait_set_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
struct WaitSetHandle {
    rcl_wait_set: rcl_wait_set_t,
    // Used to ensure the context is alive while the wait set is alive.
    #[allow(dead_code)]
    context_handle: Arc<ContextHandle>,
}

/// A struct for waiting on subscriptions and other waitable entities to become ready.
pub struct WaitSet {
    // The subscriptions that are currently registered in the wait set.
    // This correspondence is an invariant that must be maintained by all functions,
    // even in the error case.
    subscriptions: Vec<ExclusivityGuard<Arc<dyn SubscriptionBase>>>,
    clients: Vec<ExclusivityGuard<Arc<dyn ClientBase>>>,
    // The guard conditions that are currently registered in the wait set.
    guard_conditions: Vec<ExclusivityGuard<Arc<GuardCondition>>>,
    services: Vec<ExclusivityGuard<Arc<dyn ServiceBase>>>,
    handle: WaitSetHandle,
}

/// A list of entities that are ready, returned by [`WaitSet::wait`].
pub struct ReadyEntities {
    /// A list of subscriptions that have potentially received messages.
    pub subscriptions: Vec<Arc<dyn SubscriptionBase>>,
    /// A list of clients that have potentially received responses.
    pub clients: Vec<Arc<dyn ClientBase>>,
    /// A list of guard conditions that have been triggered.
    pub guard_conditions: Vec<Arc<GuardCondition>>,
    /// A list of services that have potentially received requests.
    pub services: Vec<Arc<dyn ServiceBase>>,
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

// SAFETY: While the rcl_wait_set_t does have some interior mutability (because it has
// members of non-const pointer type), this interior mutability is hidden/not used by
// the WaitSet type. Therefore, sharing &WaitSet between threads does not risk data races.
unsafe impl Sync for WaitSet {}

impl WaitSet {
    /// Creates a new wait set.
    ///
    /// The given number of subscriptions is a capacity, corresponding to how often
    /// [`WaitSet::add_subscription`] may be called.
    pub fn new(
        number_of_subscriptions: usize,
        number_of_guard_conditions: usize,
        number_of_timers: usize,
        number_of_clients: usize,
        number_of_services: usize,
        number_of_events: usize,
        context: &Context,
    ) -> Result<Self, RclrsError> {
        let rcl_wait_set = unsafe {
            // SAFETY: Getting a zero-initialized value is always safe
            let mut rcl_wait_set = rcl_get_zero_initialized_wait_set();
            let mut rcl_context = context.handle.rcl_context.lock().unwrap();
            // SAFETY: We're passing in a zero-initialized wait set and a valid context.
            // There are no other preconditions.
            rcl_wait_set_init(
                &mut rcl_wait_set,
                number_of_subscriptions,
                number_of_guard_conditions,
                number_of_timers,
                number_of_clients,
                number_of_services,
                number_of_events,
                &mut *rcl_context,
                rcutils_get_default_allocator(),
            )
            .ok()?;
            rcl_wait_set
        };
        Ok(Self {
            subscriptions: Vec::new(),
            guard_conditions: Vec::new(),
            clients: Vec::new(),
            services: Vec::new(),
            handle: WaitSetHandle {
                rcl_wait_set,
                context_handle: Arc::clone(&context.handle),
            },
        })
    }

    /// Creates a new wait set and adds all waitable entities in the node to it.
    ///
    /// The wait set is sized to fit the node exactly, so there is no capacity for adding other entities.
    pub fn new_for_node(node: &Node) -> Result<Self, RclrsError> {
        let live_subscriptions = node.live_subscriptions();
        let live_clients = node.live_clients();
        let live_guard_conditions = node.live_guard_conditions();
        let live_services = node.live_services();
        let ctx = Context {
            handle: Arc::clone(&node.handle.context_handle),
        };
        let mut wait_set = WaitSet::new(
            live_subscriptions.len(),
            live_guard_conditions.len(),
            0,
            live_clients.len(),
            live_services.len(),
            0,
            &ctx,
        )?;

        for live_subscription in &live_subscriptions {
            wait_set.add_subscription(live_subscription.clone())?;
        }

        for live_client in &live_clients {
            wait_set.add_client(live_client.clone())?;
        }

        for live_guard_condition in &live_guard_conditions {
            wait_set.add_guard_condition(live_guard_condition.clone())?;
        }

        for live_service in &live_services {
            wait_set.add_service(live_service.clone())?;
        }
        Ok(wait_set)
    }

    /// Removes all entities from the wait set.
    ///
    /// This effectively resets the wait set to the state it was in after being created by
    /// [`WaitSet::new`].
    pub fn clear(&mut self) {
        self.subscriptions.clear();
        self.guard_conditions.clear();
        self.clients.clear();
        self.services.clear();
        // This cannot fail – the rcl_wait_set_clear function only checks that the input handle is
        // valid, which it always is in our case. Hence, only debug_assert instead of returning
        // Result.
        // SAFETY: No preconditions for this function (besides passing in a valid wait set).
        let ret = unsafe { rcl_wait_set_clear(&mut self.handle.rcl_wait_set) };
        debug_assert_eq!(ret, 0);
    }

    /// Adds a subscription to the wait set.
    ///
    /// # Errors
    /// - If the subscription was already added to this wait set or another one,
    ///   [`AlreadyAddedToWaitSet`][1] will be returned
    /// - If the number of subscriptions in the wait set is larger than the
    ///   capacity set in [`WaitSet::new`], [`WaitSetFull`][2] will be returned
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::RclReturnCode
    pub fn add_subscription(
        &mut self,
        subscription: Arc<dyn SubscriptionBase>,
    ) -> Result<(), RclrsError> {
        let exclusive_subscription = ExclusivityGuard::new(
            Arc::clone(&subscription),
            Arc::clone(&subscription.handle().in_use_by_wait_set),
        )?;
        unsafe {
            // SAFETY: I'm not sure if it's required, but the subscription pointer will remain valid
            // for as long as the wait set exists, because it's stored in self.subscriptions.
            // Passing in a null pointer for the third argument is explicitly allowed.
            rcl_wait_set_add_subscription(
                &mut self.handle.rcl_wait_set,
                &*subscription.handle().lock(),
                std::ptr::null_mut(),
            )
        }
        .ok()?;
        self.subscriptions.push(exclusive_subscription);
        Ok(())
    }

    /// Adds a guard condition to the wait set.
    ///
    /// # Errors
    /// - If the guard condition was already added to this wait set or another one,
    ///   [`AlreadyAddedToWaitSet`][1] will be returned
    /// - If the number of guard conditions in the wait set is larger than the
    ///   capacity set in [`WaitSet::new`], [`WaitSetFull`][2] will be returned
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::RclReturnCode
    pub fn add_guard_condition(
        &mut self,
        guard_condition: Arc<GuardCondition>,
    ) -> Result<(), RclrsError> {
        let exclusive_guard_condition = ExclusivityGuard::new(
            Arc::clone(&guard_condition),
            Arc::clone(&guard_condition.in_use_by_wait_set),
        )?;

        unsafe {
            // SAFETY: Safe if the wait set and guard condition are initialized
            rcl_wait_set_add_guard_condition(
                &mut self.handle.rcl_wait_set,
                &*guard_condition.handle.rcl_guard_condition.lock().unwrap(),
                std::ptr::null_mut(),
            )
            .ok()?;
        }
        self.guard_conditions.push(exclusive_guard_condition);
        Ok(())
    }

    /// Adds a client to the wait set.
    ///
    /// # Errors
    /// - If the client was already added to this wait set or another one,
    ///   [`AlreadyAddedToWaitSet`][1] will be returned
    /// - If the number of clients in the wait set is larger than the
    ///   capacity set in [`WaitSet::new`], [`WaitSetFull`][2] will be returned
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::RclReturnCode
    pub fn add_client(&mut self, client: Arc<dyn ClientBase>) -> Result<(), RclrsError> {
        let exclusive_client = ExclusivityGuard::new(
            Arc::clone(&client),
            Arc::clone(&client.handle().in_use_by_wait_set),
        )?;
        unsafe {
            // SAFETY: I'm not sure if it's required, but the client pointer will remain valid
            // for as long as the wait set exists, because it's stored in self.clients.
            // Passing in a null pointer for the third argument is explicitly allowed.
            rcl_wait_set_add_client(
                &mut self.handle.rcl_wait_set,
                &*client.handle().lock() as *const _,
                core::ptr::null_mut(),
            )
        }
        .ok()?;
        self.clients.push(exclusive_client);
        Ok(())
    }

    /// Adds a service to the wait set.
    ///
    /// # Errors
    /// - If the service was already added to this wait set or another one,
    ///   [`AlreadyAddedToWaitSet`][1] will be returned
    /// - If the number of services in the wait set is larger than the
    ///   capacity set in [`WaitSet::new`], [`WaitSetFull`][2] will be returned
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::RclReturnCode
    pub fn add_service(&mut self, service: Arc<dyn ServiceBase>) -> Result<(), RclrsError> {
        let exclusive_service = ExclusivityGuard::new(
            Arc::clone(&service),
            Arc::clone(&service.handle().in_use_by_wait_set),
        )?;
        unsafe {
            // SAFETY: I'm not sure if it's required, but the service pointer will remain valid
            // for as long as the wait set exists, because it's stored in self.services.
            // Passing in a null pointer for the third argument is explicitly allowed.
            rcl_wait_set_add_service(
                &mut self.handle.rcl_wait_set,
                &*service.handle().lock() as *const _,
                core::ptr::null_mut(),
            )
        }
        .ok()?;
        self.services.push(exclusive_service);
        Ok(())
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
    pub fn wait(mut self, timeout: Option<Duration>) -> Result<ReadyEntities, RclrsError> {
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
        // We cannot currently guarantee that the wait sets may not share content, but it is
        // mentioned in the doc comment for `add_subscription`.
        // Also, the rcl_wait_set is obviously valid.
        match unsafe { rcl_wait(&mut self.handle.rcl_wait_set, timeout_ns) }.ok() {
            Ok(_) => (),
            Err(error) => match error {
                RclrsError::RclError { code, msg } => match code {
                    RclReturnCode::WaitSetEmpty => (),
                    _ => return Err(RclrsError::RclError { code, msg }),
                },
                _ => return Err(error),
            },
        }
        let mut ready_entities = ReadyEntities {
            subscriptions: Vec::new(),
            clients: Vec::new(),
            guard_conditions: Vec::new(),
            services: Vec::new(),
        };
        for (i, subscription) in self.subscriptions.iter().enumerate() {
            // SAFETY: The `subscriptions` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.subscriptions.add(i) };
            if !wait_set_entry.is_null() {
                ready_entities
                    .subscriptions
                    .push(Arc::clone(&subscription.waitable));
            }
        }

        for (i, client) in self.clients.iter().enumerate() {
            // SAFETY: The `clients` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.clients.add(i) };
            if !wait_set_entry.is_null() {
                ready_entities.clients.push(Arc::clone(&client.waitable));
            }
        }

        for (i, guard_condition) in self.guard_conditions.iter().enumerate() {
            // SAFETY: The `clients` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.guard_conditions.add(i) };
            if !wait_set_entry.is_null() {
                ready_entities
                    .guard_conditions
                    .push(Arc::clone(&guard_condition.waitable));
            }
        }

        for (i, service) in self.services.iter().enumerate() {
            // SAFETY: The `services` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.services.add(i) };
            if !wait_set_entry.is_null() {
                ready_entities.services.push(Arc::clone(&service.waitable));
            }
        }
        Ok(ready_entities)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<WaitSet>();
        assert_sync::<WaitSet>();
    }

    #[test]
    fn guard_condition_in_wait_set_readies() -> Result<(), RclrsError> {
        let context = Context::new([])?;

        let guard_condition = Arc::new(GuardCondition::new(&context));

        let mut wait_set = WaitSet::new(0, 1, 0, 0, 0, 0, &context)?;
        wait_set.add_guard_condition(Arc::clone(&guard_condition))?;
        guard_condition.trigger()?;

        let readies = wait_set.wait(Some(std::time::Duration::from_millis(10)))?;
        assert!(readies.guard_conditions.contains(&guard_condition));

        Ok(())
    }
}
