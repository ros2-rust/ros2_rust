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

use crate::error::{to_rclrs_result, RclReturnCode, RclrsError, ToResult};
use crate::rcl_bindings::*;
use crate::{ClientBase, Context, ServiceBase, SubscriptionBase};

use std::sync::Arc;
use std::time::Duration;
use std::vec::Vec;

use parking_lot::Mutex;

mod exclusivity_guard;
use exclusivity_guard::*;

/// A struct for waiting on subscriptions and other waitable entities to become ready.
pub struct WaitSet {
    rcl_wait_set: rcl_wait_set_t,
    // Used to ensure the context is alive while the wait set is alive.
    _rcl_context_mtx: Arc<Mutex<rcl_context_t>>,
    // The subscriptions that are currently registered in the wait set.
    // This correspondence is an invariant that must be maintained by all functions,
    // even in the error case.
    subscriptions: Vec<ExclusivityGuard<Arc<dyn SubscriptionBase>>>,
    clients: Vec<ExclusivityGuard<Arc<dyn ClientBase>>>,
    services: Vec<ExclusivityGuard<Arc<dyn ServiceBase>>>,
}

/// A list of entities that are ready, returned by [`WaitSet::wait`].
pub struct ReadyEntities {
    /// A list of subscriptions that have potentially received messages.
    pub subscriptions: Vec<Arc<dyn SubscriptionBase>>,
    /// A list of clients that have potentially received responses.
    pub clients: Vec<Arc<dyn ClientBase>>,
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
                &mut *context.rcl_context_mtx.lock(),
                rcutils_get_default_allocator(),
            )
            .ok()?;
            rcl_wait_set
        };
        Ok(Self {
            rcl_wait_set,
            _rcl_context_mtx: context.rcl_context_mtx.clone(),
            subscriptions: Vec::new(),
            clients: Vec::new(),
            services: Vec::new(),
        })
    }

    /// Removes all entities from the wait set.
    ///
    /// This effectively resets the wait set to the state it was in after being created by
    /// [`WaitSet::new`].
    pub fn clear(&mut self) {
        self.subscriptions.clear();
        self.clients.clear();
        self.services.clear();
        // This cannot fail – the rcl_wait_set_clear function only checks that the input handle is
        // valid, which it always is in our case. Hence, only debug_assert instead of returning
        // Result.
        // SAFETY: No preconditions for this function (besides passing in a valid wait set).
        let ret = unsafe { rcl_wait_set_clear(&mut self.rcl_wait_set) };
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
                &mut self.rcl_wait_set,
                &*subscription.handle().lock(),
                std::ptr::null_mut(),
            )
        }
        .ok()?;
        self.subscriptions.push(exclusive_subscription);
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
                &mut self.rcl_wait_set,
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
                &mut self.rcl_wait_set,
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
    /// representation, or an error will occur.
    ///
    /// This list is not comprehensive, since further errors may occur in the `rmw` or `rcl` layers.
    ///
    /// [1]: std::time::Duration::ZERO
    pub fn wait(&mut self, timeout: Option<Duration>) -> Result<ReadyEntities, RclrsError> {
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
        unsafe { rcl_wait(&mut self.rcl_wait_set, timeout_ns) }.ok()?;
        let mut ready_entities = ReadyEntities {
            subscriptions: Vec::new(),
            clients: Vec::new(),
            services: Vec::new(),
        };
        for (i, subscription) in self.subscriptions.iter().enumerate() {
            // SAFETY: The `subscriptions` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.rcl_wait_set.subscriptions.add(i) };
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
            let wait_set_entry = unsafe { *self.rcl_wait_set.clients.add(i) };
            if !wait_set_entry.is_null() {
                ready_entities.clients.push(Arc::clone(&client.waitable));
            }
        }
        for (i, service) in self.services.iter().enumerate() {
            // SAFETY: The `services` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.rcl_wait_set.services.add(i) };
            if !wait_set_entry.is_null() {
                ready_entities.services.push(Arc::clone(&service.waitable));
            }
        }
        Ok(ready_entities)
    }
}

#[cfg(test)]
mod tests {
    use crate::{Context, Node, RclrsError, WaitSet, QOS_PROFILE_DEFAULT};
    use std::sync::Arc;

    #[test]
    fn test_adding_waitable_to_wait_sets() -> Result<(), RclrsError> {
        let context = Context::new([])?;
        let mut node = Node::new(&context, "test_adding_waitable_to_wait_sets")?;
        let subscription = node.create_subscription(
            "test",
            QOS_PROFILE_DEFAULT,
            move |_: std_msgs::msg::String| {},
        )?;
        let mut wait_set_1 = WaitSet::new(1, 0, 0, 0, 0, 0, &context)?;
        let mut wait_set_2 = WaitSet::new(1, 0, 0, 0, 0, 0, &context)?;

        // Try to add the subscription to wait set 1 twice
        wait_set_1.add_subscription(Arc::clone(&subscription) as _)?;
        assert!(wait_set_1
            .add_subscription(Arc::clone(&subscription) as _)
            .is_err());

        // Try to add it to another wait set
        assert!(wait_set_2
            .add_subscription(Arc::clone(&subscription) as _)
            .is_err());

        // It works as soon as it is not anymore part of wait_set_1
        wait_set_1.clear();
        wait_set_2.add_subscription(Arc::clone(&subscription) as _)?;

        // Dropping the wait set also frees up the subscription
        drop(wait_set_2);
        wait_set_1.add_subscription(Arc::clone(&subscription) as _)?;
        Ok(())
    }
}
