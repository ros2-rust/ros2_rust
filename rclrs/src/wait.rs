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

use std::{sync::Arc, time::Duration, vec::Vec, collections::{HashSet, HashMap}};
use by_address::ByAddress;

use crate::{
    error::{to_rclrs_result, RclReturnCode, RclrsError, ToResult},
    rcl_bindings::*,
    Context, ContextHandle,
};

mod guard_condition;
pub use guard_condition::*;

mod waitable;
pub use waitable::*;

/// A struct for waiting on subscriptions and other waitable entities to become ready.
pub struct WaitSet {
    entities: HashMap<WaitableKind, Vec<Waiter>>,
    handle: WaitSetHandle,
}

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
        mut entities: WaitSetEntities,
        context: &Context,
    ) -> Result<Self, RclrsError> {
        entities.dedup();

        let count = WaitableCount::new();
        let rcl_wait_set = unsafe {
            count.initialize(&mut context.handle.rcl_context.lock().unwrap())?
        };

        let handle = WaitSetHandle {
            rcl_wait_set,
            context_handle: Arc::clone(&context.handle),
        };

        let mut wait_set = Self { entities, handle };
        wait_set.register_rcl_entities()?;
        Ok(wait_set)
    }

    /// Take all the items out of `entities` and move them into this wait set.
    pub fn add(&mut self, entities: &mut WaitSetEntities) -> Result<(), RclrsError> {
        self.entities.append(entities);
        self.entities.dedup();
        self.resize_rcl_containers()?;
        self.register_rcl_entities()?;
        Ok(())
    }

    /// Remove the specified entities from this wait set.
    fn remove(&mut self, entities: &WaitSetEntities) -> Result<(), RclrsError> {
        self.entities.remove_and_dedup(entities);
        self.resize_rcl_containers()?;
        self.register_rcl_entities()?;
        Ok(())
    }

    /// Removes all entities from the wait set.
    ///
    /// This effectively resets the wait set to the state it was in after being created by
    /// [`WaitSet::new`].
    pub fn clear(&mut self) {
        self.entities.clear();
        // This cannot fail – the rcl_wait_set_clear function only checks that the input handle is
        // valid, which it always is in our case. Hence, only debug_assert instead of returning
        // Result.
        // SAFETY: No preconditions for this function (besides passing in a valid wait set).
        let ret = unsafe { rcl_wait_set_clear(&mut self.handle.rcl_wait_set) };
        debug_assert_eq!(ret, 0);
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
    /// representation, or an error will occur.
    ///
    /// This list is not comprehensive, since further errors may occur in the `rmw` or `rcl` layers.
    ///
    /// [1]: std::time::Duration::ZERO
    pub fn wait(
        &mut self,
        timeout: Option<Duration>,
        mut f: impl FnMut(WaitSetEntity) -> Result<(), RclrsError>,
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

        for (i, subscription) in self.entities.subscriptions.iter().enumerate() {
            // SAFETY: The `subscriptions` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.subscriptions.add(i) };
            if !wait_set_entry.is_null() {
                f(WaitSetEntity::Subscription(subscription))?;
            }
        }

        for (i, client) in self.entities.clients.iter().enumerate() {
            // SAFETY: The `clients` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.clients.add(i) };
            if !wait_set_entry.is_null() {
                f(WaitSetEntity::Client(client))?;
            }
        }

        for (i, guard_condition) in self.entities.guard_conditions.iter().enumerate() {
            // SAFETY: The `clients` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.guard_conditions.add(i) };
            if !wait_set_entry.is_null() {
                f(WaitSetEntity::GuardCondition(guard_condition))?;
            }
        }

        for (i, service) in self.entities.services.iter().enumerate() {
            // SAFETY: The `services` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.rcl_wait_set.services.add(i) };
            if !wait_set_entry.is_null() {
                f(WaitSetEntity::Service(service))?;
            }
        }

        // Each time we call rcl_wait, the rcl_wait_set_t handle will have some
        // of its entities set to null, so we need to put them back in. We do
        // not need to resize the rcl_wait_set_t because no new entities could
        // have been added while we had the mutable borrow of the WaitSet.

        // Note that self.clear() will not change the allocated size of each rcl
        // entity container, so we do not need to resize before re-registering
        // the rcl entities.
        self.clear();
        self.register_rcl_entities();

        Ok(())
    }

    pub fn count(&self) -> WaitableCount {
        let mut c = WaitableCount::new();
        for (kind, collection) in &self.entities {
            c.add(*kind, collection.len());
        }
    }

    fn resize_rcl_containers(&mut self) -> Result<(), RclrsError> {
        let count = self.count();
        unsafe {
            count.resize(&mut self.handle.rcl_wait_set)?;
        }
        Ok(())
    }

    fn register_rcl_entities(&mut self) -> Result<(), RclrsError> {
        self.register_rcl_subscriptions()?;
        self.register_rcl_guard_conditions()?;
        self.register_rcl_clients()?;
        self.register_rcl_services()?;
        Ok(())
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
    fn register_rcl_subscriptions(
        &mut self,
    ) -> Result<(), RclrsError> {
        for subscription in &self.entities.subscriptions {
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
        }
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
    fn register_rcl_guard_conditions(
        &mut self,
    ) -> Result<(), RclrsError> {
        for guard_condition in &self.entities.guard_conditions {
            unsafe {
                // SAFETY: Safe if the wait set and guard condition are initialized
                rcl_wait_set_add_guard_condition(
                    &mut self.handle.rcl_wait_set,
                    &*guard_condition.handle.rcl_guard_condition.lock().unwrap(),
                    std::ptr::null_mut(),
                )
                .ok()?;
            }
        }
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
    fn register_rcl_clients(&mut self) -> Result<(), RclrsError> {
        for client in &self.entities.clients {
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
        }
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
    fn register_rcl_services(&mut self) -> Result<(), RclrsError> {
        for service in &self.entities.services {
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
        }
        Ok(())
    }
}

/// This is a container for all wait set entities that rclrs currently supports
#[derive(Clone, Default)]
pub struct WaitSetEntities {
    // The subscriptions that are currently registered in the wait set.
    // This correspondence is an invariant that must be maintained by all functions,
    // even in the error case.
    pub subscriptions: Vec<Arc<dyn SubscriptionBase>>,
    pub clients: Vec<Arc<dyn ClientBase>>,
    // The guard conditions that are currently registered in the wait set.
    pub guard_conditions: Vec<Arc<GuardCondition>>,
    pub services: Vec<Arc<dyn ServiceBase>>,
}

impl WaitSetEntities {
    /// Create a new empty container
    pub fn new() -> Self {
        Self::default()
    }

    /// Ensure there is only one instance of each entity in the collection.
    pub fn dedup(&mut self) {
        dedup_vec_by_arc_address(&mut self.subscriptions);
        dedup_vec_by_arc_address(&mut self.clients);
        dedup_vec_by_arc_address(&mut self.guard_conditions);
        dedup_vec_by_arc_address(&mut self.services);
    }

    /// Move all entities out of `other` into `self`, leaving `other` empty.
    pub fn append(&mut self, other: &mut Self) {
        self.subscriptions.append(&mut other.subscriptions);
        self.clients.append(&mut other.clients);
        self.guard_conditions.append(&mut other.guard_conditions);
        self.services.append(&mut other.services);
    }

    /// Remove all entities that are present in `other`. This will also
    /// deduplicate any items that were already in `self`.
    pub fn remove_and_dedup(&mut self, other: &Self) {
        remove_vec_by_arc_address(&mut self.subscriptions, &other.subscriptions);
        remove_vec_by_arc_address(&mut self.clients, &other.clients);
        remove_vec_by_arc_address(&mut self.guard_conditions, &other.guard_conditions);
        remove_vec_by_arc_address(&mut self.services, &other.services);
    }

    /// Clear all items from the container
    pub fn clear(&mut self) {
        self.subscriptions.clear();
        self.clients.clear();
        self.guard_conditions.clear();
        self.services.clear();
    }
}

fn dedup_vec_by_arc_address<T: ?Sized>(v: &mut Vec<Arc<T>>) {
    let mut set = HashSet::new();
    v.retain(|item| set.insert(ByAddress(Arc::clone(item))));
}

fn remove_vec_by_arc_address<T: ?Sized>(
    v: &mut Vec<Arc<T>>,
    remove: &Vec<Arc<T>>,
) {
    let mut set = HashSet::new();
    for r in remove {
        set.insert(ByAddress(Arc::clone(r)));
    }

    v.retain(|item| set.insert(ByAddress(Arc::clone(item))));
}

pub enum WaitSetEntity<'a> {
    Subscription(&'a Arc<dyn SubscriptionBase>),
    Client(&'a Arc<dyn ClientBase>),
    Service(&'a Arc<dyn ServiceBase>),
    GuardCondition(&'a Arc<GuardCondition>),
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
    rcl_wait_set: rcl_wait_set_t,
    // Used to ensure the context is alive while the wait set is alive.
    #[allow(dead_code)]
    context_handle: Arc<ContextHandle>,
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

        let mut entities = WaitSetEntities::new();
        entities.guard_conditions.push(Arc::clone(&guard_condition));

        let mut wait_set = WaitSet::new(entities, &context)?;
        guard_condition.trigger()?;

        let mut triggered = false;
        wait_set.wait(
            Some(std::time::Duration::from_millis(10)),
            |entity| {
                assert!(matches!(entity, WaitSetEntity::GuardCondition(_)));
                triggered = true;
                Ok(())
            },
        )?;
        assert!(triggered);

        Ok(())
    }
}
