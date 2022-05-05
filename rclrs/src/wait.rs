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

use crate::error::{to_rcl_result, RclReturnCode, ToResult};
use crate::rcl_bindings::*;
use crate::{Context, SubscriptionBase};

use std::sync::Arc;
use std::time::Duration;
use std::vec::Vec;

use parking_lot::Mutex;

/// A struct for waiting on subscriptions and other waitable entities to become ready.
pub struct WaitSet {
    handle: rcl_wait_set_t,
    // Used to ensure the context is alive while the wait set is alive.
    _context_handle: Arc<Mutex<rcl_context_t>>,
    // The subscriptions that are currently registered in the wait set.
    // This correspondence is an invariant that must be maintained by all functions,
    // even in the error case.
    subscriptions: Vec<Arc<dyn SubscriptionBase>>,
}

/// A list of entities that are ready, returned by [`WaitSet::wait`].
pub struct ReadyEntities {
    /// A list of subscriptions that have potentially received messages.
    pub subscriptions: Vec<Arc<dyn SubscriptionBase>>,
}

impl Drop for rcl_wait_set_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function (besides passing in a valid wait set).
        let rc = unsafe { rcl_wait_set_fini(self) };
        if let Err(e) = to_rcl_result(rc) {
            panic!("Unable to release WaitSet. {:?}", e)
        }
    }
}

impl WaitSet {
    /// Creates a new wait set.
    ///
    /// The given number of subscriptions is a capacity, corresponding to how often
    /// [`WaitSet::add_subscription`] may be called.
    pub fn new(number_of_subscriptions: usize, context: &Context) -> Result<Self, RclReturnCode> {
        let rcl_wait_set = unsafe {
            // SAFETY: Getting a zero-initialized value is always safe
            let mut rcl_wait_set = rcl_get_zero_initialized_wait_set();
            // SAFETY: We're passing in a zero-initialized wait set and a valid context.
            // There are no other preconditions.
            rcl_wait_set_init(
                &mut rcl_wait_set,
                number_of_subscriptions,
                0,
                0,
                0,
                0,
                0,
                &mut *context.handle.lock(),
                rcutils_get_default_allocator(),
            )
            .ok()?;
            rcl_wait_set
        };
        Ok(Self {
            handle: rcl_wait_set,
            _context_handle: context.handle.clone(),
            subscriptions: Vec::new(),
        })
    }

    /// Removes all entities from the wait set.
    ///
    /// This effectively resets the wait set to the state it was in after being created by
    /// [`WaitSet::new`].
    pub fn clear(&mut self) {
        self.subscriptions.clear();
        // This cannot fail – the rcl_wait_set_clear function only checks that the input handle is
        // valid, which it always is in our case. Hence, only debug_assert instead of returning
        // Result.
        // SAFETY: No preconditions for this function (besides passing in a valid wait set).
        let ret = unsafe { rcl_wait_set_clear(&mut self.handle) };
        debug_assert_eq!(ret, 0);
    }

    /// Adds a subscription to the wait set.
    ///
    /// It is possible, but not useful, to add the same subscription twice.
    ///
    /// This will return an error if the number of subscriptions in the wait set is larger than the
    /// capacity set in [`WaitSet::new`].
    ///
    /// The same subscription must not be added to multiple wait sets, because that would make it
    /// unsafe to simultaneously wait on those wait sets.
    pub fn add_subscription(
        &mut self,
        subscription: Arc<dyn SubscriptionBase>,
    ) -> Result<(), RclReturnCode> {
        unsafe {
            // SAFETY: I'm not sure if it's required, but the subscription pointer will remain valid
            // for as long as the wait set exists, because it's stored in self.subscriptions.
            // Passing in a null pointer for the third argument is explicitly allowed.
            rcl_wait_set_add_subscription(
                &mut self.handle,
                &*subscription.handle().lock(),
                std::ptr::null_mut(),
            )
        }
        .ok()?;
        self.subscriptions.push(subscription);
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
    pub fn wait(&mut self, timeout: Option<Duration>) -> Result<ReadyEntities, RclReturnCode> {
        let timeout_ns = match timeout.map(|d| d.as_nanos()) {
            None => -1,
            Some(ns) if ns <= i64::MAX as u128 => ns as i64,
            _ => {
                return Err(RclReturnCode::InvalidArgument);
            }
        };
        // SAFETY: The comments in rcl mention "This function cannot operate on the same wait set
        // in multiple threads, and the wait sets may not share content."
        // We cannot currently guarantee that the wait sets may not share content, but it is
        // mentioned in the doc comment for `add_subscription`.
        // Also, the handle is obviously valid.
        unsafe { rcl_wait(&mut self.handle, timeout_ns) }.ok()?;
        let mut ready_entities = ReadyEntities {
            subscriptions: Vec::new(),
        };
        for (i, subscription) in self.subscriptions.iter().enumerate() {
            // SAFETY: The `subscriptions` entry is an array of pointers, and this dereferencing is
            // equivalent to
            // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
            let wait_set_entry = unsafe { *self.handle.subscriptions.add(i) };
            if !wait_set_entry.is_null() {
                ready_entities.subscriptions.push(subscription.clone());
            }
        }
        Ok(ready_entities)
    }
}
