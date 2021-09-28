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

use std::borrow::BorrowMut;
use std::cell::{Ref, RefCell, RefMut};
use std::rc::{Rc, Weak};

use crate::{SubscriptionBase, error::*};
use crate::Handle;

use crate::rcl_bindings::*;

use anyhow::{Context, Error, Result};
use rclrs_common::error::WaitSetError;

pub struct WaitSet {
    pub wait_set: rcl_wait_set_t,
    initialized: bool,
}

impl WaitSet {
    /// Creates a new WaitSet object.
    ///
    /// Under the hood, this calls `rcl_get_zero_initialized_wait_set()`, and stores it
    /// within the WaitSet struct, while also noting that the returned value is uninitialized.
    pub fn new() -> Self {
        Self {
            wait_set: unsafe { rcl_get_zero_initialized_wait_set() },
            initialized: false,
        }
    }

    /// Initializes an rcl wait set with space for items to be waited on
    ///
    /// This function allocates space for the subscriptions and other wait-able
    /// entities that can be stored in the wait set, using a default allocator grabbed from
    /// `rcutils_get_default_allocator()`
    pub fn init(
        &mut self,
        number_of_subscriptions: usize,
        number_of_guard_conditions: usize,
        number_of_timers: usize,
        number_of_clients: usize,
        number_of_services: usize,
        number_of_events: usize,
        context: &mut rcl_context_s,
    ) -> Result<(), WaitSetError> {
        if self.initialized {
            return Err(WaitSetError::RclError(RclError::WaitSetInvalid));
        }
        unsafe {
            match to_rcl_result(rcl_wait_set_init(
                self.wait_set.borrow_mut() as *mut _,
                number_of_subscriptions,
                number_of_guard_conditions,
                number_of_timers,
                number_of_clients,
                number_of_services,
                number_of_events,
                context,
                rcutils_get_default_allocator(),
            )) {
                Ok(()) => {
                    self.initialized = true;
                    Ok(())
                }
                Err(err) => {
                    self.initialized = false;
                    Err(WaitSetError::RclError(err))
                }
            }
        }
    }

    /// Removes (sets to NULL) all entities in the WaitSet
    ///
    /// # Errors
    /// - `RclError::InvalidArgument` if any arguments are invalid.
    /// - `RclError::WaitSetInvalid` if the WaitSet is already zero-initialized.
    /// - `RclError::Error` for an unspecified error
    pub fn clear(&mut self) -> Result<(), WaitSetError> {
        if !self.initialized {
            return Err(WaitSetError::RclError(RclError::WaitSetInvalid));
        }
        unsafe {
            // Whether or not we successfully clear, this WaitSet will count as uninitialized
            self.initialized = false;
            to_rcl_result(rcl_wait_set_clear(self.wait_set.borrow_mut() as *mut _)).map_err(WaitSetError::RclError)
        }
    }

    pub fn add_subscription(&mut self, subscription: &Weak<dyn SubscriptionBase>) -> Result<(), WaitSetError> {
        if let Some(subscription) = subscription.upgrade() {
            let subscription_handle = &*subscription.handle().get();
            unsafe {
                return to_rcl_result(rcl_wait_set_add_subscription(
                    self.wait_set.borrow_mut() as *mut _,
                subscription_handle as *const _,
            std::ptr::null_mut())).map_err(WaitSetError::RclError);
            }
        } else {
            Err(WaitSetError::DroppedSubscription)
        }
    }

    pub fn wait(&mut self, timeout: i64) -> Result<(), WaitSetError> {
        unsafe {
            to_rcl_result(rcl_wait(self.wait_set.borrow_mut() as *mut _, timeout)).map_err(WaitSetError::RclError)
        }
    }
}

impl Drop for WaitSet {
    /// Drops the WaitSet, and clears the memory
    ///
    /// # Panics
    /// A panic is raised if `rcl` is unable to release the waitset for any reason.
    fn drop(&mut self) {
        let handle = &mut *self.wait_set.borrow_mut();
        unsafe {
            match to_rcl_result(rcl_wait_set_fini(handle as *mut _)) {
                Ok(()) => (),
                Err(err) => {
                    panic!("Unable to release WaitSet!! {:?}", err)
                }
            }
        }
    }
}
