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
use std::rc::Rc;

use crate::Handle;
use crate::error::*;

use crate::rcl_bindings::*;

use anyhow::{Context, Error};

pub struct WaitSet {
    pub wait_set: rcl_wait_set_t,
    initialized: bool,
}

impl WaitSet {
    /// Creates a new WaitSet object.
    ///
    /// Under the hood, this calls `rcl_get_zero_initialized_wait_set()`, and stores it
    /// within the WaitSet struct, while also noting that the returned value is uninitialized.
    fn new() -> Self {
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
    ///
    /// Safety: The unsafe  cases of the underlying `rcl_wait_set_init`
    fn init(
        &mut self,
        num_subs: usize,
        num_gcs: usize,
        num_timers: usize,
        num_clients: usize,
        num_services: usize,
        num_events: usize,
        context: &mut rcl_context_s) -> Result<(), RclError> {
            if self.initialized {
                return Err(RclError::WaitSetInvalid);
                // return Err(RclError::WaitSetInvalid).context(|| "WaitSet.init called with already-initialized wait_set!");
            }
            unsafe {
                match rcl_wait_set_init(
                    self.wait_set.borrow_mut() as *mut _,
                    num_subs,
                    num_gcs,
                    num_timers,
                    num_clients,
                    num_services,
                    num_events,
                    context,
                    rcutils_get_default_allocator()
                ).ok() {
                    Ok(()) => {
                        self.initialized = true;
                        Ok(())
                    },
                    Err(err) => {
                        self.initialized = false;
                        Err(err)
                    }
                }
        }
    }
}

impl Drop for WaitSet {
    fn drop(&mut self) {
        let handle = &mut *self.wait_set.borrow_mut();
        unsafe {
            match rcl_wait_set_fini(handle as *mut _).ok() {
                Ok(()) => (),
                Err(err) => {
                    panic!("Unable to release WaitSet!! {:?}", err)
                }
            }
        }
    }
}