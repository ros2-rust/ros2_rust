mod client;
mod server;

use crate::{rcl_bindings::*, RclrsError};
use std::{marker::PhantomData, sync::Arc};

pub use client::{ActionClient, ActionClientBase};
pub use server::{ActionServer, ActionServerBase};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}

unsafe impl Sync for rcl_action_goal_handle_t {}

pub type GoalUUID = [u8; RCL_ACTION_UUID_SIZE];

pub enum GoalResponse {
    Reject = 1,
    AcceptAndExecute = 2,
    AcceptAndDefer = 3,
}

pub enum CancelResponse {
    Reject = 1,
    Accept = 2,
}

pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    rcl_handle: Arc<rcl_action_goal_handle_t>,
    goal_request: Arc<T>,
    _marker: PhantomData<T>,
}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub fn new(rcl_handle: Arc<rcl_action_goal_handle_t>, goal_request: Arc<T>) -> Self {
        Self {
            rcl_handle,
            goal_request: Arc::clone(&goal_request),
            _marker: Default::default(),
        }
    }

    pub fn is_canceling(&self) -> bool {
        false
    }

    pub fn is_active(&self) -> bool {
        false
    }

    pub fn is_executing(&self) -> bool {
        false
    }

    pub fn succeed(&self, result: &T::Result) -> Result<(), RclrsError> {
        Ok(())
    }

    pub fn canceled(&self, result: &T::Result) -> Result<(), RclrsError> {
        Ok(())
    }
}
