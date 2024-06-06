use crate::{rcl_bindings::*, RclrsError};
use std::{
    marker::PhantomData,
    sync::{Arc, Mutex},
};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}

unsafe impl Sync for rcl_action_goal_handle_t {}

pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    rcl_handle: Arc<Mutex<rcl_action_goal_handle_t>>,
    goal_request: Arc<T>,
    _marker: PhantomData<T>,
}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub fn new(rcl_handle: Arc<Mutex<rcl_action_goal_handle_t>>, goal_request: Arc<T>) -> Self {
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
