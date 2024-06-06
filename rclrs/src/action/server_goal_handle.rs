use crate::{rcl_bindings::*, RclrsError, ToResult};
use std::{
    marker::PhantomData,
    sync::{Arc, Mutex},
};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}

unsafe impl Sync for rcl_action_goal_handle_t {}

// Values defined by `action_msgs/msg/GoalStatus`
#[repr(i8)]
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
enum GoalStatus {
    Unknown = 0,
    Accepted = 1,
    Executing = 2,
    Canceling = 3,
    Succeeded = 4,
    Canceled = 5,
    Aborted = 6,
}

pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    rcl_handle: Arc<Mutex<rcl_action_goal_handle_t>>,
    goal_request: Arc<T>,
}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub fn new(rcl_handle: Arc<Mutex<rcl_action_goal_handle_t>>, goal_request: Arc<T>) -> Self {
        Self {
            rcl_handle,
            goal_request: Arc::clone(&goal_request),
        }
    }

    /// Returns the goal state.
    fn get_state(&self) -> Result<GoalStatus, RclrsError> {
        let mut state = GoalStatus::Unknown as rcl_action_goal_state_t;
        {
            let rcl_handle = self.rcl_handle.lock().unwrap();
            // SAFETY: The provided goal handle is properly initialized by construction.
            unsafe { rcl_action_goal_handle_get_status(&*rcl_handle, &mut state).ok()? }
        }
        // SAFETY: state is initialized to a valid GoalStatus value and will only ever by set by
        // rcl_action_goal_handle_get_status to a valid GoalStatus value.
        Ok(unsafe { std::mem::transmute(state) })
    }

    /// Returns whether the client has requested that this goal be cancelled.
    pub fn is_canceling(&self) -> bool {
        self.get_state().unwrap() == GoalStatus::Canceling
    }

    /// Returns true if the goal is either pending or executing, or false if it has reached a
    /// terminal state.
    pub fn is_active(&self) -> bool {
        let rcl_handle = self.rcl_handle.lock().unwrap();
        // SAFETY: The provided goal handle is properly initialized by construction.
        unsafe { rcl_action_goal_handle_is_active(&*rcl_handle) }
    }

    /// Returns whether the goal is executing.
    pub fn is_executing(&self) -> bool {
        self.get_state().unwrap() == GoalStatus::Executing
    }

    /// Attempt to perform the given goal state transition.
    fn update_state(&self, event: rcl_action_goal_event_t) -> Result<(), RclrsError> {
        let mut rcl_handle = self.rcl_handle.lock().unwrap();
        // SAFETY: The provided goal handle is properly initialized by construction.
        unsafe { rcl_action_update_goal_state(&mut *rcl_handle, event).ok() }
    }

    pub fn abort(&self, result: &T::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_ABORT)?;

        // TODO: Invoke on_terminal_state callback
        Ok(())
    }

    pub fn succeed(&self, result: &T::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_SUCCEED)?;

        // TODO: Invoke on_terminal_state callback
        Ok(())
    }

    pub fn canceled(&self, result: &T::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCELED)?;

        // TODO: Invoke on_terminal_state callback
        Ok(())
    }

    pub fn execute(&self, result: &T::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_EXECUTE)?;

        // TODO: Invoke on_executing callback
        Ok(())
    }

    /// Try canceling the goal if possible.
    fn try_canceling(&mut self) -> Result<bool, RclrsError> {
        let rcl_handle = self.rcl_handle.lock().unwrap();

        // If the goal is in a cancelable state, transition to canceling.
        // SAFETY: The provided goal handle is properly initialized by construction.
        let is_cancelable = unsafe { rcl_action_goal_handle_is_cancelable(&*rcl_handle) };
        if is_cancelable {
            self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCEL_GOAL)?;
        }

        // If the goal is canceling, transition to canceled.
        if self.get_state()? == GoalStatus::Canceling {
            self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCELED)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

impl<T> Drop for ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Cancel the goal if its handle is dropped without reaching a terminal state.
    fn drop(&mut self) {
        if self.try_canceling() == Ok(true) {
            // TODO: Invoke on_terminal_state callback
        }
    }
}
