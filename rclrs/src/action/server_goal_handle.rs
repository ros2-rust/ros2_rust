use crate::{action::ActionServer, rcl_bindings::*, GoalUuid, RclrsError, ToResult};
use std::sync::{Arc, Mutex, Weak};

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

/// Handle to interact with goals on a server.
///
/// Use this to check the status of a goal and to set its result.
///
/// This type will only be created by an [`ActionServer`] when a goal is accepted and will be
/// passed to the user in the associated `handle_accepted` callback.
pub struct ServerGoalHandle<ActionT>
where
    ActionT: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl,
{
    rcl_handle: Mutex<*mut rcl_action_goal_handle_t>,
    action_server: Weak<ActionServer<ActionT>>,
    goal_request: Arc<ActionT::Goal>,
    uuid: GoalUuid,
}

// SAFETY: Send/Sync are not automatically implemented due to the contained raw pointer
// (specifically, `*mut rcl_action_goal_handle_t`). However, the `rcl_handle` field is wrapped in a
// mutex, guaranteeing that the underlying data is never simultaneously accessed on the rclrs side
// by multiple threads. Moreover, the rcl_action functions taking these handles are able to be run
// from any thread.
unsafe impl<ActionT> Send for ServerGoalHandle<ActionT> where ActionT: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl {}
unsafe impl<ActionT> Sync for ServerGoalHandle<ActionT> where ActionT: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl {}

impl<ActionT> ServerGoalHandle<ActionT>
where
    ActionT: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl,
{
    pub(crate) fn new(
        rcl_handle: *mut rcl_action_goal_handle_t,
        action_server: Weak<ActionServer<ActionT>>,
        goal_request: Arc<ActionT::Goal>,
        uuid: GoalUuid,
    ) -> Self {
        Self {
            rcl_handle: Mutex::new(rcl_handle),
            action_server,
            goal_request: Arc::clone(&goal_request),
            uuid,
        }
    }

    /// Returns the goal state.
    fn get_state(&self) -> Result<GoalStatus, RclrsError> {
        let mut state = GoalStatus::Unknown as rcl_action_goal_state_t;
        {
            let rcl_handle = self.rcl_handle.lock().unwrap();
            // SAFETY: The provided goal handle is properly initialized by construction.
            unsafe { rcl_action_goal_handle_get_status(*rcl_handle, &mut state).ok()? }
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
        unsafe { rcl_action_goal_handle_is_active(*rcl_handle) }
    }

    /// Returns whether the goal is executing.
    pub fn is_executing(&self) -> bool {
        self.get_state().unwrap() == GoalStatus::Executing
    }

    /// Attempt to perform the given goal state transition.
    fn update_state(&self, event: rcl_action_goal_event_t) -> Result<(), RclrsError> {
        let mut rcl_handle = self.rcl_handle.lock().unwrap();
        // SAFETY: The provided goal handle is properly initialized by construction.
        unsafe { rcl_action_update_goal_state(*rcl_handle, event).ok() }
    }

    /// Indicate that the goal is being cancelled.
    ///
    /// This is called when a cancel request for the goal has been accepted.
    ///
    /// Returns an error if the goal is in any state other than accepted or executing.
    pub(crate) fn cancel(&self) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCEL_GOAL)
    }

    /// Indicate that the goal could not be reached and has been aborted.
    ///
    /// Only call this if the goal is executing but cannot be completed. This is a terminal state,
    /// so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub fn abort(&self, result: &ActionT::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_ABORT)?;

        // TODO: Invoke on_terminal_state callback
        Ok(())
    }

    /// Indicate that the goal has succeeded.
    ///
    /// Only call this if the goal is executing and has reached the desired final state. This is a
    /// terminal state, so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub fn succeed(&self, result: &ActionT::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_SUCCEED)?;

        // TODO: Invoke on_terminal_state callback
        Ok(())
    }

    /// Indicate that the goal has been cancelled.
    ///
    /// Only call this if the goal is executing or pending, but has been cancelled. This is a
    /// terminal state, so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing or pending.
    pub fn canceled(&self, result: &ActionT::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCELED)?;

        // TODO: Invoke on_terminal_state callback
        Ok(())
    }

    /// Indicate that the server is starting to execute the goal.
    ///
    /// Only call this if the goal is pending. This is a terminal state, so no more methods may be
    /// called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than pending.
    pub fn execute(&self) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_EXECUTE)?;

        // Publish the state change.
        if let Some(action_server) = self.action_server.upgrade() {
            action_server.publish_status()?;
        }
        Ok(())
    }

    /// Try canceling the goal if possible.
    fn try_canceling(&mut self) -> Result<bool, RclrsError> {
        let rcl_handle = self.rcl_handle.lock().unwrap();

        // If the goal is in a cancelable state, transition to canceling.
        // SAFETY: The provided goal handle is properly initialized by construction.
        let is_cancelable = unsafe { rcl_action_goal_handle_is_cancelable(*rcl_handle) };
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

    /// Get the unique identifier of the goal.
    pub fn goal_id(&self) -> GoalUuid {
        self.uuid
    }

    /// Get the user-provided message describing the goal.
    pub fn goal(&self) -> Arc<ActionT::Goal> {
        Arc::clone(&self.goal_request)
    }

    /// Send an update about the goal's progress.
    ///
    /// This may only be called when the goal is executing.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub fn publish_feedback(&self, feedback: Arc<ActionT::Feedback>) -> Result<(), RclrsError> {
        // If the action server no longer exists, simply drop the message.
        if let Some(action_server) = self.action_server.upgrade() {
            action_server.publish_feedback(&self.uuid, &*feedback)?;
        }
        Ok(())
    }
}

impl<ActionT> Drop for ServerGoalHandle<ActionT>
where
    ActionT: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl,
{
    /// Cancel the goal if its handle is dropped without reaching a terminal state.
    fn drop(&mut self) {
        if self.try_canceling() == Ok(true) {
            // TODO: Invoke on_terminal_state callback
        }
        {
            let rcl_handle = self.rcl_handle.lock().unwrap();
            // SAFETY: The provided goal handle is properly initialized by construction. It will
            // not be accessed beyond this point.
            unsafe { rcl_action_goal_handle_fini(*rcl_handle); }
        }
    }
}
