use crate::{
    rcl_bindings::*,
    log_error,
    GoalUuid, RclrsError, ToResult,
};
use super::{GoalStatus};
use std::sync::{Mutex, MutexGuard};

/// This struct is a minimal bridge to the `rcl_action` API for action server goals.
/// While the goal is still live, it will be managed by a [`LiveActionServerGoal`][1]
/// struct. Once the [`LiveActionServerGoal`][1] is dropped, it will remain inside
/// the [`ActionServerHandle`][2] until the goal is reported as expired by `rcl_action`.
///
/// This will be created by [`RequestedActionServerGoal`][3]
///
/// [1]: super::LiveActionServerGoal
/// [2]: super::ActionServerHandle
/// [3]: super::RequestedGoal::accept
pub(super) struct ActionServerGoalHandle {
    rcl_handle: Mutex<rcl_action_goal_handle_t>,
    uuid: GoalUuid,
}

impl ActionServerGoalHandle {
    pub(super) fn new(
        rcl_handle: rcl_action_goal_handle_s,
        uuid: GoalUuid,
    ) -> Self {
        Self {
            rcl_handle: Mutex::new(rcl_handle),
            uuid,
        }
    }

    pub(super) fn lock(&self) -> MutexGuard<rcl_action_goal_handle_t> {
        self.rcl_handle.lock().unwrap()
    }

    /// Returns the goal state.
    pub(super) fn get_status(&self) -> GoalStatus {
        let mut state = GoalStatus::Unknown as rcl_action_goal_state_t;
        {
            let rcl_handle = self.rcl_handle.lock().unwrap();
            // SAFETY: The provided goal handle is properly initialized by construction.
            let r = unsafe { rcl_action_goal_handle_get_status(&*rcl_handle, &mut state).ok() };
            if let Err(err) = r {
                log_error!(
                    "ActionServerGoalHandle.get_status",
                    "Unexpected error while getting status: {err}",
                );
            }
        }
        // SAFETY: state is initialized to a valid GoalStatus value and will only ever by set by
        // rcl_action_goal_handle_get_status to a valid GoalStatus value.
        unsafe { std::mem::transmute(state) }
    }

    /// Returns whether the client has requested that this goal be cancelled.
    pub(super) fn is_cancelling(&self) -> bool {
        self.get_status() == GoalStatus::Cancelling
    }

    /// Returns true if the goal is either pending or executing, or false if it has reached a
    /// terminal state.
    pub(super) fn is_active(&self) -> bool {
        let rcl_handle = self.rcl_handle.lock().unwrap();
        // SAFETY: The provided goal handle is properly initialized by construction.
        unsafe { rcl_action_goal_handle_is_active(&*rcl_handle) }
    }

    /// Returns whether the goal is executing.
    pub(super) fn is_executing(&self) -> bool {
        self.get_status() == GoalStatus::Executing
    }

    /// Get the unique identifier of the goal.
    pub(super) fn goal_id(&self) -> &GoalUuid {
        &self.uuid
    }
}

impl Drop for ActionServerGoalHandle {
    fn drop(&mut self) {
        // SAFETY: There should not be any way for the mutex to be poisoned
        let mut rcl_handle = self.rcl_handle.lock().unwrap();
        unsafe {
            // SAFETY: The goal handle was propertly initialized, and it will
            // never be accessed again after this.
            rcl_action_goal_handle_fini(&mut *rcl_handle);
        }
    }
}

// SAFETY: The functions accessing this type don't care about the thread they are
// running in. Therefore this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}
