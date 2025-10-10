use super::GoalStatusCode;
use crate::{
    log_error, rcl_bindings::*, ActionServerHandle, GoalUuid, RclErrorMsg, RclReturnCode,
    RclrsError, ToResult,
};
use rosidl_runtime_rs::{Action, RmwResultResponse};
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
pub(super) struct ActionServerGoalHandle<A: Action> {
    rcl_handle: Mutex<rcl_action_goal_handle_t>,
    result_response: Mutex<ResponseState<A>>,
    uuid: GoalUuid,
}

impl<A: Action> ActionServerGoalHandle<A> {
    pub(super) fn new(rcl_handle: rcl_action_goal_handle_s, uuid: GoalUuid) -> Self {
        Self {
            rcl_handle: Mutex::new(rcl_handle),
            result_response: Mutex::new(ResponseState::new()),
            uuid,
        }
    }

    pub(super) fn lock(&self) -> MutexGuard<rcl_action_goal_handle_t> {
        self.rcl_handle.lock().unwrap()
    }

    /// Returns the goal state.
    pub(super) fn get_status(&self) -> GoalStatusCode {
        let mut state = GoalStatusCode::Unknown as rcl_action_goal_state_t;
        {
            let rcl_handle = self.lock();
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

    /// This is used to check if we should respond as accepting a cancellation
    /// request for this goal after it is no longer live.
    pub(super) fn is_cancelled(&self) -> bool {
        self.get_status() == GoalStatusCode::Cancelled
    }

    /// Get the unique identifier of the goal.
    pub(super) fn goal_id(&self) -> &GoalUuid {
        &self.uuid
    }

    /// Provie the result for this action goal
    pub(super) fn provide_result(
        &self,
        action_server_handle: &ActionServerHandle<A>,
        result: RmwResultResponse<A>,
    ) -> Result<(), RclrsError> {
        self.result_response
            .lock()?
            .provide_result(action_server_handle, &self.uuid, result)
    }

    /// Add a result requester for this action goal
    pub(super) fn add_result_request(
        &self,
        action_server_handle: &ActionServerHandle<A>,
        result_request: rmw_request_id_t,
    ) -> Result<(), RclrsError> {
        self.result_response
            .lock()?
            .add_result_request(action_server_handle, result_request)
    }
}

impl<A: Action> Drop for ActionServerGoalHandle<A> {
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

/// Manages the state of a goal's response.
enum ResponseState<A: Action> {
    /// The response has not arrived yet. There may be some clients waiting for
    /// the response, and they'll be listed here.
    Waiting(Vec<rmw_request_id_t>),
    /// The response is available.
    Available(RmwResultResponse<A>),
}

impl<A: Action> ResponseState<A> {
    fn new() -> Self {
        Self::Waiting(Vec::new())
    }

    fn provide_result(
        &mut self,
        action_server_handle: &ActionServerHandle<A>,
        goal_id: &GoalUuid,
        mut result: RmwResultResponse<A>,
    ) -> Result<(), RclrsError> {
        let result_requests = match self {
            Self::Waiting(waiting) => waiting,
            Self::Available(previous) => {
                log_error!(
                    "action_server_goal_handle.provide_result",
                    "Action goal {goal_id} was provided with multiple results, \
                    which is not allowed by the action server state machine and \
                    indicates a bug in rclrs. The new result will be discarded.\
                    \nPrevious result: {previous:?}\
                    \nNew result: {result:?}"
                );
                return Err(RclrsError::RclError {
                    code: RclReturnCode::ActionGoalEventInvalid,
                    msg: Some(RclErrorMsg(
                        "action goal response is already set".to_string(),
                    )),
                });
            }
        };

        if !result_requests.is_empty() {
            let action_server = action_server_handle.lock();

            // Respond to all queued requests.
            for mut result_request in result_requests {
                Self::send_result(&*action_server, &mut result_request, &mut result)?;
            }
        }

        *self = Self::Available(result);
        Ok(())
    }

    fn add_result_request(
        &mut self,
        action_server_handle: &ActionServerHandle<A>,
        mut result_request: rmw_request_id_t,
    ) -> Result<(), RclrsError> {
        match self {
            Self::Waiting(waiting) => {
                waiting.push(result_request);
            }
            Self::Available(result) => {
                let action_server = action_server_handle.lock();
                Self::send_result(&*action_server, &mut result_request, result)?;
            }
        }
        Ok(())
    }

    fn send_result(
        action_server: &rcl_action_server_t,
        result_request: &mut rmw_request_id_t,
        result_response: &mut RmwResultResponse<A>,
    ) -> Result<(), RclrsError> {
        unsafe {
            // SAFETY: The action server handle is kept valid by the
            // ActionServerHandle. The compiler ensures we have unique access
            // to the result_request and result_response structures.
            rcl_action_send_result_response(
                action_server,
                result_request,
                result_response as *mut _ as *mut _,
            )
            .ok()
        }
    }
}
