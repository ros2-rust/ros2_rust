use crate::{
    rcl_bindings::*,
    log_error,
    GoalUuid, RclrsError, RclErrorMsg, RclReturnCode, ToResult,
};
use super::{
    ActionServerGoalHandle, ActionServerHandle, CancellationState, GoalStatus, TerminalStatus,
};
use std::{
    borrow::Cow,
    sync::{Arc, Mutex},
    ops::Deref,
};
use rosidl_runtime_rs::{Action, ActionImpl, Message, Service};

/// This struct is the bridge to the rcl_action API for action server goals that
/// are still active. It can be used to perform transitions while keeping data in
/// sync between the rclrs user and the rcl_action library.
///
/// When this is dropped, the goal will automatically be transitioned into the
/// aborted status if the user did not transition the goal into a different
/// terminal status before dropping it.
pub(super) struct LiveActionServerGoal<A: ActionImpl> {
    goal_request: Arc<A::Goal>,
    result_response: Mutex<ResponseState<A>>,
    cancellation: Arc<CancellationState>,
    handle: Arc<ActionServerGoalHandle>,
    server: Arc<ActionServerHandle>,
}

impl<A: ActionImpl> LiveActionServerGoal<A> {
    pub(super) fn new(
        handle: Arc<ActionServerGoalHandle>,
        server: Arc<ActionServerHandle>,
        goal_request: Arc<A::Goal>,
    ) -> Self {
        Self {
            handle,
            server,
            goal_request,
            result_response: Mutex::new(ResponseState::new()),
            cancellation: Default::default(),
        }
    }

    /// Get the user-provided message describing the goal.
    pub(super) fn goal(&self) -> Arc<A::Goal> {
        Arc::clone(&self.goal_request)
    }

    /// Has a cancellation been requested for this goal.
    pub(super) fn cancel_requested(&self) -> bool {
        self.cancellation.cancel_requested()
    }

    pub(super) fn cancellation(&self) -> &Arc<CancellationState> {
        &self.cancellation
    }

    /// Indicate that the goal is being cancelled.
    ///
    /// This is called when a cancel request for the goal has been accepted.
    ///
    /// Returns an error if the goal is in any state other than accepted or executing.
    pub(super) fn transition_to_cancelling(&self) {
        self.cancellation.accept_cancellation(self.goal_id());
        let r = self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCEL_GOAL);
        if let Err(err) = r {
            log_error!(
                "live_action_server_goal.transition_to_cancelling",
                "Failed to transition to the cancelling state. This error should \
                not happen, please report it to the maintainers of rclrs: {err}",
            );
        }
    }

    /// If there are any open cancellation requests, they will be rejected and
    /// the [`CancellationMode`] will be restored to [`CancellationMode::None`].
    ///
    /// This function has no effect if the goal is already in the cancelling state.
    pub(super) fn reject_cancellation(&self) {
        self.cancellation.reject_cancellation(self.goal_id());
    }

    /// Indicate that the goal could not be reached and has been aborted.
    ///
    /// Only call this if the goal is executing but cannot be completed. This is a terminal state,
    /// so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub(super) fn transition_to_aborted(&self, result: &A::Result) {
        let r = self
            .update_state(rcl_action_goal_event_t::GOAL_EVENT_ABORT)
            .and_then(|_| self.terminate_goal(TerminalStatus::Aborted, result));

        if let Err(err) = r {
            log_error!(
                "live_action_server_goal.transition_to_aborted",
                "Failed to transition to the aborted state. This error should not \
                happen, please report it to the maintainers of rclrs: {err}",
            );
        }
    }

    /// Indicate that the goal has succeeded.
    ///
    /// Only call this if the goal is executing and has reached the desired final state. This is a
    /// terminal state, so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub(super) fn transition_to_succeed(&self, result: &A::Result) {
        let r = self
            .update_state(rcl_action_goal_event_t::GOAL_EVENT_SUCCEED)
            .and_then(|_| self.terminate_goal(TerminalStatus::Succeeded, result));

        if let Err(err) = r {
            log_error!(
                "live_action_server_goal.transition_to_succeed",
                "Failed to transition to the succeeded state. This error should not \
                happen, please report it to the maintainers of rclrs: {err}",
            );
        }
    }

    /// Indicate that the goal has been cancelled.
    ///
    /// Only call this if the goal is executing or pending, but has been cancelled. This is a
    /// terminal state, so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing or pending.
    pub(super) fn transition_to_cancelled(&self, result: &A::Result) {
        let r = self
            .update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCELED)
            .and_then(|_| self.terminate_goal(TerminalStatus::Cancelled, result));

        if let Err(err) = r {
            log_error!(
                "live_action_server_goal.transition_to_cancelled",
                "Failed to transition to cancelled state. This error should not \
                happen, please report it to the maintainers of rclrs: {err}",
            );
        }
    }

    /// Indicate that the server is starting to execute the goal.
    ///
    /// Only call this if the goal is pending. This is a terminal state, so no more methods may be
    /// called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than pending.
    pub(super) fn transition_to_executing(&self) {
        let r = self
            .update_state(rcl_action_goal_event_t::GOAL_EVENT_EXECUTE)
            .and_then(|_| self.server.publish_status());

        if let Err(err) = r {
            log_error!(
                "live_action_server_goal.transition_to_executing",
                "Failed to transition to executing status. This error should \
                not happen, please report it to the maintainers of rclrs: {err}",
            );
        }
    }

    /// Send an update about the goal's progress.
    ///
    /// This may only be called when the goal is executing.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub(super) fn publish_feedback(&self, feedback: &A::Feedback) {
        let feedback_rmw = <<A as Action>::Feedback as Message>::into_rmw_message(Cow::Borrowed(feedback));
        let mut feedback_msg = <A as ActionImpl>::create_feedback_message(&*self.goal_id(), feedback_rmw.into_owned());
        let r = unsafe {
            // SAFETY: The action server is locked through the handle, meaning that no other
            // non-thread-safe functions can be called on it at the same time. The feedback_msg is
            // exclusively owned here, ensuring that it won't be modified during the call.
            // rcl_action_publish_feedback() guarantees that it won't modify `feedback_msg`.
            rcl_action_publish_feedback(
                &*self.server.lock(),
                &mut feedback_msg as *mut _ as *mut _,
            )
            .ok()
        };

        if let Err(err) = r {
            // This is not an error that should be able to occur.
            log_error!(
                "live_action_server_goal.publish_feedback",
                "Failed to publish feedback for an action server goal. This should \
                not happen, please report it to the maintainers of rclrs: {err}",
            );
        }
    }

    fn terminate_goal(
        &self,
        status: TerminalStatus,
        result: &A::Result,
    ) -> Result<(), RclrsError> {
        let result_rmw = <A::Result as Message>::into_rmw_message(Cow::Borrowed(result)).into_owned();
        let response_rmw = <A as ActionImpl>::create_result_response(status as i8, result_rmw);

        // Publish the result to anyone listening.
        self.result_response.lock().unwrap().provide_result(&self.server, self.goal_id(), response_rmw);

        // Publish the state change.
        self.server.publish_status();

        // Notify rcl that a goal has terminated and to therefore recalculate the expired goal timer.
        unsafe {
            // SAFETY: The action server is locked and valid. No other preconditions.
            rcl_action_notify_goal_done(&*self.server.lock())
        }
        .ok()?;

        Ok(())
    }

    /// Attempt to perform the given goal state transition.
    fn update_state(&self, event: rcl_action_goal_event_t) -> Result<(), RclrsError> {
        let mut rcl_handle = self.handle.lock();
        // SAFETY: The provided goal handle is properly initialized by construction.
        unsafe { rcl_action_update_goal_state(&mut *rcl_handle, event).ok() }
    }
}

impl<A: ActionImpl> Deref for LiveActionServerGoal<A> {
    type Target = ActionServerGoalHandle;

    fn deref(&self) -> &Self::Target {
        self.handle.as_ref()
    }
}

impl<A: ActionImpl> Drop for LiveActionServerGoal<A> {
    fn drop(&mut self) {
        match self.get_status() {
            GoalStatus::Accepted => {
                // Transition into executing and then into aborted to reach a
                // terminal state.
                self.transition_to_executing();
                self.transition_to_aborted(&Default::default());
            }
            GoalStatus::Cancelling | GoalStatus::Executing => {
                self.transition_to_aborted(&Default::default());
            }
            GoalStatus::Succeeded | GoalStatus::Cancelled | GoalStatus::Aborted => {
                // Already in a terminal state, no need to do anything.
            }
            GoalStatus::Unknown => {
                log_error!(
                    "LiveActionServerGoal.drop",
                    "Goal status is unknown. This indicates a bug, please \
                    report this to the maintainers of rclrs."
                );
            }
        }
    }
}

pub(super) type ActionResponseRmw<A> = <<<A as ActionImpl>::GetResultService as Service>::Response as Message>::RmwMsg;

/// Manages the state of a goal's response.
enum ResponseState<A: ActionImpl> {
    /// The response has not arrived yet. There may be some clients waiting for
    /// the response, and they'll be listed here.
    Waiting(Vec<rmw_request_id_t>),
    /// The response is available.
    Available(ActionResponseRmw<A>),
}

impl<A: ActionImpl> ResponseState<A> {
    fn new() -> Self {
        Self::Waiting(Vec::new())
    }

    fn provide_result(
        &mut self,
        action_server_handle: &ActionServerHandle,
        goal_id: &GoalUuid,
        mut result: ActionResponseRmw<A>,
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
                    msg: Some(RclErrorMsg("action goal response is already set".to_string())),
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
        action_server_handle: &ActionServerHandle,
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
        result_response: &mut ActionResponseRmw<A>,
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
