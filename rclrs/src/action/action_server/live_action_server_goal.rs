use super::{
    ActionServerGoalHandle, ActionServerHandle, CancellationRequest, CancellationState,
    GoalStatusCode, TerminalStatus,
};
use crate::{log_error, rcl_bindings::*, RclrsError, ToResult};
use rosidl_runtime_rs::{Action, Message};
use std::{borrow::Cow, ops::Deref, sync::Arc};

/// This struct is the bridge to the rcl_action API for action server goals that
/// are still active. It can be used to perform transitions while keeping data in
/// sync between the rclrs user and the rcl_action library.
///
/// When this is dropped, the goal will automatically be transitioned into the
/// aborted status if the user did not transition the goal into a different
/// terminal status before dropping it.
pub(super) struct LiveActionServerGoal<A: Action> {
    goal_request: Arc<A::Goal>,
    cancellation: Arc<CancellationState<A>>,
    handle: Arc<ActionServerGoalHandle<A>>,
    server: Arc<ActionServerHandle<A>>,
}

impl<A: Action> LiveActionServerGoal<A> {
    pub(super) fn new(
        handle: Arc<ActionServerGoalHandle<A>>,
        server: Arc<ActionServerHandle<A>>,
        goal_request: Arc<A::Goal>,
    ) -> Self {
        Self {
            handle,
            server,
            goal_request,
            cancellation: Default::default(),
        }
    }

    /// Get the user-provided message describing the goal.
    pub(super) fn goal(&self) -> &Arc<A::Goal> {
        &self.goal_request
    }

    /// Has a cancellation been requested for this goal.
    pub(super) fn cancel_requested(&self) -> bool {
        self.cancellation.cancel_requested()
    }

    pub(super) fn cancellation(&self) -> &Arc<CancellationState<A>> {
        &self.cancellation
    }

    pub(super) fn request_cancellation(&self, request: CancellationRequest<A>) {
        self.cancellation
            .request_cancellation(request, self.goal_id());
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
    pub(super) fn transition_to_aborted(&self, result: A::Result) {
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
    pub(super) fn transition_to_succeed(&self, result: A::Result) {
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
    pub(super) fn transition_to_cancelled(&self, result: A::Result) {
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
    /// This should only be called in between a goal being accepted and terminated,
    /// but that is not enforced in this method.
    pub(super) fn publish_feedback(&self, feedback: A::Feedback) {
        let feedback_rmw =
            <<A as Action>::Feedback as Message>::into_rmw_message(Cow::Owned(feedback));
        let mut feedback_msg =
            <A as Action>::create_feedback_message(self.goal_id(), feedback_rmw.into_owned());
        let r = unsafe {
            // SAFETY: The action server is locked through the handle, meaning that no other
            // non-thread-safe functions can be called on it at the same time. The feedback_msg is
            // exclusively owned here, ensuring that it won't be modified during the call.
            // rcl_action_publish_feedback() guarantees that it won't modify `feedback_msg`.
            rcl_action_publish_feedback(&*self.server.lock(), &mut feedback_msg as *mut _ as *mut _)
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

    /// Atomically check the state of the goal and send feedback if the state
    /// has not terminated.
    ///
    /// This is used by [`crate::FeedbackSender`] to ensure that it does not
    /// send feedback in an unacceptable state, even if the state of the goal
    /// may change while it tries to send the feedback.
    pub(super) fn safe_publish_feedback(&self, feedback: A::Feedback) -> Result<(), A::Feedback> {
        let goal_handle = self.handle.lock();

        let mut state = GoalStatusCode::Unknown as rcl_action_goal_state_t;
        let r = unsafe {
            // SAFETY: The goal handle is properly initialized and its mutex is locked
            rcl_action_goal_handle_get_status(&*goal_handle, &mut state).ok()
        };
        if let Err(err) = r {
            log_error!(
                "LiveActionServerGoal.safe_publish_feedback",
                "Unexpected error while getting status: {err}",
            );
            return Err(feedback);
        }

        // The goal's rcl_handle mutex is locked and will prevent the goal status from
        // being changed until this function is finished.
        //
        // The publish_feedback method does not need to lock the goal's rcl_handle
        // so there is no double-lock to worry about.
        self.publish_feedback(feedback);
        Ok(())
    }

    fn terminate_goal(&self, status: TerminalStatus, result: A::Result) -> Result<(), RclrsError> {
        let result_rmw = <A::Result as Message>::into_rmw_message(Cow::Owned(result)).into_owned();
        let response_rmw = <A as Action>::create_result_response(status as i8, result_rmw);
        self.handle
            .provide_result(self.server.as_ref(), response_rmw)?;

        // Publish the state change.
        self.server.publish_status()?;

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

impl<A: Action> Deref for LiveActionServerGoal<A> {
    type Target = ActionServerGoalHandle<A>;

    fn deref(&self) -> &Self::Target {
        self.handle.as_ref()
    }
}

impl<A: Action> Drop for LiveActionServerGoal<A> {
    fn drop(&mut self) {
        match self.get_status() {
            GoalStatusCode::Accepted => {
                // Transition into executing and then into aborted to reach a
                // terminal state.
                self.transition_to_executing();
                self.transition_to_aborted(Default::default());
            }
            GoalStatusCode::Cancelling | GoalStatusCode::Executing => {
                self.transition_to_aborted(Default::default());
            }
            GoalStatusCode::Succeeded | GoalStatusCode::Cancelled | GoalStatusCode::Aborted => {
                // Already in a terminal state, no need to do anything.
            }
            GoalStatusCode::Unknown => {
                log_error!(
                    "LiveActionServerGoal.drop",
                    "Goal status is unknown. This indicates a bug, please \
                    report this to the maintainers of rclrs."
                );
            }
        }
    }
}
