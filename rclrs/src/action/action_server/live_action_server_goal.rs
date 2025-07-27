use crate::{
    rcl_bindings::*,
    vendor::{
        builtin_interfaces::msg::Time,
        action_msgs::{
            msg::GoalInfo,
            srv::CancelGoal_Response,
        },
        unique_identifier_msgs::msg::UUID,
    },
    log_error,
    CancelResponse, GoalUuid, RclrsError, RclErrorMsg, RclReturnCode, ToResult, Node,
};
use super::{
    ActionServerGoalHandle, ActionServerHandle, TerminalStatus, GoalStatus,
};
use std::{
    borrow::Cow,
    collections::HashSet,
    sync::{Arc, Mutex},
    ops::Deref,
};
use rosidl_runtime_rs::{Action, ActionImpl, Message, Service};
use tokio::sync::watch::{Sender, Receiver, channel as watch_channel};


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
    cancellation: CancellationState,
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
        self.cancellation.receiver.borrow().cancel_requested()
    }

    /// Indicate that the goal is being cancelled.
    ///
    /// This is called when a cancel request for the goal has been accepted.
    ///
    /// Returns an error if the goal is in any state other than accepted or executing.
    pub(super) fn transition_to_cancelling(&self) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCEL_GOAL)
    }

    /// Indicate that the goal could not be reached and has been aborted.
    ///
    /// Only call this if the goal is executing but cannot be completed. This is a terminal state,
    /// so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub(super) fn transition_to_abort(&self, result: &A::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_ABORT)?;
        self.terminate_goal(TerminalStatus::Aborted, result)?;
        Ok(())
    }

    /// Indicate that the goal has succeeded.
    ///
    /// Only call this if the goal is executing and has reached the desired final state. This is a
    /// terminal state, so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub(super) fn transition_to_succeed(&self, result: &A::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_SUCCEED)?;
        self.terminate_goal(TerminalStatus::Succeeded, result)?;
        Ok(())
    }

    /// Indicate that the goal has been cancelled.
    ///
    /// Only call this if the goal is executing or pending, but has been cancelled. This is a
    /// terminal state, so no more methods may be called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than executing or pending.
    pub(super) fn transition_to_cancelled(&self, result: &A::Result) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_CANCELED)?;
        self.terminate_goal(TerminalStatus::Cancelled, result)?;
        Ok(())
    }

    /// Indicate that the server is starting to execute the goal.
    ///
    /// Only call this if the goal is pending. This is a terminal state, so no more methods may be
    /// called on a goal handle after this is called.
    ///
    /// Returns an error if the goal is in any state other than pending.
    pub(super) fn transition_to_execute(&self) -> Result<(), RclrsError> {
        self.update_state(rcl_action_goal_event_t::GOAL_EVENT_EXECUTE)?;

        // Publish the state change.
        self.server.publish_status()
    }

    /// Send an update about the goal's progress.
    ///
    /// This may only be called when the goal is executing.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub(super) fn publish_feedback(&self, feedback: &A::Feedback) -> Result<(), RclrsError> {
        let feedback_rmw = <<A as Action>::Feedback as Message>::into_rmw_message(Cow::Borrowed(feedback));
        let mut feedback_msg = <A as ActionImpl>::create_feedback_message(&*self.goal_id(), feedback_rmw.into_owned());
        unsafe {
            // SAFETY: The action server is locked through the handle, meaning that no other
            // non-thread-safe functions can be called on it at the same time. The feedback_msg is
            // exclusively owned here, ensuring that it won't be modified during the call.
            // rcl_action_publish_feedback() guarantees that it won't modify `feedback_msg`.
            rcl_action_publish_feedback(
                &*self.server.lock(),
                &mut feedback_msg as *mut _ as *mut _,
            )
            .ok()
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
                self.transition_to_execute();
                self.transition_to_abort(&Default::default());
            }
            GoalStatus::Cancelling | GoalStatus::Executing => {
                self.transition_to_abort(&Default::default());
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

pub(super) struct CancellationState {
    receiver: Receiver<bool>,
    sender: Sender<bool>,

    /// We put a mutex on the mode because when we respond to cancellation
    /// requests we need to ensure that we update the cancellation mode
    /// atomically
    mode: Mutex<CancellationMode>,
}

impl CancellationState {
    /// Check if a cancellation is currently being requested.
    fn cancel_requested(&self) -> bool {
        *self.receiver.borrow()
    }

    fn request_cancellation(
        &self,
        request: CancellationRequest,
        uuid: &GoalUuid,
    ) {
        let mut mode = self.mode.lock().unwrap();
        match &mut *mode {
            CancellationMode::None => {
                let requests = Vec::from_iter([request]);
                *mode = CancellationMode::CancelRequested(requests);
            }
            CancellationMode::CancelRequested(requests) => {
                requests.push(request);
            }
            CancellationMode::Cancelling => {
                request.accept(*uuid);
            }
        }
    }

    /// Tell current cancellation requesters that their requests are rejected
    fn reject_cancellation(&self, uuid: &GoalUuid) {
        let mut mode = self.mode.lock().unwrap();
        match &mut *mode {
            CancellationMode::CancelRequested(requesters) => {
                for requester in requesters.drain(..) {
                    requester.reject(*uuid);
                }

                // Revert to not having any cancellation mode
                *mode = CancellationMode::None;
                self.sender.send(false);
            }
            CancellationMode::None => {
                // Do nothing
            }
            CancellationMode::Cancelling => {
                // Do nothing. We will not revert a cancellation state.
            }
        }
    }

    /// Tell current and future cancellation requesters that their requests are
    /// accepted
    fn accept_cancellation(&self, uuid: &GoalUuid) {
        let mut mode = self.mode.lock().unwrap();
        match &mut *mode {
            CancellationMode::CancelRequested(requesters) => {
                for requester in requesters.drain(..) {
                    requester.accept(*uuid);
                }

                // Progress to cancelling mode
                *mode = CancellationMode::Cancelling;
                // Just in case this signal was never sent, make sure we have
                // a true value in the cancel requested channel.
                self.sender.send(true);
            }
            CancellationMode::None => {
                // Skip straight to cancellation mode since the user has accepted
                // a cancellation even though it wasn't requested externally.
                *mode = CancellationMode::Cancelling;
                // Make sure the cancellation is signalled.
                self.sender.send(true);
            }
            CancellationMode::Cancelling => {
                // Do nothing
            }
        }
    }

}

impl Default for CancellationState {
    fn default() -> Self {
        let (sender, receiver) = watch_channel(CancellationMode::None);
        Self { receiver, sender }
    }
}

pub(super) enum CancellationMode {
    None,
    CancelRequested(Vec<CancellationRequest>),
    Cancelling,
}

/// This struct exists to deal with the fact that a single cancellation request
/// can trigger multiple goal cancellations at once. This allows us to
/// asynchronously receive the accept/reject results from all the different goals
/// and then issue the reply once all are received.
struct CancellationRequest {
    inner: Arc<Mutex<CancellationRequestInner>>,
}

impl CancellationRequest {
    fn accept(&self, uuid: GoalUuid) {
        let mut inner = self.inner.lock().unwrap();
        if !inner.received.insert(uuid) {
            return;
        }

        let stamp = inner.node.get_clock().now().to_ros_msg().unwrap_or_default();
        let info = GoalInfo {
            goal_id: UUID { uuid: *uuid },
            stamp,
        };

        inner.accepted.push(info);
        inner.respond_if_ready();
    }

    fn reject(&self, uuid: GoalUuid) {
        let mut inner = self.inner.lock().unwrap();
        if !inner.received.insert(uuid) {
            return;
        }

        inner.respond_if_ready();
    }

}

struct CancellationRequestInner {
    id: rmw_request_id_t,
    waiting_for: Vec<GoalUuid>,
    received: HashSet<GoalUuid>,
    accepted: Vec<GoalInfo>,
    response_sent: bool,
    server: Arc<ActionServerHandle>,
    node: Node,
}

impl CancellationRequestInner {
    fn respond_if_ready(&mut self) {
        for expected in &self.waiting_for {
            if !self.received.contains(expected) {
                return;
            }
        }

        self.respond();
    }

    fn respond(&mut self) {
        if self.response_sent {
            return;
        }

        self.response_sent = true;

        let mut response = CancelGoal_Response::default();
        response.goals_canceling = self.accepted.drain(..).collect();
        if response.goals_canceling.is_empty() {
            response.return_code = CancelResponse::Reject as i8;
        } else {
            response.return_code = CancelResponse::Accept as i8;
        }

        let mut response_rmw = CancelGoal_Response::into_rmw_message(Cow::Owned(response)).into_owned();
        let r = unsafe {
            // SAFETY: The action server handle is locked and so synchronized with other functions.
            // The request_id and response are both uniquely owned or borrowed, and so neither will
            // mutate during this function call.
            rcl_action_send_cancel_response(
                &*self.server.lock(),
                &mut self.id,
                &mut response_rmw as *mut _ as *mut _,
            )
            .ok()
        };

        if let Err(err) = r {
            log_error!(
                "CancellationRequest.respond",
                "Error occurred while responding to a cancellation request: {err}"
            )
        }
    }
}

impl Drop for CancellationRequestInner {
    fn drop(&mut self) {
        if !self.response_sent {
            // As a last resort, send the response if all possible responders
            // have dropped.
            self.respond();
        }
    }
}
