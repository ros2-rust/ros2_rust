use crate::{
    rcl_bindings::*,
    log_error,
    GoalUuid, RclrsError, ToResult, ActionServer, ActionServerHandle,
};
use std::{
    borrow::Cow,
    sync::{Arc, Mutex},
    hash::Hash,
    ops::Deref,
};
use rosidl_runtime_rs::{ActionImpl, Message, Service};
use tokio::sync::watch::{Sender, Receiver, channel as watch_channel};

/// Values defined by `action_msgs/msg/GoalStatus`
#[repr(i8)]
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
enum GoalStatus {
    Unknown = 0,
    Accepted = 1,
    Executing = 2,
    Cancelling = 3,
    Succeeded = 4,
    Cancelled = 5,
    Aborted = 6,
}

/// Possible status values for terminal states
#[repr(i8)]
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
enum TerminalStatus {
    Succeeded = 4,
    Cancelled = 5,
    Aborted = 6,
}

/// From rcl documentation for rcl_action_accept_new_goal:
///
/// If a failure occurs, `NULL` is returned and an error message is set.
/// Possible reasons for failure:
///   - action server is invalid
///   - goal info is invalid
///   - goal ID is already being tracked by the action server
///   - memory allocation failure
///
/// We have no way of diagnosing which of these errors caused the failure, so
/// all we can do is indicate that an error occurred with accepting the goal.
#[derive(Debug, Clone)]
pub struct GoalAcceptanceError;

/// A handle for an action goal that has been requested but not accepted yet.
pub(super) struct RequestedActionServerGoalHandle<A: ActionImpl> {
    server: ActionServer<A>,
    goal_request: Arc<A::Goal>,
    uuid: GoalUuid,
}

impl<A: ActionImpl> RequestedActionServerGoalHandle<A> {
    pub(super) fn transition_to_accepted(self) -> Result<ActionServerGoalHandle<A>, GoalAcceptanceError> {
        let goal_handle = {
            let mut goal_info = unsafe {
                // SAFETY: Zero-initialized rcl structs are always safe to make
                rcl_action_get_zero_initialized_goal_info()
            };
            goal_info.goal_id.uuid = *self.uuid;
            goal_info.stamp = self.server.node().get_clock().now().to_rcl().unwrap_or_default();

            let mut server_handle = self.server.handle.lock();
            let goal_handle_ptr = unsafe {
                // SAFETY: The action server handle is locked and so synchronized with other
                // functions. The request_id and response message are uniquely owned, and so will
                // not mutate during this function call. The returned goal handle pointer should be
                // valid unless it is null.
                rcl_action_accept_new_goal(&mut *server_handle, &goal_info)
            };

            if goal_handle_ptr.is_null() {
                return Err(GoalAcceptanceError);
            }

            let goal_handle = unsafe {
                // SAFETY: We receive a loose pointer since the function call is
                // fallible, but the loose pointer we receive is actually managed
                // internally by rcl_action, so we must not retain its pointer
                // value. Instead we need to copy its internal impl* into a new
                // struct which we will take ownership of.
                //
                // It remains our responsibility to call `rcl_action_goal_handle_fini`
                // on the copy of the `rcl_action_goal_handle_t` because rcl_action
                // will not manage the memory of the inner impl*.
                *goal_handle_ptr
            };

            goal_handle
        };

        Ok(Arc::new(ActionServerGoalHandle {
            rcl_handle: Mutex::new(goal_handle),
            goal_request: self.goal_request,
            uuid: self.uuid,
            server: self.server.handle,
            result_response: Default::default(),
            cancellation: Default::default(),
        }))
    }
}

/// This struct is a minimal bridge from an rclrs action server goal to the rcl
/// API.
pub(super) struct ActionServerGoalHandle<A: ActionImpl> {
    rcl_handle: Mutex<rcl_action_goal_handle_t>,
    goal_request: Arc<A::Goal>,
    uuid: GoalUuid,
    result_response: ResponseState<A>,
    server: ActionServerHandle,
    cancellation: CancellationState,
}

// SAFETY: The functions accessing this type don't care about the thread they are
// running in. Therefore this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}

impl<A: ActionImpl> ActionServerGoalHandle<A> {
    /// Returns the goal state.
    fn get_status(&self) -> Result<GoalStatus, RclrsError> {
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
    pub(super) fn is_cancelling(&self) -> bool {
        self.get_status().is_ok_and(|s| s == GoalStatus::Cancelling)
    }

    /// Returns true if the goal is either pending or executing, or false if it has reached a
    /// terminal state.
    pub(super) fn is_active(&self) -> bool {
        let rcl_handle = self.rcl_handle.lock().unwrap();
        // SAFETY: The provided goal handle is properly initialized by construction.
        unsafe { rcl_action_goal_handle_is_active(*rcl_handle) }
    }

    /// Returns whether the goal is executing.
    pub(super) fn is_executing(&self) -> bool {
        self.get_status().is_ok_and(|s| s == GoalStatus::Executing)
    }

    /// Get the unique identifier of the goal.
    pub(super) fn goal_id(&self) -> GoalUuid {
        self.uuid
    }

    /// Get the user-provided message describing the goal.
    pub(super) fn goal(&self) -> Arc<A::Goal> {
        Arc::clone(&self.goal_request)
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
        let result_rmw = <A::Result as Message>::into_rmw_message(Cow::Borrowed(result)).into_owned();
        self.terminate_goal(&self.uuid, 6, result_rmw)?;
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
        let result_rmw = <A::Result as Message>::into_rmw_message(Cow::Borrowed(result)).into_owned();
        self.terminate_goal(&self.uuid, TerminalStatus::Succeeded, result_rmw)?;
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
        let result_rmw = <A::Result as Message>::into_rmw_message(Cow::Borrowed(result)).into_owned();
        self.terminate_goal(&self.uuid, TerminalStatus::Cancelled, result_rmw)?;
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
        if let Some(action_server) = self.action_server.upgrade() {
            action_server.publish_status()?;
        }
        Ok(())
    }

    /// Send an update about the goal's progress.
    ///
    /// This may only be called when the goal is executing.
    ///
    /// Returns an error if the goal is in any state other than executing.
    pub(super) fn publish_feedback(&self, feedback: &A::Feedback) -> Result<(), RclrsError> {
        // If the action server no longer exists, simply drop the message.
        if let Some(action_server) = self.action_server.upgrade() {
            action_server.publish_feedback(&self.uuid, feedback)?;
        }
        Ok(())
    }

    fn terminate_goal(
        &self,
        goal_id: &GoalUuid,
        status: TerminalStatus,
        result: <A::Result as Message>::RmwMsg,
    ) -> Result<(), RclrsError> {
        let response_rmw = <A as ActionImpl>::create_result_response(status as i8, result);

        // Publish the result to anyone listening.
        self.publish_result(goal_id, response_rmw);

        // Publish the state change.
        self.publish_status();

        // Notify rcl that a goal has terminated and to therefore recalculate the expired goal timer.
        unsafe {
            // SAFETY: The action server is locked and valid. No other preconditions.
            rcl_action_notify_goal_done(&*self.handle.lock())
        }
        .ok()?;

        // Release ownership of the goal handle. It will persist until the user also drops it.
        self.goal_handles.lock().unwrap().remove(&goal_id);

        Ok(())
    }


    /// Attempt to perform the given goal state transition.
    fn update_state(&self, event: rcl_action_goal_event_t) -> Result<(), RclrsError> {
        let mut rcl_handle = self.rcl_handle.lock().unwrap();
        // SAFETY: The provided goal handle is properly initialized by construction.
        unsafe { rcl_action_update_goal_state(*rcl_handle, event).ok() }
    }
}

pub(super) type ActionResponseRmw<A> = <<<A as ActionImpl>::GetResultService as Service>::Response as Message>::RmwMsg;

/// Manages the state of a goal's response.
pub(super) enum ResponseState<A: ActionImpl> {
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
            }
        };

        if !result_requests.is_empty() {
            let action_server = action_server_handle.lock();

            // Respond to all queued requests.
            for mut result_request in result_requests {
                Self::send_result(action_server, result_request, &mut result)?;
            }
        }

        *self = Self::Available(result);
    }

    fn add_result_request(
        &mut self,
        action_server_handle: &ActionServerHandle,
        goal_id: &GoalUuid,
        result_request: rmw_request_id_t,
    ) -> Result<(), RclrsError> {
        match self {
            Self::Waiting(waiting) => {
                waiting.push(result_request);
            }
            Self::Available(result) => {
                let action_server = action_server_handle.lock();
                Self::send_result(action_server, &mut result_request, result)?;
            }
        }
        Ok(())
    }

    fn send_result(
        action_server: &rmw_request_id_t,
        result_request: &mut rmw_request_id_t,
        result_response: &mut ActionResponseRmw<A>,
    ) -> Result<(), RclrsError> {
        unsafe {
            // SAFETY: The action server handle is kept valid by the
            // ActionServerHandle. The compiler ensures we have unique access
            // to the result_request and result_response structures.
            rcl_action_send_result_response(
                action_server,
                &mut result_request,
                result_response as *mut _ as *mut _,
            )
            .ok()
        }
    }
}

pub(super) struct CancellationState {
    receiver: Receiver<CancellationMode>,
    sender: Sender<CancellationMode>,
}

impl Default for CancellationState {
    fn default() -> Self {
        let (sender, receiver) = watch_channel(CancellationMode::None);
        Self { receiver, sender }
    }
}

pub(super) enum CancellationMode {
    None,
    CancelRequested(Vec<rmw_request_id_t>),
    Cancelling,
}

impl<A: ActionImpl> Drop for ActionServerGoalHandle<A> {
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
