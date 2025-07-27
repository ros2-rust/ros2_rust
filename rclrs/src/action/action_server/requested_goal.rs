use crate::{
    rcl_bindings::*,
    log_error,
    GoalUuid, RclrsError, ToResult, ActionServer,
};
use super::{ActionServerGoalHandle, LiveActionServerGoal, AcceptedGoal};
use std::sync::Arc;
use rosidl_runtime_rs::ActionImpl;

#[derive(Debug, Clone)]
pub struct GoalAcceptanceError;

/// An action goal that has been requested but not accepted yet. If this is
/// dropped without being accepted then the goal request will be rejected.
pub struct RequestedGoal<A: ActionImpl> {
    server: ActionServer<A>,
    goal_request: Arc<A::Goal>,
    uuid: GoalUuid,
    goal_request_id: rmw_request_id_t,
    accepted: bool,
}

impl<A: ActionImpl> RequestedGoal<A> {
    /// Accept the requested goal. The action client will be notified that the
    /// goal was accepted, and you will be able to begin executing.
    pub fn accept(mut self) -> Result<AcceptedGoal<A>, RclrsError> {
        let handle = {
            let mut goal_info = unsafe {
                // SAFETY: Zero-initialized rcl structs are always safe to make
                rcl_action_get_zero_initialized_goal_info()
            };
            goal_info.goal_id.uuid = *self.uuid;
            goal_info.stamp = self.server.node().get_clock().now().to_rcl().unwrap_or(
                builtin_interfaces__msg__Time { sec: 0, nanosec: 0 }
            );

            let mut server_handle = self.server.handle.lock();
            let goal_handle_ptr = unsafe {
                // SAFETY: The action server handle is locked and so synchronized with other
                // functions. The request_id and response message are uniquely owned, and so will
                // not mutate during this function call. The returned goal handle pointer should be
                // valid unless it is null.
                rcl_action_accept_new_goal(&mut *server_handle, &goal_info)
            };

            if goal_handle_ptr.is_null() {
                return Err(RclrsError::GoalAcceptanceError);
            }

            let mut goal_handle = rcl_action_goal_handle_t {
                impl_: std::ptr::null_mut(),
            };

            unsafe {
                // SAFETY: We receive a loose pointer since the function call is
                // fallible, but the loose pointer we receive is actually managed
                // internally by rcl_action, so we must not retain its pointer
                // value. Instead we need to copy its internal impl* into a new
                // struct which we will take ownership of.
                //
                // It remains our responsibility to call `rcl_action_goal_handle_fini`
                // on the copy of the `rcl_action_goal_handle_t` because rcl_action
                // will not manage the memory of the inner impl*.
                goal_handle.impl_ = (*goal_handle_ptr).impl_;
            };

            Arc::new(ActionServerGoalHandle::new(goal_handle, self.uuid))
        };

        // We need to store a strong reference to the goal handle in the action
        // server handle to ensure the goal handle outlives its use within the
        // action server.
        self.server.handle.goals.lock().unwrap().insert(*handle.goal_id(), Arc::clone(&handle));

        self.accepted = true;
        self.send_goal_response();

        let live = Arc::new(LiveActionServerGoal::new(
            handle,
            Arc::clone(&self.server.handle),
            Arc::clone(&self.goal_request),
        ));

        // Add the live goal to the goal board so the action server executor can
        // interact with it.
        self.server.board.lock().unwrap().live_goals.insert(self.uuid, Arc::downgrade(&live));

        Ok(AcceptedGoal::new(live))
    }

    /// Reject this goal request.
    ///
    /// This is the same as simply allowing the [`RequestedGoal`] to drop without
    /// accepting.
    pub fn reject(self) {
        // This function need to do anything, it will simply drop the RequestedGoal,
        // and the custom Drop implementation will notify the action server that
        // the goal is rejected.
    }

    fn send_goal_response(&mut self) -> Result<(), RclrsError> {
        let stamp = self.server.node().get_clock().now().to_sec_nanosec().unwrap_or((0, 0));
        let mut response_rmw = <A as ActionImpl>::create_goal_response(self.accepted, stamp);
        unsafe {
            rcl_action_send_goal_response(
                &*self.server.handle.lock(),
                &mut self.goal_request_id,
                &mut response_rmw as *mut _ as *mut _,
            )
        }
        .ok()
    }
}

impl<A: ActionImpl> Drop for RequestedGoal<A> {
    fn drop(&mut self) {
        if !self.accepted {
            // We should notify that the goal has been rejected.
            if let Err(err) = self.send_goal_response() {
                log_error!(
                    "RequestedGoal.drop",
                    "Error occurred while attempting to reject a goal: {err}",
                );
            }
        }
    }
}
