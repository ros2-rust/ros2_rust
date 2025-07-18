use crate::{
    action::{CancelResponse, GoalResponse, GoalUuid, ServerGoalHandle},
    error::{RclReturnCode, ToResult},
    rcl_bindings::*,
    wait::WaitableNumEntities,
    Clock, DropGuard, Node, NodeHandle, QoSProfile, RclrsError, ENTITY_LIFECYCLE_MUTEX,
};
use rosidl_runtime_rs::{Action, ActionImpl, Message, Service};
use std::{
    borrow::Borrow,
    collections::HashMap,
    ffi::CString,
    sync::{atomic::AtomicBool, Arc, Mutex, MutexGuard},
};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_server_t {}

/// Manage the lifecycle of an `rcl_action_server_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_action_server_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct ActionServerHandle {
    rcl_action_server: Mutex<rcl_action_server_t>,
    node_handle: Arc<NodeHandle>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ActionServerHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_server_t> {
        self.rcl_action_server.lock().unwrap()
    }
}

impl Drop for ActionServerHandle {
    fn drop(&mut self) {
        let rcl_action_server = self.rcl_action_server.get_mut().unwrap();
        let mut rcl_node = self.node_handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_action_server_fini(rcl_action_server, &mut *rcl_node);
        }
    }
}

/// Trait to be implemented by concrete ActionServer structs.
///
/// See [`ActionServer<T>`] for an example
pub trait ActionServerBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ActionServerHandle;
    /// Returns the number of underlying entities for the action server.
    fn num_entities(&self) -> &WaitableNumEntities;
    /// Tries to run the callback for the given readiness mode.
    fn execute(self: Arc<Self>, mode: ReadyMode) -> Result<(), RclrsError>;
}

pub(crate) enum ReadyMode {
    GoalRequest,
    CancelRequest,
    ResultRequest,
    GoalExpired,
}

pub type GoalCallback<ActionT> = dyn Fn(GoalUuid, <ActionT as rosidl_runtime_rs::Action>::Goal) -> GoalResponse + 'static + Send + Sync;
pub type CancelCallback<ActionT> = dyn Fn(Arc<ServerGoalHandle<ActionT>>) -> CancelResponse + 'static + Send + Sync;
pub type AcceptedCallback<ActionT> = dyn Fn(Arc<ServerGoalHandle<ActionT>>) + 'static + Send + Sync;

/// An action server that can respond to requests sent by ROS action clients.
///
/// Create an action server using [`Node::create_action_server`][1].
///
/// ROS only supports having one server for any given fully-qualified
/// action name. "Fully-qualified" means the namespace is also taken into account
/// for uniqueness. A clone of an `ActionServer` will refer to the same server
/// instance as the original. The underlying instance is tied to [`ActionServerState`]
/// which implements the [`ActionServer`] API.
///
/// Responding to requests requires the node's executor to [spin][2].
///
/// [1]: crate::NodeState::create_action_server
/// [2]: crate::spin
pub type ActionServer<ActionT> = Arc<ActionServerState<ActionT>>;

/// The inner state of an [`ActionServer`].
///
/// This is public so that you can choose to create a [`Weak`][1] reference to it
/// if you want to be able to refer to a [`ActionServer`] in a non-owning way. It is
/// generally recommended to manage the `ActionServerState` inside of an [`Arc`],
/// and [`ActionServer`] is provided as a convenience alias for that.
///
/// The public API of the [`ActionServer`] type is implemented via `ActionServerState`.
///
/// [1]: std::sync::Weak
pub struct ActionServerState<ActionT>
where
    ActionT: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl,
{
    pub(crate) handle: Arc<ActionServerHandle>,
    num_entities: WaitableNumEntities,
    goal_callback: Box<GoalCallback<ActionT>>,
    cancel_callback: Box<CancelCallback<ActionT>>,
    accepted_callback: Box<AcceptedCallback<ActionT>>,
    // TODO(nwn): Audit these three mutexes to ensure there's no deadlocks or broken invariants. We
    // may want to join them behind a shared mutex, at least for the `goal_results` and `result_requests`.
    goal_handles: Mutex<HashMap<GoalUuid, Arc<ServerGoalHandle<ActionT>>>>,
    goal_results: Mutex<HashMap<GoalUuid, <<ActionT::GetResultService as Service>::Response as Message>::RmwMsg>>,
    result_requests: Mutex<HashMap<GoalUuid, Vec<rmw_request_id_t>>>,
    /// Ensure the parent node remains alive as long as the subscription is held.
    /// This implementation will change in the future.
    #[allow(unused)]
    node: Node,
}

impl<T> ActionServerState<T>
where
    T: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl,
{
    /// Creates a new action server.
    pub(crate) fn new<'a>(
        node: &Node,
        options: impl Into<ActionServerOptions<'a>>,
        goal_callback: impl Fn(GoalUuid, T::Goal) -> GoalResponse + 'static + Send + Sync,
        cancel_callback: impl Fn(Arc<ServerGoalHandle<T>>) -> CancelResponse + 'static + Send + Sync,
        accepted_callback: impl Fn(Arc<ServerGoalHandle<T>>) + 'static + Send + Sync,
    ) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl,
    {
        let options = options.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_server = unsafe { rcl_action_get_zero_initialized_server() };
        let type_support = T::get_type_support() as *const rosidl_action_type_support_t;
        let action_name_c_string =
            CString::new(options.action_name).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: options.action_name.into(),
            })?;

        // SAFETY: No preconditions for this function.
        let action_server_options = unsafe { rcl_action_server_get_default_options() };

        {
            let mut rcl_node = node.handle.rcl_node.lock().unwrap();
            let clock = node.get_clock();
            let rcl_clock = clock.rcl_clock();
            let mut rcl_clock = rcl_clock.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();

            // SAFETY:
            // * The rcl_action_server is zero-initialized as mandated by this function.
            // * The rcl_node is kept alive by the NodeHandle because it is a dependency of the action server.
            // * The action name and the options are copied by this function, so they can be dropped
            //   afterwards.
            // * The entity lifecycle mutex is locked to protect against the risk of global
            //   variables in the rmw implementation being unsafely modified during initialization.
            unsafe {
                rcl_action_server_init(
                    &mut rcl_action_server,
                    &mut *rcl_node,
                    &mut *rcl_clock,
                    type_support,
                    action_name_c_string.as_ptr(),
                    &action_server_options,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(ActionServerHandle {
            rcl_action_server: Mutex::new(rcl_action_server),
            node_handle: Arc::clone(&node.handle),
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        let mut num_entities = WaitableNumEntities::default();
        unsafe {
            rcl_action_server_wait_set_get_num_entities(
                &*handle.lock(),
                &mut num_entities.num_subscriptions,
                &mut num_entities.num_guard_conditions,
                &mut num_entities.num_timers,
                &mut num_entities.num_clients,
                &mut num_entities.num_services,
            )
            .ok()?;
        }

        Ok(Self {
            handle,
            num_entities,
            goal_callback: Box::new(goal_callback),
            cancel_callback: Box::new(cancel_callback),
            accepted_callback: Box::new(accepted_callback),
            goal_handles: Mutex::new(HashMap::new()),
            goal_results: Mutex::new(HashMap::new()),
            result_requests: Mutex::new(HashMap::new()),
            node: node.clone(),
        })
    }

    fn take_goal_request(&self) -> Result<(<<T::SendGoalService as Service>::Request as Message>::RmwMsg, rmw_request_id_t), RclrsError> {
        let mut request_id = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwRequest<T> = <<<T as ActionImpl>::SendGoalService as Service>::Request as Message>::RmwMsg;
        let mut request_rmw = RmwRequest::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The action server is locked by the handle. The request_id is a
            // zero-initialized rmw_request_id_t, and the request_rmw is a default-initialized
            // SendGoalService request message.
            rcl_action_take_goal_request(
                handle,
                &mut request_id,
                &mut request_rmw as *mut RmwRequest<T> as *mut _,
            )
        }
        .ok()?;

        Ok((request_rmw, request_id))
    }

    fn send_goal_response(
        &self,
        mut request_id: rmw_request_id_t,
        accepted: bool,
    ) -> Result<(), RclrsError> {
        let mut response_rmw = <T as ActionImpl>::create_goal_response(accepted, (0, 0));
        let handle = &*self.handle.lock();
        let result = unsafe {
            // SAFETY: The action server handle is locked and so synchronized with other
            // functions. The request_id and response message are uniquely owned, and so will
            // not mutate during this function call.
            // Also, when appropriate, `rcl_action_accept_new_goal()` has been called beforehand,
            // as specified in the `rcl_action` docs.
            rcl_action_send_goal_response(
                handle,
                &mut request_id,
                &mut response_rmw as *mut _ as *mut _,
            )
        }
        .ok();
        match result {
            Ok(()) => Ok(()),
            Err(RclrsError::RclError {
                code: RclReturnCode::Timeout,
                ..
            }) => {
                // TODO(nwn): Log an error and continue.
                // (See https://github.com/ros2/rclcpp/pull/2215 for reasoning.)
                Ok(())
            }
            _ => result,
        }
    }

    fn execute_goal_request(self: Arc<Self>) -> Result<(), RclrsError> {
        let (request, request_id) = match self.take_goal_request() {
            Ok(res) => res,
            Err(RclrsError::RclError {
                code: RclReturnCode::ServiceTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // action was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(err) => return Err(err),
        };

        let uuid = GoalUuid(*<T as ActionImpl>::get_goal_request_uuid(&request));

        let response: GoalResponse = {
            todo!("Optionally convert request to an idiomatic type for the user's callback.");
            todo!("Call self.goal_callback(uuid, request)");
        };

        // Don't continue if the goal was rejected by the user.
        if response == GoalResponse::Reject {
            return self.send_goal_response(request_id, false);
        }

        let goal_handle = {
            // SAFETY: No preconditions
            let mut goal_info = unsafe { rcl_action_get_zero_initialized_goal_info() };
            // Only populate the goal UUID; the timestamp will be set internally by
            // rcl_action_accept_new_goal().
            goal_info.goal_id.uuid = uuid.0;

            let server_handle = &mut *self.handle.lock();
            let goal_handle_ptr = unsafe {
                // SAFETY: The action server handle is locked and so synchronized with other
                // functions. The request_id and response message are uniquely owned, and so will
                // not mutate during this function call. The returned goal handle pointer should be
                // valid unless it is null.
                rcl_action_accept_new_goal(server_handle, &goal_info)
            };
            if goal_handle_ptr.is_null() {
                // Other than rcl_get_error_string(), there's no indication what happened.
                panic!("Failed to accept goal");
            } else {
                Arc::new(ServerGoalHandle::<T>::new(
                    goal_handle_ptr,
                    Arc::downgrade(&self),
                    todo!("Create an Arc holding the goal message"),
                    uuid,
                ))
            }
        };

        self.send_goal_response(request_id, true)?;

        self.goal_handles
            .lock()
            .unwrap()
            .insert(uuid, Arc::clone(&goal_handle));

        if response == GoalResponse::AcceptAndExecute {
            goal_handle.execute()?;
        }

        self.publish_status()?;

        // TODO: Call the user's goal_accepted callback.
        todo!("Call self.accepted_callback(goal_handle)");

        Ok(())
    }

    fn take_cancel_request(&self) -> Result<(action_msgs__srv__CancelGoal_Request, rmw_request_id_t), RclrsError> {
        let mut request_id = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        // SAFETY: No preconditions
        let mut request_rmw = unsafe { rcl_action_get_zero_initialized_cancel_request() };
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The action server is locked by the handle. The request_id is a
            // zero-initialized rmw_request_id_t, and the request_rmw is a zero-initialized
            // action_msgs__srv__CancelGoal_Request.
            rcl_action_take_cancel_request(
                handle,
                &mut request_id,
                &mut request_rmw as *mut _ as *mut _,
            )
        }
        .ok()?;

        Ok((request_rmw, request_id))
    }

    fn send_cancel_response(
        &self,
        mut request_id: rmw_request_id_t,
        response_rmw: &mut action_msgs__srv__CancelGoal_Response,
    ) -> Result<(), RclrsError> {
        let handle = &*self.handle.lock();
        let result = unsafe {
            // SAFETY: The action server handle is locked and so synchronized with other functions.
            // The request_id and response are both uniquely owned or borrowed, and so neither will
            // mutate during this function call.
            rcl_action_send_cancel_response(
                handle,
                &mut request_id,
                response_rmw as *mut _ as *mut _,
            )
        }
        .ok();
        match result {
            Ok(()) => Ok(()),
            Err(RclrsError::RclError {
                code: RclReturnCode::Timeout,
                ..
            }) => {
                // TODO(nwn): Log an error and continue.
                // (See https://github.com/ros2/rclcpp/pull/2215 for reasoning.)
                Ok(())
            }
            _ => result,
        }
    }

    fn execute_cancel_request(&self) -> Result<(), RclrsError> {
        let (request, request_id) = match self.take_cancel_request() {
            Ok(res) => res,
            Err(RclrsError::RclError {
                code: RclReturnCode::ServiceTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // action was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(err) => return Err(err),
        };

        let mut response_rmw = {
            // SAFETY: No preconditions
            let mut response_rmw = unsafe { rcl_action_get_zero_initialized_cancel_response() };
            unsafe {
                // SAFETY: The action server is locked by the handle. The request was initialized
                // by rcl_action, and the response is a zero-initialized
                // rcl_action_cancel_response_t.
                rcl_action_process_cancel_request(
                    &*self.handle.lock(),
                    &request,
                    &mut response_rmw as *mut _,
                )
            }
            .ok()?;

            DropGuard::new(response_rmw, |mut response_rmw| unsafe {
                // SAFETY: The response was initialized by rcl_action_process_cancel_request().
                // Later modifications only truncate the size of the array and shift elements,
                // without modifying the data pointer or capacity.
                rcl_action_cancel_response_fini(&mut response_rmw);
            })
        };

        let num_candidates = response_rmw.msg.goals_canceling.size;
        let mut num_accepted = 0;
        for idx in 0..response_rmw.msg.goals_canceling.size {
            let goal_info = unsafe {
                // SAFETY: The array pointed to by response_rmw.msg.goals_canceling.data is
                // guaranteed to contain at least response_rmw.msg.goals_canceling.size members.
                &*response_rmw.msg.goals_canceling.data.add(idx)
            };
            let goal_uuid = GoalUuid(goal_info.goal_id.uuid);

            let response = {
                if let Some(goal_handle) = self.goal_handles.lock().unwrap().get(&goal_uuid) {
                    let response: CancelResponse = todo!("Call self.cancel_callback(goal_handle)");
                    if response == CancelResponse::Accept {
                        // Still reject the request if the goal is no longer cancellable.
                        if goal_handle.cancel().is_ok() {
                            CancelResponse::Accept
                        } else {
                            CancelResponse::Reject
                        }
                    } else {
                        CancelResponse::Reject
                    }
                } else {
                    CancelResponse::Reject
                }
            };

            if response == CancelResponse::Accept {
                // Shift the accepted entry back to the first rejected slot, if necessary.
                if num_accepted < idx {
                    let goal_info_slot = unsafe {
                        // SAFETY: The array pointed to by response_rmw.msg.goals_canceling.data is
                        // guaranteed to contain at least response_rmw.msg.goals_canceling.size
                        // members. Since `num_accepted` is strictly less than `idx`, it is a
                        // distinct element of the array, so there is no mutable aliasing.
                        &mut *response_rmw.msg.goals_canceling.data.add(num_accepted)
                    };
                }
                num_accepted += 1;
            }
        }
        response_rmw.msg.goals_canceling.size = num_accepted;

        // If the user rejects all individual cancel requests, consider the entire request as
        // having been rejected.
        if num_accepted == 0 && num_candidates > 0 {
            // TODO(nwn): Include action_msgs__srv__CancelGoal_Response__ERROR_REJECTED in the rcl
            // bindings.
            response_rmw.msg.return_code = 1;
        }

        // If any goal states changed, publish a status update.
        if num_accepted > 0 {
            self.publish_status()?;
        }

        self.send_cancel_response(request_id, &mut response_rmw.msg)?;

        Ok(())
    }

    fn take_result_request(&self) -> Result<(<<T::GetResultService as Service>::Request as Message>::RmwMsg, rmw_request_id_t), RclrsError> {
        let mut request_id = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwRequest<T> = <<<T as ActionImpl>::GetResultService as Service>::Request as Message>::RmwMsg;
        let mut request_rmw = RmwRequest::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The action server is locked by the handle. The request_id is a
            // zero-initialized rmw_request_id_t, and the request_rmw is a default-initialized
            // GetResultService request message.
            rcl_action_take_result_request(
                handle,
                &mut request_id,
                &mut request_rmw as *mut RmwRequest<T> as *mut _,
            )
        }
        .ok()?;

        Ok((request_rmw, request_id))
    }

    fn send_result_response(
        &self,
        mut request_id: rmw_request_id_t,
        response_rmw: &mut <<<T as ActionImpl>::GetResultService as rosidl_runtime_rs::Service>::Response as Message>::RmwMsg,
    ) -> Result<(), RclrsError> {
        let handle = &*self.handle.lock();
        let result = unsafe {
            // SAFETY: The action server handle is locked and so synchronized with other functions.
            // The request_id and response are both uniquely owned or borrowed, and so neither will
            // mutate during this function call.
            rcl_action_send_result_response(
                handle,
                &mut request_id,
                response_rmw as *mut _ as *mut _,
            )
        }
        .ok();
        match result {
            Ok(()) => Ok(()),
            Err(RclrsError::RclError {
                code: RclReturnCode::Timeout,
                ..
            }) => {
                // TODO(nwn): Log an error and continue.
                // (See https://github.com/ros2/rclcpp/pull/2215 for reasoning.)
                Ok(())
            }
            _ => result,
        }
    }

    fn execute_result_request(&self) -> Result<(), RclrsError> {
        let (request, request_id) = match self.take_result_request() {
            Ok(res) => res,
            Err(RclrsError::RclError {
                code: RclReturnCode::ServiceTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // action was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(err) => return Err(err),
        };

        let uuid = GoalUuid(*<T as ActionImpl>::get_result_request_uuid(&request));

        let goal_exists = unsafe {
            // SAFETY: No preconditions
            let mut goal_info = rcl_action_get_zero_initialized_goal_info();
            goal_info.goal_id.uuid = uuid.0;

            // SAFETY: The action server is locked through the handle. The `goal_info`
            // argument points to a rcl_action_goal_info_t with the desired UUID.
            rcl_action_server_goal_exists(&*self.handle.lock(), &goal_info)
        };

        if goal_exists {
            if let Some(result) = self.goal_results.lock().unwrap().get_mut(&uuid) {
                // Respond immediately if the goal already has a response.
                self.send_result_response(request_id, result)?;
            } else {
                // Queue up the request for a response once the goal terminates.
                self.result_requests.lock().unwrap().entry(uuid).or_insert(vec![]).push(request_id);
            }
        } else {
            // TODO(nwn): Include action_msgs__msg__GoalStatus__STATUS_UNKNOWN in the rcl
            // bindings.
            let null_response = <T::Result as Message>::RmwMsg::default();
            let mut response_rmw = <T as ActionImpl>::create_result_response(0, null_response);
            self.send_result_response(request_id, &mut response_rmw)?;
        }

        Ok(())
    }

    fn execute_goal_expired(&self) -> Result<(), RclrsError> {
        // We assume here that only one goal expires at a time. If not, the only consequence is
        // that we'll call rcl_action_expire_goals() more than necessary.

        // SAFETY: No preconditions
        let mut expired_goal = unsafe { rcl_action_get_zero_initialized_goal_info() };
        let mut num_expired = 1;

        loop {
            unsafe {
                // SAFETY: The action server is locked through the handle. The `expired_goal`
                // argument points to an array of one rcl_action_goal_info_t and num_expired points
                // to a `size_t`.
                rcl_action_expire_goals(&*self.handle.lock(), &mut expired_goal, 1, &mut num_expired)
            }
            .ok()?;

            if num_expired > 0 {
                // Clean up the expired goal.
                let uuid = GoalUuid(expired_goal.goal_id.uuid);
                self.goal_results.lock().unwrap().remove(&uuid);
                self.result_requests.lock().unwrap().remove(&uuid);
                self.goal_handles.lock().unwrap().remove(&uuid);
            } else {
                break;
            }
        }

        Ok(())
    }

    // TODO(nwn): Replace `status` with a "properly typed" action_msgs::msg::GoalStatus enum.
    pub(crate) fn terminate_goal(&self, goal_id: &GoalUuid, status: i8, result: <T::Result as Message>::RmwMsg) -> Result<(), RclrsError> {
        let response_rmw = <T as ActionImpl>::create_result_response(status, result);

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

    pub(crate) fn publish_status(&self) -> Result<(), RclrsError> {
        let mut goal_statuses = DropGuard::new(
            unsafe {
                // SAFETY: No preconditions
                rcl_action_get_zero_initialized_goal_status_array()
            },
            |mut goal_statuses| unsafe {
                // SAFETY: The goal_status array is either zero-initialized and empty or populated by
                // `rcl_action_get_goal_status_array`. In either case, it can be safely finalized.
                rcl_action_goal_status_array_fini(&mut goal_statuses);
            },
        );

        unsafe {
            // SAFETY: The action server is locked through the handle and goal_statuses is
            // zero-initialized.
            rcl_action_get_goal_status_array(&*self.handle.lock(), &mut *goal_statuses)
        }
        .ok()?;

        unsafe {
            // SAFETY: The action server is locked through the handle and goal_statuses.msg is a
            // valid `action_msgs__msg__GoalStatusArray` by construction.
            rcl_action_publish_status(
                &*self.handle.lock(),
                &goal_statuses.msg as *const _ as *const std::ffi::c_void,
            )
        }
        .ok()
    }

    pub(crate) fn publish_feedback(&self, goal_id: &GoalUuid, feedback: &<T as rosidl_runtime_rs::Action>::Feedback) -> Result<(), RclrsError> {
        let feedback_rmw = <<T as rosidl_runtime_rs::Action>::Feedback as Message>::into_rmw_message(std::borrow::Cow::Borrowed(feedback));
        let mut feedback_msg = <T as rosidl_runtime_rs::ActionImpl>::create_feedback_message(&goal_id.0, feedback_rmw.into_owned());
        unsafe {
            // SAFETY: The action server is locked through the handle, meaning that no other
            // non-thread-safe functions can be called on it at the same time. The feedback_msg is
            // exclusively owned here, ensuring that it won't be modified during the call.
            // rcl_action_publish_feedback() guarantees that it won't modify `feedback_msg`.
            rcl_action_publish_feedback(
                &*self.handle.lock(),
                &mut feedback_msg as *mut _ as *mut std::ffi::c_void,
            )
        }
        .ok()
    }

    fn publish_result(&self, goal_id: &GoalUuid, mut result: <<<T as ActionImpl>::GetResultService as Service>::Response as Message>::RmwMsg) -> Result<(), RclrsError> {
        let goal_exists = unsafe {
            // SAFETY: No preconditions
            let mut goal_info = rcl_action_get_zero_initialized_goal_info();
            goal_info.goal_id.uuid = goal_id.0;

            // SAFETY: The action server is locked through the handle. The `goal_info`
            // argument points to a rcl_action_goal_info_t with the desired UUID.
            rcl_action_server_goal_exists(&*self.handle.lock(), &goal_info)
        };
        if !goal_exists {
            panic!("Cannot publish result for unknown goal")
        }

        // TODO(nwn): Fix synchronization problem between goal_results and result_requests.
        // Currently, there is a gap between the request queue being drained and the result being
        // stored for future requests. Any requests received during that gap would never receive a
        // response. Fixing this means we'll need combined locking over these two hash maps.

        // Respond to all queued requests.
        if let Some(result_requests) = self.result_requests.lock().unwrap().remove(&goal_id) {
            for mut result_request in result_requests {
                self.send_result_response(result_request, &mut result)?;
            }
        }

        self.goal_results.lock().unwrap().insert(*goal_id, result);

        Ok(())
    }
}

impl<T> ActionServerBase for ActionServerState<T>
where
    T: rosidl_runtime_rs::Action + rosidl_runtime_rs::ActionImpl,
{
    fn handle(&self) -> &ActionServerHandle {
        &self.handle
    }

    fn num_entities(&self) -> &WaitableNumEntities {
        &self.num_entities
    }

    fn execute(self: Arc<Self>, mode: ReadyMode) -> Result<(), RclrsError> {
        match mode {
            ReadyMode::GoalRequest => self.execute_goal_request(),
            ReadyMode::CancelRequest => self.execute_cancel_request(),
            ReadyMode::ResultRequest => self.execute_result_request(),
            ReadyMode::GoalExpired => self.execute_goal_expired(),
        }
    }
}

/// `ActionServerOptions` are used by [`Node::create_action_server`][1] to initialize an
/// [`ActionServer`].
///
/// [1]: crate::Node::create_action_server
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct ActionServerOptions<'a> {
    /// The name of the action implemented by this server
    pub action_name: &'a str,
    /// The quality of service profile for the goal service
    pub goal_service_qos: QoSProfile,
    /// The quality of service profile for the result service
    pub result_service_qos: QoSProfile,
    /// The quality of service profile for the cancel service
    pub cancel_service_qos: QoSProfile,
    /// The quality of service profile for the feedback topic
    pub feedback_topic_qos: QoSProfile,
    /// The quality of service profile for the status topic
    pub status_topic_qos: QoSProfile,
    // TODO(nwn): result_timeout
}

impl<'a> ActionServerOptions<'a> {
    /// Initialize a new [`ActionServerOptions`] with default settings.
    pub fn new(action_name: &'a str) -> Self {
        Self {
            action_name,
            goal_service_qos: QoSProfile::services_default(),
            result_service_qos: QoSProfile::services_default(),
            cancel_service_qos: QoSProfile::services_default(),
            feedback_topic_qos: QoSProfile::topics_default(),
            status_topic_qos: QoSProfile::action_status_default(),
        }
    }
}

impl<'a, T: Borrow<str> + ?Sized + 'a> From<&'a T> for ActionServerOptions<'a> {
    fn from(value: &'a T) -> Self {
        Self::new(value.borrow())
    }
}
