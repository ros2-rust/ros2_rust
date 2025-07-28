use crate::{
    action::GoalUuid,
    error::ToResult,
    rcl_bindings::*,
    DropGuard, Node, NodeHandle, QoSProfile, RclPrimitive, RclrsError,
    RclPrimitiveHandle, RclPrimitiveKind, ReadyKind, Waitable, WaitableLifecycle, ENTITY_LIFECYCLE_MUTEX,
};
use rosidl_runtime_rs::{Action, Message, RmwGoalRequest, RmwResultRequest};
use std::{
    any::Any,
    borrow::Borrow,
    collections::HashMap,
    ffi::CString,
    panic::UnwindSafe,
    sync::{Arc, Mutex, MutexGuard, Weak},
};
use futures::future::BoxFuture;
use tokio::sync::mpsc::UnboundedSender;

mod accepted_goal;
pub use accepted_goal::*;

mod action_server_goal_handle;
use action_server_goal_handle::*;

mod cancellation_state;
use cancellation_state::*;

mod cancelling_goal;
use cancelling_goal::*;

mod executing_goal;
pub use executing_goal::*;

mod live_action_server_goal;
use live_action_server_goal::*;

mod requested_goal;
pub use requested_goal::*;

mod terminated_goal;
pub use terminated_goal::*;

/// `ActionServerOptions` are used by [`Node::create_action_server`][1] to initialize an
/// [`ActionServer`].
///
/// [1]: crate::NodeState::create_action_server
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
pub type ActionServer<A> = Arc<ActionServerState<A>>;

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
struct ActionServerState<A: Action> {
    board: Arc<ActionServerGoalBoard<A>>,

    /// Holding onto this keeps the waitable for this action server alive in the
    /// wait set of the executor.
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
}

impl<A: Action> ActionServerState<A> {
    /// Creates a new action server.
    pub(crate) fn create<'a>(
        node: &Node,
        options: impl Into<ActionServerOptions<'a>>,
        dispatch: GoalDispatch<A>,
    ) -> Result<Self, RclrsError> {
        let options = options.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_server = unsafe { rcl_action_get_zero_initialized_server() };
        let type_support = A::get_type_support() as *const rosidl_action_type_support_t;
        let action_name_c_string =
            CString::new(options.action_name).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: options.action_name.into(),
            })?;

        // SAFETY: No preconditions for this function.
        let action_server_options = unsafe { rcl_action_server_get_default_options() };

        {
            let mut rcl_node = node.handle().rcl_node.lock().unwrap();
            let clock = node.get_clock();
            let rcl_clock = clock.get_rcl_clock();
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
            node_handle: Arc::clone(&node.handle()),
            goals: Default::default(),
        });

        let board = Arc::new(ActionServerGoalBoard::new(dispatch, handle, node));

        let async_commands = node.commands().async_worker_commands();
        let (waitable, lifecycle) = Waitable::new(
            Box::new(ActionServerExecutable {
                board: Arc::clone(&board),
            }),
            Some(Arc::clone(async_commands.get_guard_condition())),
        );
        async_commands.add_to_wait_set(waitable);

        Ok(Self { board, lifecycle })
    }
}

pub struct ActionServerGoalBoard<A: Action> {
    /// These goals have a live handle held by the user. We refer to them with a
    /// Weak to prevent a circular reference. When the user drops the live handle
    /// it will automatically be moved into the dropped_goals map.
    live_goals: Mutex<HashMap<GoalUuid, Weak<LiveActionServerGoal<A>>>>,
    dispatch: Mutex<GoalDispatch<A>>,
    handle: Arc<ActionServerHandle<A>>,
    node: Node,
}

impl<A: Action> ActionServerGoalBoard<A> {
    fn new(
        dispatch: GoalDispatch<A>,
        handle: Arc<ActionServerHandle<A>>,
        node: &Node,
    ) -> Self {
        Self {
            dispatch: Mutex::new(dispatch),
            handle,
            node: Arc::clone(node),
            live_goals: Default::default(),
        }
    }

    pub fn node(&self) -> &Node {
        &self.node
    }

    fn execute_goal_request(self: &Arc<Self>) -> Result<(), RclrsError> {
        let (request, goal_request_id) = match self.handle.take_goal_request() {
            Ok(res) => res,
            Err(err) => {
                if err.is_take_failed() {
                    // Spurious wakeup – this may happen even when a waitset indicated that this
                    // action was ready, so it shouldn't be an error.
                    return Ok(());
                }

                return Err(err);
            }
        };

        let (uuid, request) = <A as Action>::split_goal_request(request);
        let requested_goal = RequestedGoal::new(
            Arc::clone(self),
            Arc::new(Message::from_rmw_message(request)),
            GoalUuid(uuid),
            goal_request_id,
        );

        match &mut *self.dispatch.lock()? {
            GoalDispatch::Callback(callback) => {
                let f = callback(requested_goal);
                let _ = self.node.commands().run(f);
            }
            GoalDispatch::Sender(sender) => {
                // A send error means the user has dropped their receiver, so
                // the requested goal will be dropped and then the goal will be
                // automatically rejected, so we don't need to do anyting with
                // SendErrors from here.
                let _ = sender.send(requested_goal);
            }
        }

        Ok(())
    }

    fn execute_cancel_request(&self) -> Result<(), RclrsError> {
        let (request, request_id) = match self.handle.take_cancel_request() {
            Ok(res) => res,
            Err(err) => {
                if err.is_take_failed() {
                    // Spurious wakeup – this may happen even when a waitset indicated that this
                    // action was ready, so it shouldn't be an error.
                    return Ok(());
                }

                return Err(err);
            }
        };

        let response_rmw = {
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

        let mut waiting_for = Vec::new();
        for idx in 0..response_rmw.msg.goals_canceling.size {
            let goal_info = unsafe {
                // SAFETY: The array pointed to by response_rmw.msg.goals_canceling.data is
                // guaranteed to contain at least response_rmw.msg.goals_canceling.size members.
                &*response_rmw.msg.goals_canceling.data.add(idx)
            };
            let goal_uuid = GoalUuid(goal_info.goal_id.uuid);
            waiting_for.push(goal_uuid);
        }

        let cancellation_request = CancellationRequest::new(
            request_id,
            waiting_for.clone(),
            Arc::clone(&self.handle),
            Arc::clone(&self.node),
        );

        let live_goals = self.live_goals.lock()?;
        for goal in waiting_for {
            if let Some(live_goal) = live_goals.get(&goal).and_then(|goal| goal.upgrade()) {
                live_goal.request_cancellation(cancellation_request.clone());
            } else {
                if let Some(handle) = self.handle.goals.lock()?.get(&goal) {
                    // If the goal is already cancelled then we will say that we
                    // accept the cancellation request. There is no need to
                    // check for the cancelling state since non-live goals must
                    // be in a terminal state.
                    if handle.is_cancelled() {
                        cancellation_request.accept(goal);
                    }
                }
            }
        }

        Ok(())
    }

    fn execute_result_request(&self) -> Result<(), RclrsError> {
        let (request, mut request_id) = match self.handle.take_result_request() {
            Ok(res) => res,
            Err(err) => {
                if err.is_take_failed() {
                    return Ok(());
                }
                return Err(err);
            },
        };

        let uuid = GoalUuid(*<A as Action>::get_result_request_uuid(&request));
        if let Some(goal) = self.handle.goals.lock()?.get(&uuid) {
            goal.add_result_request(&self.handle, request_id)?;
        } else {
            // The goal either never existed or expired, so we give back an
            // unknown response
            let result_rmw = <<A::Result as Message>::RmwMsg as Default>::default();
            let mut response_rmw = A::create_result_response(GoalStatus::Unknown as i8, result_rmw);

            let server = self.handle.lock();
            unsafe {
                // SAFETY: The action server handle is kept valid by the mutex.
                // The compiler ensures we have unique access to the result_request
                // and result_response structures.
                rcl_action_send_result_response(
                    &*server,
                    &mut request_id,
                    &mut response_rmw as *mut _ as *mut _
                )
            }
            .ok()?;
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
                self.live_goals.lock().unwrap().remove(&uuid);
                self.handle.goals.lock().unwrap().remove(&uuid);
            } else {
                break;
            }
        }

        // Clear any lingering dropped goals from the board to avoid leaks
        self.live_goals.lock().unwrap().retain(
            |_, weak| weak.upgrade().is_some()
        );

        Ok(())
    }
}

struct ActionServerExecutable<A: Action> {
    board: Arc<ActionServerGoalBoard<A>>,
}

impl<A: Action> RclPrimitive for ActionServerExecutable<A> {
    unsafe fn execute(
        &mut self,
        ready: ReadyKind,
        _payload: &mut dyn Any,
    ) -> Result<(), RclrsError> {
        let ready = ready.for_action_server()?;

        if ready.goal_request {
            self.board.execute_goal_request()?;
        }

        if ready.cancel_request {
            self.board.execute_cancel_request()?;
        }

        if ready.result_request {
            self.board.execute_result_request()?;
        }

        if ready.goal_expired {
            self.board.execute_goal_expired()?;
        }

        Ok(())
    }

    fn kind(&self) -> crate::RclPrimitiveKind {
        RclPrimitiveKind::ActionServer
    }

    fn handle(&self) -> RclPrimitiveHandle {
        RclPrimitiveHandle::ActionServer(self.board.handle.lock())
    }
}

/// Manage the lifecycle of an `rcl_action_server_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_action_server_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub(crate) struct ActionServerHandle<A: Action> {
    rcl_action_server: Mutex<rcl_action_server_t>,
    /// Ensure the node remains active while the action server is running
    node_handle: Arc<NodeHandle>,
    /// Ensure the `impl_*` of the action server goals remain valid until they
    /// have expired or until the rcl_action_server_t gets fini-ed.
    pub(super) goals: Mutex<HashMap<GoalUuid, Arc<ActionServerGoalHandle<A>>>>,
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_server_t {}

impl<A: Action> ActionServerHandle<A> {
    pub(super) fn lock(&self) -> MutexGuard<rcl_action_server_t> {
        self.rcl_action_server.lock().unwrap()
    }

    pub(super) fn publish_status(&self) -> Result<(), RclrsError> {
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

        let rcl_handle = self.lock();
        unsafe {
            // SAFETY: The action server is locked through the handle and goal_statuses is
            // zero-initialized.
            rcl_action_get_goal_status_array(&*rcl_handle, &mut *goal_statuses)
        }
        .ok()?;

        unsafe {
            // SAFETY: The action server is locked through the handle and goal_statuses.msg is a
            // valid `action_msgs__msg__GoalStatusArray` by construction.
            rcl_action_publish_status(
                &*rcl_handle,
                &goal_statuses.msg as *const _ as *const std::ffi::c_void,
            )
        }
        .ok()
    }

    fn take_goal_request(&self) -> Result<(RmwGoalRequest<A>, rmw_request_id_t), RclrsError> {
        let mut request_id = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        let mut request_rmw = RmwGoalRequest::<A>::default();
        let handle = self.lock();
        unsafe {
            // SAFETY: The action server is locked by the handle. The request_id is a
            // zero-initialized rmw_request_id_t, and the request_rmw is a default-initialized
            // SendGoalService request message.
            rcl_action_take_goal_request(
                &*handle,
                &mut request_id,
                &mut request_rmw as *mut RmwGoalRequest<A> as *mut _,
            )
        }
        .ok()?;

        Ok((request_rmw, request_id))
    }

    fn take_cancel_request(&self) -> Result<(action_msgs__srv__CancelGoal_Request, rmw_request_id_t), RclrsError> {
        let mut request_id = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        // SAFETY: No preconditions
        let mut request_rmw = unsafe { rcl_action_get_zero_initialized_cancel_request() };
        let handle = self.lock();
        unsafe {
            // SAFETY: The action server is locked by the handle. The request_id is a
            // zero-initialized rmw_request_id_t, and the request_rmw is a zero-initialized
            // action_msgs__srv__CancelGoal_Request.
            rcl_action_take_cancel_request(
                &*handle,
                &mut request_id,
                &mut request_rmw as *mut _ as *mut _,
            )
        }
        .ok()?;

        Ok((request_rmw, request_id))
    }

    fn take_result_request(&self) -> Result<(RmwResultRequest<A>, rmw_request_id_t), RclrsError> {
        let mut request_id = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };

        let mut request_rmw = RmwResultRequest::<A>::default();
        let handle = self.lock();
        unsafe {
            // SAFETY: The action server is locked by the handle. The request_id is a
            // zero-initialized rmw_request_id_t, and the request_rmw is a default-initialized
            // GetResultService request message.
            rcl_action_take_result_request(
                &*handle,
                &mut request_id,
                &mut request_rmw as *mut RmwResultRequest<A> as *mut _,
            )
        }
        .ok()?;

        Ok((request_rmw, request_id))
    }
}

enum GoalDispatch<A: Action> {
    Callback(Box<dyn FnMut(RequestedGoal<A>) -> BoxFuture<'static, TerminatedGoal> + Send + Sync + UnwindSafe>),
    Sender(UnboundedSender<RequestedGoal<A>>),
}

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
