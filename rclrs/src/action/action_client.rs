use crate::{
    rcl_bindings::*,
    vendor::builtin_interfaces::msg::Time,
    DropGuard, GoalStatus, GoalUuid, ToResult, Node, NodeHandle,
    QoSProfile, RclrsError, TakeFailedAsNone, ENTITY_LIFECYCLE_MUTEX,
};
use std::{
    borrow::Borrow,
    collections::HashMap,
    ffi::CString,
    sync::{Arc, Mutex, MutexGuard},
};
use tokio::sync::{
    watch::Sender as WatchSender,
    mpsc::UnboundedSender,
    oneshot::Sender,
};
use rosidl_runtime_rs::{Action, Message, RmwFeedbackMessage};

mod feedback_client;
pub use feedback_client::*;

mod goal_client;
pub use goal_client::*;

mod status_client;
pub use status_client::*;

/// `ActionClientOptions` are used by [`Node::create_action_client`][1] to initialize an
/// [`ActionClient`].
///
/// [1]: crate::Node::create_action_client
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct ActionClientOptions<'a> {
    /// The name of the action that this client will send requests to
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
}

impl<'a> ActionClientOptions<'a> {
    /// Initialize a new [`ActionClientOptions`] with default settings.
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

impl<'a, T: Borrow<str> + ?Sized + 'a> From<&'a T> for ActionClientOptions<'a> {
    fn from(value: &'a T) -> Self {
        Self::new(value.borrow())
    }
}

/// Main class responsible for sending goals to a ROS action server.
///
/// Create a client using [`Node::create_action_client`][1].
///
/// Receiving feedback and results requires the node's executor to [spin][2].
///
/// [1]: crate::NodeState::create_action_client
/// [2]: crate::spin
pub type ActionClient<A> = Arc<ActionClientState<A>>;

/// The inner state of an [`ActionClient`].
///
/// This is public so that you can choose to create a [`Weak`][1] reference to it
/// if you want to be able to refer to an [`ActionClient`] in a non-owning way. It is
/// generally recommended to manage the `ActionClientState` inside of an [`Arc`],
/// and [`ActionClient`] is provided as a convenience alias for that.
///
/// The public API of the [`ActionClient`] type is implemented via `ActionClientState`.
///
/// [1]: std::sync::Weak
pub struct ActionClientState<A: Action> {
    board: Arc<ActionClientGoalBoard<A>>,
}

impl<A: Action> ActionClientState<A> {
    /// Creates a new action client.
    pub(crate) fn new<'a>(
        node: &Node,
        options: impl Into<ActionClientOptions<'a>>,
    ) -> Result<Self, RclrsError>
    where
        A: rosidl_runtime_rs::Action,
    {
        let options = options.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_client = unsafe { rcl_action_get_zero_initialized_client() };
        let type_support = A::get_type_support() as *const rosidl_action_type_support_t;
        let action_name_c_string =
            CString::new(options.action_name).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: options.action_name.into(),
            })?;

        // SAFETY: No preconditions for this function.
        let action_client_options = unsafe { rcl_action_client_get_default_options() };

        {
            let mut rcl_node = node.handle().rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();

            // SAFETY:
            // * The rcl_action_client was zero-initialized as expected by this function.
            // * The rcl_node is kept alive by the NodeHandle because it is a dependency of the action client.
            // * The action name and the options are copied by this function, so they can be dropped
            //   afterwards.
            // * The entity lifecycle mutex is locked to protect against the risk of global
            //   variables in the rmw implementation being unsafely modified during initialization.
            unsafe {
                rcl_action_client_init(
                    &mut rcl_action_client,
                    &mut *rcl_node,
                    type_support,
                    action_name_c_string.as_ptr(),
                    &action_client_options,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(ActionClientHandle {
            rcl_action_client: Mutex::new(rcl_action_client),
            node_handle: Arc::clone(&node.handle()),
        });

        let board = Arc::new(ActionClientGoalBoard {
            handle,
            node: Arc::clone(node),
            feedback_senders: Default::default(),
            status_senders: Default::default(),
            result_senders: Default::default(),
        });

        Ok(Self { board })
    }
}

struct ActionClientGoalBoard<A: Action> {
    feedback_senders: Mutex<HashMap<GoalUuid, Vec<UnboundedSender<A::Feedback>>>>,
    status_senders: Mutex<HashMap<GoalUuid, WatchSender<GoalStatus>>>,
    result_senders: Mutex<HashMap<rmw_request_id_t, Sender<A::Result>>>,
    handle: Arc<ActionClientHandle>,
    /// Ensure the parent node remains alive as long as the subscription is held.
    /// This implementation will change in the future.
    #[allow(unused)]
    node: Node,
}

impl<A: Action> ActionClientGoalBoard<A> {
    fn execute_feedback(&self) -> Result<(), RclrsError> {
        let Some(feedback_rmw) = self.handle.take_feedback::<A>().take_failed_as_none()? else {
            return Ok(());
        };

        let (goal_uuid, feedback) = A::split_feedback_message(feedback_rmw);
        let feedback: A::Feedback = Message::from_rmw_message(feedback);
        if let Some(senders) = self.feedback_senders.lock().unwrap().get(&GoalUuid(goal_uuid)) {
            // Avoid making any unnecessary clones
            for sender in senders.iter().take(senders.len() - 1) {
                sender.send(feedback.clone());
            }

            if let Some(last_sender) = senders.last() {
                last_sender.send(feedback);
            }
        }

        Ok(())
    }

    fn execute_status(&self) -> Result<(), RclrsError> {
        let Some(goal_statuses) = self.handle.take_status().take_failed_as_none()? else {
            return Ok(());
        };

        let all_status_senders = self.status_senders.lock().unwrap();
        for index in 0..goal_statuses.msg.status_list.size {
            let rcl_status = unsafe {
                &*goal_statuses.msg.status_list.data.add(index)
            };

            let stamp = &rcl_status.goal_info.stamp;
            let goal_id = GoalUuid(rcl_status.goal_info.goal_id.uuid);
            let status = GoalStatus {
                goal_id,
                code: rcl_status.status.into(),
                stamp: Time { sec: stamp.sec, nanosec: stamp.nanosec },
            };

            if let Some(sender) = all_status_senders.get(&goal_id) {
                sender.send_modify(|watched_status| *watched_status = status);
            }
        }

        Ok(())
    }

    fn execute_goal_response(&self) -> Result<(), RclrsError> {
        todo!()
    }

    fn execute_cancel_response(&self) -> Result<(), RclrsError> {
        todo!()
    }

    fn execute_result_response(&self) -> Result<(), RclrsError> {
        todo!()
    }
}

/// Once all of the constituent clients for the goal are dropped, this will
/// remove the [`GoalClientSender`][super::GoalClientSender] from the goal
/// board.
struct GoalClientLifecycle<A: Action> {
    kind: GoalClientKind,
    board: Arc<ActionClientGoalBoard<A>>,
}

enum GoalClientKind {
    Feedback(GoalUuid),
    Status(GoalUuid),
    Result(rmw_request_id_t),
    Cancellation,
}

impl<A: Action> Drop for GoalClientLifecycle<A> {
    fn drop(&mut self) {
        match self.kind {
            GoalClientKind::Feedback(goal_uuid) => {
                let mut all_feedback_senders = self.board.feedback_senders.lock().unwrap();

                let mut is_empty = false;
                if let Some(senders) = all_feedback_senders.get_mut(&goal_uuid) {
                    senders.retain(|sender| !sender.is_closed());
                    is_empty = senders.is_empty();
                }
                if is_empty {
                    all_feedback_senders.remove(&goal_uuid);
                }
            }
            GoalClientKind::Status(goal_uuid) => {
                let mut all_status_senders = self.board.status_senders.lock().unwrap();

                let remove = all_status_senders.get(&goal_uuid).is_some_and(|sender| sender.is_closed());
                if remove {
                    all_status_senders.remove(&goal_uuid);
                }
            }
        }
    }
}

struct ActionClientExecutable<A: Action> {
    board: Arc<ActionClientGoalBoard<A>>,
}

/// Manage the lifecycle of an `rcl_action_client_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_action_client_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct ActionClientHandle {
    rcl_action_client: Mutex<rcl_action_client_t>,
    node_handle: Arc<NodeHandle>,
}

impl ActionClientHandle {
    fn lock(&self) -> MutexGuard<rcl_action_client_t> {
        self.rcl_action_client.lock().unwrap()
    }

    fn take_feedback<A: Action>(&self) -> Result<RmwFeedbackMessage<A>, RclrsError> {
        let mut feedback_rmw = RmwFeedbackMessage::<A>::default();
        unsafe {
            let handle = self.lock();
            rcl_action_take_feedback(&*handle, &mut feedback_rmw as *mut _ as *mut _)
        }
        .ok()?;

        Ok(feedback_rmw)
    }

    fn take_status(&self) -> Result<DropGuard<rcl_action_goal_status_array_t>, RclrsError> {
        let mut goal_statuses = DropGuard::new(
            unsafe {
                // SAFETY: No preconditions
                rcl_action_get_zero_initialized_goal_status_array()
            },
            |mut goal_statuses| unsafe {
                // SAFETY: The goal_status array is either zero-initialized and empty or populated by
                // `rcl_action_get_goal_status_array`. In either case, it can be safely finalized.
                rcl_action_goal_status_array_fini(&mut goal_statuses);
            }
        );

        unsafe {
            let handle = self.lock();
            rcl_action_take_status(&*handle, &mut *goal_statuses as *mut _ as *mut _)
        }
        .ok()?;

        Ok(goal_statuses)
    }
}

impl Drop for ActionClientHandle {
    fn drop(&mut self) {
        let rcl_action_client = self.rcl_action_client.get_mut().unwrap();
        let mut rcl_node = self.node_handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_action_client_fini(rcl_action_client, &mut *rcl_node);
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_client_t {}
