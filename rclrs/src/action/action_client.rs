use super::empty_goal_status_array;
use crate::{
    log_warn,
    rcl_bindings::*,
    vendor::{action_msgs::srv::CancelGoal_Response, builtin_interfaces::msg::Time},
    CancelResponse, CancelResponseCode, DropGuard, GoalStatus, GoalStatusCode, GoalUuid,
    MultiCancelResponse, Node, NodeHandle, QoSProfile, RclPrimitive, RclPrimitiveHandle,
    RclPrimitiveKind, RclrsError, ReadyKind, TakeFailedAsNone, ToResult, Waitable,
    WaitableLifecycle, ENTITY_LIFECYCLE_MUTEX,
};
use rosidl_runtime_rs::{Action, Message, RmwFeedbackMessage, RmwGoalResponse, RmwResultResponse};
use std::{
    any::Any,
    borrow::{Borrow, Cow},
    collections::HashMap,
    ffi::CString,
    sync::{Arc, Mutex, MutexGuard, Weak},
};
use tokio::sync::{
    mpsc::{unbounded_channel, UnboundedSender},
    oneshot::{channel as oneshot_channel, Sender},
    watch::Sender as WatchSender,
};

mod cancellation_client;
pub use cancellation_client::*;

mod feedback_client;
pub use feedback_client::*;

mod goal_client;
pub use goal_client::*;

mod status_client;
pub use status_client::*;

mod result_client;
pub use result_client::*;

mod requested_goal_client;
pub use requested_goal_client::*;

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

/// Trait to implicitly convert a compatible object into [`ActionClientOptions`].
pub trait IntoActionClientOptions<'a>: Sized {
    /// Change this into an [`ActionClientOptions`].
    fn into_action_client_options(self) -> ActionClientOptions<'a>;

    /// Set the quality of service profile for the goal service
    fn goal_service_qos(self, profile: QoSProfile) -> ActionClientOptions<'a> {
        let mut options = self.into_action_client_options();
        options.goal_service_qos = profile;
        options
    }

    /// Set the quality of service profile for the result service
    fn result_service_qos(self, profile: QoSProfile) -> ActionClientOptions<'a> {
        let mut options = self.into_action_client_options();
        options.result_service_qos = profile;
        options
    }

    /// Set the quality of service profile for the cancel service
    fn cancel_service_qos(self, profile: QoSProfile) -> ActionClientOptions<'a> {
        let mut options = self.into_action_client_options();
        options.cancel_service_qos = profile;
        options
    }

    /// Set the quality of service profile for the feedback topic
    fn feedback_topic_qos(self, profile: QoSProfile) -> ActionClientOptions<'a> {
        let mut options = self.into_action_client_options();
        options.feedback_topic_qos = profile;
        options
    }

    /// Set the quality of service profile for the status topic
    fn status_topic_qos(self, profile: QoSProfile) -> ActionClientOptions<'a> {
        let mut options = self.into_action_client_options();
        options.status_topic_qos = profile;
        options
    }
}

impl<'a, T: Borrow<str> + ?Sized + 'a> IntoActionClientOptions<'a> for &'a T {
    fn into_action_client_options(self) -> ActionClientOptions<'a> {
        ActionClientOptions::new(self.borrow())
    }
}

impl<'a> IntoActionClientOptions<'a> for ActionClientOptions<'a> {
    fn into_action_client_options(self) -> ActionClientOptions<'a> {
        self
    }
}

impl<'a> From<&'_ ActionClientOptions<'a>> for rcl_action_client_options_t {
    fn from(value: &'_ ActionClientOptions<'a>) -> Self {
        rcl_action_client_options_s {
            goal_service_qos: value.goal_service_qos.into(),
            result_service_qos: value.result_service_qos.into(),
            cancel_service_qos: value.cancel_service_qos.into(),
            feedback_topic_qos: value.feedback_topic_qos.into(),
            status_topic_qos: value.status_topic_qos.into(),
            // SAFETY: No preconditions for this function
            allocator: unsafe { rcutils_get_default_allocator() },
        }
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
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
}

impl<A: Action> ActionClientState<A> {
    /// Request the action server to execute a goal. You will receive a
    /// [`RequestedGoalClient`] which you can use to wait for the reply from the
    /// action server that indicates whether the goal was accepted.
    ///
    /// If the goal is accepted, the [`RequestedGoalClient`] will yield a
    /// [`GoalClient`]. If the goal was rejected by the action server then it will
    /// yield a [`None`]. The easiest way to get the response is to use `.await`
    /// in an async function.
    ///
    /// In the unlikely event of an error at the rcl layer, this will panic.
    /// Use [`Self::try_request_goal`] to handle errors without panicking.
    pub fn request_goal(self: &Arc<Self>, goal: A::Goal) -> RequestedGoalClient<A> {
        self.try_request_goal(goal).unwrap()
    }

    /// A version of [`Self::request_goal`] which allows you to handle any errors
    /// that may happen at the rcl layer.
    pub fn try_request_goal(
        self: &Arc<Self>,
        goal: A::Goal,
    ) -> Result<RequestedGoalClient<A>, RclrsError> {
        self.board.request_goal(self, goal)
    }

    /// Get a client to receive feedback for a specific goal.
    pub fn receive_feedback(self: &Arc<Self>, goal_id: GoalUuid) -> FeedbackClient<A> {
        self.board.create_feedback_client(self, goal_id)
    }

    /// Get a client to receive all status updates for a specific goal. If you
    /// only care about the latest status update, consider using [`Self::watch_status`]
    /// instead.
    pub fn receive_status(self: &Arc<Self>, goal_id: GoalUuid) -> StatusClient<A> {
        self.board.create_status_client(self, goal_id)
    }

    /// Get a client to watch the status of a specific goal.
    pub fn watch_status(self: &Arc<Self>, goal_id: GoalUuid) -> StatusWatcher<A> {
        self.board.create_status_watcher(self, goal_id)
    }

    /// Ask to receive the result of a goal. You can `.await` the [`ResultClient`]
    /// to get the result asynchronously.
    ///
    /// In the unlikely event of an error at the rcl layer, this will panic.
    /// Use [`Self::try_receive_result`] to catch errors without panicking.
    pub fn receive_result(self: &Arc<Self>, goal_id: GoalUuid) -> ResultClient<A> {
        self.try_receive_result(goal_id).unwrap()
    }

    /// Try to receive the result of a goal. If an rcl error happens, you can
    /// handle it.
    pub fn try_receive_result(
        self: &Arc<Self>,
        goal_id: GoalUuid,
    ) -> Result<ResultClient<A>, RclrsError> {
        self.board.request_result(Arc::clone(self), goal_id)
    }

    /// Ask the action server to cancel a single goal.
    ///
    /// In the unlikely event of an error at the rcl layer, this will panic.
    /// Use [`Self::try_cancel_goal`] to handle errors without panicking.
    pub fn cancel_goal(self: &Arc<Self>, goal_id: GoalUuid) -> CancelResponseClient<A> {
        self.try_cancel_goal(goal_id).unwrap()
    }

    /// Try to ask the action server to cancel a single goal. If an rcl error
    /// happens, you can handle it.
    pub fn try_cancel_goal(
        self: &Arc<Self>,
        goal_id: GoalUuid,
    ) -> Result<CancelResponseClient<A>, RclrsError> {
        self.board.request_single_cancel(self, goal_id)
    }

    /// Ask the action server to cancel all of its goals.
    ///
    /// In the unlikely event of an error at the rcl layer, this will panic.
    /// Use [`Self::try_cancel_all_goals`] to catch errors without panicking.
    pub fn cancel_all_goals(self: &Arc<Self>) -> MultiCancelResponseClient<A> {
        self.try_cancel_all_goals().unwrap()
    }

    /// Try to ask the action server to cancel all of its goals. If an rcl error
    /// happens, you can handle it.
    pub fn try_cancel_all_goals(
        self: &Arc<Self>,
    ) -> Result<MultiCancelResponseClient<A>, RclrsError> {
        self.board.request_multi_cancel(self, None)
    }

    /// Ask the action server to cancel all of its goals that started before a
    /// specified time stamp.
    ///
    /// <div class="warning">Make sure the time stamp is based on the time according to the action server.</div>
    ///
    /// In the unlikely event of an error at the rcl layer, this will panic.
    /// Use [`Self::try_cancel_goals_prior_to`] to catch errors without panicking.
    pub fn cancel_goals_prior_to(self: &Arc<Self>, time: Time) -> MultiCancelResponseClient<A> {
        self.try_cancel_goals_prior_to(time).unwrap()
    }

    /// Try to ask the action server to cancel all of its goals that started before
    /// a specified time stamp. If an rcl error happens, you can handle it.
    ///
    /// <div class="warning">Make sure the time stamp is based on the time according to the action server.</div>
    pub fn try_cancel_goals_prior_to(
        self: &Arc<Self>,
        time: Time,
    ) -> Result<MultiCancelResponseClient<A>, RclrsError> {
        self.board.request_multi_cancel(self, Some(time))
    }

    /// Creates a new action client.
    pub(crate) fn create<'a>(
        node: &Node,
        options: impl IntoActionClientOptions<'a>,
    ) -> Result<Arc<Self>, RclrsError> {
        let options = options.into_action_client_options();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_client = unsafe { rcl_action_get_zero_initialized_client() };
        let type_support = A::get_type_support() as *const rosidl_action_type_support_t;
        let action_name_c_string =
            CString::new(options.action_name).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: options.action_name.into(),
            })?;

        // SAFETY: No preconditions for this function.
        let action_client_options = (&options).into();

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
            pending_goal_clients: Default::default(),
            feedback_senders: Default::default(),
            status_senders: Default::default(),
            status_posters: Default::default(),
            cancel_response_senders: Default::default(),
            result_senders: Default::default(),
            client: Default::default(),
        });

        let async_commands = node.commands().async_worker_commands();
        let (waitable, lifecycle) = Waitable::new(
            Box::new(ActionClientExecutable {
                board: Arc::clone(&board),
            }),
            Some(Arc::clone(async_commands.get_guard_condition())),
        );
        async_commands.add_to_wait_set(waitable);

        let client = Arc::new(Self { board, lifecycle });
        *client.board.client.lock().unwrap() = Arc::downgrade(&client);
        Ok(client)
    }
}

struct ActionClientGoalBoard<A: Action> {
    pending_goal_clients: Mutex<HashMap<i64, PendingGoalClient<A>>>,
    feedback_senders: Mutex<HashMap<GoalUuid, Vec<UnboundedSender<A::Feedback>>>>,
    status_senders: Mutex<HashMap<GoalUuid, Vec<UnboundedSender<GoalStatus>>>>,
    status_posters: Mutex<HashMap<GoalUuid, WatchSender<GoalStatus>>>,
    cancel_response_senders: Mutex<HashMap<i64, CancelResponseSender>>,
    result_senders: Mutex<HashMap<i64, Sender<(GoalStatusCode, A::Result)>>>,
    handle: Arc<ActionClientHandle>,
    client: Mutex<Weak<ActionClientState<A>>>,
    /// Ensure the parent node remains alive as long as the subscription is held.
    /// This implementation will change in the future.
    #[allow(unused)]
    node: Node,
}

enum CancelResponseSender {
    /// Used when only a single goal is being cancelled
    SingleGoalCancel(Sender<CancelResponse>),
    /// Used when multiple goals are being cancelled
    MultiGoalCancel(Sender<MultiCancelResponse>),
}

impl<A: Action> ActionClientGoalBoard<A> {
    fn execute_feedback(&self) -> Result<(), RclrsError> {
        let Some(feedback_rmw) = self.handle.take_feedback::<A>().take_failed_as_none()? else {
            return Ok(());
        };

        let (goal_uuid, feedback) = A::split_feedback_message(feedback_rmw);
        let feedback: A::Feedback = Message::from_rmw_message(feedback);
        if let Some(senders) = self
            .feedback_senders
            .lock()
            .unwrap()
            .get(&GoalUuid(goal_uuid))
        {
            // Avoid making any unnecessary clones
            for sender in senders.iter().take(senders.len() - 1) {
                let _ = sender.send(feedback.clone());
            }

            if let Some(last_sender) = senders.last() {
                let _ = last_sender.send(feedback);
            }
        }

        Ok(())
    }

    fn execute_status(&self) -> Result<(), RclrsError> {
        let Some(goal_statuses) = self.handle.take_status().take_failed_as_none()? else {
            return Ok(());
        };

        let all_status_senders = self.status_senders.lock().unwrap();
        let all_status_watchers = self.status_posters.lock().unwrap();
        for index in 0..goal_statuses.msg.status_list.size {
            let rcl_status = unsafe { &*goal_statuses.msg.status_list.data.add(index) };

            let stamp = &rcl_status.goal_info.stamp;
            let goal_id = GoalUuid(rcl_status.goal_info.goal_id.uuid);
            let status = GoalStatus {
                goal_id,
                code: rcl_status.status.into(),
                stamp: Time {
                    sec: stamp.sec,
                    nanosec: stamp.nanosec,
                },
            };

            if let Some(senders) = all_status_senders.get(&goal_id) {
                for sender in senders {
                    let _ = sender.send(status.clone());
                }
            }

            if let Some(watcher) = all_status_watchers.get(&goal_id) {
                watcher.send_modify(|watched_status| *watched_status = status);
            }
        }

        Ok(())
    }

    fn execute_goal_response(&self) -> Result<(), RclrsError> {
        let Some(client) = self.client.lock().unwrap().upgrade() else {
            // The action client has already expired which means the user is not
            // waiting for this response any longer. We can just discard it.
            return Ok(());
        };

        let Some((response_rmw, header)) = self
            .handle
            .take_goal_response::<A>()
            .take_failed_as_none()?
        else {
            return Ok(());
        };

        let accepted = A::get_goal_response_accepted(&response_rmw);
        let stamp = A::get_goal_response_stamp(&response_rmw);

        let Some(pending) = self
            .pending_goal_clients
            .lock()
            .unwrap()
            .remove(&header.sequence_number)
        else {
            // This suggests that the RequestedGoalClient was dropped before the result arrived.
            return Ok(());
        };

        if accepted {
            let _ = pending.sender.send(Some(GoalClient {
                feedback: pending.feedback,
                status: pending.status,
                result: self.request_result(Arc::clone(&client), pending.goal_id)?,
                cancellation: CancellationClient::new(client, pending.goal_id),
                stamp: Time {
                    sec: stamp.0,
                    nanosec: stamp.1,
                },
            }));
        } else {
            let _ = pending.sender.send(None);
        }

        Ok(())
    }

    fn execute_cancel_response(&self) -> Result<(), RclrsError> {
        let Some((result, header)) = self.handle.take_cancel_response().take_failed_as_none()?
        else {
            return Ok(());
        };

        let seq = header.sequence_number;
        let Some(sender) = self.cancel_response_senders.lock().unwrap().remove(&seq) else {
            // If the sender doesn't exist, most likely it means the CancelResponseClient
            // has been dropped.
            return Ok(());
        };

        let code: CancelResponseCode = result.return_code.into();
        match sender {
            CancelResponseSender::SingleGoalCancel(sender) => {
                if result.goals_canceling.len() > 1 {
                    log_warn!(
                        "action_client.execute_cancel_response",
                        "Multiple goals were cancelled when only one was expected. \
                        This may indicate a client library implementation bug.",
                    );
                }

                // Use the first goal's info to get the time stamp. We should
                // only be expecting one goal to be cancelled anyway.
                let stamp = result.goals_canceling.first().map(|g| g.stamp.clone());
                let _ = sender.send(CancelResponse { code, stamp });
            }
            CancelResponseSender::MultiGoalCancel(sender) => {
                let mut stamps = HashMap::default();
                for info in result.goals_canceling {
                    stamps.insert(GoalUuid(info.goal_id.uuid), info.stamp);
                }
                let _ = sender.send(MultiCancelResponse { code, stamps });
            }
        }

        Ok(())
    }

    fn execute_result_response(&self) -> Result<(), RclrsError> {
        let Some((result_rmw, header)) = self
            .handle
            .take_result_response::<A>()
            .take_failed_as_none()?
        else {
            return Ok(());
        };

        let (status, result) = A::split_result_response(result_rmw);
        let status_code = status.into();

        let seq = header.sequence_number;
        let Some(sender) = self.result_senders.lock().unwrap().remove(&seq) else {
            // If the sender doesn't exist, most likely it means the ResultClient
            // has been dropped.
            return Ok(());
        };

        let result = Message::from_rmw_message(result);
        let _ = sender.send((status_code, result));
        Ok(())
    }

    fn request_goal(
        &self,
        client: &ActionClient<A>,
        goal: A::Goal,
    ) -> Result<RequestedGoalClient<A>, RclrsError> {
        let goal_id: GoalUuid = uuid::Uuid::new_v4().as_bytes().into();
        let goal_rmw = <A::Goal as Message>::into_rmw_message(Cow::Owned(goal)).into_owned();
        let request = A::create_goal_request(&*goal_id, goal_rmw);

        let mut seq: i64 = 0;
        unsafe {
            let handle = self.handle.lock();
            rcl_action_send_goal_request(&*handle, &request as *const _ as *const _, &mut seq)
        }
        .ok()?;

        let (sender, receiver) = oneshot_channel();
        let feedback = self.create_feedback_client(client, goal_id);
        let status = self.create_status_watcher(client, goal_id);

        self.pending_goal_clients.lock().unwrap().insert(
            seq,
            PendingGoalClient {
                goal_id,
                feedback,
                status,
                sender,
            },
        );

        Ok(RequestedGoalClient::new(
            goal_id,
            receiver,
            GoalClientLifecycle {
                kind: GoalClientKind::Request(seq),
                client: Arc::clone(client),
            },
        ))
    }

    fn create_feedback_client(
        &self,
        client: &ActionClient<A>,
        goal_id: GoalUuid,
    ) -> FeedbackClient<A> {
        let (sender, receiver) = unbounded_channel();
        self.feedback_senders
            .lock()
            .unwrap()
            .entry(goal_id)
            .or_default()
            .push(sender);
        FeedbackClient::new(
            receiver,
            goal_id,
            GoalClientLifecycle {
                kind: GoalClientKind::Feedback(goal_id),
                client: Arc::clone(client),
            },
        )
    }

    fn create_status_client(&self, client: &ActionClient<A>, goal_id: GoalUuid) -> StatusClient<A> {
        let (sender, receiver) = unbounded_channel();
        self.status_senders
            .lock()
            .unwrap()
            .entry(goal_id)
            .or_default()
            .push(sender);
        StatusClient::new(
            receiver,
            goal_id,
            GoalClientLifecycle {
                kind: GoalClientKind::Status(goal_id),
                client: Arc::clone(client),
            },
        )
    }

    fn create_status_watcher(
        &self,
        client: &ActionClient<A>,
        goal_id: GoalUuid,
    ) -> StatusWatcher<A> {
        let watcher = self
            .status_posters
            .lock()
            .unwrap()
            .entry(goal_id)
            .or_insert_with(|| {
                WatchSender::new(GoalStatus {
                    code: GoalStatusCode::Unknown,
                    goal_id,
                    stamp: Time { sec: 0, nanosec: 0 },
                })
            })
            .subscribe();

        StatusWatcher::new(
            watcher,
            Arc::new(GoalClientLifecycle {
                kind: GoalClientKind::Status(goal_id),
                client: Arc::clone(client),
            }),
        )
    }

    fn request_single_cancel(
        &self,
        client: &ActionClient<A>,
        goal_id: GoalUuid,
    ) -> Result<CancelResponseClient<A>, RclrsError> {
        let seq = self
            .handle
            .send_cancel_goal(goal_id, Time { sec: 0, nanosec: 0 })?;
        let (sender, receiver) = oneshot_channel();
        self.cancel_response_senders
            .lock()
            .unwrap()
            .insert(seq, CancelResponseSender::SingleGoalCancel(sender));
        Ok(CancelResponseClient::new(
            receiver,
            GoalClientLifecycle {
                kind: GoalClientKind::CancelResponse(seq),
                client: Arc::clone(client),
            },
        ))
    }

    fn request_multi_cancel(
        &self,
        client: &ActionClient<A>,
        stamp: Option<Time>,
    ) -> Result<MultiCancelResponseClient<A>, RclrsError> {
        let goal_id = GoalUuid::zero();
        let stamp = stamp.unwrap_or(Time { sec: 0, nanosec: 0 });
        let seq = self.handle.send_cancel_goal(goal_id, stamp)?;
        let (sender, receiver) = oneshot_channel();
        self.cancel_response_senders
            .lock()
            .unwrap()
            .insert(seq, CancelResponseSender::MultiGoalCancel(sender));
        Ok(MultiCancelResponseClient::new(
            receiver,
            GoalClientLifecycle {
                kind: GoalClientKind::CancelResponse(seq),
                client: Arc::clone(client),
            },
        ))
    }

    fn request_result(
        &self,
        client: ActionClient<A>,
        goal_id: GoalUuid,
    ) -> Result<ResultClient<A>, RclrsError> {
        let request_rmw = A::create_result_request(&*goal_id);
        let mut seq: i64 = 0;
        unsafe {
            let handle = self.handle.lock();
            rcl_action_send_result_request(&*handle, &request_rmw as *const _ as *const _, &mut seq)
        }
        .ok()?;

        let (sender, receiver) = oneshot_channel();
        self.result_senders.lock().unwrap().insert(seq, sender);
        let lifecycle = GoalClientLifecycle {
            client,
            kind: GoalClientKind::Result(seq),
        };
        Ok(ResultClient::new(receiver, lifecycle))
    }
}

/// This struct represents a goal client that has not been accepted yet. We
/// create the feedback and status channels before sending the request just in
/// case the action server sends updates for the goal before we process that the
/// goal was accepted. This ensures that no information can be lost by accident.
struct PendingGoalClient<A: Action> {
    goal_id: GoalUuid,
    feedback: FeedbackClient<A>,
    status: StatusWatcher<A>,
    sender: Sender<Option<GoalClient<A>>>,
}

/// Once all of the constituent clients for the goal are dropped, this will
/// remove the [`GoalClientSender`][super::GoalClientSender] from the goal
/// board.
///
/// This also ensures that the action client remains alive for as long as any
/// of its constituent clients are using it.
struct GoalClientLifecycle<A: Action> {
    kind: GoalClientKind,
    client: ActionClient<A>,
}

enum GoalClientKind {
    Request(i64),
    Feedback(GoalUuid),
    Status(GoalUuid),
    Result(i64),
    CancelResponse(i64),
}

impl<A: Action> Drop for GoalClientLifecycle<A> {
    fn drop(&mut self) {
        match self.kind {
            GoalClientKind::Request(seq) => {
                let mut all_pending_goal_clients =
                    self.client.board.pending_goal_clients.lock().unwrap();
                all_pending_goal_clients.remove(&seq);
            }
            GoalClientKind::Feedback(goal_uuid) => {
                let mut all_feedback_senders = self.client.board.feedback_senders.lock().unwrap();

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
                {
                    let mut all_status_senders = self.client.board.status_senders.lock().unwrap();
                    let mut is_empty = false;
                    if let Some(senders) = all_status_senders.get_mut(&goal_uuid) {
                        senders.retain(|sender| !sender.is_closed());
                        is_empty = senders.is_empty();
                    }
                    if is_empty {
                        all_status_senders.remove(&goal_uuid);
                    }
                }

                {
                    let mut all_status_posters = self.client.board.status_posters.lock().unwrap();
                    let remove = all_status_posters
                        .get(&goal_uuid)
                        .is_some_and(|sender| sender.is_closed());
                    if remove {
                        all_status_posters.remove(&goal_uuid);
                    }
                }
            }
            GoalClientKind::CancelResponse(seq) => {
                let mut all_cancel_response_senders =
                    self.client.board.cancel_response_senders.lock().unwrap();
                all_cancel_response_senders.remove(&seq);
            }
            GoalClientKind::Result(seq) => {
                let mut all_result_senders = self.client.board.result_senders.lock().unwrap();
                all_result_senders.remove(&seq);
            }
        }
    }
}

struct ActionClientExecutable<A: Action> {
    board: Arc<ActionClientGoalBoard<A>>,
}

impl<A: Action> RclPrimitive for ActionClientExecutable<A> {
    unsafe fn execute(&mut self, ready: ReadyKind, _: &mut dyn Any) -> Result<(), RclrsError> {
        let ready = ready.for_action_client()?;

        if ready.goal_response {
            self.board.execute_goal_response()?;
        }

        if ready.feedback {
            self.board.execute_feedback()?;
        }

        if ready.status {
            self.board.execute_status()?;
        }

        if ready.cancel_response {
            self.board.execute_cancel_response()?;
        }

        if ready.result_response {
            self.board.execute_result_response()?;
        }

        Ok(())
    }

    fn kind(&self) -> RclPrimitiveKind {
        RclPrimitiveKind::ActionClient
    }

    fn handle(&self) -> crate::RclPrimitiveHandle {
        RclPrimitiveHandle::ActionClient(self.board.handle.lock())
    }
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

    fn take_goal_response<A: Action>(
        &self,
    ) -> Result<(RmwGoalResponse<A>, rmw_request_id_t), RclrsError> {
        let mut response_rmw = RmwGoalResponse::<A>::default();
        let mut response_header = rmw_request_id_t {
            writer_guid: [0; RCL_ACTION_UUID_SIZE],
            sequence_number: 0,
        };

        unsafe {
            let handle = self.lock();
            rcl_action_take_goal_response(
                &*handle,
                &mut response_header,
                &mut response_rmw as *mut _ as *mut _,
            )
        }
        .ok()?;

        Ok((response_rmw, response_header))
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
        let mut goal_statuses = empty_goal_status_array();
        unsafe {
            let handle = self.lock();
            rcl_action_take_status(&*handle, &mut *goal_statuses as *mut _ as *mut _)
        }
        .ok()?;

        Ok(goal_statuses)
    }

    fn take_result_response<A: Action>(
        &self,
    ) -> Result<(RmwResultResponse<A>, rmw_request_id_t), RclrsError> {
        let mut result_rmw = RmwResultResponse::<A>::default();
        let mut response_header = rmw_request_id_t {
            writer_guid: [0; RCL_ACTION_UUID_SIZE],
            sequence_number: 0,
        };

        unsafe {
            let handle = self.lock();
            rcl_action_take_result_response(
                &*handle,
                &mut response_header,
                &mut result_rmw as *mut _ as *mut _,
            )
        }
        .ok()?;

        Ok((result_rmw, response_header))
    }

    fn send_cancel_goal(&self, goal_id: GoalUuid, stamp: Time) -> Result<i64, RclrsError> {
        let cancel_request_rmw = action_msgs__srv__CancelGoal_Request {
            goal_info: action_msgs__msg__GoalInfo {
                goal_id: unique_identifier_msgs__msg__UUID { uuid: *goal_id },
                stamp: builtin_interfaces__msg__Time {
                    sec: stamp.sec,
                    nanosec: stamp.nanosec,
                },
            },
        };
        let mut seq: i64 = 0;

        unsafe {
            let handle = self.lock();
            rcl_action_send_cancel_request(
                &*handle,
                &cancel_request_rmw as *const _ as *const _,
                &mut seq,
            )
        }
        .ok()?;

        Ok(seq)
    }

    fn take_cancel_response(&self) -> Result<(CancelGoal_Response, rmw_request_id_t), RclrsError> {
        let mut result_rmw = <CancelGoal_Response as Message>::RmwMsg::default();
        let mut header = rmw_request_id_t {
            writer_guid: [0; RCL_ACTION_UUID_SIZE],
            sequence_number: 0,
        };

        unsafe {
            let handle = self.lock();
            rcl_action_take_cancel_response(
                &*handle,
                &mut header,
                &mut result_rmw as *mut _ as *mut _,
            )
        }
        .ok()?;

        let result = Message::from_rmw_message(result_rmw);
        Ok((result, header))
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
