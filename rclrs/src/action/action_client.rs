use crate::{
    error::ToResult, rcl_bindings::*, wait::WaitableNumEntities, Node, NodeHandle, QoSProfile,
    RclrsError, ENTITY_LIFECYCLE_MUTEX,
};
use std::{
    borrow::Borrow,
    ffi::CString,
    marker::PhantomData,
    sync::{atomic::AtomicBool, Arc, Mutex, MutexGuard},
};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_client_t {}

/// Manage the lifecycle of an `rcl_action_client_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_action_client_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct ActionClientHandle {
    rcl_action_client: Mutex<rcl_action_client_t>,
    node_handle: Arc<NodeHandle>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ActionClientHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_client_t> {
        self.rcl_action_client.lock().unwrap()
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

/// Trait to be implemented by concrete ActionClient structs.
///
/// See [`ActionClient<T>`] for an example
pub trait ActionClientBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ActionClientHandle;
    /// Returns the number of underlying entities for the action client.
    fn num_entities(&self) -> &WaitableNumEntities;
    /// Tries to run the callback for the given readiness mode.
    fn execute(&self, mode: ReadyMode) -> Result<(), RclrsError>;
}

pub(crate) enum ReadyMode {
    Feedback,
    Status,
    GoalResponse,
    CancelResponse,
    ResultResponse,
}

///
/// Main class responsible for sending goals to a ROS action server.
///
/// Create a client using [`Node::create_action_client`][1].
///
/// Receiving feedback and results requires the node's executor to [spin][2].
///
/// [1]: crate::NodeState::create_action_client
/// [2]: crate::spin
pub type ActionClient<ActionT> = Arc<ActionClientState<ActionT>>;

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
pub struct ActionClientState<ActionT>
where
    ActionT: rosidl_runtime_rs::Action,
{
    _marker: PhantomData<fn() -> ActionT>,
    pub(crate) handle: Arc<ActionClientHandle>,
    num_entities: WaitableNumEntities,
    /// Ensure the parent node remains alive as long as the subscription is held.
    /// This implementation will change in the future.
    #[allow(unused)]
    node: Node,
}

impl<T> ActionClientState<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new action client.
    pub(crate) fn new<'a>(
        node: &Node,
        options: impl Into<ActionClientOptions<'a>>,
    ) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action,
    {
        let options = options.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_client = unsafe { rcl_action_get_zero_initialized_client() };
        let type_support = T::get_type_support() as *const rosidl_action_type_support_t;
        let action_name_c_string =
            CString::new(options.action_name).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: options.action_name.into(),
            })?;

        // SAFETY: No preconditions for this function.
        let action_client_options = unsafe { rcl_action_client_get_default_options() };

        {
            let mut rcl_node = node.handle.rcl_node.lock().unwrap();
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
            node_handle: Arc::clone(&node.handle),
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        let mut num_entities = WaitableNumEntities::default();
        unsafe {
            rcl_action_client_wait_set_get_num_entities(
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
            _marker: Default::default(),
            handle,
            num_entities,
            node: Arc::clone(node),
        })
    }

    fn execute_feedback(&self) -> Result<(), RclrsError> {
        todo!()
    }

    fn execute_status(&self) -> Result<(), RclrsError> {
        todo!()
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

impl<T> ActionClientBase for ActionClientState<T>
where
    T: rosidl_runtime_rs::Action,
{
    fn handle(&self) -> &ActionClientHandle {
        &self.handle
    }

    fn num_entities(&self) -> &WaitableNumEntities {
        &self.num_entities
    }

    fn execute(&self, mode: ReadyMode) -> Result<(), RclrsError> {
        match mode {
            ReadyMode::Feedback => self.execute_feedback(),
            ReadyMode::Status => self.execute_status(),
            ReadyMode::GoalResponse => self.execute_goal_response(),
            ReadyMode::CancelResponse => self.execute_cancel_response(),
            ReadyMode::ResultResponse => self.execute_result_response(),
        }
    }
}

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
