use std::any::Any;
use std::boxed::Box;
use std::ffi::{CStr, CString};
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex};

use futures::future::BoxFuture;

use super::{
    get_type_support_handle, get_type_support_library, DynamicMessage, DynamicMessageMetadata,
    MessageStructure, MessageTypeName,
};
use crate::rcl_bindings::*;
use crate::{
    MessageInfo, Node, NodeHandle, RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, RclrsError,
    SubscriptionHandle, SubscriptionOptions, ToResult, Waitable, WaitableLifecycle, WorkScope,
    Worker, WorkerCommands, ENTITY_LIFECYCLE_MUTEX,
};

pub type DynamicSubscription = Arc<DynamicSubscriptionState<Node>>;

pub type WorkerDynamicSubscription<Payload> = Arc<DynamicSubscriptionState<Worker<Payload>>>;

struct DynamicSubscriptionExecutable<Payload> {
    handle: Arc<SubscriptionHandle>,
    callback: Arc<Mutex<DynamicSubscriptionCallback<Payload>>>,
    commands: Arc<WorkerCommands>,
    metadata: Arc<DynamicMessageMetadata>,
}

// TODO(luca) consider making these enums if we want different callback types
// TODO(luca) make fields private
pub struct NodeDynamicSubscriptionCallback(
    pub Box<dyn Fn(DynamicMessage, MessageInfo) + Send + Sync>,
);
pub struct NodeAsyncDynamicSubscriptionCallback(
    pub Box<dyn FnMut(DynamicMessage, MessageInfo) -> BoxFuture<'static, ()> + Send + Sync>,
);
pub struct WorkerDynamicSubscriptionCallback<Payload>(
    pub Box<dyn FnMut(&mut Payload, DynamicMessage, MessageInfo) + Send + Sync>,
);

impl Deref for NodeDynamicSubscriptionCallback {
    type Target = Box<dyn Fn(DynamicMessage, MessageInfo) + 'static + Send + Sync>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Deref for NodeAsyncDynamicSubscriptionCallback {
    type Target =
        Box<dyn FnMut(DynamicMessage, MessageInfo) -> BoxFuture<'static, ()> + Send + Sync>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for NodeDynamicSubscriptionCallback {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl DerefMut for NodeAsyncDynamicSubscriptionCallback {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<Payload> Deref for WorkerDynamicSubscriptionCallback<Payload> {
    type Target = Box<dyn FnMut(&mut Payload, DynamicMessage, MessageInfo) + Send + Sync>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<Payload> DerefMut for WorkerDynamicSubscriptionCallback<Payload> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub enum DynamicSubscriptionCallback<Payload> {
    /// A callback with the message and the message info as arguments.
    Node(NodeAsyncDynamicSubscriptionCallback),
    /// A callback with the payload, message, and the message info as arguments.
    Worker(WorkerDynamicSubscriptionCallback<Payload>),
}

impl From<NodeDynamicSubscriptionCallback> for DynamicSubscriptionCallback<()> {
    fn from(value: NodeDynamicSubscriptionCallback) -> Self {
        let func = Arc::new(value);
        DynamicSubscriptionCallback::Node(NodeAsyncDynamicSubscriptionCallback(Box::new(
            move |message, info| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(message, info);
                })
            },
        )))
    }
}

impl From<NodeAsyncDynamicSubscriptionCallback> for DynamicSubscriptionCallback<()> {
    fn from(value: NodeAsyncDynamicSubscriptionCallback) -> Self {
        DynamicSubscriptionCallback::Node(value)
    }
}

impl<Payload> From<WorkerDynamicSubscriptionCallback<Payload>>
    for DynamicSubscriptionCallback<Payload>
{
    fn from(value: WorkerDynamicSubscriptionCallback<Payload>) -> Self {
        DynamicSubscriptionCallback::Worker(value)
    }
}

impl<Payload: 'static> DynamicSubscriptionCallback<Payload> {
    fn execute(
        &mut self,
        executable: &DynamicSubscriptionExecutable<Payload>,
        any_payload: &mut dyn Any,
        commands: &WorkerCommands,
    ) -> Result<(), RclrsError> {
        let Some(payload) = any_payload.downcast_mut::<Payload>() else {
            return Err(RclrsError::InvalidPayload {
                expected: std::any::TypeId::of::<Payload>(),
                received: (*any_payload).type_id(),
            });
        };
        match self {
            Self::Node(cb) => {
                let (msg, msg_info) = executable.take()?;
                commands.run_async(cb(msg, msg_info));
            }
            Self::Worker(cb) => {
                let (msg, msg_info) = executable.take()?;
                cb(payload, msg, msg_info);
            }
        }
        Ok(())
    }
}

impl<Payload> DynamicSubscriptionExecutable<Payload> {
    pub fn take(&self) -> Result<(DynamicMessage, MessageInfo), RclrsError> {
        let mut dynamic_message = self.metadata.create()?;
        let rmw_message = dynamic_message.storage.as_mut_ptr();
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        let rcl_subscription = &mut *self.handle.lock();
        unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                rcl_subscription,
                rmw_message as *mut _,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?
        };
        Ok((
            dynamic_message,
            MessageInfo::from_rmw_message_info(&message_info),
        ))
    }
}

impl<Payload: 'static> RclPrimitive for DynamicSubscriptionExecutable<Payload> {
    unsafe fn execute(&mut self, payload: &mut dyn Any) -> Result<(), RclrsError> {
        self.callback
            .lock()
            .unwrap()
            .execute(&self, payload, &self.commands)
    }

    fn kind(&self) -> RclPrimitiveKind {
        RclPrimitiveKind::Subscription
    }

    fn handle(&self) -> RclPrimitiveHandle {
        RclPrimitiveHandle::Subscription(self.handle.lock())
    }
}

/// Struct for receiving messages whose type is only known at runtime.
pub struct DynamicSubscriptionState<Scope>
where
    Scope: WorkScope,
{
    /// This handle is used to access the data that rcl holds for this subscription.
    handle: Arc<SubscriptionHandle>,
    /// This allows us to replace the callback in the subscription task.
    ///
    /// Holding onto this sender will keep the subscription task alive. Once
    /// this sender is dropped, the subscription task will end itself.
    callback: Arc<Mutex<DynamicSubscriptionCallback<Scope::Payload>>>,
    /// Holding onto this keeps the waiter for this subscription alive in the
    /// wait set of the executor.
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
    metadata: Arc<DynamicMessageMetadata>,
    // This is the regular type support library, not the introspection one.
    #[allow(dead_code)]
    type_support_library: Arc<libloading::Library>,
}

impl<Scope> DynamicSubscriptionState<Scope>
where
    Scope: WorkScope,
{
    /// Creates a new dynamic subscription.
    ///
    /// This is not a public function, by the same rationale as `Subscription::new()`.
    pub(crate) fn create<'a>(
        topic_type: MessageTypeName,
        options: impl Into<SubscriptionOptions<'a>>,
        callback: impl Into<DynamicSubscriptionCallback<Scope::Payload>>,
        node_handle: &Arc<NodeHandle>,
        commands: &Arc<WorkerCommands>,
    ) -> Result<Arc<Self>, RclrsError> {
        // TODO(luca) a lot of duplication with nomral, refactor
        // This loads the introspection type support library.
        let metadata = DynamicMessageMetadata::new(topic_type)?;
        let SubscriptionOptions { topic, qos } = options.into();
        // However, we also need the regular type support library –
        // the rosidl_typesupport_c one.
        let message_type = &metadata.message_type;
        let type_support_library =
            get_type_support_library(&message_type.package_name, "rosidl_typesupport_c")?;
        // SAFETY: The symbol type of the type support getter function can be trusted
        // assuming the install dir hasn't been tampered with.
        // The pointer returned by this function is kept valid by keeping the library loaded.
        let type_support_ptr = unsafe {
            get_type_support_handle(
                type_support_library.as_ref(),
                "rosidl_typesupport_c",
                message_type,
            )?
        };

        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let mut rcl_subscription_options = unsafe { rcl_subscription_get_default_options() };
        rcl_subscription_options.qos = qos.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        {
            let rcl_node = node_handle.rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            unsafe {
                // SAFETY:
                // * The rcl_subscription is zero-initialized as mandated by this function.
                // * The rcl_node is kept alive by the NodeHandle because it is a dependency of the subscription.
                // * The topic name and the options are copied by this function, so they can be dropped afterwards.
                // * The entity lifecycle mutex is locked to protect against the risk of global
                //   variables in the rmw implementation being unsafely modified during cleanup.
                rcl_subscription_init(
                    &mut rcl_subscription,
                    &*rcl_node,
                    type_support_ptr,
                    topic_c_string.as_ptr(),
                    &rcl_subscription_options,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(SubscriptionHandle {
            rcl_subscription: Mutex::new(rcl_subscription),
            node_handle: Arc::clone(node_handle),
        });

        let callback = Arc::new(Mutex::new(callback.into()));
        let metadata = Arc::new(metadata);

        let (waitable, lifecycle) = Waitable::new(
            Box::new(DynamicSubscriptionExecutable {
                handle: Arc::clone(&handle),
                callback: Arc::clone(&callback),
                commands: Arc::clone(commands),
                metadata: Arc::clone(&metadata),
            }),
            Some(Arc::clone(commands.get_guard_condition())),
        );
        commands.add_to_wait_set(waitable);

        Ok(Arc::new(Self {
            handle,
            callback,
            lifecycle,
            metadata,
            type_support_library,
        }))
    }

    /// Returns the topic name of the subscription.
    ///
    /// This returns the topic name after remapping, so it is not necessarily the
    /// topic name which was used when creating the subscription.
    pub fn topic_name(&self) -> String {
        // SAFETY: No preconditions for the function used
        // The unsafe variables get converted to safe types before being returned
        unsafe {
            let raw_topic_pointer = rcl_subscription_get_topic_name(&*self.handle.lock());
            CStr::from_ptr(raw_topic_pointer)
                .to_string_lossy()
                .into_owned()
        }
    }

    /// Returns a description of the message structure.
    pub fn structure(&self) -> &MessageStructure {
        &self.metadata.structure
    }

    /// Fetches a new message.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1].
    ///
    /// [1]: crate::RclrsError
    //
    // ```text
    // +-------------+
    // | rclrs::take |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rcl_take   |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rmw_take   |
    // +-------------+
    // ```
    pub fn take(&self) -> Result<DynamicMessage, RclrsError> {
        let mut dynamic_message = self.metadata.create()?;
        let rmw_message = dynamic_message.storage.as_mut_ptr();
        let rcl_subscription = &mut *self.handle.lock();
        unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                rcl_subscription,
                rmw_message as *mut _,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
            .ok()?
        };
        Ok(dynamic_message)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;
    use test_msgs::msg;

    fn assert_send<T: Send>() {}
    fn assert_sync<T: Sync>() {}

    #[test]
    fn dynamic_subscription_is_sync_and_send() {
        assert_send::<DynamicSubscription>();
        assert_sync::<DynamicSubscription>();
    }

    #[test]
    fn test_dynamic_subscriptions() -> Result<(), RclrsError> {
        use crate::TopicEndpointInfo;

        let namespace = "/test_dynamic_subscriptions_graph";
        let graph = construct_test_graph(namespace)?;

        let node_2_empty_subscription = graph.node2.create_dynamic_subscription::<_>(
            "test_msgs/msg/Empty".try_into().unwrap(),
            "graph_test_topic_1",
            |_, _| {},
        )?;
        let topic1 = node_2_empty_subscription.topic_name();
        let node_2_basic_types_subscription =
            graph.node2.create_dynamic_subscription::<_>(
                "test_msgs/msg/BasicTypes".try_into().unwrap(),
                "graph_test_topic_2",
                |_, _| {},
            )?;
        let topic2 = node_2_basic_types_subscription.topic_name();

        let node_1_defaults_subscription = graph.node1.create_dynamic_subscription::<_>(
            "test_msgs/msg/Defaults".try_into().unwrap(),
            "graph_test_topic_3",
            |_, _| {},
        )?;
        let topic3 = node_1_defaults_subscription.topic_name();

        std::thread::sleep(std::time::Duration::from_millis(100));

        // Test count_subscriptions()
        assert_eq!(graph.node2.count_subscriptions(&topic1)?, 1);
        assert_eq!(graph.node2.count_subscriptions(&topic2)?, 1);

        // Test get_subscription_names_and_types_by_node()
        let node_1_subscription_names_and_types = graph
            .node1
            .get_subscription_names_and_types_by_node(&graph.node1.name(), namespace)?;

        let types = node_1_subscription_names_and_types.get(&topic3).unwrap();
        assert!(types.contains(&"test_msgs/msg/Defaults".to_string()));

        let node_2_subscription_names_and_types = graph
            .node2
            .get_subscription_names_and_types_by_node(&graph.node2.name(), namespace)?;

        let types = node_2_subscription_names_and_types.get(&topic1).unwrap();
        assert!(types.contains(&"test_msgs/msg/Empty".to_string()));

        let types = node_2_subscription_names_and_types.get(&topic2).unwrap();
        assert!(types.contains(&"test_msgs/msg/BasicTypes".to_string()));

        // Test get_subscriptions_info_by_topic()
        let expected_subscriptions_info = vec![TopicEndpointInfo {
            node_name: String::from("graph_test_node_2"),
            node_namespace: String::from(namespace),
            topic_type: String::from("test_msgs/msg/Empty"),
        }];
        assert_eq!(
            graph.node1.get_subscriptions_info_by_topic(&topic1)?,
            expected_subscriptions_info
        );
        assert_eq!(
            graph.node2.get_subscriptions_info_by_topic(&topic1)?,
            expected_subscriptions_info
        );
        Ok(())
    }
}
