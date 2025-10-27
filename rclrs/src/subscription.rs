use std::{
    any::Any,
    ffi::{CStr, CString},
    sync::{Arc, Mutex, MutexGuard},
};

use rosidl_runtime_rs::{Message, RmwMessage};

use crate::{
    error::ToResult, qos::QoSProfile, rcl_bindings::*, IntoPrimitiveOptions, Node, NodeHandle,
    RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, RclrsError, ReadyKind, Waitable,
    WaitableLifecycle, WorkScope, Worker, WorkerCommands, ENTITY_LIFECYCLE_MUTEX,
};

mod any_subscription_callback;
pub use any_subscription_callback::*;

mod node_subscription_callback;
pub use node_subscription_callback::*;

mod into_async_subscription_callback;
pub use into_async_subscription_callback::*;

mod into_node_subscription_callback;
pub use into_node_subscription_callback::*;

mod into_worker_subscription_callback;
pub use into_worker_subscription_callback::*;

mod message_info;
pub use message_info::*;

mod readonly_loaned_message;
pub use readonly_loaned_message::*;

mod worker_subscription_callback;
pub use worker_subscription_callback::*;

/// Struct for receiving messages of type `T`.
///
/// Create a subscription using [`NodeState::create_subscription()`][1]
/// or [`NodeState::create_async_subscription`][2].
///
/// There can be multiple subscriptions for the same topic, in different nodes or the same node.
/// A clone of a `Subscription` will refer to the same subscription instance as the original.
/// The underlying instance is tied to [`SubscriptionState`] which implements the [`Subscription`] API.
///
/// Receiving messages requires the node's executor to [spin][3].
///
/// When a subscription is created, it may take some time to get "matched" with a corresponding
/// publisher.
///
/// [1]: crate::NodeState::create_subscription
/// [2]: crate::NodeState::create_async_subscription
/// [3]: crate::Executor::spin
pub type Subscription<T> = Arc<SubscriptionState<T, Node>>;

/// A [`Subscription`] that runs on a [`Worker`].
///
/// Create a worker subscription using [`WorkerState::create_subscription`][1].
///
/// [1]: crate::WorkerState::create_subscription
pub type WorkerSubscription<T, Payload> = Arc<SubscriptionState<T, Worker<Payload>>>;

/// The inner state of a [`Subscription`].
///
/// This is public so that you can choose to create a [`Weak`][1] reference to it
/// if you want to be able to refer to a [`Subscription`] in a non-owning way. It is
/// generally recommended to manage the `SubscriptionState` inside of an [`Arc`],
/// and [`Subscription`] is provided as a convenience alias for that.
///
/// The public API of the [`Subscription`] type is implemented via `SubscriptionState`.
///
/// [1]: std::sync::Weak
pub struct SubscriptionState<T, Scope>
where
    T: Message,
    Scope: WorkScope,
{
    /// This handle is used to access the data that rcl holds for this subscription.
    handle: Arc<SubscriptionHandle>,
    /// This allows us to replace the callback in the subscription task.
    ///
    /// Holding onto this sender will keep the subscription task alive. Once
    /// this sender is dropped, the subscription task will end itself.
    callback: Arc<Mutex<AnySubscriptionCallback<T, Scope::Payload>>>,
    /// Holding onto this keeps the waiter for this subscription alive in the
    /// wait set of the executor.
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
}

impl<T, Scope> SubscriptionState<T, Scope>
where
    T: Message,
    Scope: WorkScope,
{
    /// Returns the topic name of the subscription.
    ///
    /// This returns the topic name after remapping, so it is not necessarily the
    /// topic name which was used when creating the subscription.
    pub fn topic_name(&self) -> String {
        // SAFETY: The subscription handle is valid because its lifecycle is managed by an Arc.
        // The unsafe variables get converted to safe types before being returned
        unsafe {
            let raw_topic_pointer = rcl_subscription_get_topic_name(&*self.handle.lock());
            CStr::from_ptr(raw_topic_pointer)
        }
        .to_string_lossy()
        .into_owned()
    }

    /// Used by [`Node`][crate::Node] to create a new subscription.
    pub(crate) fn create<'a>(
        options: impl Into<SubscriptionOptions<'a>>,
        callback: AnySubscriptionCallback<T, Scope::Payload>,
        node_handle: &Arc<NodeHandle>,
        commands: &Arc<WorkerCommands>,
    ) -> Result<Arc<Self>, RclrsError> {
        let SubscriptionOptions { topic, qos } = options.into();
        let callback = Arc::new(Mutex::new(callback));

        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let mut rcl_subscription_options = unsafe { rcl_subscription_get_default_options() };
        rcl_subscription_options.qos = qos.into();

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
                    type_support,
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

        let (waitable, lifecycle) = Waitable::new(
            Box::new(SubscriptionExecutable {
                handle: Arc::clone(&handle),
                callback: Arc::clone(&callback),
                commands: Arc::clone(commands),
            }),
            Some(Arc::clone(commands.get_guard_condition())),
        );
        commands.add_to_wait_set(waitable);

        Ok(Arc::new(Self {
            handle,
            callback,
            lifecycle,
        }))
    }
}

impl<T: Message> SubscriptionState<T, Node> {
    /// Set the callback of this subscription, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the subscription previously used an async callback.
    ///
    /// This can only be called when the `Scope` of the [`SubscriptionState`] is [`Node`].
    /// If the `Scope` is [`Worker<Payload>`] then use [`Self::set_worker_callback`] instead.
    pub fn set_callback<Args>(&self, callback: impl IntoNodeSubscriptionCallback<T, Args>) {
        let callback = callback.into_node_subscription_callback();
        *self.callback.lock().unwrap() = callback;
    }

    /// Set the callback of this subscription, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the subscription previously used a non-async callback.
    ///
    /// This can only be called when the `Scope` of the [`SubscriptionState`] is [`Node`].
    /// If the `Scope` is [`Worker<Payload>`] then use [`Self::set_worker_callback`] instead.
    pub fn set_async_callback<Args>(&self, callback: impl IntoAsyncSubscriptionCallback<T, Args>) {
        let callback = callback.into_async_subscription_callback();
        *self.callback.lock().unwrap() = callback;
    }
}

impl<T: Message, Payload: 'static + Send + Sync> SubscriptionState<T, Worker<Payload>> {
    /// Set the callback of this subscription, replacing the callback that was
    /// previously set.
    ///
    /// This can only be called when the `Scope` of the [`SubscriptionState`] is [`Worker`].
    /// If the `Scope` is [`Node`] then use [`Self::set_callback`] or
    /// [`Self::set_async_callback`] instead.
    pub fn set_worker_callback<Args>(
        &self,
        callback: impl IntoWorkerSubscriptionCallback<T, Payload, Args>,
    ) {
        let callback = callback.into_worker_subscription_callback();
        *self.callback.lock().unwrap() = callback;
    }
}

/// `SubscriptionOptions` are used by [`Node::create_subscription`][1] to initialize
/// a [`Subscription`].
///
/// [1]: crate::NodeState::create_subscription
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct SubscriptionOptions<'a> {
    /// The topic name for the subscription.
    pub topic: &'a str,
    /// The quality of service settings for the subscription.
    pub qos: QoSProfile,
}

impl<'a> SubscriptionOptions<'a> {
    /// Initialize a new [`SubscriptionOptions`] with default settings.
    pub fn new(topic: &'a str) -> Self {
        Self {
            topic,
            qos: QoSProfile::topics_default(),
        }
    }
}

impl<'a, T: IntoPrimitiveOptions<'a>> From<T> for SubscriptionOptions<'a> {
    fn from(value: T) -> Self {
        let primitive = value.into_primitive_options();
        let mut options = Self::new(primitive.name);
        primitive.apply_to(&mut options.qos);
        options
    }
}

struct SubscriptionExecutable<T: Message, Payload> {
    handle: Arc<SubscriptionHandle>,
    callback: Arc<Mutex<AnySubscriptionCallback<T, Payload>>>,
    commands: Arc<WorkerCommands>,
}

impl<T, Payload: 'static> RclPrimitive for SubscriptionExecutable<T, Payload>
where
    T: Message,
{
    unsafe fn execute(
        &mut self,
        ready: ReadyKind,
        payload: &mut dyn Any,
    ) -> Result<(), RclrsError> {
        ready.for_basic()?;
        self.callback
            .lock()
            .unwrap()
            .execute(&self.handle, payload, &self.commands)
    }

    fn kind(&self) -> RclPrimitiveKind {
        RclPrimitiveKind::Subscription
    }

    fn handle(&self) -> RclPrimitiveHandle {
        RclPrimitiveHandle::Subscription(self.handle.lock())
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_subscription_t {}

/// Manage the lifecycle of an `rcl_subscription_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_subscription_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
struct SubscriptionHandle {
    rcl_subscription: Mutex<rcl_subscription_t>,
    node_handle: Arc<NodeHandle>,
}

impl SubscriptionHandle {
    fn lock(&self) -> MutexGuard<rcl_subscription_t> {
        self.rcl_subscription.lock().unwrap()
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
    fn take<T: Message>(&self) -> Result<(T, MessageInfo), RclrsError> {
        let mut rmw_message = <T as Message>::RmwMsg::default();
        let message_info = Self::take_inner::<T>(self, &mut rmw_message)?;
        Ok((T::from_rmw_message(rmw_message), message_info))
    }

    /// This is a version of take() that returns a boxed message.
    ///
    /// This can be more efficient for messages containing large arrays.
    fn take_boxed<T: Message>(&self) -> Result<(Box<T>, MessageInfo), RclrsError> {
        let mut rmw_message = Box::<<T as Message>::RmwMsg>::default();
        let message_info = Self::take_inner::<T>(self, &mut *rmw_message)?;
        // TODO: This will still use the stack in general. Change signature of
        // from_rmw_message to allow placing the result in a Box directly.
        let message = Box::new(T::from_rmw_message(*rmw_message));
        Ok((message, message_info))
    }

    // Inner function, to be used by both regular and boxed versions.
    fn take_inner<T: Message>(
        &self,
        rmw_message: &mut <T as Message>::RmwMsg,
    ) -> Result<MessageInfo, RclrsError> {
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        let rcl_subscription = &mut *self.lock();
        unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                rcl_subscription,
                rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?
        };
        Ok(MessageInfo::from_rmw_message_info(&message_info))
    }

    /// Obtains a read-only handle to a message owned by the middleware.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1].
    ///
    /// This is the counterpart to [`Publisher::borrow_loaned_message()`][2]. See its documentation
    /// for more information.
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::Publisher::borrow_loaned_message
    fn take_loaned<T: Message>(
        self: &Arc<Self>,
    ) -> Result<(ReadOnlyLoanedMessage<T>, MessageInfo), RclrsError> {
        let mut msg_ptr = std::ptr::null_mut();
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        unsafe {
            // SAFETY: The third argument (message_info) and fourth argument (allocation) may be null.
            // The second argument (loaned_message) contains a null ptr as expected.
            rcl_take_loaned_message(
                &*self.lock(),
                &mut msg_ptr,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?;
        }
        let read_only_loaned_msg = ReadOnlyLoanedMessage {
            msg_ptr: msg_ptr as *const T::RmwMsg,
            handle: Arc::clone(self),
        };
        Ok((
            read_only_loaned_msg,
            MessageInfo::from_rmw_message_info(&message_info),
        ))
    }
}

impl Drop for SubscriptionHandle {
    fn drop(&mut self) {
        let rcl_subscription = self.rcl_subscription.get_mut().unwrap();
        let mut rcl_node = self.node_handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_subscription_fini(rcl_subscription, &mut *rcl_node);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{test_helpers::*, vendor::test_msgs::msg};

    #[test]
    fn traits() {
        assert_send::<SubscriptionState<msg::BoundedSequences, Node>>();
        assert_sync::<SubscriptionState<msg::BoundedSequences, Node>>();
    }

    #[test]
    fn test_subscriptions() -> Result<(), RclrsError> {
        use crate::TopicEndpointInfo;

        let namespace = "/test_subscriptions_graph";
        let graph = construct_test_graph(namespace)?;

        let node_2_empty_subscription = graph
            .node2
            .create_subscription::<msg::Empty, _>("graph_test_topic_1", |_msg: msg::Empty| {})?;
        let topic1 = node_2_empty_subscription.topic_name();
        let node_2_basic_types_subscription =
            graph.node2.create_subscription::<msg::BasicTypes, _>(
                "graph_test_topic_2",
                |_msg: msg::BasicTypes| {},
            )?;
        let topic2 = node_2_basic_types_subscription.topic_name();
        let node_1_defaults_subscription = graph.node1.create_subscription::<msg::Defaults, _>(
            "graph_test_topic_3",
            |_msg: msg::Defaults| {},
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

    #[test]
    fn test_node_subscription_raii() {
        use crate::*;
        use std::sync::atomic::{AtomicBool, Ordering};

        let mut executor = Context::default().create_basic_executor();

        let triggered = Arc::new(AtomicBool::new(false));
        let inner_triggered = Arc::clone(&triggered);
        let callback = move |_: msg::Empty| {
            inner_triggered.store(true, Ordering::Release);
        };

        let (_subscription, publisher) = {
            let node = executor
                .create_node(&format!("test_node_subscription_raii_{}", line!()))
                .unwrap();

            let qos = QoSProfile::default().keep_all().reliable();
            let subscription = node
                .create_subscription::<msg::Empty, _>("test_topic".qos(qos), callback)
                .unwrap();
            let publisher = node
                .create_publisher::<msg::Empty>("test_topic".qos(qos))
                .unwrap();

            (subscription, publisher)
        };

        publisher.publish(msg::Empty::default()).unwrap();
        let start_time = std::time::Instant::now();
        while !triggered.load(Ordering::Acquire) {
            assert!(executor.spin(SpinOptions::spin_once()).is_empty());
            assert!(start_time.elapsed() < std::time::Duration::from_secs(10));
        }
    }

    #[test]
    fn test_delayed_subscription() {
        use crate::{vendor::example_interfaces::msg::Empty, *};
        use futures::{
            channel::{mpsc, oneshot},
            StreamExt,
        };
        use std::sync::atomic::{AtomicBool, Ordering};

        let mut executor = Context::default().create_basic_executor();
        let node = executor
            .create_node(
                format!("test_delayed_subscription_{}", line!())
                    // We need to turn off parameter services because their activity will
                    // wake up the wait set, which defeats the purpose of this test.
                    .start_parameter_services(false),
            )
            .unwrap();

        let (promise, receiver) = oneshot::channel();
        let promise = Arc::new(Mutex::new(Some(promise)));

        let success = Arc::new(AtomicBool::new(false));
        let send_success = Arc::clone(&success);

        let publisher = node.create_publisher("test_delayed_subscription").unwrap();

        let commands = Arc::clone(executor.commands());
        std::thread::spawn(move || {
            // Wait a little while so the executor can start spinning and guard
            // conditions can settle down.
            std::thread::sleep(std::time::Duration::from_millis(10));

            let _ = commands.run(async move {
                let (sender, mut receiver) = mpsc::unbounded();
                let _subscription = node
                    .create_subscription("test_delayed_subscription", move |_: Empty| {
                        let _ = sender.unbounded_send(());
                    })
                    .unwrap();

                // Make sure the message doesn't get dropped due to the subscriber
                // not being connected yet.
                let _ = publisher.notify_on_subscriber_ready().await;

                // Publish the message, which should trigger the executor to stop spinning
                publisher.publish(Empty::default()).unwrap();

                if let Some(_) = receiver.next().await {
                    send_success.store(true, Ordering::Release);
                    if let Some(promise) = promise.lock().unwrap().take() {
                        promise.send(()).unwrap();
                    }
                }
            });
        });

        let r = executor.spin(
            SpinOptions::default()
                .until_promise_resolved(receiver)
                .timeout(std::time::Duration::from_secs(10)),
        );

        assert!(r.is_empty(), "{r:?}");
        let message_was_received = success.load(Ordering::Acquire);
        assert!(message_was_received);
    }
}
