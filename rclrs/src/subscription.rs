use std::{
    ffi::{CStr, CString},
    marker::PhantomData,
    sync::{atomic::AtomicBool, Arc, Mutex, MutexGuard},
};

use rosidl_runtime_rs::{Message, RmwMessage};

use crate::{
    error::{RclReturnCode, ToResult},
    qos::QoSProfile,
    rcl_bindings::*,
    IntoPrimitiveOptions, Node, NodeHandle, RclrsError, ENTITY_LIFECYCLE_MUTEX,
};

mod callback;
mod message_info;
mod readonly_loaned_message;
pub use callback::*;
pub use message_info::*;
pub use readonly_loaned_message::*;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_subscription_t {}

/// Manage the lifecycle of an `rcl_subscription_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_subscription_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct SubscriptionHandle {
    rcl_subscription: Mutex<rcl_subscription_t>,
    node_handle: Arc<NodeHandle>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl SubscriptionHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_subscription_t> {
        self.rcl_subscription.lock().unwrap()
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

/// Trait to be implemented by concrete [`Subscription`]s.
pub trait SubscriptionBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &SubscriptionHandle;
    /// Tries to take a new message and run the callback with it.
    fn execute(&self) -> Result<(), RclrsError>;
}

/// Struct for receiving messages of type `T`.
///
/// Create a subscription using [`Node::create_subscription`][1].
///
/// There can be multiple subscriptions for the same topic, in different nodes or the same node.
/// A clone of a `Subscription` will refer to the same subscription instance as the original.
/// The underlying instance is tied to [`SubscriptionState`] which implements the [`Subscription`] API.
///
/// Receiving messages requires the node's executor to [spin][2].
///
/// When a subscription is created, it may take some time to get "matched" with a corresponding
/// publisher.
///
/// [1]: crate::NodeState::create_subscription
/// [2]: crate::spin
pub type Subscription<T> = Arc<SubscriptionState<T>>;

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
pub struct SubscriptionState<T>
where
    T: Message,
{
    pub(crate) handle: Arc<SubscriptionHandle>,
    /// The callback function that runs when a message was received.
    pub callback: Mutex<AnySubscriptionCallback<T>>,
    /// Ensure the parent node remains alive as long as the subscription is held.
    /// This implementation will change in the future.
    #[allow(unused)]
    node: Node,
    message: PhantomData<T>,
}

impl<T> SubscriptionState<T>
where
    T: Message,
{
    /// Creates a new subscription.
    pub(crate) fn new<'a, Args>(
        node: &Node,
        options: impl Into<SubscriptionOptions<'a>>,
        callback: impl SubscriptionCallback<T, Args>,
    ) -> Result<Self, RclrsError>
    // This uses pub(crate) visibility to avoid instantiating this struct outside
    // [`Node::create_subscription`], see the struct's documentation for the rationale
    where
        T: Message,
    {
        let SubscriptionOptions { topic, qos } = options.into();
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
            let rcl_node = node.handle.rcl_node.lock().unwrap();
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
            node_handle: Arc::clone(&node.handle),
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(callback.into_callback()),
            node: Arc::clone(node),
            message: PhantomData,
        })
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
    pub fn take(&self) -> Result<(T, MessageInfo), RclrsError> {
        let mut rmw_message = <T as Message>::RmwMsg::default();
        let message_info = self.take_inner(&mut rmw_message)?;
        Ok((T::from_rmw_message(rmw_message), message_info))
    }

    /// This is a version of take() that returns a boxed message.
    ///
    /// This can be more efficient for messages containing large arrays.
    pub fn take_boxed(&self) -> Result<(Box<T>, MessageInfo), RclrsError> {
        let mut rmw_message = Box::<<T as Message>::RmwMsg>::default();
        let message_info = self.take_inner(&mut *rmw_message)?;
        // TODO: This will still use the stack in general. Change signature of
        // from_rmw_message to allow placing the result in a Box directly.
        let message = Box::new(T::from_rmw_message(*rmw_message));
        Ok((message, message_info))
    }

    // Inner function, to be used by both regular and boxed versions.
    fn take_inner(
        &self,
        rmw_message: &mut <T as Message>::RmwMsg,
    ) -> Result<MessageInfo, RclrsError> {
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        let rcl_subscription = &mut *self.handle.lock();
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
    /// This is the counterpart to [`PublisherState::borrow_loaned_message()`][2]. See its documentation
    /// for more information.
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::PublisherState::borrow_loaned_message
    pub fn take_loaned(&self) -> Result<(ReadOnlyLoanedMessage<'_, T>, MessageInfo), RclrsError> {
        let mut msg_ptr = std::ptr::null_mut();
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        unsafe {
            // SAFETY: The third argument (message_info) and fourth argument (allocation) may be null.
            // The second argument (loaned_message) contains a null ptr as expected.
            rcl_take_loaned_message(
                &*self.handle.lock(),
                &mut msg_ptr,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?;
        }
        let read_only_loaned_msg = ReadOnlyLoanedMessage {
            msg_ptr: msg_ptr as *const T::RmwMsg,
            subscription: self,
        };
        Ok((
            read_only_loaned_msg,
            MessageInfo::from_rmw_message_info(&message_info),
        ))
    }
}

/// `SubscriptionOptions` are used by [`Node::create_subscription`][1] to initialize
/// a [`Subscription`].
///
/// [1]: crate::Node::create_subscription
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

impl<T> SubscriptionBase for SubscriptionState<T>
where
    T: Message,
{
    fn handle(&self) -> &SubscriptionHandle {
        &self.handle
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let evaluate = || {
            match &mut *self.callback.lock().unwrap() {
                AnySubscriptionCallback::Regular(cb) => {
                    let (msg, _) = self.take()?;
                    cb(msg)
                }
                AnySubscriptionCallback::RegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = self.take()?;
                    cb(msg, msg_info)
                }
                AnySubscriptionCallback::Boxed(cb) => {
                    let (msg, _) = self.take_boxed()?;
                    cb(msg)
                }
                AnySubscriptionCallback::BoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = self.take_boxed()?;
                    cb(msg, msg_info)
                }
                AnySubscriptionCallback::Loaned(cb) => {
                    let (msg, _) = self.take_loaned()?;
                    cb(msg)
                }
                AnySubscriptionCallback::LoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = self.take_loaned()?;
                    cb(msg, msg_info)
                }
            }
            Ok(())
        };

        // Immediately evaluated closure, to handle SubscriptionTakeFailed
        // outside this match
        match evaluate() {
            Err(RclrsError::RclError {
                code: RclReturnCode::SubscriptionTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // subscription was ready, so it shouldn't be an error.
                Ok(())
            }
            other => other,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;
    use test_msgs::msg;

    #[test]
    fn traits() {
        assert_send::<SubscriptionState<msg::BoundedSequences>>();
        assert_sync::<SubscriptionState<msg::BoundedSequences>>();
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
        use std::sync::atomic::Ordering;

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
}
