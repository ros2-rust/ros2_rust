use std::{
    borrow::Cow,
    ffi::{CStr, CString},
    marker::PhantomData,
    sync::{Arc, Mutex},
};

use rosidl_runtime_rs::{Message, RmwMessage};

use crate::{
    error::{RclrsError, ToResult},
    qos::QoSProfile,
    rcl_bindings::*,
    IntoPrimitiveOptions, Node, Promise, ENTITY_LIFECYCLE_MUTEX,
};

mod loaned_message;
pub use loaned_message::*;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_publisher_t {}

/// Manage the lifecycle of an `rcl_publisher_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_publisher_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
struct PublisherHandle {
    rcl_publisher: Mutex<rcl_publisher_t>,
    /// We store the whole node here because we use some of its user-facing API
    /// in some of the Publisher methods.
    node: Node,
}

impl Drop for PublisherHandle {
    fn drop(&mut self) {
        let mut rcl_node = self.node.handle().rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_publisher_fini(self.rcl_publisher.get_mut().unwrap(), &mut *rcl_node);
        }
    }
}

/// Struct for sending messages of type `T`.
///
/// Create a publisher using [`Node::create_publisher`][1].
///
/// Multiple publishers can be created for the same topic, in different nodes or the same node.
/// A clone of a `Publisher` will refer to the same publisher instance as the original.
/// The underlying instance is tied to [`PublisherState`] which implements the [`Publisher`] API.
///
/// The underlying RMW will decide on the concrete delivery mechanism (network stack, shared
/// memory, or intraprocess).
///
/// Sending messages does not require the node's executor to [spin][2].
///
/// [1]: crate::NodeState::create_publisher
/// [2]: crate::Executor::spin
pub type Publisher<T> = Arc<PublisherState<T>>;

/// The inner state of a [`Publisher`].
///
/// This is public so that you can choose to create a [`Weak`][1] reference to it
/// if you want to be able to refer to a [`Publisher`] in a non-owning way. It is
/// generally recommended to manage the `PublisherState` inside of an [`Arc`],
/// and [`Publisher`] is provided as a convenience alias for that.
///
/// The public API of the [`Publisher`] type is implemented via `PublisherState`.
///
/// [1]: std::sync::Weak
pub struct PublisherState<T>
where
    T: Message,
{
    // The data pointed to by type_support_ptr has static lifetime;
    // it is global data in the type support library.
    type_support_ptr: *const rosidl_message_type_support_t,
    message: PhantomData<T>,
    handle: PublisherHandle,
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl<T> Send for PublisherState<T> where T: Message {}
// SAFETY: The type_support_ptr prevents the default Sync impl.
// rosidl_message_type_support_t is a read-only type without interior mutability.
unsafe impl<T> Sync for PublisherState<T> where T: Message {}

impl<T> PublisherState<T>
where
    T: Message,
{
    /// Creates a new `Publisher`.
    ///
    /// Node and namespace changes are always applied _before_ topic remapping.
    pub(crate) fn create<'a>(
        options: impl Into<PublisherOptions<'a>>,
        node: Node,
    ) -> Result<Arc<Self>, RclrsError>
    where
        T: Message,
    {
        let PublisherOptions { topic, qos } = options.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_publisher = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support_ptr =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let mut publisher_options = unsafe { rcl_publisher_get_default_options() };
        publisher_options.qos = qos.into();

        {
            let rcl_node = node.handle().rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            unsafe {
                // SAFETY:
                // * The rcl_publisher is zero-initialized as mandated by this function.
                // * The rcl_node is kept alive by the NodeHandle because it is a dependency of the publisher.
                // * The topic name and the options are copied by this function, so they can be dropped afterwards.
                // * The entity lifecycle mutex is locked to protect against the risk of global
                //   variables in the rmw implementation being unsafely modified during cleanup.
                rcl_publisher_init(
                    &mut rcl_publisher,
                    &*rcl_node,
                    type_support_ptr,
                    topic_c_string.as_ptr(),
                    &publisher_options,
                )
                .ok()?;
            }
        }

        Ok(Arc::new(Self {
            type_support_ptr,
            message: PhantomData,
            handle: PublisherHandle {
                rcl_publisher: Mutex::new(rcl_publisher),
                node,
            },
        }))
    }

    /// Returns the topic name of the publisher.
    ///
    /// This returns the topic name after remapping, so it is not necessarily the
    /// topic name which was used when creating the publisher.
    pub fn topic_name(&self) -> String {
        // SAFETY: No preconditions for the functions called.
        // The unsafe variables created get converted to safe types before being returned
        unsafe {
            let raw_topic_pointer =
                rcl_publisher_get_topic_name(&*self.handle.rcl_publisher.lock().unwrap());
            CStr::from_ptr(raw_topic_pointer)
                .to_string_lossy()
                .into_owned()
        }
    }

    /// Returns the number of subscriptions of the publisher.
    pub fn get_subscription_count(&self) -> Result<usize, RclrsError> {
        let mut subscription_count = 0;
        // SAFETY: No preconditions for the function called.
        unsafe {
            rcl_publisher_get_subscription_count(
                &*self.handle.rcl_publisher.lock().unwrap(),
                &mut subscription_count,
            )
            .ok()?
        };
        Ok(subscription_count)
    }

    /// Get a promise that will be fulfilled when at least one subscriber is
    /// listening to this publisher.
    pub fn notify_on_subscriber_ready(self: &Arc<PublisherState<T>>) -> Promise<()> {
        let publisher = Arc::clone(self);
        self.handle.node.notify_on_graph_change(move || {
            publisher
                .get_subscription_count()
                .is_ok_and(|count| count > 0)
        })
    }

    /// Publishes a message.
    ///
    /// The [`MessageCow`] trait is implemented by any
    /// [`Message`] as well as any reference to a `Message`.
    ///
    /// The reason for allowing owned messages is that publishing owned messages can be more
    /// efficient in the case of idiomatic messages[^note].
    ///
    /// [^note]: See the [`Message`] trait for an explanation of "idiomatic".
    ///
    /// Hence, when a message will not be needed anymore after publishing, pass it by value.
    /// When a message will be needed again after publishing, pass it by reference, instead of cloning and passing by value.
    ///
    /// Calling `publish()` is a potentially blocking call, see [this issue][1] for details.
    ///
    /// [1]: https://github.com/ros2/ros2/issues/255
    pub fn publish<'a, M: MessageCow<'a, T>>(&self, message: M) -> Result<(), RclrsError> {
        let rmw_message = T::into_rmw_message(message.into_cow());
        let rcl_publisher = &mut *self.handle.rcl_publisher.lock().unwrap();
        unsafe {
            // SAFETY: The message type is guaranteed to match the publisher type by the type system.
            // The message does not need to be valid beyond the duration of this function call.
            // The third argument is explictly allowed to be NULL.
            rcl_publish(
                rcl_publisher,
                rmw_message.as_ref() as *const <T as Message>::RmwMsg as *mut _,
                std::ptr::null_mut(),
            )
            .ok()
        }
    }
}

impl<T> PublisherState<T>
where
    T: RmwMessage,
{
    /// Obtains a writable handle to a message owned by the middleware.
    ///
    /// This lets the middleware control how and where to allocate memory for the
    /// message.
    /// The purpose of this is typically to achieve *zero-copy communication* between publishers and
    /// subscriptions on the same machine: the message is placed directly in a shared memory region,
    /// and a reference to the same memory is returned by [`Subscription::take_loaned_message()`][1].
    /// No copying or serialization/deserialization takes place, which is much more efficient,
    /// especially as the message size grows.
    ///
    /// # Conditions for zero-copy communication
    /// 1. A middleware with support for shared memory is used, e.g. `CycloneDDS` with `iceoryx`
    /// 1. Shared memory transport is enabled in the middleware configuration
    /// 1. Publishers and subscriptions are on the same machine
    /// 1. The message is a "plain old data" type containing no variable-size members, whether bounded or unbounded
    /// 1. The publisher's QOS settings are compatible with zero-copy, e.g. the [default QOS][2]
    /// 1. `Publisher::borrow_loaned_message()` is used and the subscription uses a callback taking a
    ///    [`ReadOnlyLoanedMessage`][1]
    ///
    /// This function is only implemented for [`RmwMessage`]s since the "idiomatic" message type
    /// does not have a typesupport library.
    ///
    /// [1]: crate::ReadOnlyLoanedMessage
    /// [2]: crate::QOS_PROFILE_DEFAULT
    //
    // TODO: Explain more, e.g.
    // - Zero-copy communication between rclcpp and rclrs possible?
    // - What errors occur when?
    // - What happens when only *some* subscribers are local?
    // - What QOS settings are required exactly? https://cyclonedds.io/docs/cyclonedds/latest/shared_memory.html
    pub fn borrow_loaned_message(&self) -> Result<LoanedMessage<'_, T>, RclrsError> {
        let mut msg_ptr = std::ptr::null_mut();
        unsafe {
            // SAFETY: msg_ptr contains a null ptr as expected by this function.
            rcl_borrow_loaned_message(
                &*self.handle.rcl_publisher.lock().unwrap(),
                self.type_support_ptr,
                &mut msg_ptr,
            )
            .ok()?;
        }
        Ok(LoanedMessage {
            publisher: self,
            msg_ptr: msg_ptr as *mut T,
        })
    }

    /// Returns true if message loans are possible, false otherwise.
    pub fn can_loan_messages(&self) -> bool {
        unsafe { rcl_publisher_can_loan_messages(&*self.handle.rcl_publisher.lock().unwrap()) }
    }
}

/// `PublisherOptions` are used by [`NodeState::create_publisher`][1] to initialize
/// a [`Publisher`].
///
/// [1]: crate::NodeState::create_publisher
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct PublisherOptions<'a> {
    /// The topic name for the publisher.
    pub topic: &'a str,
    /// The quality of service settings for the publisher.
    pub qos: QoSProfile,
}

impl<'a> PublisherOptions<'a> {
    /// Initialize a new [`PublisherOptions`] with default settings.
    pub fn new(topic: &'a str) -> Self {
        Self {
            topic,
            qos: QoSProfile::topics_default(),
        }
    }
}

impl<'a, T: IntoPrimitiveOptions<'a>> From<T> for PublisherOptions<'a> {
    fn from(value: T) -> Self {
        let primitive = value.into_primitive_options();
        let mut options = Self::new(primitive.name);
        primitive.apply_to(&mut options.qos);
        options
    }
}

/// Convenience trait for [`PublisherState::publish`].
pub trait MessageCow<'a, T: Message> {
    /// Wrap the owned or borrowed message in a `Cow`.
    fn into_cow(self) -> Cow<'a, T>;
}

impl<'a, T: Message> MessageCow<'a, T> for T {
    fn into_cow(self) -> Cow<'a, T> {
        Cow::Owned(self)
    }
}

impl<'a, T: Message> MessageCow<'a, T> for &'a T {
    fn into_cow(self) -> Cow<'a, T> {
        Cow::Borrowed(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn traits() {
        use crate::vendor::test_msgs;
        assert_send::<Publisher<test_msgs::msg::BoundedSequences>>();
        assert_sync::<Publisher<test_msgs::msg::BoundedSequences>>();
    }

    #[test]
    fn test_publishers() -> Result<(), RclrsError> {
        use crate::{vendor::test_msgs::msg, TopicEndpointInfo};

        let namespace = "/test_publishers_graph";
        let graph = construct_test_graph(namespace)?;

        let node_1_empty_publisher = graph
            .node1
            .create_publisher::<msg::Empty>("graph_test_topic_1")?;
        let topic1 = node_1_empty_publisher.topic_name();
        let node_1_basic_types_publisher = graph
            .node1
            .create_publisher::<msg::BasicTypes>("graph_test_topic_2")?;
        let topic2 = node_1_basic_types_publisher.topic_name();
        let node_2_default_publisher = graph
            .node2
            .create_publisher::<msg::Defaults>("graph_test_topic_3")?;
        let topic3 = node_2_default_publisher.topic_name();

        std::thread::sleep(std::time::Duration::from_millis(100));

        // Test count_publishers()
        assert_eq!(graph.node1.count_publishers(&topic1)?, 1);
        assert_eq!(graph.node1.count_publishers(&topic2)?, 1);
        assert_eq!(graph.node1.count_publishers(&topic3)?, 1);

        // Test get_publisher_names_and_types_by_node()
        let node_1_publisher_names_and_types = graph
            .node1
            .get_publisher_names_and_types_by_node(&graph.node1.name(), namespace)?;

        let types = node_1_publisher_names_and_types.get(&topic1).unwrap();
        assert!(types.contains(&"test_msgs/msg/Empty".to_string()));

        let types = node_1_publisher_names_and_types.get(&topic2).unwrap();
        assert!(types.contains(&"test_msgs/msg/BasicTypes".to_string()));

        let node_2_publisher_names_and_types = graph
            .node1
            .get_publisher_names_and_types_by_node(&graph.node2.name(), namespace)?;

        let types = node_2_publisher_names_and_types.get(&topic3).unwrap();
        assert!(types.contains(&"test_msgs/msg/Defaults".to_string()));

        // Test get_publishers_info_by_topic()
        let expected_publishers_info = vec![TopicEndpointInfo {
            node_name: String::from("graph_test_node_1"),
            node_namespace: String::from(namespace),
            topic_type: String::from("test_msgs/msg/Empty"),
        }];
        assert_eq!(
            graph.node1.get_publishers_info_by_topic(&topic1)?,
            expected_publishers_info
        );
        assert_eq!(
            graph.node2.get_publishers_info_by_topic(&topic1)?,
            expected_publishers_info
        );

        // Test get_subscription_count()
        assert_eq!(node_1_empty_publisher.get_subscription_count(), Ok(0));
        assert_eq!(node_1_basic_types_publisher.get_subscription_count(), Ok(0));
        assert_eq!(node_2_default_publisher.get_subscription_count(), Ok(0));
        let _node_1_empty_subscriber = graph
            .node1
            .create_subscription("graph_test_topic_1", |_msg: msg::Empty| {});
        let _node_1_basic_types_subscriber = graph
            .node1
            .create_subscription("graph_test_topic_2", |_msg: msg::BasicTypes| {});
        let _node_2_default_subscriber = graph
            .node2
            .create_subscription("graph_test_topic_3", |_msg: msg::Defaults| {});
        assert_eq!(node_1_empty_publisher.get_subscription_count(), Ok(1));
        assert_eq!(node_1_basic_types_publisher.get_subscription_count(), Ok(1));
        assert_eq!(node_2_default_publisher.get_subscription_count(), Ok(1));

        Ok(())
    }
}
