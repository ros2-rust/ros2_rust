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
    NodeHandle, ENTITY_LIFECYCLE_MUTEX,
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
    node_handle: Arc<NodeHandle>,
}

impl Drop for PublisherHandle {
    fn drop(&mut self) {
        let mut rcl_node = self.node_handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_publisher_fini(self.rcl_publisher.get_mut().unwrap(), &mut **rcl_node);
        }
    }
}

/// Struct for sending messages of type `T`.
///
/// Multiple publishers can be created for the same topic, in different nodes or the same node.
///
/// The underlying RMW will decide on the concrete delivery mechanism (network stack, shared
/// memory, or intraprocess).
///
/// Sending messages does not require calling [`spin`][1] on the publisher's node.
///
/// [1]: crate::spin
pub struct Publisher<T>
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
unsafe impl<T> Send for Publisher<T> where T: Message {}
// SAFETY: The type_support_ptr prevents the default Sync impl.
// rosidl_message_type_support_t is a read-only type without interior mutability.
unsafe impl<T> Sync for Publisher<T> where T: Message {}

impl<T> Publisher<T>
where
    T: Message,
{
    /// Creates a new `Publisher`.
    ///
    /// Node and namespace changes are always applied _before_ topic remapping.
    pub(crate) fn new(
        node_handle: Arc<NodeHandle>,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Self, RclrsError>
    where
        T: Message,
    {
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
            let rcl_node = node_handle.rcl_node.lock().unwrap();
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
                    &**rcl_node,
                    type_support_ptr,
                    topic_c_string.as_ptr(),
                    &publisher_options,
                )
                .ok()?;
            }
        }

        Ok(Self {
            type_support_ptr,
            message: PhantomData,
            handle: PublisherHandle {
                rcl_publisher: Mutex::new(rcl_publisher),
                node_handle,
            },
        })
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

impl<T> Publisher<T>
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
}

/// Convenience trait for [`Publisher::publish`].
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
        assert_send::<Publisher<test_msgs::msg::BoundedSequences>>();
        assert_sync::<Publisher<test_msgs::msg::BoundedSequences>>();
    }

    #[test]
    fn test_publishers() -> Result<(), RclrsError> {
        use crate::{TopicEndpointInfo, QOS_PROFILE_SYSTEM_DEFAULT};
        use test_msgs::msg;

        let namespace = "/test_publishers_graph";
        let graph = construct_test_graph(namespace)?;

        let node_1_empty_publisher = graph
            .node1
            .create_publisher::<msg::Empty>("graph_test_topic_1", QOS_PROFILE_SYSTEM_DEFAULT)?;
        let topic1 = node_1_empty_publisher.topic_name();
        let node_1_basic_types_publisher = graph.node1.create_publisher::<msg::BasicTypes>(
            "graph_test_topic_2",
            QOS_PROFILE_SYSTEM_DEFAULT,
        )?;
        let topic2 = node_1_basic_types_publisher.topic_name();
        let node_2_default_publisher = graph
            .node2
            .create_publisher::<msg::Defaults>("graph_test_topic_3", QOS_PROFILE_SYSTEM_DEFAULT)?;
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

        Ok(())
    }
}
