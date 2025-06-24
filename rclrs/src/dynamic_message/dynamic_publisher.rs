use std::ffi::CString;
use std::sync::{Arc, Mutex};

use super::{
    get_type_support_handle, get_type_support_library, DynamicMessage, DynamicMessageError,
    DynamicMessageMetadata, MessageTypeName,
};
use crate::error::{RclrsError, ToResult};
use crate::rcl_bindings::*;
use crate::{NodeHandle, PublisherHandle, PublisherOptions, ENTITY_LIFECYCLE_MUTEX};

/// Struct for sending dynamic messages.
///
/// Create a dynamic publisher using [`Node::create_dynamic_publisher`][1].
/// Refer to [`crate::Publisher`] for details of the behavior.
///
/// [1]: crate::NodeState::create_dynamic_publisher
pub type DynamicPublisher = Arc<DynamicPublisherState>;

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
pub struct DynamicPublisherState {
    handle: PublisherHandle,
    metadata: DynamicMessageMetadata,
    // This is the regular type support library, not the introspection one.
    #[allow(dead_code)]
    type_support_library: Arc<libloading::Library>,
}

impl DynamicPublisherState {
    /// Creates a new `DynamicPublisherState`.
    ///
    /// Node and namespace changes are always applied _before_ topic remapping.
    pub(crate) fn create<'a>(
        topic_type: MessageTypeName,
        options: impl Into<PublisherOptions<'a>>,
        node_handle: Arc<NodeHandle>,
    ) -> Result<Arc<Self>, RclrsError> {
        // This loads the introspection type support library.
        let metadata = DynamicMessageMetadata::new(topic_type)?;
        let PublisherOptions { topic, qos } = options.into();
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

        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_publisher = unsafe { rcl_get_zero_initialized_publisher() };
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
                // SAFETY: The rcl_publisher is zero-initialized as expected by this function.
                // The rcl_node is kept alive because it is co-owned by the subscription.
                // The topic name and the options are copied by this function, so they can be dropped
                // afterwards.
                // TODO: type support?
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
            handle: PublisherHandle {
                rcl_publisher: Mutex::new(rcl_publisher),
                node_handle,
            },
            metadata,
            type_support_library,
        }))
    }

    /// Returns the topic name of the publisher.
    ///
    /// This returns the topic name after remapping, so it is not necessarily the
    /// topic name which was used when creating the publisher.
    pub fn topic_name(&self) -> String {
        self.handle.topic_name()
    }

    /// Returns the number of subscriptions of the publisher.
    pub fn get_subscription_count(&self) -> Result<usize, RclrsError> {
        self.handle.get_subscription_count()
    }

    /// Publishes a message.
    ///
    /// Calling `publish()` is a potentially blocking call, see [this issue][1] for details.
    ///
    /// [1]: https://github.com/ros2/ros2/issues/255
    pub fn publish(&self, mut message: DynamicMessage) -> Result<(), RclrsError> {
        if message.metadata.message_type != self.metadata.message_type {
            return Err(DynamicMessageError::MessageTypeMismatch.into());
        }
        let rcl_publisher = &mut *self.handle.rcl_publisher.lock().unwrap();
        unsafe {
            // SAFETY: The message type is guaranteed to match the publisher type by the type system.
            // The message does not need to be valid beyond the duration of this function call.
            // The third argument is explictly allowed to be NULL.
            rcl_publish(
                rcl_publisher,
                message.storage.as_mut_ptr() as *mut _,
                std::ptr::null_mut(),
            )
            .ok()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    fn assert_send<T: Send>() {}
    fn assert_sync<T: Sync>() {}

    #[test]
    fn dynamic_publisher_is_sync_and_send() {
        assert_send::<DynamicPublisher>();
        assert_sync::<DynamicPublisher>();
    }

    #[test]
    fn test_dynamic_publishers() -> Result<(), RclrsError> {
        use crate::TopicEndpointInfo;
        use test_msgs::msg;

        let namespace = "/test_dynamic_publishers_graph";
        let graph = construct_test_graph(namespace)?;

        let node_1_empty_publisher = graph
            .node1
            .create_dynamic_publisher("test_msgs/msg/Empty".try_into()?, "graph_test_topic_1")?;
        let topic1 = node_1_empty_publisher.topic_name();
        let node_1_basic_types_publisher = graph.node1.create_dynamic_publisher(
            "test_msgs/msg/BasicTypes".try_into()?,
            "graph_test_topic_2",
        )?;
        let topic2 = node_1_basic_types_publisher.topic_name();
        let node_2_default_publisher = graph
            .node2
            .create_dynamic_publisher("test_msgs/msg/Defaults".try_into()?, "graph_test_topic_3")?;
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
        // Test subscription with static types
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

        // Test subscription with dynamic types
        let _node_1_empty_subscriber = graph.node1.create_dynamic_subscription(
            "test_msgs/msg/Empty".try_into().unwrap(),
            "graph_test_topic_1",
            |_, _| {},
        );
        let _node_1_basic_types_subscriber = graph.node1.create_dynamic_subscription(
            "test_msgs/msg/BasicTypes".try_into().unwrap(),
            "graph_test_topic_2",
            |_, _| {},
        );
        let _node_2_default_subscriber = graph.node2.create_dynamic_subscription(
            "test_msgs/msg/Defaults".try_into().unwrap(),
            "graph_test_topic_3",
            |_, _| {},
        );
        assert_eq!(node_1_empty_publisher.get_subscription_count(), Ok(2));
        assert_eq!(node_1_basic_types_publisher.get_subscription_count(), Ok(2));
        assert_eq!(node_2_default_publisher.get_subscription_count(), Ok(2));

        Ok(())
    }
}
