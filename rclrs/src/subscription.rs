use std::{
    ffi::{CStr, CString},
    sync::{Arc, Mutex, MutexGuard},
};

use rosidl_runtime_rs::{Message, RmwMessage};

use futures::channel::mpsc::{unbounded, UnboundedSender, TrySendError};

use crate::{
    error::ToResult,
    qos::QoSProfile,
    rcl_bindings::*,
    ExecutorCommands, NodeHandle, RclrsError, Waitable, Executable, ExecutableHandle,
    ExecutableKind, GuardCondition, WaiterLifecycle, ENTITY_LIFECYCLE_MUTEX,
};

mod any_subscription_callback;
pub use any_subscription_callback::*;

mod subscription_async_callback;
pub use subscription_async_callback::*;

mod subscription_callback;
pub use subscription_callback::*;

mod message_info;
pub use message_info::*;

mod readonly_loaned_message;
pub use readonly_loaned_message::*;

mod subscription_task;
use subscription_task::*;

/// Struct for receiving messages of type `T`.
///
/// The only way to instantiate a subscription is via [`Node::create_subscription()`][2]
/// or [`Node::create_async_subscription`][3].
///
/// There can be multiple subscriptions for the same topic, in different nodes or the same node.
///
/// Receiving messages requires calling [`spin`][1] on the `Executor` of subscription's [Node][4].
///
/// When a subscription is created, it may take some time to get "matched" with a corresponding
/// publisher.
///
/// [1]: crate::Executor::spin
/// [2]: crate::Node::create_subscription
/// [3]: crate::Node::create_async_subscription
/// [4]: crate::Node
pub struct Subscription<T>
where
    T: Message,
{
    /// This handle is used to access the data that rcl holds for this subscription.
    handle: Arc<SubscriptionHandle>,
    /// This allows us to replace the callback in the subscription task.
    ///
    /// Holding onto this sender will keep the subscription task alive. Once
    /// this sender is dropped, the subscription task will end itself.
    action: UnboundedSender<SubscriptionAction<T>>,
    /// Holding onto this keeps the waiter for this subscription alive in the
    /// wait set of the executor.
    lifecycle: WaiterLifecycle,
}

impl<T> Subscription<T>
where
    T: Message,
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

    /// Set the callback of this subscription, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the subscription previously used an async callback.
    pub fn set_callback<Args>(
        &self,
        callback: impl SubscriptionCallback<T, Args>,
    ) -> Result<(), TrySendError<SubscriptionAction<T>>> {
        let callback = callback.into_subscription_callback();
        self.action.unbounded_send(SubscriptionAction::SetCallback(callback))
    }

    /// Set the callback of this subscription, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the subscription previously used a non-async callback.
    pub fn set_async_callback<Args>(
        &self,
        callback: impl SubscriptionAsyncCallback<T, Args>,
    ) -> Result<(), TrySendError<SubscriptionAction<T>>> {
        let callback = callback.into_subscription_async_callback();
        self.action.unbounded_send(SubscriptionAction::SetCallback(callback))
    }

    /// Used by [`Node`][crate::Node] to create a new subscription.
    pub(crate) fn create(
        topic: &str,
        qos: QoSProfile,
        callback: AnySubscriptionCallback<T>,
        node_handle: &Arc<NodeHandle>,
        commands: &Arc<ExecutorCommands>,
    ) -> Result<Arc<Self>, RclrsError> {
        let (sender, receiver) = unbounded();
        let (subscription, waiter) = Self::new(
            topic,
            qos,
            sender,
            Arc::clone(&node_handle),
            Arc::clone(commands.get_guard_condition()),
        )?;

        commands.run_detached(subscription_task(
            callback,
            receiver,
            Arc::clone(&subscription.handle),
            Arc::clone(&commands),
        ));

        commands.add_to_wait_set(waiter);

        Ok(subscription)
    }

    /// Instantiate the Subscription.
    fn new(
        topic: &str,
        qos: QoSProfile,
        action: UnboundedSender<SubscriptionAction<T>>,
        node_handle: Arc<NodeHandle>,
        guard_condition: Arc<GuardCondition>,
    ) -> Result<(Arc<Self>, Waitable), RclrsError>
    // This uses pub(crate) visibility to avoid instantiating this struct outside
    // [`Node::create_subscription`], see the struct's documentation for the rationale
    where
        T: Message,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let mut subscription_options = unsafe { rcl_subscription_get_default_options() };
        subscription_options.qos = qos.into();

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
                    &subscription_options,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(SubscriptionHandle {
            rcl_subscription: Mutex::new(rcl_subscription),
            node_handle,
        });

        let (waiter, lifecycle) = Waitable::new(
            Box::new(SubscriptionWaitable {
                handle: Arc::clone(&handle),
                action: action.clone(),
            }),
            Some(guard_condition),
        );

        let subscription = Arc::new(Self { handle, action, lifecycle });

        Ok((subscription, waiter))
    }
}

struct SubscriptionWaitable<T: Message> {
    handle: Arc<SubscriptionHandle>,
    action: UnboundedSender<SubscriptionAction<T>>,
}

impl<T> Executable for SubscriptionWaitable<T>
where
    T: Message,
{
    fn execute(&mut self) -> Result<(), RclrsError> {
        self.action.unbounded_send(SubscriptionAction::Execute).ok();
        Ok(())
    }

    fn kind(&self) -> crate::ExecutableKind {
        ExecutableKind::Subscription
    }

    fn handle(&self) -> ExecutableHandle {
        ExecutableHandle::Subscription(self.handle.lock())
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
    use crate::test_helpers::*;
    use test_msgs::msg;

    #[test]
    fn traits() {
        assert_send::<Subscription<msg::BoundedSequences>>();
        assert_sync::<Subscription<msg::BoundedSequences>>();
    }

    #[test]
    fn test_subscriptions() -> Result<(), RclrsError> {
        use crate::{TopicEndpointInfo, QOS_PROFILE_SYSTEM_DEFAULT};

        let namespace = "/test_subscriptions_graph";
        let graph = construct_test_graph(namespace)?;

        let node_2_empty_subscription = graph.node2.create_subscription::<msg::Empty, _>(
            "graph_test_topic_1",
            QOS_PROFILE_SYSTEM_DEFAULT,
            |_msg: msg::Empty| {},
        )?;
        let topic1 = node_2_empty_subscription.topic_name();
        let node_2_basic_types_subscription =
            graph.node2.create_subscription::<msg::BasicTypes, _>(
                "graph_test_topic_2",
                QOS_PROFILE_SYSTEM_DEFAULT,
                |_msg: msg::BasicTypes| {},
            )?;
        let topic2 = node_2_basic_types_subscription.topic_name();
        let node_1_defaults_subscription = graph.node1.create_subscription::<msg::Defaults, _>(
            "graph_test_topic_3",
            QOS_PROFILE_SYSTEM_DEFAULT,
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
}
