mod options;
pub use options::*;

mod graph;
pub use graph::*;

mod node_graph_task;
use node_graph_task::*;

use std::{
    cmp::PartialEq,
    ffi::CStr,
    fmt,
    os::raw::c_char,
    sync::{Arc, Mutex},
    time::Duration,
};

use futures::{
    StreamExt,
    channel::mpsc::{unbounded, UnboundedSender},
};

// use async_std::future::timeout;

use rosidl_runtime_rs::Message;

use crate::{
    rcl_bindings::*,
    Client, Clock, ContextHandle, Promise, ParameterBuilder, ParameterInterface,
    ParameterVariant, Parameters, Publisher, QoSProfile, RclrsError, Service,
    Subscription, SubscriptionCallback, SubscriptionAsyncCallback, ServiceCallback,
    ServiceAsyncCallback, ExecutorCommands, TimeSource, ENTITY_LIFECYCLE_MUTEX,
};


use std::io::Write;


/// A processing unit that can communicate with other nodes.
///
/// Nodes are a core concept in ROS 2. Refer to the official ["Understanding ROS 2 nodes"][1]
/// tutorial for an introduction.
///
/// Ownership of the node is shared with all [`Publisher`]s and [`Subscription`]s created from it.
/// That means that even after the node itself is dropped, it will continue to exist and be
/// displayed by e.g. `ros2 topic` as long as its publishers and subscriptions are not dropped.
///
/// # Naming
/// A node has a *name* and a *namespace*.
/// The node namespace will be prefixed to the node name to form the *fully qualified
/// node name*. This is the name that is shown e.g. in `ros2 node list`.
/// Similarly, the node namespace will be prefixed to all names of topics and services
/// created from this node.
///
/// By convention, a node name with a leading underscore marks the node as hidden.
///
/// It's a good idea for node names in the same executable to be unique.
///
/// ## Remapping
/// The namespace and name given when creating the node can be overriden through the command line.
/// In that sense, the parameters to the node creation functions are only the _default_ namespace and
/// name.
/// See also the [official tutorial][1] on the command line arguments for ROS nodes, and the
/// [`Node::namespace()`] and [`Node::name()`] functions for examples.
///
/// ## Rules for valid names
/// The rules for valid node names and node namespaces are explained in
/// [`NodeBuilder::new()`][3] and [`NodeBuilder::namespace()`][4].
///
/// [1]: https://docs.ros.org/en/rolling/Tutorials/Understanding-ROS2-Nodes.html
/// [2]: https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html
/// [3]: crate::NodeBuilder::new
/// [4]: crate::NodeBuilder::namespace
pub struct Node {
    time_source: TimeSource,
    parameter: ParameterInterface,
    commands: Arc<ExecutorCommands>,
    graph_change_action: UnboundedSender<NodeGraphAction>,
    handle: Arc<NodeHandle>,
}

/// This struct manages the lifetime of an `rcl_node_t`, and accounts for its
/// dependency on the lifetime of its `rcl_context_t` by ensuring that this
/// dependency is [dropped after][1] the `rcl_node_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub(crate) struct NodeHandle {
    pub(crate) rcl_node: Mutex<rcl_node_t>,
    pub(crate) context_handle: Arc<ContextHandle>,
}

impl Drop for NodeHandle {
    fn drop(&mut self) {
        let _context_lock = self.context_handle.rcl_context.lock().unwrap();
        let mut rcl_node = self.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe { rcl_node_fini(&mut *rcl_node) };
    }
}

impl Eq for Node {}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.handle, &other.handle)
    }
}

impl fmt::Debug for Node {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        f.debug_struct("Node")
            .field("fully_qualified_name", &self.fully_qualified_name())
            .finish()
    }
}

impl Node {
    /// Returns the clock associated with this node.
    pub fn get_clock(&self) -> Clock {
        self.time_source.get_clock()
    }

    /// Returns the name of the node.
    ///
    /// This returns the name after remapping, so it is not necessarily the same as the name that
    /// was used when creating the node.
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclrsError};
    /// // Without remapping
    /// let context = Context::new([])?;
    /// let node = rclrs::create_node(&context, "my_node")?;
    /// assert_eq!(node.name(), "my_node");
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__node:=your_node"].map(String::from);
    /// let context_r = Context::new(remapping)?;
    /// let node_r = rclrs::create_node(&context_r, "my_node")?;
    /// assert_eq!(node_r.name(), "your_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn name(&self) -> String {
        self.call_string_getter(rcl_node_get_name)
    }

    /// Returns the namespace of the node.
    ///
    /// This returns the namespace after remapping, so it is not necessarily the same as the
    /// namespace that was used when creating the node.
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclrsError};
    /// // Without remapping
    /// let context = Context::new([])?;
    /// let node =
    ///   rclrs::create_node_builder(&context, "my_node")
    ///   .namespace("/my/namespace")
    ///   .build()?;
    /// assert_eq!(node.namespace(), "/my/namespace");
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__ns:=/your_namespace"].map(String::from);
    /// let context_r = Context::new(remapping)?;
    /// let node_r = rclrs::create_node(&context_r, "my_node")?;
    /// assert_eq!(node_r.namespace(), "/your_namespace");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn namespace(&self) -> String {
        self.call_string_getter(rcl_node_get_namespace)
    }

    /// Returns the fully qualified name of the node.
    ///
    /// The fully qualified name of the node is the node namespace combined with the node name.
    /// It is subject to the remappings shown in [`Node::name()`] and [`Node::namespace()`].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclrsError};
    /// let context = Context::new([])?;
    /// let node =
    ///   rclrs::create_node_builder(&context, "my_node")
    ///   .namespace("/my/namespace")
    ///   .build()?;
    /// assert_eq!(node.fully_qualified_name(), "/my/namespace/my_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn fully_qualified_name(&self) -> String {
        self.call_string_getter(rcl_node_get_fully_qualified_name)
    }

    /// Creates a [`Client`][1].
    ///
    /// [1]: crate::Client
    // TODO: make client's lifetime depend on node's lifetime
    pub fn create_client<T>(
        self: &Arc<Self>,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Arc<Client<T>>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        Client::<T>::create(topic, qos, &self)
    }

    /// Creates a [`Publisher`][1].
    ///
    /// [1]: crate::Publisher
    // TODO: make publisher's lifetime depend on node's lifetime
    pub fn create_publisher<T>(
        &self,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Arc<Publisher<T>>, RclrsError>
    where
        T: Message,
    {
        Ok(Arc::new(Publisher::<T>::new(Arc::clone(&self.handle), topic, qos)?))
    }

    /// Creates a [`Service`] with an ordinary callback.
    ///
    /// Even though this takes in a blocking (non-async) function, the callback
    /// may run in parallel with other callbacks. This callback may even run
    /// multiple times simultaneously with different incoming requests.
    ///
    /// Any internal state that needs to be mutated will need to be wrapped in
    /// [`Mutex`] to ensure it is synchronized across multiple simultaneous runs
    /// of the callback. To share internal state outside of the callback you will
    /// need to wrap it in [`Arc`] or `Arc<Mutex<S>>`.
    //
    // TODO(@mxgrey): Add examples showing each supported signature
    pub fn create_service<T, Args>(
        &self,
        topic: &str,
        qos: QoSProfile,
        callback: impl ServiceCallback<T, Args>,
    ) -> Result<Arc<Service<T>>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        Service::<T>::create(
            topic,
            qos,
            callback.into_service_callback(),
            &self.handle,
            &self.commands,
        )
    }

    /// Creates a [`Service`] with an async callback.
    ///
    /// This callback may run in parallel with other callbacks. It may even run
    /// multiple times simultaneously with different incoming requests. This
    /// parallelism will depend on the executor that is being used. When the
    /// callback uses `.await`, it will not block anything else from running.
    ///
    /// Any internal state that needs to be mutated will need to be wrapped in
    /// [`Mutex`] to ensure it is synchronized across multiple runs of the
    /// callback. To share internal state outside of the callback you will need
    /// to wrap it in [`Arc`] (immutable) or `Arc<Mutex<S>>` (mutable).
    //
    // TODO(@mxgrey): Add examples showing each supported signature
    pub fn create_async_service<T, Args>(
        &self,
        topic: &str,
        qos: QoSProfile,
        callback: impl ServiceAsyncCallback<T, Args>,
    ) -> Result<Arc<Service<T>>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        Service::<T>::create(
            topic,
            qos,
            callback.into_service_async_callback(),
            &self.handle,
            &self.commands,
        )
    }

    /// Creates a [`Subscription`] with an ordinary callback.
    ///
    /// Even though this takes in a blocking (non-async) function, the callback
    /// may run in parallel with other callbacks. This callback may even run
    /// multiple times simultaneously with different incoming messages. This
    /// parallelism will depend on the executor that is being used.
    ///
    /// Any internal state that needs to be mutated will need to be wrapped in
    /// [`Mutex`] to ensure it is synchronized across multiple simultaneous runs
    /// of the callback. To share internal state outside of the callback you will
    /// need to wrap it in [`Arc`] or `Arc<Mutex<S>>`.
    //
    // TODO(@mxgrey): Add examples showing each supported signature
    pub fn create_subscription<T, Args>(
        &self,
        topic: &str,
        qos: QoSProfile,
        callback: impl SubscriptionCallback<T, Args>,
    ) -> Result<Arc<Subscription<T>>, RclrsError>
    where
        T: Message,
    {
        Subscription::<T>::create(
            topic,
            qos,
            callback.into_subscription_callback(),
            &self.handle,
            &self.commands,
        )
    }

    /// Creates a [`Subscription`] with an async callback.
    ///
    /// This callback may run in parallel with other callbacks. It may even run
    /// multiple times simultaneously with different incoming messages. This
    /// parallelism will depend on the executor that is being used. When the
    /// callback uses `.await`, it will not block anything else from running.
    ///
    /// Any internal state that needs to be mutated will need to be wrapped in
    /// [`Mutex`] to ensure it is synchronized across multiple runs of the
    /// callback. To share internal state outside of the callback you will need
    /// to wrap it in [`Arc`] or `Arc<Mutex<S>>`.
    //
    // TODO(@mxgrey): Add examples showing each supported signature
    pub fn create_async_subscription<T, Args>(
        &self,
        topic: &str,
        qos: QoSProfile,
        callback: impl SubscriptionAsyncCallback<T, Args>,
    ) -> Result<Arc<Subscription<T>>, RclrsError>
    where
        T: Message,
    {
        Subscription::<T>::create(
            topic,
            qos,
            callback.into_subscription_async_callback(),
            &self.handle,
            &self.commands,
        )
    }

    /// Returns the ROS domain ID that the node is using.
    ///
    /// The domain ID controls which nodes can send messages to each other, see the [ROS 2 concept article][1].
    /// It can be set through the `ROS_DOMAIN_ID` environment variable.
    ///
    /// [1]: https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclrsError};
    /// // Set default ROS domain ID to 10 here
    /// std::env::set_var("ROS_DOMAIN_ID", "10");
    /// let context = Context::new([])?;
    /// let node = rclrs::create_node(&context, "domain_id_node")?;
    /// let domain_id = node.domain_id();
    /// assert_eq!(domain_id, 10);
    /// # Ok::<(), RclrsError>(())
    /// ```
    // TODO: If node option is supported,
    // add description about this function is for getting actual domain_id
    // and about override of domain_id via node option
    pub fn domain_id(&self) -> usize {
        let rcl_node = self.handle.rcl_node.lock().unwrap();
        let mut domain_id: usize = 0;
        let ret = unsafe {
            // SAFETY: No preconditions for this function.
            rcl_node_get_domain_id(&*rcl_node, &mut domain_id)
        };

        debug_assert_eq!(ret, 0);
        domain_id
    }

    /// Creates a [`ParameterBuilder`] that can be used to set parameter declaration options and
    /// declare a parameter as [`OptionalParameter`](crate::parameter::OptionalParameter),
    /// [`MandatoryParameter`](crate::parameter::MandatoryParameter), or
    /// [`ReadOnly`](crate::parameter::ReadOnlyParameter).
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, ParameterRange, RclrsError};
    /// let context = Context::new([])?;
    /// let node = rclrs::create_node(&context, "domain_id_node")?;
    /// // Set it to a range of 0-100, with a step of 2
    /// let range = ParameterRange {
    ///     lower: Some(0),
    ///     upper: Some(100),
    ///     step: Some(2),
    /// };
    /// let param = node.declare_parameter("int_param")
    ///                 .default(10)
    ///                 .range(range)
    ///                 .mandatory()
    ///                 .unwrap();
    /// assert_eq!(param.get(), 10);
    /// param.set(50).unwrap();
    /// assert_eq!(param.get(), 50);
    /// // Out of range, will return an error
    /// assert!(param.set(200).is_err());
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn declare_parameter<'a, T: ParameterVariant + 'a>(
        &'a self,
        name: impl Into<Arc<str>>,
    ) -> ParameterBuilder<'a, T> {
        self.parameter.declare(name.into())
    }

    /// Enables usage of undeclared parameters for this node.
    ///
    /// Returns a [`Parameters`] struct that can be used to get and set all parameters.
    pub fn use_undeclared_parameters(&self) -> Parameters {
        self.parameter.allow_undeclared();
        Parameters {
            interface: &self.parameter,
        }
    }

    /// Same as [`Self::notify_on_graph_change_with_period`] but uses a
    /// recommended default period of 100ms.
    pub fn notify_on_graph_change(
        &self,
        condition: impl FnMut() -> bool + Send + 'static,
    ) -> Promise<()> {
        self.notify_on_graph_change_with_period(Duration::from_millis(100), condition)
    }

    /// This function allows you to track when a specific graph change happens.
    ///
    /// Provide a function that will be called each time a graph change occurs.
    /// You will be given a [`Promise`] that will be fulfilled when the condition
    /// returns true. The condition will be checked under these conditions:
    /// - once immediately as this function is run
    /// - each time rcl notifies us that a graph change has happened
    /// - each time the period elapses
    ///
    /// We specify a period because it is possible that race conditions at the
    /// rcl layer could trigger a notification of a graph change before your
    /// API calls will be able to observe it.
    ///
    ///
    pub fn notify_on_graph_change_with_period(
        &self,
        period: Duration,
        mut condition: impl FnMut() -> bool + Send + 'static,
    ) -> Promise<()> {
        let (listener, mut on_graph_change_receiver) = unbounded();
        let promise = self.commands.query(async move {
            dbg!();
            std::io::stdout().lock().flush().unwrap();
            loop {
                // match timeout(period, on_graph_change_receiver.next()).await {
                //     Ok(Some(_)) | Err(_) => {
                //         // Either we received a notification that there was a
                //         // graph change, or the timeout elapsed. Either way, we
                //         // want to check the condition and break out of the loop
                //         // if the condition is true.
                //         if condition() {
                //             return;
                //         }
                //     }
                //     Ok(None) => {
                //         // We've been notified that the graph change sender is
                //         // closed which means we will never receive another
                //         // graph change update. This only happens when a node
                //         // is being torn down, so go ahead and exit this loop.
                //         return;
                //     }
                // }

                match on_graph_change_receiver.next().await {
                    Some(_) => {
                        dbg!();
                        std::io::stdout().lock().flush().unwrap();
                        if condition() {
                            // Condition is met
                            dbg!();
                            std::io::stdout().lock().flush().unwrap();
                            return;
                        }
                    }
                    None => {
                        dbg!();
                        std::io::stdout().lock().flush().unwrap();
                        // Graph change sender is closed
                        return;
                    }
                }
            }
        });

        self.graph_change_action
            .unbounded_send(NodeGraphAction::NewGraphListener(listener))
            .ok();

        promise
    }

    /// Get the [`ExecutorCommands`] used by this Node.
    pub fn commands(&self) -> &Arc<ExecutorCommands> {
        &self.commands
    }

    // Helper for name(), namespace(), fully_qualified_name()
    fn call_string_getter(
        &self,
        getter: unsafe extern "C" fn(*const rcl_node_t) -> *const c_char,
    ) -> String {
        let rcl_node = self.handle.rcl_node.lock().unwrap();
        unsafe { call_string_getter_with_rcl_node(&rcl_node, getter) }
    }

    pub(crate) fn handle(&self) -> &Arc<NodeHandle> {
        &self.handle
    }
}

// Helper used to implement call_string_getter(), but also used to get the FQN in the Node::new()
// function, which is why it's not merged into Node::call_string_getter().
// This function is unsafe since it's possible to pass in an rcl_node_t with dangling
// pointers etc.
pub(crate) unsafe fn call_string_getter_with_rcl_node(
    rcl_node: &rcl_node_t,
    getter: unsafe extern "C" fn(*const rcl_node_t) -> *const c_char,
) -> String {
    let char_ptr = getter(rcl_node);
    debug_assert!(!char_ptr.is_null());
    // SAFETY: The returned CStr is immediately converted to an owned string,
    // so the lifetime is no issue. The ptr is valid as per the documentation
    // of rcl_node_get_name.
    let cstr = CStr::from_ptr(char_ptr);
    cstr.to_string_lossy().into_owned()
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_node_t {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn traits() {
        assert_send::<Node>();
        assert_sync::<Node>();
    }

    #[test]
    fn test_topic_names_and_types() -> Result<(), RclrsError> {
        use crate::QOS_PROFILE_SYSTEM_DEFAULT;
        use test_msgs::msg;

        let graph = construct_test_graph("test_topics_graph")?;

        let _node_1_defaults_subscription = graph.node1.create_subscription::<msg::Defaults, _>(
            "graph_test_topic_3",
            QOS_PROFILE_SYSTEM_DEFAULT,
            |_msg: msg::Defaults| {},
        )?;
        let _node_2_empty_subscription = graph.node2.create_subscription::<msg::Empty, _>(
            "graph_test_topic_1",
            QOS_PROFILE_SYSTEM_DEFAULT,
            |_msg: msg::Empty| {},
        )?;
        let _node_2_basic_types_subscription =
            graph.node2.create_subscription::<msg::BasicTypes, _>(
                "graph_test_topic_2",
                QOS_PROFILE_SYSTEM_DEFAULT,
                |_msg: msg::BasicTypes| {},
            )?;

        std::thread::sleep(std::time::Duration::from_millis(100));

        let topic_names_and_types = graph.node1.get_topic_names_and_types()?;

        let types = topic_names_and_types
            .get("/test_topics_graph/graph_test_topic_1")
            .unwrap();
        assert!(types.contains(&"test_msgs/msg/Empty".to_string()));
        let types = topic_names_and_types
            .get("/test_topics_graph/graph_test_topic_2")
            .unwrap();
        assert!(types.contains(&"test_msgs/msg/BasicTypes".to_string()));

        let types = topic_names_and_types
            .get("/test_topics_graph/graph_test_topic_3")
            .unwrap();
        assert!(types.contains(&"test_msgs/msg/Defaults".to_string()));

        Ok(())
    }
}
