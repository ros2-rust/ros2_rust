mod builder;
mod graph;
use std::{
    cmp::PartialEq,
    ffi::CStr,
    fmt,
    os::raw::c_char,
    sync::{Arc, Mutex, Weak, atomic::AtomicBool},
    vec::Vec,
};

use rosidl_runtime_rs::Message;

pub use self::{builder::*, graph::*};
use crate::{
    rcl_bindings::*, Client, ClientBase, Clock, Context, ContextHandle, GuardCondition,
    ParameterBuilder, ParameterInterface, ParameterVariant, Parameters, Publisher, QoSProfile,
    RclrsError, Service, ServiceBase, Subscription, SubscriptionBase, SubscriptionCallback,
    TimeSource, ENTITY_LIFECYCLE_MUTEX,
};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_node_t {}

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
/// The namespace and name given when creating the node can be overridden through the command line.
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
    pub(crate) clients_mtx: Mutex<Vec<Weak<dyn ClientBase>>>,
    pub(crate) guard_conditions_mtx: Mutex<Vec<Weak<GuardCondition>>>,
    pub(crate) services_mtx: Mutex<Vec<Weak<dyn ServiceBase>>>,
    pub(crate) subscriptions_mtx: Mutex<Vec<Weak<dyn SubscriptionBase>>>,
    time_source: TimeSource,
    parameter: ParameterInterface,
    pub(crate) handle: Arc<NodeHandle>,
}

/// This struct manages the lifetime of an `rcl_node_t`, and accounts for its
/// dependency on the lifetime of its `rcl_context_t` by ensuring that this
/// dependency is [dropped after][1] the `rcl_node_t`.
/// Note: we capture the rcl_node_t returned from rcl_get_zero_initialized_node()
/// to guarantee that the node handle exists until we drop the NodeHandle
/// instance. This addresses an issue where previously the address of the variable
/// in the builder.rs was being used, and whose lifespan was (just) shorter than the
/// NodeHandle instance.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub(crate) struct NodeHandle {
    pub(crate) rcl_node: Mutex<rcl_node_t>,
    pub(crate) context_handle: Arc<ContextHandle>,
    /// In the humbe distro, rcl is sensitive to the address of the rcl_node_t
    /// object being moved (this issue seems to be gone in jazzy), so we need
    /// to initialize the rcl_node_t in-place inside this struct. In the event
    /// that the initialization fails (e.g. it was created with an invalid name)
    /// we need to make sure that we do not call rcl_node_fini on it while
    /// dropping the NodeHandle, so we keep track of successful initialization
    /// with this variable.
    ///
    /// We may be able to restructure this in the future when we no longer need
    /// to support Humble.
    pub(crate) initialized: AtomicBool,
}

impl Drop for NodeHandle {
    fn drop(&mut self) {
        if !self.initialized.load(std::sync::atomic::Ordering::Acquire) {
            // The node was not correctly initialized, e.g. it was created with
            // an invalid name, so we must not try to finalize it or else we
            // will get undefined behavior.
            return;
        }

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
    /// Creates a new node in the empty namespace.
    ///
    /// See [`NodeBuilder::new()`] for documentation.
    #[allow(clippy::new_ret_no_self)]
    pub fn new(context: &Context, node_name: &str) -> Result<Arc<Node>, RclrsError> {
        Self::builder(context, node_name).build()
    }

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

    // Helper for name(), namespace(), fully_qualified_name()
    fn call_string_getter(
        &self,
        getter: unsafe extern "C" fn(*const rcl_node_t) -> *const c_char,
    ) -> String {
        let rcl_node = self.handle.rcl_node.lock().unwrap();
        unsafe { call_string_getter_with_rcl_node(&rcl_node, getter) }
    }

    /// Creates a [`Client`][1].
    ///
    /// [1]: crate::Client
    // TODO: make client's lifetime depend on node's lifetime
    pub fn create_client<T>(&self, topic: &str) -> Result<Arc<Client<T>>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        let client = Arc::new(Client::<T>::new(Arc::clone(&self.handle), topic)?);
        { self.clients_mtx.lock().unwrap() }.push(Arc::downgrade(&client) as Weak<dyn ClientBase>);
        Ok(client)
    }

    /// Creates a [`GuardCondition`][1] with no callback.
    ///
    /// A weak pointer to the `GuardCondition` is stored within this node.
    /// When this node is added to a wait set (e.g. when calling `spin_once`[2]
    /// with this node as an argument), the guard condition can be used to
    /// interrupt the wait.
    ///
    /// [1]: crate::GuardCondition
    /// [2]: crate::spin_once
    pub fn create_guard_condition(&self) -> Arc<GuardCondition> {
        let guard_condition = Arc::new(GuardCondition::new_with_context_handle(
            Arc::clone(&self.handle.context_handle),
            None,
        ));
        { self.guard_conditions_mtx.lock().unwrap() }
            .push(Arc::downgrade(&guard_condition) as Weak<GuardCondition>);
        guard_condition
    }

    /// Creates a [`GuardCondition`][1] with a callback.
    ///
    /// A weak pointer to the `GuardCondition` is stored within this node.
    /// When this node is added to a wait set (e.g. when calling `spin_once`[2]
    /// with this node as an argument), the guard condition can be used to
    /// interrupt the wait.
    ///
    /// [1]: crate::GuardCondition
    /// [2]: crate::spin_once
    pub fn create_guard_condition_with_callback<F>(&mut self, callback: F) -> Arc<GuardCondition>
    where
        F: Fn() + Send + Sync + 'static,
    {
        let guard_condition = Arc::new(GuardCondition::new_with_context_handle(
            Arc::clone(&self.handle.context_handle),
            Some(Box::new(callback) as Box<dyn Fn() + Send + Sync>),
        ));
        { self.guard_conditions_mtx.lock().unwrap() }
            .push(Arc::downgrade(&guard_condition) as Weak<GuardCondition>);
        guard_condition
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
        let publisher = Arc::new(Publisher::<T>::new(Arc::clone(&self.handle), topic, qos)?);
        Ok(publisher)
    }

    /// Creates a [`Service`][1].
    ///
    /// [1]: crate::Service
    // TODO: make service's lifetime depend on node's lifetime
    pub fn create_service<T, F>(
        &self,
        topic: &str,
        callback: F,
    ) -> Result<Arc<Service<T>>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
        F: Fn(&rmw_request_id_t, T::Request) -> T::Response + 'static + Send,
    {
        let service = Arc::new(Service::<T>::new(
            Arc::clone(&self.handle),
            topic,
            callback,
        )?);
        { self.services_mtx.lock().unwrap() }
            .push(Arc::downgrade(&service) as Weak<dyn ServiceBase>);
        Ok(service)
    }

    /// Creates a [`Subscription`][1].
    ///
    /// [1]: crate::Subscription
    // TODO: make subscription's lifetime depend on node's lifetime
    pub fn create_subscription<T, Args>(
        &self,
        topic: &str,
        qos: QoSProfile,
        callback: impl SubscriptionCallback<T, Args>,
    ) -> Result<Arc<Subscription<T>>, RclrsError>
    where
        T: Message,
    {
        let subscription = Arc::new(Subscription::<T>::new(
            Arc::clone(&self.handle),
            topic,
            qos,
            callback,
        )?);
        { self.subscriptions_mtx.lock() }
            .unwrap()
            .push(Arc::downgrade(&subscription) as Weak<dyn SubscriptionBase>);
        Ok(subscription)
    }

    /// Returns the subscriptions that have not been dropped yet.
    pub(crate) fn live_subscriptions(&self) -> Vec<Arc<dyn SubscriptionBase>> {
        { self.subscriptions_mtx.lock().unwrap() }
            .iter()
            .filter_map(Weak::upgrade)
            .collect()
    }

    pub(crate) fn live_clients(&self) -> Vec<Arc<dyn ClientBase>> {
        { self.clients_mtx.lock().unwrap() }
            .iter()
            .filter_map(Weak::upgrade)
            .collect()
    }

    pub(crate) fn live_guard_conditions(&self) -> Vec<Arc<GuardCondition>> {
        { self.guard_conditions_mtx.lock().unwrap() }
            .iter()
            .filter_map(Weak::upgrade)
            .collect()
    }

    pub(crate) fn live_services(&self) -> Vec<Arc<dyn ServiceBase>> {
        { self.services_mtx.lock().unwrap() }
            .iter()
            .filter_map(Weak::upgrade)
            .collect()
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

    /// Creates a [`NodeBuilder`][1] with the given name.
    ///
    /// Convenience function equivalent to [`NodeBuilder::new()`][2].
    ///
    /// [1]: crate::NodeBuilder
    /// [2]: crate::NodeBuilder::new
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, Node, RclrsError};
    /// let context = Context::new([])?;
    /// let node = Node::builder(&context, "my_node").build()?;
    /// assert_eq!(node.name(), "my_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn builder(context: &Context, node_name: &str) -> NodeBuilder {
        NodeBuilder::new(context, node_name)
    }

    /// Returns the logger name of the node.
    pub fn logger_name(&self) -> &str {
        let rcl_node = self.handle.rcl_node.lock().unwrap();
        // SAFETY: No pre-conditions for this function
        let name_raw_ptr = unsafe { rcl_node_get_logger_name(&*rcl_node) };
        if name_raw_ptr.is_null() {
            return "";
        }
        // SAFETY: The returned CStr is immediately converted to a string slice,
        // so the lifetime is no issue. The ptr is valid as per the documentation
        // of rcl_node_get_logger_name.
        let name_cstr = unsafe { CStr::from_ptr(name_raw_ptr) };
        name_cstr.to_str().unwrap_or("")
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

    #[test]
    fn test_logger_name() -> Result<(), RclrsError> {
        // Use helper to create 2 nodes for us
        let graph = construct_test_graph("test_topics_graph")?;

        assert_eq!(
            graph.node1.logger_name(),
            "test_topics_graph.graph_test_node_1"
        );
        assert_eq!(
            graph.node2.logger_name(),
            "test_topics_graph.graph_test_node_2"
        );

        Ok(())
    }
}
