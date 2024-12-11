mod node_options;
pub use node_options::*;

mod primitive_options;
pub use primitive_options::*;

mod graph;
pub use graph::*;

mod node_graph_task;
use node_graph_task::*;

use std::{
    cmp::PartialEq,
    ffi::CStr,
    fmt,
    os::raw::c_char,
    sync::{atomic::AtomicBool, Arc, Mutex},
    time::Duration,
};

use futures::{
    channel::mpsc::{unbounded, UnboundedSender},
    StreamExt,
};

use async_std::future::timeout;

use rosidl_runtime_rs::Message;

use crate::{
    rcl_bindings::*, Client, ClientOptions, ClientState, Clock, ContextHandle, ExecutorCommands,
    LogParams, Logger, ParameterBuilder, ParameterInterface, ParameterVariant, Parameters, Promise,
    Publisher, PublisherOptions, PublisherState, RclrsError, Service, IntoAsyncServiceCallback,
    IntoNodeServiceCallback, ServiceOptions, ServiceState, Subscription, IntoAsyncSubscriptionCallback,
    IntoNodeSubscriptionCallback, SubscriptionOptions, SubscriptionState, TimeSource, ToLogParams,
    ENTITY_LIFECYCLE_MUTEX, IntoWorkerOptions, Worker, WorkerState,
};

/// A processing unit that can communicate with other nodes.
///
/// Nodes are a core concept in ROS 2. Refer to the official ["Understanding ROS 2 nodes"][1]
/// tutorial for an introduction.
///
/// Ownership of the node is shared with all the primitives such as [`Publisher`]s and [`Subscription`]s
/// that are created from it. That means that even after the `Node` itself is dropped, it will continue
/// to exist and be displayed by e.g. `ros2 topic` as long as any one of its primitives is not dropped.
///
/// # Creating
/// Use [`Executor::create_node`][7] to create a new node. Pass in [`NodeOptions`] to set all the different
/// options for node creation, or just pass in a string for the node's name if the default options are okay.
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
/// [`Node::namespace()`][3] and [`Node::name()`][4] functions for examples.
///
/// ## Rules for valid names
/// The rules for valid node names and node namespaces are explained in
/// [`NodeOptions::new()`][5] and [`NodeOptions::namespace()`][6].
///
/// [1]: https://docs.ros.org/en/rolling/Tutorials/Understanding-ROS2-Nodes.html
/// [2]: https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html
/// [3]: Node::namespace
/// [4]: Node::name
/// [5]: crate::NodeOptions::new
/// [6]: crate::NodeOptions::namespace
/// [7]: crate::Executor::create_node

pub type Node = Arc<NodeState>;

/// The inner state of a [`Node`].
///
/// This is public so that you can choose to put it inside a [`Weak`] if you
/// want to be able to refer to a [`Node`] in a non-owning way. It is generally
/// recommended to manage the [`NodeState`] inside of an [`Arc`], and [`Node`]
/// recommended to manage the `NodeState` inside of an [`Arc`], and [`Node`]
/// is provided as convenience alias for that.
///
/// The public API of the [`Node`] type is implemented via `NodeState`.
pub struct NodeState {
    time_source: TimeSource,
    parameter: ParameterInterface,
    logger: Logger,
    commands: Arc<ExecutorCommands>,
    graph_change_action: UnboundedSender<NodeGraphAction>,
    handle: Arc<NodeHandle>,
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
    /// In the humble distro, rcl is sensitive to the address of the rcl_node_t
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

impl Eq for NodeState {}

impl PartialEq for NodeState {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.handle, &other.handle)
    }
}

impl fmt::Debug for NodeState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        f.debug_struct("Node")
            .field("fully_qualified_name", &self.fully_qualified_name())
            .finish()
    }
}

impl NodeState {
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
    /// # use rclrs::{Context, InitOptions, RclrsError};
    /// // Without remapping
    /// let executor = Context::default().create_basic_executor();
    /// let node = executor.create_node("my_node")?;
    /// assert_eq!(node.name(), "my_node");
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__node:=your_node"].map(String::from);
    /// let executor_r = Context::new(remapping, InitOptions::default())?.create_basic_executor();
    /// let node_r = executor_r.create_node("my_node")?;
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
    /// # use rclrs::{Context, InitOptions, RclrsError, IntoNodeOptions};
    /// // Without remapping
    /// let executor = Context::default().create_basic_executor();
    /// let node = executor.create_node(
    ///     "my_node"
    ///     .namespace("/my/namespace")
    /// )?;
    /// assert_eq!(node.namespace(), "/my/namespace");
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__ns:=/your_namespace"].map(String::from);
    /// let executor_r = Context::new(remapping, InitOptions::default())?.create_basic_executor();
    /// let node_r = executor_r.create_node("my_node")?;
    /// assert_eq!(node_r.namespace(), "/your_namespace");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn namespace(&self) -> String {
        self.call_string_getter(rcl_node_get_namespace)
    }

    /// Returns the fully qualified name of the node.
    ///
    /// The fully qualified name of the node is the node namespace combined with the node name.
    /// It is subject to the remappings shown in [`NodeState::name()`] and [`NodeState::namespace()`].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclrsError, IntoNodeOptions};
    /// let executor = Context::default().create_basic_executor();
    /// let node = executor.create_node(
    ///     "my_node"
    ///     .namespace("/my/namespace")
    /// )?;
    /// assert_eq!(node.fully_qualified_name(), "/my/namespace/my_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn fully_qualified_name(&self) -> String {
        self.call_string_getter(rcl_node_get_fully_qualified_name)
    }

    /// Create a new [`Worker`] for this Node.
    //
    // TODO(@mxgrey): Write some usage examples.
    pub fn create_worker<'a, Payload>(
        &self,
        options: impl IntoWorkerOptions<Payload>,
    ) -> Worker<Payload>
    where
        Payload: 'static + Send,
    {
        let options = options.into_worker_options();
        let commands = self.commands.create_worker_commands(Box::new(options.payload));
        WorkerState::create(Arc::clone(&self.handle), commands)
    }

    /// Creates a [`Client`][1].
    ///
    /// Pass in only the service name for the `options` argument to use all default client options:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let client = node.create_client::<test_msgs::srv::Empty>(
    ///     "my_service"
    /// )
    /// .unwrap();
    /// ```
    ///
    /// Take advantage of the [`IntoPrimitiveOptions`] API to easily build up the
    /// client options:
    ///
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let client = node.create_client::<test_msgs::srv::Empty>(
    ///     "my_service"
    ///     .keep_all()
    ///     .transient_local()
    /// )
    /// .unwrap();
    /// ```
    ///
    /// Any quality of service options that you explicitly specify will override
    /// the default service options. Any that you do not explicitly specify will
    /// remain the default service options. Note that clients are generally
    /// expected to use [reliable][1], so it's best not to change the reliability
    /// setting unless you know what you are doing.
    ///
    /// [1]: crate::QoSReliabilityPolicy::Reliable
    pub fn create_client<'a, T>(
        self: &Arc<Self>,
        options: impl Into<ClientOptions<'a>>,
    ) -> Result<Client<T>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        ClientState::<T>::create(options, self)
    }

    /// Creates a [`Publisher`][1].
    ///
    /// Pass in only the topic name for the `options` argument to use all default publisher options:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let publisher = node.create_publisher::<test_msgs::msg::Empty>(
    ///     "my_topic"
    /// )
    /// .unwrap();
    /// ```
    ///
    /// Take advantage of the [`IntoPrimitiveOptions`] API to easily build up the
    /// publisher options:
    ///
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let publisher = node.create_publisher::<test_msgs::msg::Empty>(
    ///     "my_topic"
    ///     .keep_last(100)
    ///     .transient_local()
    /// )
    /// .unwrap();
    ///
    /// let reliable_publisher = node.create_publisher::<test_msgs::msg::Empty>(
    ///     "my_topic"
    ///     .reliable()
    /// )
    /// .unwrap();
    /// ```
    ///
    pub fn create_publisher<'a, T>(
        &self,
        options: impl Into<PublisherOptions<'a>>,
    ) -> Result<Publisher<T>, RclrsError>
    where
        T: Message,
    {
        PublisherState::<T>::create(options, Arc::clone(&self.handle))
    }

    /// Creates a [`Service`] with an ordinary callback.
    ///
    /// # Behavior
    ///
    /// Even though this takes in a blocking (non-async) function, the callback
    /// may run in parallel with other callbacks. This callback may even run
    /// multiple times simultaneously with different incoming requests.
    ///
    /// Any internal state that needs to be mutated will need to be wrapped in
    /// [`Mutex`] to ensure it is synchronized across multiple simultaneous runs
    /// of the callback. To share internal state outside of the callback you will
    /// need to wrap it in [`Arc`] or `Arc<Mutex<S>>`.
    ///
    /// # Usage
    ///
    /// Pass in only the service name for the `options` argument to use all default service options:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let service = node.create_service::<test_msgs::srv::Empty, _>(
    ///     "my_service",
    ///     |_request: test_msgs::srv::Empty_Request| {
    ///         println!("Received request!");
    ///         test_msgs::srv::Empty_Response::default()
    ///     },
    /// );
    /// ```
    ///
    /// Take advantage of the [`IntoPrimitiveOptions`] API to easily build up the
    /// service options:
    ///
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let service = node.create_service::<test_msgs::srv::Empty, _>(
    ///     "my_service"
    ///     .keep_all()
    ///     .transient_local(),
    ///     |_request: test_msgs::srv::Empty_Request| {
    ///         println!("Received request!");
    ///         test_msgs::srv::Empty_Response::default()
    ///     },
    /// );
    /// ```
    ///
    /// Any quality of service options that you explicitly specify will override
    /// the default service options. Any that you do not explicitly specify will
    /// remain the default service options. Note that services are generally
    /// expected to use [reliable][2], so it's best not to change the reliability
    /// setting unless you know what you are doing.
    ///
    /// [1]: crate::Service
    /// [2]: crate::QoSReliabilityPolicy::Reliable
    //
    // TODO(@mxgrey): Add examples showing each supported signature
    pub fn create_service<'a, T, Args>(
        &self,
        options: impl Into<ServiceOptions<'a>>,
        callback: impl IntoNodeServiceCallback<T, Args>,
    ) -> Result<Service<T>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        ServiceState::<T, Node>::create(
            options,
            callback.into_node_service_callback(),
            &self.handle,
            self.commands.async_worker_commands(),
        )
    }

    /// Creates a [`Service`] with an async callback.
    ///
    /// # Behavior
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
    ///
    /// # Usage
    ///
    /// See [create_service][Node::create_service#Usage] for usage.
    //
    // TODO(@mxgrey): Add examples showing each supported signature
    pub fn create_async_service<'a, T, Args>(
        &self,
        options: impl Into<ServiceOptions<'a>>,
        callback: impl IntoAsyncServiceCallback<T, Args>,
    ) -> Result<Service<T>, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        ServiceState::<T, Node>::create(
            options,
            callback.into_async_service_callback(),
            &self.handle,
            self.commands.async_worker_commands(),
        )
    }

    /// Creates a [`Subscription`] with an ordinary callback.
    ///
    /// # Behavior
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
    ///
    /// # Usage
    ///
    /// Pass in only the topic name for the `options` argument to use all default subscription options:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let subscription = node.create_subscription(
    ///     "my_topic",
    ///     |_msg: test_msgs::msg::Empty| {
    ///         println!("Received message!");
    ///     },
    /// );
    /// ```
    ///
    /// Take advantage of the [`IntoPrimitiveOptions`] API to easily build up the
    /// subscription options:
    ///
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let subscription = node.create_subscription(
    ///     "my_topic"
    ///     .keep_last(100)
    ///     .transient_local(),
    ///     |_msg: test_msgs::msg::Empty| {
    ///         println!("Received message!");
    ///     },
    /// );
    ///
    /// let reliable_subscription = node.create_subscription(
    ///     "my_reliable_topic"
    ///     .reliable(),
    ///     |_msg: test_msgs::msg::Empty| {
    ///         println!("Received message!");
    ///     },
    /// );
    /// ```
    ///
    //
    // TODO(@mxgrey): Add examples showing each supported callback signatures
    pub fn create_subscription<'a, T, Args>(
        &self,
        options: impl Into<SubscriptionOptions<'a>>,
        callback: impl IntoNodeSubscriptionCallback<T, Args>,
    ) -> Result<Subscription<T>, RclrsError>
    where
        T: Message,
    {
        SubscriptionState::<T, Node>::create(
            options,
            callback.into_node_subscription_callback(),
            &self.handle,
            self.commands.async_worker_commands(),
        )
    }

    /// Creates a [`Subscription`] with an async callback.
    ///
    /// # Behavior
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
    ///
    /// # Usage
    ///
    /// See [create_subscription][Node::create_subscription#Usage] for usage.
    //
    // TODO(@mxgrey): Add examples showing each supported signature
    pub fn create_async_subscription<'a, T, Args>(
        &self,
        options: impl Into<SubscriptionOptions<'a>>,
        callback: impl IntoAsyncSubscriptionCallback<T, Args>,
    ) -> Result<Subscription<T>, RclrsError>
    where
        T: Message,
    {
        SubscriptionState::<T, Node>::create(
            options,
            callback.into_async_subscription_callback(),
            &self.handle,
            self.commands.async_worker_commands(),
        )
    }

    /// Returns the ROS domain ID that the node is using.
    ///
    /// The domain ID controls which nodes can send messages to each other, see the [ROS 2 concept article][1].
    /// It can be set through the `ROS_DOMAIN_ID` environment variable or by
    /// passing custom [`InitOptions`][2] into [`Context::new`][3] or [`Context::from_env`][4].
    ///
    /// [1]: https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html
    /// [2]: crate::InitOptions
    /// [3]: crate::Context::new
    /// [4]: crate::Context::from_env
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclrsError};
    /// // Set default ROS domain ID to 10 here
    /// std::env::set_var("ROS_DOMAIN_ID", "10");
    /// let executor = Context::default().create_basic_executor();
    /// let node = executor.create_node("domain_id_node")?;
    /// let domain_id = node.domain_id();
    /// assert_eq!(domain_id, 10);
    /// # Ok::<(), RclrsError>(())
    /// ```
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
    /// let executor = Context::default().create_basic_executor();
    /// let node = executor.create_node("domain_id_node")?;
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
            loop {
                match timeout(period, on_graph_change_receiver.next()).await {
                    Ok(Some(_)) | Err(_) => {
                        // Either we received a notification that there was a
                        // graph change, or the timeout elapsed. Either way, we
                        // want to check the condition and break out of the loop
                        // if the condition is true.
                        if condition() {
                            return;
                        }
                    }
                    Ok(None) => {
                        // We've been notified that the graph change sender is
                        // closed which means we will never receive another
                        // graph change update. This only happens when a node
                        // is being torn down, so go ahead and exit this loop.
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

    /// Get the logger associated with this Node.
    pub fn logger(&self) -> &Logger {
        &self.logger
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

impl<'a> ToLogParams<'a> for &'a NodeState {
    fn to_log_params(self) -> LogParams<'a> {
        self.logger().to_log_params()
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
    use crate::{test_helpers::*, *};

    #[test]
    fn traits() {
        assert_send::<NodeState>();
        assert_sync::<NodeState>();
    }

    #[test]
    fn test_topic_names_and_types() -> Result<(), RclrsError> {
        use test_msgs::msg;

        let graph = construct_test_graph("test_topics_graph")?;

        let _node_1_defaults_subscription = graph.node1.create_subscription::<msg::Defaults, _>(
            "graph_test_topic_3",
            |_msg: msg::Defaults| {},
        )?;
        let _node_2_empty_subscription = graph
            .node2
            .create_subscription::<msg::Empty, _>("graph_test_topic_1", |_msg: msg::Empty| {})?;
        let _node_2_basic_types_subscription =
            graph.node2.create_subscription::<msg::BasicTypes, _>(
                "graph_test_topic_2",
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
        let graph = construct_test_graph("test_logger_name")?;

        assert_eq!(
            graph.node1.logger().name(),
            "test_logger_name.graph_test_node_1"
        );
        assert_eq!(
            graph.node2.logger().name(),
            "test_logger_name.graph_test_node_2"
        );

        Ok(())
    }
}
