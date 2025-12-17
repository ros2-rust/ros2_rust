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
    future::Future,
    os::raw::c_char,
    sync::{atomic::AtomicBool, Arc, Mutex},
    time::Duration,
};

use futures::{
    channel::mpsc::{unbounded, UnboundedSender},
    StreamExt,
};

use async_std::future::timeout;

use rosidl_runtime_rs::{Action, Message};

use crate::{
    rcl_bindings::*, ActionClient, ActionClientState, ActionGoalReceiver, ActionServer,
    ActionServerState, AnyTimerCallback, Client, ClientOptions, ClientState, Clock, ContextHandle,
    ExecutorCommands, IntoActionClientOptions, IntoActionServerOptions, IntoAsyncServiceCallback,
    IntoAsyncSubscriptionCallback, IntoNodeServiceCallback, IntoNodeSubscriptionCallback,
    IntoNodeTimerOneshotCallback, IntoNodeTimerRepeatingCallback, IntoTimerOptions, LogParams,
    Logger, ParameterBuilder, ParameterInterface, ParameterVariant, Parameters, Promise, Publisher,
    PublisherOptions, PublisherState, RclrsError, RequestedGoal, Service, ServiceOptions,
    ServiceState, Subscription, SubscriptionOptions, SubscriptionState, TerminatedGoal, TimeSource,
    Timer, TimerState, ToLogParams, Worker, WorkerOptions, WorkerState, ENTITY_LIFECYCLE_MUTEX,
};

/// A processing unit that can communicate with other nodes. See the API of
/// [`NodeState`] to find out what methods you can call on a [`Node`].
///
/// Nodes are a core concept in ROS 2. Refer to the official ["Understanding ROS 2 nodes"][1]
/// tutorial for an introduction.
///
/// Ownership of the node is shared with all the primitives such as [`Publisher`]s and [`Subscription`]s
/// that are created from it. That means that even after the `Node` itself is dropped, it will continue
/// to exist and be displayed by e.g. `ros2 topic` as long as any one of its primitives is not dropped.
///
/// # Creating
/// Use [`Executor::create_node`][5] to create a new node. Pass in [`NodeOptions`] to set all the different
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
/// [`NodeState::namespace()`] and [`NodeState::name()`] functions for examples.
///
/// ## Rules for valid names
/// The rules for valid node names and node namespaces are explained in
/// [`NodeOptions::new()`][3] and [`NodeOptions::namespace()`][4].
///
/// [1]: https://docs.ros.org/en/rolling/Tutorials/Understanding-ROS2-Nodes.html
/// [2]: https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html
/// [3]: crate::NodeOptions::new
/// [4]: crate::NodeOptions::namespace
/// [5]: crate::Executor::create_node
pub type Node = Arc<NodeState>;

/// The inner state of a [`Node`].
///
/// This is public so that you can choose to put it inside a [`Weak`][1] if you
/// want to be able to refer to a [`Node`] in a non-owning way. It is generally
/// recommended to manage the [`NodeState`] inside of an [`Arc`], and [`Node`]
/// is provided as convenience alias for that.
///
/// The public API of the [`Node`] type is implemented via `NodeState`.
///
/// [1]: std::sync::Weak
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
    /// # use rclrs::*;
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
    /// # use rclrs::*;
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
    /// # use rclrs::*;
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
    ///
    /// Workers carry a data "payload" that they can lend out to callbacks that
    /// are created using the worker. This makes it easy to share data between
    /// callbacks.
    ///
    /// In some cases the payload type can be inferred by Rust:
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// let executor = Context::default().create_basic_executor();
    /// let node = executor.create_node("my_node").unwrap();
    ///
    /// let worker = node.create_worker(String::new());
    ///
    /// let subscription = worker.create_subscription(
    ///     "input_topic",
    ///     move |data: &mut String, msg: example_interfaces::msg::String| {
    ///         *data = msg.data;
    ///     }
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// If the compiler complains about not knowing the payload type, you can
    /// specify it explicitly in two ways:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let worker: Worker<String> = node.create_worker(String::new());
    /// ```
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// let worker = node.create_worker::<String>(String::new());
    /// ```
    ///
    /// The data given to the worker can be any custom data type:
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    ///
    /// #[derive(Default)]
    /// struct MyNodeData {
    ///     addition_client: Option<Client<example_interfaces::srv::AddTwoInts>>,
    ///     result_publisher: Option<Publisher<example_interfaces::msg::Int64>>,
    /// }
    ///
    /// let worker = node.create_worker(MyNodeData::default());
    /// ```
    ///
    /// In the above example, `addition_client` and `result_publisher` can be
    /// created later inside a subscription or service callback using the [`Node`].
    pub fn create_worker<'a, Payload>(
        self: &Arc<Self>,
        options: impl Into<WorkerOptions<Payload>>,
    ) -> Worker<Payload>
    where
        Payload: 'static + Send + Sync,
    {
        let options = options.into();
        let commands = self
            .commands
            .create_worker_commands(Box::new(options.payload));
        WorkerState::create(Arc::clone(self), commands)
    }

    /// Creates a [`Client`].
    ///
    /// Pass in only the service name for the `options` argument to use all default client options:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// # use crate::rclrs::vendor::test_msgs;
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
    /// # use crate::rclrs::vendor::test_msgs;
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
    /// Creates an [`ActionClient`][1].
    ///
    /// [1]: crate::ActionClient
    // TODO: make action client's lifetime depend on node's lifetime
    pub fn create_action_client<'a, A: Action>(
        self: &Arc<Self>,
        options: impl IntoActionClientOptions<'a>,
    ) -> Result<ActionClient<A>, RclrsError> {
        ActionClientState::create(self, options)
    }

    /// Creates an [`ActionServer`].
    //
    // TODO(@mxgrey): Add extensive documentation and usage examples
    pub fn create_action_server<'a, A: Action, Task>(
        self: &Arc<Self>,
        options: impl IntoActionServerOptions<'a>,
        callback: impl FnMut(RequestedGoal<A>) -> Task + Send + Sync + 'static,
    ) -> Result<ActionServer<A>, RclrsError>
    where
        Task: Future<Output = TerminatedGoal> + Send + Sync + 'static,
    {
        ActionServerState::create(self, options, callback)
    }

    /// Creates an [`ActionGoalReceiver`].
    //
    // TODO(@mxgrey): Add extensive documentation and usage examples
    pub fn create_goal_receiver<'a, A: Action>(
        self: &Arc<Self>,
        options: impl IntoActionServerOptions<'a>,
    ) -> Result<ActionGoalReceiver<A>, RclrsError> {
        ActionGoalReceiver::new(self, options)
    }

    /// Creates a [`Publisher`].
    ///
    /// Pass in only the topic name for the `options` argument to use all default publisher options:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// # use crate::rclrs::vendor::test_msgs;
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
    /// # use crate::rclrs::vendor::test_msgs;
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
        self: &Arc<Self>,
        options: impl Into<PublisherOptions<'a>>,
    ) -> Result<Publisher<T>, RclrsError>
    where
        T: Message,
    {
        PublisherState::<T>::create(options, Arc::clone(self))
    }

    /// Creates a [`Service`] with an ordinary callback.
    ///
    /// # Behavior
    ///
    /// Even though this takes in a blocking (non-async) function, the callback
    /// may run in parallel with other callbacks. This callback may even run
    /// multiple times simultaneously with different incoming requests. This
    /// will depend on what kind of executor is running.
    ///
    /// If you want to ensure that calls to this service can only run
    /// one-at-a-time then consider creating a [`Worker`] and using that to
    /// [create a service][worker-service].
    ///
    /// [worker-service]: WorkerState::create_service
    ///
    /// # Service Options
    ///
    /// Pass in only the service name for the `options` argument to use all default service options:
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::test_msgs;
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
    /// # use crate::rclrs::vendor::test_msgs;
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
    ///
    /// # Service Callbacks
    ///
    /// Three callback signatures are supported:
    /// - [`Fn`] ( `Request` ) -> `Response`
    /// - [`Fn`] ( `Request`, [`RequestId`][3] ) -> `Response`
    /// - [`Fn`] ( `Request`, [`ServiceInfo`][4] ) -> `Response`
    ///
    /// [3]: crate::RequestId
    /// [4]: crate::ServiceInfo
    ///
    /// Any internal state captured into the callback that needs to be mutated
    /// will need to be wrapped in [`Mutex`] to ensure it is synchronized across
    /// multiple simultaneous runs of the callback. For example:
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::sync::Mutex;
    ///
    /// let counter = Mutex::new(0usize);
    /// let service = node.create_service::<example_interfaces::srv::Trigger, _>(
    ///     "trigger_counter",
    ///     move |_request: example_interfaces::srv::Trigger_Request| {
    ///         let mut counter = counter.lock().unwrap();
    ///         *counter += 1;
    ///         println!("Triggered {} times", *counter);
    ///         example_interfaces::srv::Trigger_Response {
    ///             success: true,
    ///             message: "no problems here".to_string(),
    ///         }
    ///     }
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    /// To share the internal state outside of the callback you will need to
    /// wrap it in [`Arc`] or `Arc<Mutex<S>>` and then clone the [`Arc`] before
    /// capturing it in the closure:
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::sync::{Arc, Mutex};
    ///
    /// let counter = Arc::new(Mutex::new(0usize));
    ///
    /// let counter_in_service = Arc::clone(&counter);
    /// let service = node.create_service::<example_interfaces::srv::Trigger, _>(
    ///     "trigger_counter",
    ///     move |_request: example_interfaces::srv::Trigger_Request| {
    ///         let mut counter = counter_in_service.lock().unwrap();
    ///         *counter += 1;
    ///         println!("Triggered {} times", *counter);
    ///         example_interfaces::srv::Trigger_Response {
    ///             success: true,
    ///             message: "no problems here".to_string(),
    ///         }
    ///     }
    /// )?;
    ///
    /// // TODO(@mxgrey): Replace this with a timer when timers become available
    /// std::thread::spawn(move || {
    ///     loop {
    ///         std::thread::sleep(std::time::Duration::from_secs(1));
    ///         println!("Last count of triggers: {}", counter.lock().unwrap());
    ///     }
    /// });
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// In general, when you need to manage some state within a blocking service,
    /// it may be a good idea to create a service using a [`WorkerState`] instead of
    /// creating one directly from the node. The [`WorkerState`] can carry your state
    /// as a payload so you don't have to worry about locking and sharing.
    ///
    /// The advantage of creating a service directly from the [`NodeState`] is you
    /// can create async services using [`NodeState::create_async_service`].
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
    /// # Service Options
    ///
    /// See [`create_service`][NodeState::create_service] for examples of setting
    /// the service options.
    ///
    /// # Async Service Callbacks
    ///
    /// Three callback signatures are supported:
    /// - [`FnMut`] ( `Request` ) -> impl [`Future`][5]<Output=`Response`>
    /// - [`FnMut`] ( `Request`, [`RequestId`][3] ) -> impl [`Future`][5]<Output=`Response`>
    /// - [`FnMut`] ( `Request`, [`ServiceInfo`][4] ) -> impl [`Future`][5]<Output=`Response`>
    ///
    /// [3]: crate::RequestId
    /// [4]: crate::ServiceInfo
    /// [5]: std::future::Future
    ///
    /// In this case the closure can be [`FnMut`], allowing internal state to
    /// be mutable, but it should be noted that the function is expected to
    /// immediately return a [`Future`][5], so in many cases any internal state
    /// that needs to be mutated will still need to be wrapped in [`Arc`] and
    /// [`Mutex`] to ensure it is synchronized across multiple runs of the
    /// `Future` that the callback produces.
    ///
    /// However unlike the blocking callbacks that can be provided to
    /// [`NodeState::create_service`], callbacks for async services can take
    /// advantage of `.await`. This allows you to capture [`Client`]s or
    /// [`Worker`]s into the closure, run tasks on them, and await the outcome.
    /// This allows one async service to share state data across multiple workers.
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node")?;
    /// use std::sync::Arc;
    ///
    /// let worker_a = node.create_worker(0_i64);
    /// let worker_b = node.create_worker(0_i64);
    ///
    /// let service = node.create_async_service::<example_interfaces::srv::AddTwoInts, _>(
    ///     "add",
    ///     move |request: example_interfaces::srv::AddTwoInts_Request| {
    ///         // Clone the workers so they can be captured into the async block
    ///         let worker_a = Arc::clone(&worker_a);
    ///         let worker_b = Arc::clone(&worker_b);
    ///         async move {
    ///             // Save the requested values into each worker
    ///             let a = request.a;
    ///             let _ = worker_a.run(move |last_a: &mut i64| {
    ///                 *last_a = a;
    ///             }).await;
    ///
    ///             let b = request.b;
    ///             let _ = worker_b.run(move |last_b: &mut i64| {
    ///                 *last_b = b;
    ///             }).await;
    ///
    ///             // Awaiting above ensures that each number from the
    ///             // request is saved in its respective worker before
    ///             // we give back a response.
    ///             example_interfaces::srv::AddTwoInts_Response { sum: a + b }
    ///        }
    ///     }
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
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
    /// need to wrap it in [`Arc`] or `Arc<Mutex<S>>`. In cases where you need to
    /// manage a changing state for the subscription, consider using
    /// [`WorkerState::create_subscription`] instead of this one.
    ///
    /// # Subscription Options
    ///
    /// Pass in only the topic name for the `options` argument to use all default subscription options:
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::test_msgs;
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
    /// # use crate::rclrs::vendor::test_msgs;
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
    /// # Subscription Callbacks
    ///
    /// Subscription callbacks support six signatures:
    /// - [`Fn`] ( `Message` )
    /// - [`Fn`] ( `Message`, [`MessageInfo`][1] )
    /// - [`Fn`] ( [`Box`]<`Message`> )
    /// - [`Fn`] ( [`Box`]<`Message`>, [`MessageInfo`][1] )
    /// - [`Fn`] ( [`ReadOnlyLoanedMessage`][2]<`Message`> )
    /// - [`Fn`] ( [`ReadOnlyLoanedMessage`][2]<`Message`>, [`MessageInfo`][1] )
    ///
    /// [1]: crate::MessageInfo
    /// [2]: crate::ReadOnlyLoanedMessage
    ///
    /// All function signatures use [`Fn`] since the callback may be run
    /// multiple times simultaneously across different threads depending on the
    /// executor runtime that is being used. Because of this, any internal state
    /// captured into the callback that needs to be mutated will need to be
    /// wrapped in [`Mutex`] to ensure it is synchronized across multiple
    /// simultaneous runs of the callback. For example:
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::sync::Mutex;
    ///
    /// let num_messages = Mutex::new(0usize);
    /// let subscription = node.create_subscription(
    ///     "topic",
    ///     move |msg: example_interfaces::msg::String| {
    ///         let mut num = num_messages.lock().unwrap();
    ///         *num += 1;
    ///         println!("#{} | I heard: '{}'", *num, msg.data);
    ///     },
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// To share the internal state outside of the callback you will need to
    /// wrap it in [`Arc`] (or `Arc<Mutex<S>>` for mutability) and then clone
    /// the [`Arc`] before capturing it in the closure:
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::sync::{Arc, Mutex};
    ///
    /// let data = Arc::new(Mutex::new(String::new()));
    ///
    /// let data_in_subscription = Arc::clone(&data);
    /// let subscription = node.create_subscription(
    ///     "topic",
    ///     move |msg: example_interfaces::msg::String| {
    ///         let mut data = data_in_subscription.lock().unwrap();
    ///         *data = msg.data;
    ///     },
    /// )?;
    ///
    /// // TODO(@mxgrey): Replace this with a timer when timers become available
    /// std::thread::spawn(move || {
    ///     loop {
    ///         std::thread::sleep(std::time::Duration::from_secs(1));
    ///         println!("Last message received: {}", data.lock().unwrap());
    ///     }
    /// });
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// You can change the subscription at any time by calling
    /// [`SubscriptionState::set_callback`] or
    /// [`SubscriptionState::set_async_callback`]. Even if the subscription is
    /// initially created with a regular callback, it can be changed to an async
    /// callback at any time.
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
    /// parallelism will depend on the executor that is being used.
    ///
    /// The key advantage of an async subscription is that the callback can use
    /// `.await`, and other callbacks will be able to run concurrently without
    /// being blocked by this callback. You can also pass in an `async fn` as
    /// the callback, but in most cases you will probably need to use a closure
    /// that returns an `async { ... }` block so that you can capture some state
    /// into the closure.
    ///
    /// If you don't need async language features for your callback, then
    /// consider using [`NodeState::create_subscription`] or
    /// [`WorkerState::create_subscription`].
    ///
    /// # Subscription Options
    ///
    /// See [`create_subscription`][NodeState::create_subscription] for examples
    /// of setting the subscription options.
    ///
    /// # Async Subscription Callbacks
    ///
    /// Async subscription callbacks support six signatures:
    /// - [`FnMut`] ( `Message` ) -> impl [`Future`][1]<Output=()>
    /// - [`FnMut`] ( `Message`, [`MessageInfo`][2] ) -> impl [`Future`][1]<Output=()>
    /// - [`FnMut`] ( [`Box`]<`Message`> ) -> impl [`Future`][1]<Output=()>
    /// - [`FnMut`] ( [`Box`]<`Message`>, [`MessageInfo`][2] ) -> impl [`Future`][1]<Output=()>
    /// - [`FnMut`] ( [`ReadOnlyLoanedMessage`][3]<`Message`> ) -> impl [`Future`][1]<Output=()>
    /// - [`FnMut`] ( [`ReadOnlyLoanedMessage`][3]<`Message`>, [`MessageInfo`][2] ) -> impl [`Future`][1]<Output=()>
    ///
    /// [1]: std::future::Future
    /// [2]: crate::MessageInfo
    /// [3]: crate::ReadOnlyLoanedMessage
    ///
    /// In this case the closure can be [`FnMut`], allowing internal state to be
    /// mutable, but it should be noted that the function is expected to
    /// immediately return a [`Future`][1], so in many cases any internal state
    /// that needs to be mutable will still need to be wrapped in [`Arc`] and
    /// [`Mutex`] to ensure it is synchronized across mutliple runs of the
    /// `Future` that the callback produces.
    ///
    /// However unlike the blocking callbacks that can be provided to
    /// [`NodeState::create_subscription`], callbacks for async subscriptions
    /// can take advantage of `.await`. This allows you to capture [`Client`]s
    /// or [`Worker`]s into the closure, run tasks on them, and await the
    /// outcome. This allows one async subscription to share state data across
    /// multiple workers.
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    ///
    /// use std::sync::Arc;
    ///
    /// let count_worker = node.create_worker(0_usize);
    /// let data_worker = node.create_worker(String::new());
    ///
    /// let service = node.create_async_subscription::<example_interfaces::msg::String, _>(
    ///     "topic",
    ///     move |msg: example_interfaces::msg::String| {
    ///         // Clone the workers so they can be captured into the async block
    ///         let count_worker = Arc::clone(&count_worker);
    ///         let data_worker = Arc::clone(&data_worker);
    ///         async move {
    ///             // Update the message count
    ///             let current_count = count_worker.run(move |count: &mut usize| {
    ///                 *count += 1;
    ///                 *count
    ///             }).await.unwrap();
    ///
    ///             // Change the data in the data_worker and get back the data
    ///             // that was previously put in there.
    ///             let previous = data_worker.run(move |data: &mut String| {
    ///                 std::mem::replace(data, msg.data)
    ///             }).await.unwrap();
    ///
    ///             println!("Current count is {current_count}, data was previously {previous}");
    ///        }
    ///     }
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// You can change the subscription at any time by calling
    /// [`SubscriptionState::set_callback`] or
    /// [`SubscriptionState::set_async_callback`]. Even if the subscription is
    /// initially created with an async callback, it can be changed to a regular
    /// callback at any time.
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

    /// Create a [`Timer`] with a repeating callback.
    ///
    /// This has similar behavior to `rclcpp::Node::create_timer` by periodically
    /// triggering the callback of the timer. For a one-shot timer alternative,
    /// see [`NodeState::create_timer_oneshot`].
    ///
    /// See also:
    /// * [`Self::create_timer_oneshot`]
    /// * [`Self::create_timer_inert`]
    ///
    /// # Behavior
    ///
    /// While the callback of this timer is running, no other callbacks associated
    /// with this node will be able to run. This is in contrast to callbacks given
    /// to [`Self::create_subscription`] which can run multiple times in parallel.
    ///
    /// Since the callback of this timer may block other callbacks from being able
    /// to run, it is strongly recommended to ensure that the callback returns
    /// quickly. If the callback needs to trigger long-running behavior then you
    /// can consider using [`std::thread::spawn`], or for async behaviors you can
    /// capture an [`ExecutorCommands`] in your callback and use [`ExecutorCommands::run`]
    /// to issue a task for the executor to run in its async task pool.
    ///
    /// Since these callbacks are blocking, you may use [`FnMut`] here instead of
    /// being limited to [`Fn`].
    ///
    /// # Timer Options
    ///
    /// You can choose both
    /// 1. a timer period (duration) which determines how often the callback is triggered
    /// 2. a clock to measure the passage of time
    ///
    /// Both of these choices are expressed by [`TimerOptions`][1].
    ///
    /// By default the steady clock time will be used, but you could choose
    /// node time instead if you want the timer to automatically use simulated
    /// time when running as part of a simulation:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::time::Duration;
    ///
    /// let timer = node.create_timer_repeating(
    ///     TimerOptions::new(Duration::from_secs(1))
    ///     .node_time(),
    ///     || {
    ///         println!("Triggering once each simulated second");
    ///     },
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// If there is a specific manually-driven clock you want to use, you can
    /// also select that:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::time::Duration;
    ///
    /// let (my_clock, my_source) = Clock::with_source();
    ///
    /// let timer = node.create_timer_repeating(
    ///     TimerOptions::new(Duration::from_secs(1))
    ///     .clock(&my_clock),
    ///     || {
    ///         println!("Triggering once each simulated second");
    ///     },
    /// )?;
    ///
    /// my_source.set_ros_time_override(1_500_000_000);
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// If you are okay with the default choice of clock (steady clock) then you
    /// can choose to simply pass a duration in as the options:
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::time::Duration;
    ///
    /// let timer = node.create_timer_repeating(
    ///     Duration::from_secs(1),
    ///     || {
    ///         println!("Triggering per steady clock second");
    ///     },
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// # Node Timer Repeating Callbacks
    ///
    /// Node Timer repeating callbacks support three signatures:
    /// - <code>[FnMut] ()</code>
    /// - <code>[FnMut] ([Time][2])</code>
    /// - <code>[FnMut] (&[Timer])</code>
    ///
    /// You can choose to receive the current time when the callback is being
    /// triggered.
    ///
    /// Or instead of the current time, you can get a borrow of the [`Timer`]
    /// itself, that way if you need to access it from inside the callback, you
    /// do not need to worry about capturing a [`Weak`][3] and then locking it.
    /// This is useful if you need to change the callback of the timer from inside
    /// the callback of the timer.
    ///
    /// For an [`FnOnce`] instead of [`FnMut`], use [`Self::create_timer_oneshot`].
    ///
    /// [1]: crate::TimerOptions
    /// [2]: crate::Time
    /// [3]: std::sync::Weak
    pub fn create_timer_repeating<'a, Args>(
        self: &Arc<Self>,
        options: impl IntoTimerOptions<'a>,
        callback: impl IntoNodeTimerRepeatingCallback<Args>,
    ) -> Result<Timer, RclrsError> {
        self.create_timer_internal(options, callback.into_node_timer_repeating_callback())
    }

    /// Create a [`Timer`] whose callback will be triggered once after the period
    /// of the timer has elapsed. After that you will need to use
    /// [`TimerState::set_repeating`] or [`TimerState::set_oneshot`] or else
    /// nothing will happen the following times that the `Timer` elapses.
    ///
    /// This does not have an equivalent in `rclcpp`.
    ///
    /// See also:
    /// * [`Self::create_timer_repeating`]
    /// * [`Self::create_timer_inert`]
    ///
    /// # Behavior
    ///
    /// While the callback of this timer is running, no other callbacks associated
    /// with this node will be able to run. This is in contrast to callbacks given
    /// to [`Self::create_subscription`] which can run multiple times in parallel.
    ///
    /// Since the callback of this timer may block other callbacks from being able
    /// to run, it is strongly recommended to ensure that the callback returns
    /// quickly. If the callback needs to trigger long-running behavior then you
    /// can consider using [`std::thread::spawn`], or for async behaviors you can
    /// capture an [`ExecutorCommands`] in your callback and use [`ExecutorCommands::run`]
    /// to issue a task for the executor to run in its async task pool.
    ///
    /// Since these callbacks will only be triggered once, you may use [`FnOnce`] here.
    ///
    /// # Timer Options
    ///
    /// See [`NodeSate::create_timer_repeating`][3] for examples of setting the
    /// timer options.
    ///
    /// # Node Timer Oneshot Callbacks
    ///
    /// Node Timer OneShot callbacks support three signatures:
    /// - <code>[FnOnce] ()</code>
    /// - <code>[FnOnce] ([Time][2])</code>
    /// - <code>[FnOnce] (&[Timer])</code>
    ///
    /// You can choose to receive the current time when the callback is being
    /// triggered.
    ///
    /// Or instead of the current time, you can get a borrow of the [`Timer`]
    /// itself, that way if you need to access it from inside the callback, you
    /// do not need to worry about capturing a [`Weak`][3] and then locking it.
    /// This is useful if you need to change the callback of the timer from inside
    /// the callback of the timer.
    ///
    /// [2]: crate::Time
    /// [3]: std::sync::Weak
    pub fn create_timer_oneshot<'a, Args>(
        self: &Arc<Self>,
        options: impl IntoTimerOptions<'a>,
        callback: impl IntoNodeTimerOneshotCallback<Args>,
    ) -> Result<Timer, RclrsError> {
        self.create_timer_internal(options, callback.into_node_timer_oneshot_callback())
    }

    /// Create a [`Timer`] without a callback. Nothing will happen when this
    /// `Timer` elapses until you use [`TimerState::set_repeating`] or
    /// [`TimerState::set_oneshot`].
    ///
    /// This function is not usually what you want. An inert timer is usually
    /// just a follow-up state to a oneshot timer which is waiting to be given
    /// a new callback to run. However, you could use this method to declare a
    /// timer whose callbacks you will start to feed in at a later.
    ///
    /// There is no equivalent to this function in `rclcpp`.
    ///
    /// See also:
    /// * [`Self::create_timer_repeating`]
    /// * [`Self::create_timer_oneshot`]
    pub fn create_timer_inert<'a>(
        self: &Arc<Self>,
        options: impl IntoTimerOptions<'a>,
    ) -> Result<Timer, RclrsError> {
        self.create_timer_internal(options, AnyTimerCallback::Inert)
    }

    /// Used internally to create any kind of [`Timer`].
    ///
    /// Downstream users should instead use:
    /// * [`Self::create_timer_repeating`]
    /// * [`Self::create_timer_oneshot`]
    /// * [`Self::create_timer_inert`]
    fn create_timer_internal<'a>(
        self: &Arc<Self>,
        options: impl IntoTimerOptions<'a>,
        callback: AnyTimerCallback<Node>,
    ) -> Result<Timer, RclrsError> {
        let options = options.into_timer_options();
        let clock = options.clock.as_clock(self);
        let node = options.clock.is_node_time().then(|| Arc::clone(self));
        TimerState::create(
            options.period,
            clock,
            callback,
            self.commands.async_worker_commands(),
            &self.handle.context_handle,
            node,
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
    /// # use rclrs::*;
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
    /// # use rclrs::*;
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
        use crate::vendor::test_msgs::msg;

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
