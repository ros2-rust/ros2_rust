use std::{
    ffi::CString,
    sync::{Arc, Mutex},
};

use crate::{
    rcl_bindings::*,
    ClockType, Node, NodeState, NodeHandle, ParameterInterface, ContextHandle,
    QoSProfile, RclrsError, TimeSource, ToResult, ENTITY_LIFECYCLE_MUTEX, QOS_PROFILE_CLOCK,
};

/// A builder for creating a [`Node`][1].
///
/// The builder pattern allows selectively setting some fields, and leaving all others at their default values.
/// This struct instance can be created via [`Node::builder()`][2].
///
/// The default values for optional fields are:
/// - `namespace: "/"`
/// - `use_global_arguments: true`
/// - `arguments: []`
/// - `enable_rosout: true`
/// - `start_parameter_services: true`
/// - `clock_type: ClockType::RosTime`
/// - `clock_qos: QOS_PROFILE_CLOCK`
///
/// # Example
/// ```
/// # use rclrs::{Context, NodeOptions, Node, RclrsError};
/// let executor = Context::default().create_basic_executor();
/// // Building a node in a single expression
/// let node = executor.create_node(NodeOptions::new("foo_node").namespace("/bar"))?;
/// assert_eq!(node.name(), "foo_node");
/// assert_eq!(node.namespace(), "/bar");
/// // Building a node via NodeOptions
/// let node = executor.create_node(NodeOptions::new("bar_node"))?;
/// assert_eq!(node.name(), "bar_node");
/// // Building a node step-by-step
/// let mut options = NodeOptions::new("goose");
/// options = options.namespace("/duck/duck");
/// let node = executor.create_node(options)?;
/// assert_eq!(node.fully_qualified_name(), "/duck/duck/goose");
/// # Ok::<(), RclrsError>(())
/// ```
///
/// [1]: crate::Node
/// [2]: crate::Node::builder
pub struct NodeOptions {
    name: String,
    namespace: String,
    use_global_arguments: bool,
    arguments: Vec<String>,
    enable_rosout: bool,
    start_parameter_services: bool,
    clock_type: ClockType,
    clock_qos: QoSProfile,
}

impl NodeOptions {
    /// Creates a builder for a node with the given name.
    ///
    /// See the [`Node` docs][1] for general information on node names.
    ///
    /// # Rules for valid node names
    ///
    /// The rules for a valid node name are checked by the [`rmw_validate_node_name()`][2]
    /// function. They are:
    /// - Must contain only the `a-z`, `A-Z`, `0-9`, and `_` characters
    /// - Must not be empty and not be longer than `RMW_NODE_NAME_MAX_NAME_LENGTH`
    /// - Must not start with a number
    ///
    /// Note that node name validation is delayed until [`NodeBuilder::build()`][3].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, NodeOptions, RclrsError, RclReturnCode};
    /// let executor = Context::default().create_basic_executor();
    /// // This is a valid node name
    /// assert!(executor.create_node(NodeOptions::new("my_node")).is_ok());
    /// // This is another valid node name (although not a good one)
    /// assert!(executor.create_node(NodeOptions::new("_______")).is_ok());
    /// // This is an invalid node name
    /// assert!(matches!(
    ///     executor.create_node(NodeOptions::new("röböt")).unwrap_err(),
    ///     RclrsError::RclError { code: RclReturnCode::NodeInvalidName, .. }
    /// ));
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::Node#naming
    /// [2]: https://docs.ros2.org/latest/api/rmw/validate__node__name_8h.html#a5690a285aed9735f89ef11950b6e39e3
    /// [3]: NodeBuilder::build
    pub fn new(name: impl ToString) -> NodeOptions {
        NodeOptions {
            name: name.to_string(),
            namespace: "/".to_string(),
            use_global_arguments: true,
            arguments: vec![],
            enable_rosout: true,
            start_parameter_services: true,
            clock_type: ClockType::RosTime,
            clock_qos: QOS_PROFILE_CLOCK,
        }
    }

    /// Sets the node namespace.
    ///
    /// See the [`Node` docs][1] for general information on namespaces.
    ///
    /// # Rules for valid namespaces
    ///
    /// The rules for a valid node namespace are based on the [rules for a valid topic][2]
    /// and are checked by the [`rmw_validate_namespace()`][3] function. However, a namespace
    /// without a leading forward slash is automatically changed to have a leading forward slash
    /// before it is checked with this function.
    ///
    /// Thus, the effective rules are:
    /// - Must contain only the `a-z`, `A-Z`, `0-9`, `_`, and `/` characters
    /// - Must not have a number at the beginning, or after a `/`
    /// - Must not contain two or more `/` characters in a row
    /// - Must not have a `/` character at the end, except if `/` is the full namespace
    ///
    /// Note that namespace validation is delayed until [`NodeBuilder::build()`][4].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, Node, NodeOptions, RclrsError, RclReturnCode};
    /// let executor = Context::default().create_basic_executor();
    /// // This is a valid namespace
    /// let options_ok_ns = NodeOptions::new("my_node").namespace("/some/nested/namespace");
    /// assert!(executor.create_node(options_ok_ns).is_ok());
    /// // This is an invalid namespace
    /// assert!(matches!(
    ///     executor.create_node(
    ///         NodeOptions::new("my_node")
    ///         .namespace("/10_percent_luck/20_percent_skill")
    ///     ).unwrap_err(),
    ///     RclrsError::RclError { code: RclReturnCode::NodeInvalidNamespace, .. }
    /// ));
    /// // A missing forward slash at the beginning is automatically added
    /// assert_eq!(
    ///     executor.create_node(
    ///         NodeOptions::new("my_node")
    ///         .namespace("foo")
    ///     )?.namespace(),
    ///     "/foo"
    /// );
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::Node#naming
    /// [2]: http://design.ros2.org/articles/topic_and_service_names.html
    /// [3]: https://docs.ros2.org/latest/api/rmw/validate__namespace_8h.html#a043f17d240cf13df01321b19a469ee49
    /// [4]: NodeBuilder::build
    pub fn namespace(mut self, namespace: impl ToString) -> Self {
        self.namespace = namespace.to_string();
        self
    }

    /// Enables or disables using global arguments.
    ///
    /// The "global" arguments are those used in [creating the context][1].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, InitOptions, Node, NodeOptions, RclrsError};
    /// let context_args = ["--ros-args", "--remap", "__node:=your_node"]
    ///   .map(String::from);
    /// let executor = Context::new(context_args, InitOptions::default())?.create_basic_executor();
    /// // Ignore the global arguments:
    /// let node_without_global_args = executor.create_node(
    ///     NodeOptions::new("my_node")
    ///     .use_global_arguments(false)
    /// )?;
    /// assert_eq!(node_without_global_args.name(), "my_node");
    /// // Do not ignore the global arguments:
    /// let node_with_global_args = executor.create_node(
    ///     NodeOptions::new("my_other_node")
    ///     .use_global_arguments(true)
    /// )?;
    /// assert_eq!(node_with_global_args.name(), "your_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::Context::new
    pub fn use_global_arguments(mut self, enable: bool) -> Self {
        self.use_global_arguments = enable;
        self
    }

    /// Sets node-specific command line arguments.
    ///
    /// These arguments are parsed the same way as those for [`Context::new()`][1].
    /// However, the node-specific command line arguments have higher precedence than the arguments
    /// used in creating the context.
    ///
    /// For more details about command line arguments, see [here][2].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, InitOptions, Node, NodeOptions, RclrsError};
    /// // Usually, this would change the name of "my_node" to "context_args_node":
    /// let context_args = ["--ros-args", "--remap", "my_node:__node:=context_args_node"]
    ///   .map(String::from);
    /// let executor = Context::new(context_args, InitOptions::default())?.create_basic_executor();
    /// // But the node arguments will change it to "node_args_node":
    /// let node_args = ["--ros-args", "--remap", "my_node:__node:=node_args_node"]
    ///   .map(String::from);
    /// let node = executor.create_node(
    ///     NodeOptions::new("my_node")
    ///     .arguments(node_args)
    /// )?;
    /// assert_eq!(node.name(), "node_args_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::Context::new
    /// [2]: https://design.ros2.org/articles/ros_command_line_arguments.html
    pub fn arguments<Args: IntoIterator>(mut self, arguments: Args) -> Self
    where
        Args::Item: ToString,
    {
        self.arguments = arguments.into_iter().map(|item| item.to_string()).collect();
        self
    }

    /// Enables or disables logging to rosout.
    ///
    /// When enabled, log messages are published to the `/rosout` topic in addition to
    /// standard output.
    ///
    /// This option is currently unused in `rclrs`.
    pub fn enable_rosout(mut self, enable: bool) -> Self {
        self.enable_rosout = enable;
        self
    }

    /// Enables or disables parameter services.
    ///
    /// Parameter services can be used to allow external nodes to list, get and set
    /// parameters for this node.
    pub fn start_parameter_services(mut self, start: bool) -> Self {
        self.start_parameter_services = start;
        self
    }

    /// Sets the node's clock type.
    pub fn clock_type(mut self, clock_type: ClockType) -> Self {
        self.clock_type = clock_type;
        self
    }

    /// Sets the QoSProfile for the clock subscription.
    pub fn clock_qos(mut self, clock_qos: QoSProfile) -> Self {
        self.clock_qos = clock_qos;
        self
    }

    /// Builds the node instance.
    ///
    /// Only used internally. Downstream users should call
    /// [`Executor::create_node`].
    pub(crate) fn build(
        self,
        context: &Arc<ContextHandle>,
    ) -> Result<Node, RclrsError> {
        let node_name =
            CString::new(self.name.as_str()).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: self.name.clone(),
            })?;
        let node_namespace =
            CString::new(self.namespace.as_str()).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: self.namespace.clone(),
            })?;
        let rcl_node_options = self.create_rcl_node_options()?;
        let rcl_context = &mut *context.rcl_context.lock().unwrap();

        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_node = unsafe { rcl_get_zero_initialized_node() };
        unsafe {
            // SAFETY:
            // * The rcl_node is zero-initialized as mandated by this function.
            // * The strings and node options are copied by this function, so we don't need to keep them alive.
            // * The rcl_context is kept alive by the ContextHandle because it is a dependency of the node.
            // * The entity lifecycle mutex is locked to protect against the risk of
            //   global variables in the rmw implementation being unsafely modified during cleanup.
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            rcl_node_init(
                &mut rcl_node,
                node_name.as_ptr(),
                node_namespace.as_ptr(),
                rcl_context,
                &rcl_node_options,
            )
            .ok()?;
        };

        let handle = Arc::new(NodeHandle {
            rcl_node: Mutex::new(rcl_node),
            context_handle: Arc::clone(&context),
        });
        let parameter = {
            let rcl_node = handle.rcl_node.lock().unwrap();
            ParameterInterface::new(
                &rcl_node,
                &rcl_node_options.arguments,
                &rcl_context.global_arguments,
            )?
        };

        let node = Arc::new(NodeState {
            clients_mtx: Mutex::default(),
            guard_conditions_mtx: Mutex::default(),
            services_mtx: Mutex::default(),
            subscriptions_mtx: Mutex::default(),
            time_source: TimeSource::builder(self.clock_type)
                .clock_qos(self.clock_qos)
                .build(),
            parameter,
            handle,
        });
        node.time_source.attach_node(&node);

        if self.start_parameter_services {
            node.parameter.create_services(&node)?;
        }

        Ok(node)
    }

    /// Creates a rcl_node_options_t struct from this builder.
    ///
    /// Any fields not present in the builder will have their default value.
    /// For detail about default values, see [`NodeBuilder`][1] docs.
    ///
    /// [1]: crate::NodeBuilder
    fn create_rcl_node_options(&self) -> Result<rcl_node_options_t, RclrsError> {
        // SAFETY: No preconditions for this function.
        let mut rcl_node_options = unsafe { rcl_node_get_default_options() };

        let cstring_args = self
            .arguments
            .iter()
            .map(|s| match CString::new(s.as_str()) {
                Ok(cstr) => Ok(cstr),
                Err(err) => Err(RclrsError::StringContainsNul { s: s.clone(), err }),
            })
            .collect::<Result<Vec<_>, _>>()?;

        let cstring_arg_ptrs = cstring_args.iter().map(|s| s.as_ptr()).collect::<Vec<_>>();
        unsafe {
            // SAFETY: This function does not store the ephemeral cstring_args_ptrs
            // pointers. We are passing in a zero-initialized arguments struct as expected.
            rcl_parse_arguments(
                cstring_arg_ptrs.len() as i32,
                cstring_arg_ptrs.as_ptr(),
                rcutils_get_default_allocator(),
                &mut rcl_node_options.arguments,
            )
        }
        .ok()?;

        rcl_node_options.use_global_arguments = self.use_global_arguments;
        rcl_node_options.enable_rosout = self.enable_rosout;
        // SAFETY: No preconditions for this function.
        rcl_node_options.allocator = unsafe { rcutils_get_default_allocator() };

        Ok(rcl_node_options)
    }
}

impl<T: ToString> From<T> for NodeOptions {
    fn from(name: T) -> Self {
        NodeOptions::new(name)
    }
}

impl Drop for rcl_node_options_t {
    fn drop(&mut self) {
        // SAFETY: Do not finish this struct except here.
        unsafe {
            // This also finalizes the `rcl_arguments_t` contained in `rcl_node_options_t`.
            rcl_node_options_fini(self).ok().unwrap();
        }
    }
}
