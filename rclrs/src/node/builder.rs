use std::{
    ffi::{CStr, CString},
    sync::{atomic::AtomicBool, Arc, Mutex},
};

use crate::{
    rcl_bindings::*, ClockType, Context, ContextHandle, Logger, Node, NodeHandle,
    ParameterInterface, QoSProfile, RclrsError, TimeSource, ToResult, ENTITY_LIFECYCLE_MUTEX,
    QOS_PROFILE_CLOCK,
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
/// # use rclrs::{Context, NodeBuilder, Node, RclrsError};
/// let context = Context::new([])?;
/// // Building a node in a single expression
/// let node = NodeBuilder::new(&context, "foo_node").namespace("/bar").build()?;
/// assert_eq!(node.name(), "foo_node");
/// assert_eq!(node.namespace(), "/bar");
/// // Building a node via Node::builder()
/// let node = Node::builder(&context, "bar_node").build()?;
/// assert_eq!(node.name(), "bar_node");
/// // Building a node step-by-step
/// let mut builder = Node::builder(&context, "goose");
/// builder = builder.namespace("/duck/duck");
/// let node = builder.build()?;
/// assert_eq!(node.fully_qualified_name(), "/duck/duck/goose");
/// # Ok::<(), RclrsError>(())
/// ```
///
/// [1]: crate::Node
/// [2]: crate::Node::builder
pub struct NodeBuilder {
    context: Arc<ContextHandle>,
    name: String,
    namespace: String,
    use_global_arguments: bool,
    arguments: Vec<String>,
    enable_rosout: bool,
    start_parameter_services: bool,
    clock_type: ClockType,
    clock_qos: QoSProfile,
}

impl NodeBuilder {
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
    /// # use rclrs::{Context, NodeBuilder, RclrsError, RclReturnCode};
    /// let context = Context::new([])?;
    /// // This is a valid node name
    /// assert!(NodeBuilder::new(&context, "my_node").build().is_ok());
    /// // This is another valid node name (although not a good one)
    /// assert!(NodeBuilder::new(&context, "_______").build().is_ok());
    /// // This is an invalid node name
    /// assert!(matches!(
    ///     NodeBuilder::new(&context, "röböt")
    ///         .build()
    ///         .unwrap_err(),
    ///     RclrsError::RclError { code: RclReturnCode::NodeInvalidName, .. }
    /// ));
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::Node#naming
    /// [2]: https://docs.ros2.org/latest/api/rmw/validate__node__name_8h.html#a5690a285aed9735f89ef11950b6e39e3
    /// [3]: NodeBuilder::build
    pub fn new(context: &Context, name: &str) -> NodeBuilder {
        NodeBuilder {
            context: Arc::clone(&context.handle),
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
    /// # use rclrs::{Context, Node, RclrsError, RclReturnCode};
    /// let context = Context::new([])?;
    /// // This is a valid namespace
    /// let builder_ok_ns = Node::builder(&context, "my_node").namespace("/some/nested/namespace");
    /// assert!(builder_ok_ns.build().is_ok());
    /// // This is an invalid namespace
    /// assert!(matches!(
    ///     Node::builder(&context, "my_node")
    ///         .namespace("/10_percent_luck/20_percent_skill")
    ///         .build()
    ///         .unwrap_err(),
    ///     RclrsError::RclError { code: RclReturnCode::NodeInvalidNamespace, .. }
    /// ));
    /// // A missing forward slash at the beginning is automatically added
    /// assert_eq!(
    ///     Node::builder(&context, "my_node")
    ///         .namespace("foo")
    ///         .build()?
    ///         .namespace(),
    ///     "/foo"
    /// );
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::Node#naming
    /// [2]: http://design.ros2.org/articles/topic_and_service_names.html
    /// [3]: https://docs.ros2.org/latest/api/rmw/validate__namespace_8h.html#a043f17d240cf13df01321b19a469ee49
    /// [4]: NodeBuilder::build
    pub fn namespace(mut self, namespace: &str) -> Self {
        self.namespace = namespace.to_string();
        self
    }

    /// Enables or disables using global arguments.
    ///
    /// The "global" arguments are those used in [creating the context][1].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, Node, NodeBuilder, RclrsError};
    /// let context_args = ["--ros-args", "--remap", "__node:=your_node"]
    ///   .map(String::from);
    /// let context = Context::new(context_args)?;
    /// // Ignore the global arguments:
    /// let node_without_global_args =
    ///   rclrs::create_node_builder(&context, "my_node")
    ///   .use_global_arguments(false)
    ///   .build()?;
    /// assert_eq!(node_without_global_args.name(), "my_node");
    /// // Do not ignore the global arguments:
    /// let node_with_global_args =
    ///   rclrs::create_node_builder(&context, "my_other_node")
    ///   .use_global_arguments(true)
    ///   .build()?;
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
    /// # use rclrs::{Context, Node, NodeBuilder, RclrsError};
    /// // Usually, this would change the name of "my_node" to "context_args_node":
    /// let context_args = ["--ros-args", "--remap", "my_node:__node:=context_args_node"]
    ///   .map(String::from);
    /// let context = Context::new(context_args)?;
    /// // But the node arguments will change it to "node_args_node":
    /// let node_args = ["--ros-args", "--remap", "my_node:__node:=node_args_node"]
    ///   .map(String::from);
    /// let node =
    ///   rclrs::create_node_builder(&context, "my_node")
    ///   .arguments(node_args)
    ///   .build()?;
    /// assert_eq!(node.name(), "node_args_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::Context::new
    /// [2]: https://design.ros2.org/articles/ros_command_line_arguments.html
    pub fn arguments(mut self, arguments: impl IntoIterator<Item = String>) -> Self {
        self.arguments = arguments.into_iter().collect();
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
    /// Node name and namespace validation is performed in this method.
    ///
    /// For example usage, see the [`NodeBuilder`][1] docs.
    ///
    /// [1]: crate::NodeBuilder
    pub fn build(&self) -> Result<Arc<Node>, RclrsError> {
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
        let rcl_context = &mut *self.context.rcl_context.lock().unwrap();

        let handle = Arc::new(NodeHandle {
            // SAFETY: Getting a zero-initialized value is always safe.
            rcl_node: Mutex::new(unsafe { rcl_get_zero_initialized_node() }),
            context_handle: Arc::clone(&self.context),
            initialized: AtomicBool::new(false),
        });

        unsafe {
            // SAFETY:
            // * The rcl_node is zero-initialized as mandated by this function.
            // * The strings and node options are copied by this function, so we don't need to keep them alive.
            // * The rcl_context is kept alive by the ContextHandle because it is a dependency of the node.
            // * The entity lifecycle mutex is locked to protect against the risk of
            //   global variables in the rmw implementation being unsafely modified during cleanup.
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            rcl_node_init(
                &mut *handle.rcl_node.lock().unwrap(),
                node_name.as_ptr(),
                node_namespace.as_ptr(),
                rcl_context,
                &rcl_node_options,
            )
            .ok()?;
        };

        handle
            .initialized
            .store(true, std::sync::atomic::Ordering::Release);

        let parameter = {
            let rcl_node = handle.rcl_node.lock().unwrap();
            ParameterInterface::new(
                &rcl_node,
                &rcl_node_options.arguments,
                &rcl_context.global_arguments,
            )?
        };

        let logger_name = {
            let rcl_node = handle.rcl_node.lock().unwrap();
            let logger_name_raw_ptr = unsafe { rcl_node_get_logger_name(&*rcl_node) };
            if logger_name_raw_ptr.is_null() {
                ""
            } else {
                // SAFETY: rcl_node_get_logger_name will either return a nullptr
                // if the provided node was invalid or provide a valid null-terminated
                // const char* if the provided node was valid. We have already
                // verified that it is not a nullptr. We are also preventing the
                // pointed-to value from being modified while we view it by locking
                // the mutex of rcl_node while we view it. This means all the
                // safety conditions of CStr::from_ptr are met.
                unsafe { CStr::from_ptr(logger_name_raw_ptr) }
                    .to_str()
                    .unwrap_or("")
            }
        };

        let node = Arc::new(Node {
            handle,
            clients_mtx: Mutex::new(vec![]),
            guard_conditions_mtx: Mutex::new(vec![]),
            services_mtx: Mutex::new(vec![]),
            subscriptions_mtx: Mutex::new(vec![]),
            time_source: TimeSource::builder(self.clock_type)
                .clock_qos(self.clock_qos)
                .build(),
            parameter,
            logger: Logger::new(logger_name)?,
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

impl Drop for rcl_node_options_t {
    fn drop(&mut self) {
        // SAFETY: Do not finish this struct except here.
        unsafe {
            // This also finalizes the `rcl_arguments_t` contained in `rcl_node_options_t`.
            rcl_node_options_fini(self).ok().unwrap();
        }
    }
}
