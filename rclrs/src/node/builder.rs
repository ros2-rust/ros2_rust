use crate::rcl_bindings::*;
use crate::{Context, Node, RclrsError, ToResult};

use std::ffi::CString;

use parking_lot::Mutex;
use std::sync::Arc;

/// A builder for creating a [`Node`][1].
///
/// The builder pattern allows selectively setting some fields, and leaving all others at their default values.
/// This struct instance can be created via [`Node::builder()`][2].
///
/// The default values for optional fields are:
/// - `namespace: "/"`
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
///
pub struct NodeBuilder {
    context: Arc<Mutex<rcl_context_t>>,
    name: String,
    namespace: String,
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
    /// # use rclrs::{Context, NodeBuilder, RclrsError, RclReturnCode, NodeErrorCode};
    /// let context = Context::new([])?;
    /// // This is a valid node name
    /// assert!(NodeBuilder::new(&context, "my_node").build().is_ok());
    /// // This is another valid node name (although not a good one)
    /// assert!(NodeBuilder::new(&context, "_______").build().is_ok());
    /// // This is an invalid node name
    /// assert_eq!(
    ///     NodeBuilder::new(&context, "röböt")
    ///         .build()
    ///         .unwrap_err()
    ///         .code,
    ///     RclReturnCode::NodeError(NodeErrorCode::NodeInvalidName)
    /// );
    /// # Ok::<(), RclrsError>(())
    /// ```    
    ///    
    /// [1]: crate::Node#naming
    /// [2]: https://docs.ros2.org/latest/api/rmw/validate__node__name_8h.html#a5690a285aed9735f89ef11950b6e39e3
    /// [3]: NodeBuilder::build
    pub fn new(context: &Context, name: &str) -> NodeBuilder {
        NodeBuilder {
            context: context.handle.clone(),
            name: name.to_string(),
            namespace: "/".to_string(),
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
    /// # use rclrs::{Context, Node, RclrsError, RclReturnCode, NodeErrorCode};
    /// let context = Context::new([])?;
    /// // This is a valid namespace
    /// let builder_ok_ns = Node::builder(&context, "my_node").namespace("/some/nested/namespace");
    /// assert!(builder_ok_ns.build().is_ok());
    /// // This is an invalid namespace
    /// assert_eq!(
    ///     Node::builder(&context, "my_node")
    ///         .namespace("/10_percent_luck/20_percent_skill")
    ///         .build()
    ///         .unwrap_err()
    ///         .code,
    ///     RclReturnCode::NodeError(NodeErrorCode::NodeInvalidNamespace)
    /// );
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

    /// Builds the node instance.
    ///
    /// Node name and namespace validation is performed in this method.
    ///
    /// For example usage, see the [`NodeBuilder`][1] docs.
    ///
    /// [1]: crate::NodeBuilder
    pub fn build(&self) -> Result<Node, RclrsError> {
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

        // SAFETY: No preconditions for this function.
        let mut node_handle = unsafe { rcl_get_zero_initialized_node() };

        unsafe {
            // SAFETY: No preconditions for this function.
            let context_handle = &mut *self.context.lock();
            // SAFETY: No preconditions for this function.
            let node_options = rcl_node_get_default_options();

            // SAFETY: The node handle is zero-initialized as expected by this function.
            // The strings and node options are copied by this function, so we don't need
            // to keep them alive.
            // The context handle has to be kept alive because it is co-owned by the node.
            rcl_node_init(
                &mut node_handle,
                node_name.as_ptr(),
                node_namespace.as_ptr(),
                context_handle,
                &node_options,
            )
            .ok()?;
        };

        let handle = Arc::new(Mutex::new(node_handle));

        Ok(Node {
            handle,
            context: self.context.clone(),
            subscriptions: std::vec![],
        })
    }
}
