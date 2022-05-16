mod publisher;
mod subscription;
pub use self::publisher::*;
pub use self::subscription::*;

use crate::rcl_bindings::*;
use crate::{Context, QoSProfile, RclrsError, ToResult};
use std::ffi::{CStr, CString};

use std::cmp::PartialEq;
use std::fmt;
use std::sync::{Arc, Weak};
use std::vec::Vec;

use libc::c_char;
use parking_lot::Mutex;

use rosidl_runtime_rs::Message;

impl Drop for rcl_node_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        unsafe { rcl_node_fini(self).ok().unwrap() };
    }
}

/// A processing unit that can communicate with other nodes.
///
/// Nodes are a core concept in ROS 2. Refer to the official ["Understanding ROS 2 nodes"][1]
/// tutorial for an introduction.
///
/// Ownership of the node is shared with all [`Publisher`]s and [`Subscription`]s created from it.
/// That means that even after the node itself is dropped, it will continue to exist and be
/// displayed by e.g. `ros2 topic` as long as its publishers and subscriptions are not dropped.
///
/// [1]: https://docs.ros.org/en/rolling/Tutorials/Understanding-ROS2-Nodes.html
pub struct Node {
    handle: Arc<Mutex<rcl_node_t>>,
    pub(crate) context: Arc<Mutex<rcl_context_t>>,
    pub(crate) subscriptions: Vec<Weak<dyn SubscriptionBase>>,
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
    /// See [`Node::new_with_namespace`] for documentation.
    #[allow(clippy::new_ret_no_self)]
    pub fn new(node_name: &str, context: &Context) -> Result<Node, RclrsError> {
        Self::new_with_namespace("", node_name, context)
    }

    /// Creates a new node in a namespace.
    ///
    /// A namespace without a leading forward slash is automatically changed to have a leading
    /// forward slash.
    ///
    /// # Naming
    /// The node namespace will be prefixed to the node name itself to form the *fully qualified
    /// node name*. This is the name that is shown e.g. in `ros2 node list`.
    /// Similarly, the node namespace will be prefixed to all names of topics and services
    /// created from this node.
    ///
    /// By convention, a node name with a leading underscore marks the node as hidden.
    ///
    /// It's a good idea for node names in the same executable to be unique.
    ///
    /// ## Remapping
    /// The namespace and name given here can be overriden through the command line.
    /// In that sense, the parameters to this functions are only the _default_ namespace and name.
    /// See also the [official tutorial][1] on the command line arguments for ROS nodes, and the
    /// [`Node::namespace()`] and [`Node::name()`] functions for examples.
    ///
    /// ## Rules for valid names
    /// The node namespace needs to fulfill the criteria of
    /// [`rmw_validate_namespace()`][2], otherwise an error will be returned.
    /// Likewise, the node name needs to fulfill the criteria of [`rmw_validate_node_name()`][3].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, Node, RclReturnCode, NodeErrorCode};
    /// let ctx = Context::new([])?;
    /// // This is a valid namespace and name
    /// assert!(Node::new_with_namespace("/my/nested/namespace", "my_node", &ctx).is_ok());
    /// // This name contains invalid characters, although the empty namespace is valid
    /// assert_eq!(
    ///     Node::new_with_namespace("", "röböt", &ctx),
    ///     Err(RclReturnCode::NodeError(NodeErrorCode::NodeInvalidName))
    /// );
    /// // This namespace starts with a number
    /// assert_eq!(
    ///     Node::new_with_namespace("/123/4", "my_node", &ctx),
    ///     Err(RclReturnCode::NodeError(NodeErrorCode::NodeInvalidNamespace))
    /// );
    /// # Ok::<(), RclReturnCode>(())
    /// ```
    ///
    /// # Panics
    /// When the node namespace or node name contain interior null bytes.
    ///
    /// [1]: https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html
    /// [2]: https://docs.ros2.org/latest/api/rmw/validate__namespace_8h.html
    /// [3]: https://docs.ros2.org/latest/api/rmw/validate__node__name_8h.html
    pub fn new_with_namespace(
        node_ns: &str,
        node_name: &str,
        context: &Context,
    ) -> Result<Node, RclrsError> {
        let raw_node_name = CString::new(node_name).unwrap();
        let raw_node_ns = CString::new(node_ns).unwrap();

        // SAFETY: Getting a zero-initialized value is always safe.
        let mut node_handle = unsafe { rcl_get_zero_initialized_node() };
        let context_handle = &mut *context.handle.lock();

        unsafe {
            // SAFETY: No preconditions for this function.
            let node_options = rcl_node_get_default_options();
            // SAFETY: The node handle is zero-initialized as expected by this function.
            // The strings and node options are copied by this function, so we don't need
            // to keep them alive.
            // The context handle is kept alive because it is co-owned by the node.
            rcl_node_init(
                &mut node_handle,
                raw_node_name.as_ptr(),
                raw_node_ns.as_ptr(),
                context_handle,
                &node_options,
            )
            .ok()?;
        }

        let handle = Arc::new(Mutex::new(node_handle));

        Ok(Node {
            handle,
            context: context.handle.clone(),
            subscriptions: std::vec![],
        })
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
    /// let node = context.create_node("my_node")?;
    /// assert_eq!(node.name(), "my_node");
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__node:=your_node"].map(String::from);
    /// let context_r = Context::new(remapping)?;
    /// let node_r = context_r.create_node("my_node")?;
    /// assert_eq!(node_r.name(), "your_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn name(&self) -> String {
        self.get_string(rcl_node_get_name)
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
    /// let node = context.create_node_with_namespace("/my/namespace", "my_node")?;
    /// assert_eq!(node.namespace(), "/my/namespace");
    /// // With remapping
    /// let remapping = ["--ros-args", "-r", "__ns:=/your_namespace"].map(String::from);
    /// let context_r = Context::new(remapping)?;
    /// let node_r = context_r.create_node("my_node")?;
    /// assert_eq!(node_r.namespace(), "/your_namespace");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn namespace(&self) -> String {
        self.get_string(rcl_node_get_namespace)
    }

    /// Returns the fully qualified name of the node.
    ///
    /// The fully qualified name of the node is the node namespace combined with the node name.
    /// It is subject to the remappings shown in [`Node::name`] and [`Node::namespace`].
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclrsError};
    /// let context = Context::new([])?;
    /// let node = context.create_node_with_namespace("/my/namespace", "my_node")?;
    /// assert_eq!(node.fully_qualified_name(), "/my/namespace/my_node");
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn fully_qualified_name(&self) -> String {
        self.get_string(rcl_node_get_fully_qualified_name)
    }

    // Helper for name(), namespace(), fully_qualified_name()
    fn get_string(
        &self,
        getter: unsafe extern "C" fn(*const rcl_node_t) -> *const c_char,
    ) -> String {
        let char_ptr = unsafe {
            // SAFETY: The node handle is valid.
            getter(&*self.handle.lock())
        };
        debug_assert!(!char_ptr.is_null());
        let cstr = unsafe {
            // SAFETY: The returned CStr is immediately converted to an owned string,
            // so the lifetime is no issue. The ptr is valid as per the documentation
            // of rcl_node_get_name.
            CStr::from_ptr(char_ptr)
        };
        cstr.to_string_lossy().into_owned()
    }

    /// Creates a [`Publisher`][1].
    ///
    /// [1]: crate::Publisher
    // TODO: make publisher's lifetime depend on node's lifetime
    pub fn create_publisher<T>(
        &self,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Publisher<T>, RclrsError>
    where
        T: Message,
    {
        Publisher::<T>::new(self, topic, qos)
    }

    /// Creates a [`Subscription`][1].
    ///
    /// [1]: crate::Subscription
    // TODO: make subscription's lifetime depend on node's lifetime
    pub fn create_subscription<T, F>(
        &mut self,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Arc<Subscription<T>>, RclrsError>
    where
        T: Message,
        F: FnMut(T) + 'static,
    {
        let subscription = Arc::new(Subscription::<T>::new(self, topic, qos, callback)?);
        self.subscriptions
            .push(Arc::downgrade(&subscription) as Weak<dyn SubscriptionBase>);
        Ok(subscription)
    }

    /// Returns the subscriptions that have not been dropped yet.
    pub(crate) fn live_subscriptions(&self) -> Vec<Arc<dyn SubscriptionBase>> {
        self.subscriptions
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
    /// # use rclrs::{Context, RclReturnCode};
    /// // set default ROS domain ID to 10 here
    /// std::env::set_var("ROS_DOMAIN_ID", "10");
    /// let context = Context::new([])?;
    /// let node = context.create_node("domain_id_node")?;
    /// let domain_id = node.domain_id();    
    /// assert_eq!(domain_id, 10);
    /// # Ok::<(), RclReturnCode>(())
    /// ```
    // TODO: If node option is supported,
    // add description about this function is for getting actual domain_id
    // and about override of domain_id via node option
    pub fn domain_id(&self) -> usize {
        let handle = &*self.handle.lock();
        let mut domain_id: usize = 0;
        let ret = unsafe {
            // SAFETY: No preconditions for this function.
            rcl_node_get_domain_id(handle, &mut domain_id)
        };

        debug_assert_eq!(ret, 0);
        domain_id
    }
}
