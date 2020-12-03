use crate::rcl_bindings as ffi;
use crate::Context;
use crate::qos::QoSProfile;
use std::ffi::{CString, CStr};
use std::sync::Mutex;
use thiserror::Error;

pub struct NodeOptions {
    node_options: ffi::rcl_node_options_t
}

impl Default for NodeOptions {
    /// Get the default options for a Node.
    fn default() -> Self {
        // Safety: cannot fail, will always return valid default options.
        let default_options = unsafe { ffi::rcl_node_get_default_options() };
        Self {
            node_options: default_options
        }
    }
}

impl Drop for NodeOptions {
    fn drop(&mut self) {
        // This is safe to call, because `node_options` will always be non-NULL
        // and valid, and will always had `rcl_node_get_default_options()`
        // called on it, because this is done on construction. Therefore,
        // it is safe to ignore potential errors that might occur here.
        //
        // However, there might still be an "unspecified error", in which case
        // it is not safe to continue. Because drop() does not support returning
        // `Result`s, we have to panic here.
        let return_code = unsafe { ffi::rcl_node_options_fini(&mut self.node_options) };
        if return_code as u32 == ffi::RCL_RET_ERROR {
            panic!("Unspecified error occured while dropping NodeOptions")
        }
    }
}

impl NodeOptions {
    /// Flag to enable /rosout for this node. By default, this is enabled.
    /// # Example
    /// ```
    /// # use rclrs::NodeOptions;
    /// let mut node_options = NodeOptions::default();
    /// node_options.enable_rosout(true);
    /// ```
    pub fn enable_rosout(&mut self, state: bool) {
        self.node_options.enable_rosout = state;
    }

    /// Set the quality of service profile for the messages posted
    /// to the /rosout topic.
    /// # Example
    /// ```
    /// # use rclrs::NodeOptions;
    /// # use rclrs::qos::QOS_PROFILE_SYSTEM_DEFAULT;
    /// let mut node_options = NodeOptions::default();
    /// node_options.set_rosout_qos_profile(QOS_PROFILE_SYSTEM_DEFAULT);
    /// ```
    pub fn set_rosout_qos_profile(&mut self, qos: QoSProfile) {
        self.node_options.rosout_qos = qos.into();
    }
}

#[derive(Error, Debug)]
pub enum NodeError {
    #[error("The given node has already been initialized")]
    AlreadyInitialized,
    #[error("The given context is invalid")]
    InvalidContext,
    #[error("One of the provided arguments is invalid")]
    InvalidArgument,
    #[error("Allocating memory failed")]
    BadAllocation,
    #[error("The given name is invalid")]
    InvalidNodeName,
    #[error("The given namespace is invalid")]
    InvalidNamespaceName,
    #[error("Unspecified error occured")]
    UnspecifiedError,
}

pub struct Node<'context> {
    context: &'context Context,
    node: Mutex<ffi::rcl_node_t>
}

impl<'context> Node<'context> {
    pub(crate) fn new(context: &'context Context, name: &str, namespace: &str, options: &NodeOptions) -> Result<Self, NodeError> {
        // Safety: reserves space for a struct, cannot fail
        let mut node = unsafe { ffi::rcl_get_zero_initialized_node() };

        let c_name = CString::new(name).map_err(|_| NodeError::InvalidNodeName).unwrap();
        let c_namespace = CString::new(namespace).map_err(|_| NodeError::InvalidNamespaceName).unwrap();

        let mut context_lock = context.handle.lock().unwrap();
        // Safety: This is safe, because
        // - node is zero initialized and new
        // - c_name and c_namespace are valid CStrings with a lifetime that
        //   exceeds this function. They are copied into the node and
        //   therefore, the node cannot have dangling pointers to
        //   c_name and c_namespace in case they are dropped.
        // - The context is valid, because an invalid Context object cannot
        //   be via the Context::new() function.
        // - Options are valid and deep copied into the node. It is therefore
        //   safe to drop the options after the initialization of this node.
        let return_code = unsafe { ffi::rcl_node_init(&mut node, c_name.as_ptr(), c_namespace.as_ptr(), &mut *context_lock, &options.node_options) };
        drop(context_lock);

        match return_code {
            RCL_RET_OK => Ok(Node {
                context,
                node: Mutex::new(node)
            }),
            RCL_RET_ALREADY_INIT => Err(NodeError::AlreadyInitialized),
            RCL_RET_NOT_INIT => Err(NodeError::InvalidContext),
            RCL_RET_INVALID_ARGUMENT => Err(NodeError::InvalidArgument),
            RCL_RET_BAD_ALLOC => Err(NodeError::BadAllocation),
            RCL_RET_NODE_INVALID_NAME => Err(NodeError::InvalidNodeName),
            RCL_RET_NODE_INVALID_NAMESPACE => Err(NodeError::InvalidNamespaceName),
            _ => Err(NodeError::UnspecifiedError)
        }
    }

    /// Get the name of this Node
    /// # Example
    /// ```
    /// # use rclrs::{ Context, NodeOptions };
    /// let context = Context::new().unwrap();
    /// let node = context.create_node("NodeName", "/this/is/a/namespace", &NodeOptions::default()).unwrap();
    /// assert_eq!(node.get_name(), "NodeName".to_string());
    /// ```
    pub fn get_name(&self) -> String {
        let node = self.node.lock().unwrap();
        // Safety: Node is always initiliazed, otherwise this function cannot
        // be called.
        let c_buffer = unsafe { ffi::rcl_node_get_name(&*node) };
        // Safety: Rust documentation lists these issues:
        //  - There is no guarantee to the validity of ptr.
        //      > This can be guaranteed if we trust that rcl_node_get_name
        //      > actually returns a valid pointer.
        //  - The returned lifetime is not guaranteed to be the actual lifetime of ptr.
        //      > The RCL docs specify that the returned pointer is only valid as long
        //      > as the given rcl_node_t is valid. Given that this function can only
        //      > be called on valid nodes, this is also not a problem.
        //  - There is no guarantee that the memory pointed to by ptr contains a
        //  - valid nul terminator byte at the end of the string.
        //      > Because the string has been constructed from a Rust &str, we can
        //      > ensure that the string has a valid nul terminator. We have to
        //      > trust RCL to keep a valid nul terminator at the end of the string.
        //  - It is not guaranteed that the memory pointed by ptr won't change
        //  - before the CStr has been destroyed.
        //      > This node can only be changed if someone else has access to it.
        //      > Because a mutex is locked at the beginning of this function,
        //      > it can be ensured that nobody else can access the node from Rust.
        let c_str = unsafe { CStr::from_ptr(c_buffer) };
        let str_slice = c_str.to_str().unwrap();
        str_slice.to_owned()
    }

    /// Get the namespace of this Node.
    /// # Example
    /// ```
    /// # use rclrs::{ Context, NodeOptions };
    /// let context = Context::new().unwrap();
    /// let node = context.create_node("NodeName", "/this/is/a/namespace", &NodeOptions::default()).unwrap();
    /// assert_eq!(node.get_namespace(), "/this/is/a/namespace".to_string());
    /// ```
    pub fn get_namespace(&self) -> String {
        let node = self.node.lock().unwrap();
        // Safety: these two functions have the same safety assumptions
        // as the two functions in get_name(). The same assumptions are
        // true here.
        let c_buffer = unsafe { ffi::rcl_node_get_namespace(&*node) };
        let c_str = unsafe { CStr::from_ptr(c_buffer) };
        let str_slice = c_str.to_str().unwrap();
        str_slice.to_owned()
    }

    /// Get the fully qualified name of this Node.
    /// # Example
    /// ```
    /// # use rclrs::{ Context, NodeOptions };
    /// let context = Context::new().unwrap();
    /// let node = context.create_node("NodeName", "/this/is/a/namespace", &NodeOptions::default()).unwrap();
    /// assert_eq!(node.get_fully_qualified_name(), "/this/is/a/namespace/NodeName".to_string());
    /// ```
    pub fn get_fully_qualified_name(&self) -> String {
        let node = self.node.lock().unwrap();
        // Safety: these two functions have the same safety assumptions
        // as the two functions in get_name(). The same assumptions are
        // true here.
        let c_buffer = unsafe { ffi::rcl_node_get_fully_qualified_name(&*node) };
        let c_str = unsafe { CStr::from_ptr(c_buffer) };
        let str_slice = c_str.to_str().unwrap();
        str_slice.to_owned()
    }
}

impl Drop for Node<'_> {
    fn drop(&mut self) {
        let mut node = self.node.lock().unwrap();
        // Safety: This is safe to call, because a node will always be non-NULL
        // and valid, because a node will always be initialized by the
        // init function that function cannot succeed if the node is invalid.
        //
        // However, there might still be an "unspecified error", in which case
        // it is not safe to continue. Because drop() does not support returning
        // `Result`s, we have to panic here.
        let return_code = unsafe { ffi::rcl_node_fini(&mut *node) } as u32;
        if return_code == ffi::RCL_RET_ERROR {
            panic!("Unspecified error occured while dropping Node")
        }
    }
}

/*
pub mod publisher;
pub use self::publisher::*;
pub mod subscription;
pub use self::subscription::*;

pub struct NodeHandle(RefCell<rcl_node_t>);

impl<'a> Handle<rcl_node_t> for &'a NodeHandle {
    type DerefT = Ref<'a, rcl_node_t>;
    type DerefMutT = RefMut<'a, rcl_node_t>;

    fn get(self) -> Self::DerefT {
        self.0.borrow()
    }

    fn get_mut(self) -> Self::DerefMutT {
        self.0.borrow_mut()
    }
}

impl Drop for NodeHandle {
    fn drop(&mut self) {
        let handle = &mut *self.get_mut();
        unsafe {
            rcl_node_fini(handle as *mut _).unwrap();
        }
    }
}

pub struct Node {
    handle: Rc<NodeHandle>,
    pub(crate) context: Rc<ContextHandle>,
    pub(crate) subscriptions: Vec<Weak<dyn SubscriptionBase>>,
}

impl Node {
    #[allow(clippy::new_ret_no_self)]
    pub fn new(node_name: &str, context: &Context) -> RclResult<Node> {
        Self::new_with_namespace(node_name, "", context)
    }

    pub fn new_with_namespace(
        node_name: &str,
        node_ns: &str,
        context: &Context,
    ) -> RclResult<Node> {
        let raw_node_name = CString::new(node_name).unwrap();
        let raw_node_ns = CString::new(node_ns).unwrap();

        let mut node_handle = unsafe { rcl_get_zero_initialized_node() };
        let context_handle = &mut *context.handle.get_mut();

        unsafe {
            let node_options = rcl_node_get_default_options();
            rcl_node_init(
                &mut node_handle as *mut _,
                raw_node_name.as_ptr(),
                raw_node_ns.as_ptr(),
                context_handle as *mut _,
                &node_options as *const _,
            )
            .ok()?;
        }

        let handle = Rc::new(NodeHandle(RefCell::new(node_handle)));

        Ok(Node {
            handle,
            context: context.handle.clone(),
            subscriptions: vec![],
        })
    }

    // TODO: make publisher's lifetime depend on node's lifetime
    pub fn create_publisher<T>(&self, topic: &str, qos: QoSProfile) -> RclResult<Publisher<T>>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        Publisher::<T>::new(self, topic, qos)
    }n

    // TODO: make subscription's lifetime depend on node's lifetime
    pub fn create_subscription<T, F>(
        &mut self,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> RclResult<Rc<Subscription<T>>>
    where
        T: rclrs_common::traits::MessageDefinition<T> + Default,
        F: FnMut(&T) + Sized + 'static,
    {
        let subscription = Rc::new(Subscription::<T>::new(self, topic, qos, callback)?);
        self.subscriptions
            .push(Rc::downgrade(&subscription) as Weak<dyn SubscriptionBase>);
        Ok(subscription)
    }
}
*/