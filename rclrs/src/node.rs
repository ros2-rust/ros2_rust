//use crate::qos::QoSProfile;
use crate::rcl_bindings as ffi;
use crate::Context;
use crate::qos::QoSProfile;
//use std::cell::{Ref, RefCell, RefMut};
//use std::ffi::CString;
//use std::rc::{Rc, Weak};

pub struct NodeOptions {
    node_options: ffi::rcl_node_options_t
}

impl Default for NodeOptions {
    /// Get the default options for a Node.
    fn default() -> Self {
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
    ///
    /// let mut node_options = NodeOptions::default();
    /// node_options.set_rosout_qos_profile(QOS_PROFILE_SYSTEM_DEFAULT);
    /// ```
    pub fn set_rosout_qos_profile(&mut self, qos: QoSProfile) {
        self.node_options.rosout_qos = qos.into();
    }
}

pub struct Node<'context> {
    context: &'context Context
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