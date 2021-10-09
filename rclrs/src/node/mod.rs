use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::ToResult;
use crate::{Context, ContextHandle};
use rclrs_common::error::RclError;
use std::ffi::CString;
use std::sync::{Arc, Weak};

use parking_lot::{Mutex, MutexGuard};

pub mod publisher;
pub use self::publisher::*;
pub mod subscription;
pub use self::subscription::*;

pub struct NodeHandle(Mutex<rcl_node_t>);

impl NodeHandle {
    pub fn get_mut(&mut self) -> &mut rcl_node_t {
        self.0.get_mut()
    }

    pub fn lock(&self) -> MutexGuard<rcl_node_t> {
        self.0.lock()
    }

    pub fn try_lock(&self) -> Option<MutexGuard<rcl_node_t>> {
        self.0.try_lock()
    }
}

impl Drop for NodeHandle {
    fn drop(&mut self) {
        let handle = &mut *self.get_mut();
        unsafe { rcl_node_fini(handle as *mut _).unwrap() };
    }
}

pub struct Node {
    handle: Arc<NodeHandle>,
    pub(crate) context: Arc<ContextHandle>,
    pub(crate) subscriptions: Vec<Weak<dyn SubscriptionBase>>,
}

impl Node {
    #[allow(clippy::new_ret_no_self)]
    pub fn new<'ctxt>(node_name: &str, context: &Context) -> Result<Node, RclError> {
        Self::new_with_namespace(node_name, "", context)
    }

    pub fn new_with_namespace<'ctxt>(
        node_name: &str,
        node_ns: &str,
        context: &Context,
    ) -> Result<Node, RclError> {
        let raw_node_name = CString::new(node_name).unwrap();
        let raw_node_ns = CString::new(node_ns).unwrap();

        let mut node_handle = unsafe { rcl_get_zero_initialized_node() };
        let context_handle = &mut *context.handle.lock();

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

        let handle = Arc::new(NodeHandle(Mutex::new(node_handle)));

        Ok(Node {
            handle,
            context: context.handle.clone(),
            subscriptions: vec![],
        })
    }

    // TODO: make publisher's lifetime depend on node's lifetime
    pub fn create_publisher<T>(
        &self,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Publisher<T>, RclError>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        Publisher::<T>::new(self, topic, qos)
    }

    // TODO: make subscription's lifetime depend on node's lifetime
    pub fn create_subscription<T, F>(
        &mut self,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Arc<Subscription<T>>, RclError>
    where
        T: rclrs_common::traits::MessageDefinition<T> + Default,
        F: FnMut(&T) + Sized + 'static,
    {
        let subscription = Arc::new(Subscription::<T>::new(self, topic, qos, callback)?);
        self.subscriptions
            .push(Arc::downgrade(&subscription) as Weak<dyn SubscriptionBase>);
        Ok(subscription)
    }
}
