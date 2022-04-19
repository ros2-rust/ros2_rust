use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};

use crate::error::{RclReturnCode, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::Context;

use rosidl_runtime_rs::Message;

use cstr_core::CString;

pub mod publisher;
pub use self::publisher::*;
pub mod subscription;
pub use self::subscription::*;

#[cfg(not(feature = "std"))]
use spin::Mutex;

#[cfg(feature = "std")]
use parking_lot::Mutex;

impl Drop for rcl_node_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        unsafe { rcl_node_fini(self as *mut _).unwrap() };
    }
}

pub struct Node {
    handle: Arc<Mutex<rcl_node_t>>,
    pub(crate) context: Arc<Mutex<rcl_context_t>>,
    pub(crate) subscriptions: Vec<Weak<dyn SubscriptionBase>>,
}

impl Node {
    #[allow(clippy::new_ret_no_self)]
    pub fn new(node_name: &str, context: &Context) -> Result<Node, RclReturnCode> {
        Self::new_with_namespace(node_name, "", context)
    }

    pub fn new_with_namespace(
        node_name: &str,
        node_ns: &str,
        context: &Context,
    ) -> Result<Node, RclReturnCode> {
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
                &mut node_handle as *mut _,
                raw_node_name.as_ptr(),
                raw_node_ns.as_ptr(),
                context_handle as *mut _,
                &node_options as *const _,
            )
            .ok()?;
        }

        let handle = Arc::new(Mutex::new(node_handle));

        Ok(Node {
            handle,
            context: context.handle.clone(),
            subscriptions: alloc::vec![],
        })
    }

    // TODO: make publisher's lifetime depend on node's lifetime
    pub fn create_publisher<T>(
        &self,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Publisher<T>, RclReturnCode>
    where
        T: Message,
    {
        Publisher::<T>::new(self, topic, qos)
    }

    // TODO: make subscription's lifetime depend on node's lifetime
    pub fn create_subscription<T, F>(
        &mut self,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Arc<Subscription<T>>, RclReturnCode>
    where
        T: Message,
        F: FnMut(T) + Sized + 'static,
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
}
