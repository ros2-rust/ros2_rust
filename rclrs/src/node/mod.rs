use crate::error::{RclResult, ToRclResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::{Context, ContextHandle, Handle};
use std::cell::{Ref, RefCell, RefMut};
use std::ffi::CString;
use std::rc::{Rc, Weak};

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
    }

    // TODO: make subscription's lifetime depend on node's lifetime 
    pub fn create_subscription<T>(
        &mut self,
        topic: &str,
        qos: QoSProfile,
        callback: fn(&T),
    ) -> RclResult<Rc<Subscription<T>>>
    where
        T: rclrs_common::traits::MessageDefinition<T> + Default,
    {
        let subscription = Rc::new(Subscription::<T>::new(self, topic, qos, callback)?);
        self.subscriptions
            .push(Rc::downgrade(&subscription) as Weak<dyn SubscriptionBase>);
        Ok(subscription)
    }
}
