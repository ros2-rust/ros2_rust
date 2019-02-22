use crate::error::{RclError, RclResult, ToRclResult};
use crate::qos::QoSProfile;
use crate::{Context, ContextHandle, Handle};
use crate::rcl_bindings::*;
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
    context: Rc<ContextHandle>,
    subscriptions: Vec<Weak<SubscriptionBase>>,
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

    pub fn create_publisher<T>(&self, topic: &str, qos: QoSProfile) -> RclResult<Publisher<T>>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        Publisher::<T>::new(self, topic, qos)
    }

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
            .push(Rc::downgrade(&subscription) as Weak<SubscriptionBase>);
        Ok(subscription)
    }

    pub fn spin(&self) -> RclResult {
        let context_handle = &mut *self.context.get_mut();
        while unsafe { rcl_context_is_valid(context_handle) } {
            if let Some(error) = self.spin_once(500).err() {
                match error {
                    RclError::Timeout => continue,
                    _ => return Err(error),
                }
            }
        }

        Ok(())
    }

    pub fn spin_once(&self, timeout: i64) -> RclResult {
        let mut wait_set_handle = unsafe { rcl_get_zero_initialized_wait_set() };

        let number_of_subscriptions = self.subscriptions.len();
        let number_of_guard_conditions = 0;
        let number_of_timers = 0;
        let number_of_clients = 0;
        let number_of_services = 0;

        unsafe {
            rcl_wait_set_init(
                &mut wait_set_handle as *mut _,
                number_of_subscriptions,
                number_of_guard_conditions,
                number_of_timers,
                number_of_clients,
                number_of_services,
                rcutils_get_default_allocator(),
            )
            .ok()?;
        }

        unsafe {
            rcl_wait_set_clear(&mut wait_set_handle as *mut _).ok()?;
        }

        for subscription in &self.subscriptions {
            if let Some(subscription) = subscription.upgrade() {
                let subscription_handle = &*subscription.handle().get();
                unsafe {
                    rcl_wait_set_add_subscription(
                        &mut wait_set_handle as *mut _,
                        subscription_handle as *const _,
                        std::ptr::null_mut(),
                    )
                    .ok()?;
                }
            }
        }

        unsafe {
            rcl_wait(&mut wait_set_handle as *mut _, timeout).ok()?;
        }

        for subscription in &self.subscriptions {
            if let Some(subscription) = subscription.upgrade() {
                let mut message = subscription.create_message();
                let result = subscription.take(&mut *message).unwrap();
                if result {
                    subscription.callback_fn(message);
                }
            }
        }
        unsafe {
            rcl_wait_set_fini(&mut wait_set_handle as *mut _).ok()?;
        }

        Ok(())
    }
}
