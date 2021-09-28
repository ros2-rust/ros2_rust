use crate::error::{RclError, ToResult};
use crate::qos::QoSProfile;
use crate::{Handle, Node, NodeHandle};
use crate::rcl_bindings::*;
use std::borrow::Borrow;
use std::cell::{Ref, RefCell, RefMut};
use std::ffi::CString;
use std::marker::PhantomData;
use std::rc::Rc;
use anyhow::Result;

pub struct SubscriptionHandle {
    handle: RefCell<rcl_subscription_t>,
    node_handle: Rc<NodeHandle>,
}

impl SubscriptionHandle {
    fn node_handle(&self) -> &NodeHandle {
        self.node_handle.borrow()
    }
}

impl<'a> Handle<rcl_subscription_t> for &'a SubscriptionHandle {
    type DerefT = Ref<'a, rcl_subscription_t>;
    type DerefMutT = RefMut<'a, rcl_subscription_t>;

    fn get(self) -> Self::DerefT {
        self.handle.borrow()
    }

    fn get_mut(self) -> Self::DerefMutT {
        self.handle.borrow_mut()
    }
}

impl Drop for SubscriptionHandle {
    fn drop(&mut self) {
        let handle = &mut *self.get_mut();
        let node_handle = &mut *self.node_handle().get_mut();
        unsafe {
            rcl_subscription_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Trait to be implemented by concrete Subscriber structs
/// See [`Subscription<T>`] for an example
pub trait SubscriptionBase {
    fn handle(&self) -> &SubscriptionHandle;
    fn create_message(&self) -> Box<dyn rclrs_common::traits::Message>;
    fn callback_fn(&self, message: Box<dyn rclrs_common::traits::Message>) -> ();

    /// Ask RMW for the data
    ///
    /// +-------------+
    /// | rclrs::take |
    /// +------+------+
    ///        |
    ///        |
    /// +------v------+
    /// |  rcl_take   |
    /// +------+------+
    ///        |
    ///        |
    /// +------v------+
    /// |  rmw_take   |
    /// +-------------+
    fn take(&self, message: &mut dyn rclrs_common::traits::Message) -> Result<bool, RclError> {
        let handle = &*self.handle().get();
        let message_handle = message.get_native_message();

        let result = unsafe {
            rcl_take(
                handle as *const _,
                message_handle as *mut _,
                std::ptr::null_mut(),
                std::ptr::null_mut()
            )
        };

        let result = match result.into() {
            RclError::Ok => {
                message.read_handle(message_handle);
                Ok(true)
            }
            RclError::SubscriptionTakeFailed => Ok(false),
            error => Err(error),
        };

        message.destroy_native_message(message_handle);

        result
    }
}

/// Main class responsible for subscribing to topics and receiving data over IPC in ROS
pub struct Subscription<T>
where
    T: rclrs_common::traits::Message,
{
    pub handle: Rc<SubscriptionHandle>,
    // The callback's lifetime should last as long as we need it to
    pub callback: RefCell<Box<dyn FnMut(&T) + 'static>>,
    message: PhantomData<T>,
}

impl<T> Subscription<T>
where
    T: rclrs_common::traits::Message,
{
    pub fn new<F>(node: &Node, topic: &str, qos: QoSProfile, callback: F) -> Result<Self, RclError>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
        F: FnMut(&T) + Sized + 'static,
    {
        let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support = T::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.get_mut();

        unsafe {
            let mut subscription_options = rcl_subscription_get_default_options();
            subscription_options.qos = qos.into();
            rcl_subscription_init(
                &mut subscription_handle as *mut _,
                node_handle as *mut _,
                type_support,
                topic_c_string.as_ptr(),
                &subscription_options as *const _,
            )
            .ok()?;
        }

        let handle = Rc::new(SubscriptionHandle {
            handle: RefCell::new(subscription_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            callback: RefCell::new(Box::new(callback)),
            message: PhantomData,
        })
    }

    pub fn take(&self, message: &mut T) -> Result<(), RclError> {
        let handle = &*self.handle.get();
        let message_handle = message.get_native_message();
        let ret = unsafe {
            rcl_take(
                handle as *const _,
                message_handle as *mut _,
                std::ptr::null_mut(),
                std::ptr::null_mut()
            )
        };
        message.read_handle(message_handle);
        message.destroy_native_message(message_handle);
        ret.ok()
    }

    fn callback_ext(&self, message: Box<dyn rclrs_common::traits::Message>) {
        let msg = message.downcast_ref::<T>().unwrap();
        (&mut *self.callback.borrow_mut())(msg);
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: rclrs_common::traits::MessageDefinition<T> + std::default::Default,
{
    fn handle(&self) -> &SubscriptionHandle {
        self.handle.borrow()
    }

    fn create_message(&self) -> Box<dyn rclrs_common::traits::Message> {
        Box::new(T::default())
    }

    fn callback_fn(&self, message: Box<dyn rclrs_common::traits::Message>) {
        self.callback_ext(message);
    }
}
