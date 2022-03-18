use crate::error::ToResult;
use crate::qos::QoSProfile;
use crate::{rcl_bindings::*, RclReturnCode};
use crate::{Node, NodeHandle};
use alloc::boxed::Box;
use alloc::sync::Arc;
use core::borrow::Borrow;
use core::marker::PhantomData;
use cstr_core::CString;
use rosidl_runtime_rs::{Message, RmwMessage};

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

pub struct SubscriptionHandle {
    handle: Mutex<rcl_subscription_t>,
    node_handle: Arc<NodeHandle>,
}

impl SubscriptionHandle {
    fn node_handle(&self) -> &NodeHandle {
        self.node_handle.borrow()
    }

    pub fn get_mut(&mut self) -> &mut rcl_subscription_t {
        self.handle.get_mut()
    }

    pub fn lock(&self) -> MutexGuard<rcl_subscription_t> {
        self.handle.lock()
    }

    pub fn try_lock(&self) -> Option<MutexGuard<rcl_subscription_t>> {
        self.handle.try_lock()
    }
}

impl Drop for SubscriptionHandle {
    fn drop(&mut self) {
        let handle = self.handle.get_mut();
        let node_handle = &mut *self.node_handle.lock();
        unsafe {
            rcl_subscription_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Trait to be implemented by concrete Subscriber structs
/// See [`Subscription<T>`] for an example
pub trait SubscriptionBase {
    fn handle(&self) -> &SubscriptionHandle;
    fn execute(&self) -> Result<(), RclReturnCode>;
}

/// Main class responsible for subscribing to topics and receiving data over IPC in ROS
pub struct Subscription<T>
where
    T: Message,
{
    pub handle: Arc<SubscriptionHandle>,
    // The callback's lifetime should last as long as we need it to
    pub callback: Mutex<Box<dyn FnMut(&T) + 'static>>,
    message: PhantomData<T>,
}

impl<T> Subscription<T>
where
    T: Message,
{
    pub fn new<F>(
        node: &Node,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Self, RclReturnCode>
    where
        T: Message,
        F: FnMut(&T) + Sized + 'static,
    {
        let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

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

        let handle = Arc::new(SubscriptionHandle {
            handle: Mutex::new(subscription_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(Box::new(callback)),
            message: PhantomData,
        })
    }

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
    pub fn take(&self) -> Result<T, RclReturnCode> {
        let mut rmw_message = <T as Message>::RmwMsg::default();
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_take(
                handle as *const _,
                &mut rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            )
        };
        ret.ok()?;
        Ok(T::from_rmw_message(rmw_message))
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: Message,
{
    fn handle(&self) -> &SubscriptionHandle {
        self.handle.borrow()
    }

    fn execute(&self) -> Result<(), RclReturnCode> {
        let msg = self.take()?;
        (&mut *self.callback.lock())(&msg);
        Ok(())
    }
}
