use crate::error::{RclReturnCode, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::{Node, NodeHandle};
use alloc::sync::Arc;
use core::borrow::Borrow;
use core::marker::PhantomData;
use cstr_core::CString;
use rosidl_runtime_rs::{Message, RmwMessage};

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

pub struct PublisherHandle {
    handle: Mutex<rcl_publisher_t>,
    node_handle: Arc<NodeHandle>,
}

impl PublisherHandle {
    fn node_handle(&self) -> &NodeHandle {
        self.node_handle.borrow()
    }

    fn get_mut(&mut self) -> &mut rcl_publisher_t {
        self.handle.get_mut()
    }

    fn lock(&self) -> MutexGuard<rcl_publisher_t> {
        self.handle.lock()
    }

    fn try_lock(&self) -> Option<MutexGuard<rcl_publisher_t>> {
        self.handle.try_lock()
    }
}

impl Drop for PublisherHandle {
    fn drop(&mut self) {
        let handle = self.handle.get_mut();
        let node_handle = &mut *self.node_handle.lock();
        unsafe {
            rcl_publisher_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Main class responsible for publishing data to ROS topics
pub struct Publisher<T>
where
    T: Message,
{
    pub handle: Arc<PublisherHandle>,
    message: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: Message,
{
    pub fn new(node: &Node, topic: &str, qos: QoSProfile) -> Result<Self, RclReturnCode>
    where
        T: Message,
    {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

        unsafe {
            let mut publisher_options = rcl_publisher_get_default_options();
            publisher_options.qos = qos.into();

            rcl_publisher_init(
                &mut publisher_handle as *mut _,
                node_handle as *mut _,
                type_support,
                topic_c_string.as_ptr(),
                &publisher_options as *const _,
            )
            .ok()?;
        }

        let handle = Arc::new(PublisherHandle {
            handle: Mutex::new(publisher_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            message: PhantomData,
        })
    }

    pub fn publish(&self, message: T) -> Result<(), RclReturnCode> {
        let handle = &mut *self.handle.lock();
        let mut rmw_message = message.into_rmw_message();
        let ret = unsafe {
            rcl_publish(
                handle as *mut _,
                &mut rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                core::ptr::null_mut(),
            )
        };
        ret.ok()
    }
}
