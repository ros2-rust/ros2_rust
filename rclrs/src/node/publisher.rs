use crate::error::ToResult;
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::{Node, NodeHandle};
use alloc::sync::Arc;
use core::borrow::Borrow;
use core::marker::PhantomData;
use cstr_core::CString;
use rclrs_common::error::RclReturnCode;

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
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub handle: Arc<PublisherHandle>,
    message: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub fn new(node: &Node, topic: &str, qos: QoSProfile) -> Result<Self, RclReturnCode>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support = T::get_type_support() as *const rosidl_message_type_support_t;
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

    pub fn publish(&self, message: &T) -> Result<(), RclReturnCode> {
        let native_message_ptr = message.get_native_message();
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_publish(
                handle as *mut _,
                native_message_ptr as *mut _,
                core::ptr::null_mut(),
            )
        };
        message.destroy_native_message(native_message_ptr);
        ret.ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Context, Publisher, QOS_PROFILE_DEFAULT};
    use alloc::{borrow::ToOwned, vec::Vec};
    use rclrs_common::error::RclReturnCode;
    use std_msgs;
    use std::{env, println};

    fn default_context() -> Context {
        let args: Vec<CString> = env::args()
            .filter_map(|arg| CString::new(arg).ok())
            .collect();
        println!("<test_publisher> Context args: {:?}", args);
        Context::default(args)
    }

    #[test]
    fn test_new_publisher() -> Result<(), RclReturnCode> {
        let context = default_context();
        let node = context.create_node( "test_new_publisher")?;
        Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT).map(|_x| ())
    }

    #[test]
    fn test_publish() -> Result<(), RclReturnCode> {
        let context = default_context();
        let node = context.create_node("test_publish")?;
        let publisher = Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT)?;
        let mut message = std_msgs::msg::String::default();
        message.data = "Hello world!".to_owned();
        publisher.publish(&message)
    }
}