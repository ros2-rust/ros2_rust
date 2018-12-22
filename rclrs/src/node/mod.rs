use crate::error::{RclResult, ToRclResult};
use crate::rcl_bindings::*;
use crate::Context;
use std::ffi::CString;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::RwLock;

pub mod publisher;
pub use self::publisher::*;
pub mod subscription;
pub use self::subscription::*;

pub struct Node<'a> {
    handle: Arc<RwLock<rcl_node_t>>,
    context: PhantomData<&'a Context>,
}

impl<'a> Node<'a> {
    #[allow(clippy::new_ret_no_self)]
    pub fn new(node_name: &str, context: &'a Context) -> RclResult<Node<'a>> {
        Self::new_with_namespace(node_name, "", context)
    }

    pub fn new_with_namespace(node_name: &str, node_ns: &str, context: &'a Context) -> RclResult<Node<'a>> {
        let raw_node_name = CString::new(node_name).unwrap();
        let raw_node_ns = CString::new(node_ns).unwrap();

        let node_handle = Arc::new(RwLock::new(unsafe { rcl_get_zero_initialized_node() }));

        unsafe {
            let node_options = rcl_node_get_default_options();
            rcl_node_init(
                &mut *node_handle.write().unwrap() as *mut _,
                raw_node_name.as_ptr(),
                raw_node_ns.as_ptr(),
                &mut *context.inner.write().unwrap() as *mut _,
                &node_options as *const _,
            )
            .ok()?;
        }

        Ok(Node {
            handle: node_handle,
            context: PhantomData,
        })
    }

    pub fn advertise<T>(&self, topic: &str) -> RclResult<Publisher<T>>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        let mut publisher = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support = T::get_type_support() as *const rosidl_message_type_support_t;

        unsafe {
            let publisher_options = rcl_publisher_get_default_options();
            rcl_publisher_init(
                &mut publisher as *mut _,
                &*self.handle.read().unwrap() as *const _,
                type_support,
                CString::new(topic).unwrap().as_ptr(),
                &publisher_options as *const _,
            ).ok()?;
        }

        Ok(Publisher::<T> {
            node: self,
            publisher,
            message: PhantomData,
        })
    }

    pub fn subscribe<T>(&self, topic: &str) -> RclResult<Subscription<T>>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        let mut subscription = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support = T::get_type_support() as *const rosidl_message_type_support_t;

        unsafe {
            let subscription_options = rcl_subscription_get_default_options();
            rcl_subscription_init(
                &mut subscription as *mut _,
                &*self.handle.read().unwrap() as *const _,
                type_support,
                CString::new(topic).unwrap().as_ptr(),
                &subscription_options as *const _,
            ).ok()?;
        }

        Ok(Subscription::<T> {
            node: self,
            subscription,
            message: PhantomData,
        })
    }
}

impl Drop for Node<'_> {
    fn drop(&mut self) {
        let mut raw_node = self.handle.write().unwrap();
        let raw_node_ptr: *mut _ = &mut *raw_node;
        unsafe {
            rcl_node_fini(raw_node_ptr).unwrap();
        }
    }
}
