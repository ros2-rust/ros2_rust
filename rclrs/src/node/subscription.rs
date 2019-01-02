use crate::error::{RclResult, ToRclResult};
use rcl_sys::*;
use std::marker::PhantomData;
use super::Node;

pub struct Subscription<'a, 'b, T>
where
    T: rclrs_common::traits::Message,
{
    pub(crate) node: &'a Node<'b>,
    pub(crate) subscription: rcl_subscription_t,
    pub(crate) message: PhantomData<T>,
}

impl<'a, 'b, T> Subscription<'a, 'b, T>
where
    T: rclrs_common::traits::Message,
{
    pub fn take(&self, message: &mut T) -> RclResult {
        let message_handle = message.get_native_message();
        let ret = unsafe {
            rcl_take(&self.subscription as *const _, message_handle as *mut _, std::ptr::null_mut())
        };
        message.read_handle(message_handle);
        message.destroy_native_message(message_handle);
        ret.ok()
    }
}

impl<'a, 'b, T> Drop for Subscription<'a, 'b, T>
where
    T: rclrs_common::traits::Message,
{
    fn drop(&mut self) {
        unsafe {
            rcl_subscription_fini(
                &mut self.subscription as *mut _,
                &mut *self.node.handle.write().unwrap() as *mut _,
            ).unwrap();
        }
    }
}
