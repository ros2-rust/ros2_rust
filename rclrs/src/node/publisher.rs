use crate::error::{RclResult, ToRclResult};
use crate::rcl_bindings::*;
use std::marker::PhantomData;
use super::Node;

pub struct Publisher<'a, 'b, T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub(crate) node: &'a Node<'b>,
    pub(crate) publisher: rcl_publisher_t,
    pub(crate) message: PhantomData<T>,
}

impl<'a, 'b, T> Publisher<'a, 'b, T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub fn publish(&self, message: &T) -> RclResult {
        let native_message_ptr = message.get_native_message();
        let ret = unsafe {
            rcl_publish(&self.publisher as *const _, native_message_ptr as *mut _)
        };
        message.destroy_native_message(native_message_ptr);
        ret.ok()
    }
}

impl<'a, 'b, T> Drop for Publisher<'a, 'b, T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    fn drop(&mut self) {
        unsafe {
            rcl_publisher_fini(
                &mut self.publisher as *mut _,
                &mut *self.node.handle.write().unwrap() as *mut _,
            ).unwrap();
        }
    }
}
