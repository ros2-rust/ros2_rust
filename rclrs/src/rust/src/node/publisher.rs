use std::marker::PhantomData;

use libc::uintptr_t;
use libc::c_int;
use rclrs_common;

use rclrs_common::error::RCLError;
use generate_rcl_error;

#[link(name = "rclrs")]
extern "C" {
    fn rclrs_native_publish(publisher_handle: uintptr_t, message_handle: uintptr_t) -> c_int;
}

pub struct Publisher<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    handle: uintptr_t,
    topic: &'static str,
    publisher_type: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub fn new(handle: uintptr_t, topic: &'static str) -> Publisher<T> {
        Publisher::<T> {
            handle: handle,
            topic: topic,
            publisher_type: PhantomData,
        }
    }

    pub fn publish(self: &Self, message: &T) -> Result<(), RCLError> {
        let native_message_ptr = message.get_native_message();
        let ret = unsafe { rclrs_native_publish(self.handle, native_message_ptr) };
        message.destroy_native_message(native_message_ptr);
        match ret {
            0 => Ok(()),
            _ => Err(generate_rcl_error(ret)),
        }
    }
}
