use std;

use libc::uintptr_t;
use libc::c_int;
use rclrs_common;

use rclrs_common::error::RCLError;
use generate_rcl_error;
use Handle;

use std::any::Any;

pub trait SubscriptionBase: Handle {
    fn create_message(&self) -> Box<rclrs_common::traits::Message>;
    fn callback_fn(&self, message: Box<rclrs_common::traits::Message>) -> ();
}

pub struct Subscription<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    handle: uintptr_t,
    topic: &'static str,
    callback: fn(&T),
    subscription_type: std::marker::PhantomData<T>,
}

impl<T> Handle for Subscription<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    fn handle(&self) -> uintptr_t {
        return self.handle;
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: rclrs_common::traits::MessageDefinition<T> + std::default::Default,
{
    fn create_message(&self) -> Box<rclrs_common::traits::Message> {
        let message: T = Default::default();
        return Box::new(message);
    }

    fn callback_fn(&self, message: Box<rclrs_common::traits::Message>) -> () {
        self.callback_ext(message);
    }
}

impl<T> Subscription<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub fn new(handle: uintptr_t, topic: &'static str, callback: fn(&T)) -> Subscription<T> {
        Subscription::<T> {
            handle: handle,
            topic: topic,
            callback: callback,
            subscription_type: std::marker::PhantomData,
        }
    }

    fn callback_ext(&self, message: Box<rclrs_common::traits::Message>) -> () {
        let msg = message.downcast_ref::<T>().unwrap();
        (self.callback)(msg);
    }
}
