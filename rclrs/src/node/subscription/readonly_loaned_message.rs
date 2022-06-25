use crate::rcl_bindings::*;
use crate::{Subscription, ToResult};

use rosidl_runtime_rs::RmwMessage;

use std::ops::Deref;

/// A message that is owned by the middleware, loaned out for reading.
///
/// It dereferences to a `&T`.
///
/// This type is returned by [`Subscription::take_loaned_message()`].
///
/// The loan is returned by dropping the `ReadOnlyLoanedMessage`.
pub struct ReadOnlyLoanedMessage<'a, T>
where
    T: RmwMessage,
{
    pub(super) msg_ptr: *const T,
    pub(super) subscription: &'a Subscription<T>,
}

impl<'a, T> Deref for ReadOnlyLoanedMessage<'a, T>
where
    T: RmwMessage,
{
    type Target = T;
    fn deref(&self) -> &Self::Target {
        unsafe { &*self.msg_ptr }
    }
}

impl<'a, T> Drop for ReadOnlyLoanedMessage<'a, T>
where
    T: RmwMessage,
{
    fn drop(&mut self) {
        unsafe {
            rcl_return_loaned_message_from_subscription(
                &*self.subscription.handle.lock(),
                self.msg_ptr as *mut _,
            )
            .ok()
            .unwrap();
        }
    }
}
