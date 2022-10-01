use std::ops::Deref;

use rosidl_runtime_rs::Message;

use crate::rcl_bindings::*;
use crate::{Subscription, ToResult};

/// A message that is owned by the middleware, loaned out for reading.
///
/// It dereferences to a `&T::RmwMsg`. That is, if `T` is already an RMW-native
/// message, it's the same as `&T`, and otherwise it's the corresponding RMW-native
/// message.
///
/// This type is returned by [`Subscription::take_loaned()`] and may be used in
/// subscription callbacks.
///
/// The loan is returned by dropping the `ReadOnlyLoanedMessage`.
pub struct ReadOnlyLoanedMessage<'a, T>
where
    T: Message,
{
    pub(super) msg_ptr: *const T::RmwMsg,
    pub(super) subscription: &'a Subscription<T>,
}

impl<'a, T> Deref for ReadOnlyLoanedMessage<'a, T>
where
    T: Message,
{
    type Target = T::RmwMsg;
    fn deref(&self) -> &Self::Target {
        unsafe { &*self.msg_ptr }
    }
}

impl<'a, T> Drop for ReadOnlyLoanedMessage<'a, T>
where
    T: Message,
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

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl<'a, T> Send for ReadOnlyLoanedMessage<'a, T> where T: Message {}
// SAFETY: This type has no interior mutability, in fact it has no mutability at all.
unsafe impl<'a, T> Sync for ReadOnlyLoanedMessage<'a, T> where T: Message {}
