use std::ops::{Deref, DerefMut};

use rosidl_runtime_rs::RmwMessage;

use crate::rcl_bindings::*;
use crate::{Publisher, RclrsError, ToResult};

/// A message that is owned by the middleware, loaned for publishing.
///
/// It dereferences to a `&mut T`.
///
/// This type is returned by [`Publisher::borrow_loaned_message()`], see the documentation of
/// that function for more information.
///
/// The loan is returned by dropping the message or [publishing it][1].
///
/// [1]: LoanedMessage::publish
pub struct LoanedMessage<'a, T>
where
    T: RmwMessage,
{
    pub(super) msg_ptr: *mut T,
    pub(super) publisher: &'a Publisher<T>,
}

impl<'a, T> Deref for LoanedMessage<'a, T>
where
    T: RmwMessage,
{
    type Target = T;
    fn deref(&self) -> &Self::Target {
        // SAFETY: msg_ptr is a valid pointer, obtained through rcl_borrow_loaned_message.
        unsafe { &*self.msg_ptr }
    }
}

impl<'a, T> DerefMut for LoanedMessage<'a, T>
where
    T: RmwMessage,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: msg_ptr is a valid pointer, obtained through rcl_borrow_loaned_message.
        unsafe { &mut *self.msg_ptr }
    }
}

impl<'a, T> Drop for LoanedMessage<'a, T>
where
    T: RmwMessage,
{
    fn drop(&mut self) {
        // Check whether the loan was already returned with
        // rcl_publish_loaned_message()
        if !self.msg_ptr.is_null() {
            unsafe {
                // SAFETY: These two pointers are valid, and the msg_ptr is not used afterwards.
                rcl_return_loaned_message_from_publisher(
                    &*self.publisher.rcl_publisher_mtx.lock().unwrap(),
                    self.msg_ptr as *mut _,
                )
                .ok()
                .unwrap()
            }
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl<'a, T> Send for LoanedMessage<'a, T> where T: RmwMessage {}
// SAFETY: There is no interior mutability in this type. All mutation happens through &mut references.
unsafe impl<'a, T> Sync for LoanedMessage<'a, T> where T: RmwMessage {}

impl<'a, T> LoanedMessage<'a, T>
where
    T: RmwMessage,
{
    /// Publishes the loaned message, falling back to regular publishing if needed.
    pub fn publish(mut self) -> Result<(), RclrsError> {
        unsafe {
            // SAFETY: These two pointers are valid, and the msg_ptr is not used afterwards.
            rcl_publish_loaned_message(
                &*self.publisher.rcl_publisher_mtx.lock().unwrap(),
                self.msg_ptr as *mut _,
                std::ptr::null_mut(),
            )
            .ok()?;
        }
        // Set the msg_ptr to null, as a signal to the drop impl that this
        // loan was already returned.
        self.msg_ptr = std::ptr::null_mut();
        Ok(())
    }
}
