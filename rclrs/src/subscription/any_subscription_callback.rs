use rosidl_runtime_rs::Message;

use super::{MessageInfo, SubscriptionHandle};
use crate::{
    error::ToResult,
    rcl_bindings::*,
    ReadOnlyLoanedMessage, ExecutorCommands, RclrsError, RclReturnCode,
};

use futures::future::BoxFuture;

use std::sync::Arc;

/// An enum capturing the various possible function signatures for subscription callbacks.
///
/// The correct enum variant is deduced by the [`SubscriptionCallback`][1] or
/// [`SubscriptionAsyncCallback`][2] trait.
///
/// [1]: crate::SubscriptionCallback
/// [2]: crate::SubscriptionAsyncCallback
pub enum AnySubscriptionCallback<T>
where
    T: Message,
{
    /// A callback with only the message as an argument.
    Regular(Box<dyn FnMut(T) -> BoxFuture<'static, ()> + Send>),
    /// A callback with the message and the message info as arguments.
    RegularWithMessageInfo(Box<dyn FnMut(T, MessageInfo) -> BoxFuture<'static, ()> + Send>),
    /// A callback with only the boxed message as an argument.
    Boxed(Box<dyn FnMut(Box<T>) -> BoxFuture<'static, ()> + Send>),
    /// A callback with the boxed message and the message info as arguments.
    BoxedWithMessageInfo(Box<dyn FnMut(Box<T>, MessageInfo) -> BoxFuture<'static, ()> + Send>),
    /// A callback with only the loaned message as an argument.
    #[allow(clippy::type_complexity)]
    Loaned(Box<dyn for<'a> FnMut(ReadOnlyLoanedMessage<T>) -> BoxFuture<'static, ()> + Send>),
    /// A callback with the loaned message and the message info as arguments.
    #[allow(clippy::type_complexity)]
    LoanedWithMessageInfo(Box<dyn for<'a> FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) -> BoxFuture<'static, ()> + Send>),
}

impl<T: Message> AnySubscriptionCallback<T> {
    pub(super) fn execute(
        &mut self,
        handle: &Arc<SubscriptionHandle>,
        commands: &Arc<ExecutorCommands>,
    ) -> Result<(), RclrsError> {
        // Immediately evaluated closure, to handle SubscriptionTakeFailed
        // outside this match
        let mut evaluate = || {
            match self {
                AnySubscriptionCallback::Regular(cb) => {
                    let (msg, _) = Self::take(handle)?;
                    commands.run(cb(msg));
                }
                AnySubscriptionCallback::RegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = Self::take(handle)?;
                    commands.run(cb(msg, msg_info));
                }
                AnySubscriptionCallback::Boxed(cb) => {
                    let (msg, _) = Self::take_boxed(handle)?;
                    commands.run(cb(msg));
                }
                AnySubscriptionCallback::BoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = Self::take_boxed(handle)?;
                    commands.run(cb(msg, msg_info));
                }
                AnySubscriptionCallback::Loaned(cb) => {
                    let (msg, _) = Self::take_loaned(handle)?;
                    commands.run(cb(msg));
                }
                AnySubscriptionCallback::LoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = Self::take_loaned(handle)?;
                    commands.run(cb(msg, msg_info));
                }
            }
            Ok(())
        };

        match evaluate() {
            Err(RclrsError::RclError {
                code: RclReturnCode::SubscriptionTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // subscription was ready, so it shouldn't be an error.
                Ok(())
            }
            other => other,
        }
    }

    /// Fetches a new message.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1].
    ///
    /// [1]: crate::RclrsError
    //
    // ```text
    // +-------------+
    // | rclrs::take |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rcl_take   |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rmw_take   |
    // +-------------+
    // ```
    fn take(handle: &SubscriptionHandle) -> Result<(T, MessageInfo), RclrsError> {
        let mut rmw_message = <T as Message>::RmwMsg::default();
        let message_info = Self::take_inner(handle, &mut rmw_message)?;
        Ok((T::from_rmw_message(rmw_message), message_info))
    }

    /// This is a version of take() that returns a boxed message.
    ///
    /// This can be more efficient for messages containing large arrays.
    fn take_boxed(handle: &SubscriptionHandle) -> Result<(Box<T>, MessageInfo), RclrsError> {
        let mut rmw_message = Box::<<T as Message>::RmwMsg>::default();
        let message_info = Self::take_inner(handle, &mut *rmw_message)?;
        // TODO: This will still use the stack in general. Change signature of
        // from_rmw_message to allow placing the result in a Box directly.
        let message = Box::new(T::from_rmw_message(*rmw_message));
        Ok((message, message_info))
    }

    // Inner function, to be used by both regular and boxed versions.
    fn take_inner(
        handle: &SubscriptionHandle,
        rmw_message: &mut <T as Message>::RmwMsg,
    ) -> Result<MessageInfo, RclrsError> {
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        let rcl_subscription = &mut *handle.lock();
        unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                rcl_subscription,
                rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?
        };
        Ok(MessageInfo::from_rmw_message_info(&message_info))
    }

    /// Obtains a read-only handle to a message owned by the middleware.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1].
    ///
    /// This is the counterpart to [`Publisher::borrow_loaned_message()`][2]. See its documentation
    /// for more information.
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::Publisher::borrow_loaned_message
    fn take_loaned(
        handle: &Arc<SubscriptionHandle>,
    ) -> Result<(ReadOnlyLoanedMessage<T>, MessageInfo), RclrsError> {
        let mut msg_ptr = std::ptr::null_mut();
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        unsafe {
            // SAFETY: The third argument (message_info) and fourth argument (allocation) may be null.
            // The second argument (loaned_message) contains a null ptr as expected.
            rcl_take_loaned_message(
                &*handle.lock(),
                &mut msg_ptr,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?;
        }
        let read_only_loaned_msg = ReadOnlyLoanedMessage {
            msg_ptr: msg_ptr as *const T::RmwMsg,
            handle: Arc::clone(handle),
        };
        Ok((
            read_only_loaned_msg,
            MessageInfo::from_rmw_message_info(&message_info),
        ))
    }
}
