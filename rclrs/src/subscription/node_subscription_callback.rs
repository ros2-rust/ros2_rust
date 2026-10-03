use rosidl_runtime_rs::Message;

use super::{MessageInfo, SubscriptionHandle};
use crate::{RclrsError, ReadOnlyLoanedMessage};

use futures::future::BoxFuture;

use std::sync::Arc;

/// An enum capturing the various possible function signatures for subscription callbacks
/// that can be used with the async worker.
///
/// The correct enum variant is deduced by the [`IntoNodeSubscriptionCallback`][1] or
/// [`IntoAsyncSubscriptionCallback`][2] trait.
///
/// [1]: crate::IntoNodeSubscriptionCallback
/// [2]: crate::IntoAsyncSubscriptionCallback
pub enum NodeSubscriptionCallback<T: Message> {
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
    Loaned(Box<dyn FnMut(ReadOnlyLoanedMessage<T>) -> BoxFuture<'static, ()> + Send>),
    /// A callback with the loaned message and the message info as arguments.
    #[allow(clippy::type_complexity)]
    LoanedWithMessageInfo(
        Box<dyn FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) -> BoxFuture<'static, ()> + Send>,
    ),
}

impl<T: Message> NodeSubscriptionCallback<T> {
    pub(super) fn take_and_call(
        &mut self,
        handle: &Arc<SubscriptionHandle>,
    ) -> Result<Option<BoxFuture<'static, ()>>, RclrsError> {
        let mut evaluate = || {
            let task = match self {
                NodeSubscriptionCallback::Regular(cb) => {
                    let (msg, _) = handle.take::<T>()?;
                    cb(msg)
                }
                NodeSubscriptionCallback::RegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take::<T>()?;
                    cb(msg, msg_info)
                }
                NodeSubscriptionCallback::Boxed(cb) => {
                    let (msg, _) = handle.take_boxed::<T>()?;
                    cb(msg)
                }
                NodeSubscriptionCallback::BoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_boxed::<T>()?;
                    cb(msg, msg_info)
                }
                NodeSubscriptionCallback::Loaned(cb) => {
                    let (msg, _) = handle.take_loaned::<T>()?;
                    cb(msg)
                }
                NodeSubscriptionCallback::LoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_loaned::<T>()?;
                    cb(msg, msg_info)
                }
            };
            Ok::<_, RclrsError>(task)
        };

        match evaluate() {
            Ok(task) => Ok(Some(task)),
            Err(err) if err.is_take_failed() => Ok(None),
            Err(err) => Err(err),
        }
    }
}
