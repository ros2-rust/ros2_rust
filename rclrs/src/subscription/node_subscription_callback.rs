use rosidl_runtime_rs::Message;

use super::{MessageInfo, SubscriptionHandle};
use crate::{WorkerCommands, RclrsError, ReadOnlyLoanedMessage, RclrsErrorFilter};

use futures::future::BoxFuture;

use std::sync::Arc;

/// An enum capturing the various possible function signatures for subscription callbacks.
///
/// The correct enum variant is deduced by the [`SubscriptionCallback`][1] or
/// [`SubscriptionAsyncCallback`][2] trait.
///
/// [1]: crate::SubscriptionCallback
/// [2]: crate::SubscriptionAsyncCallback
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
    pub(super) fn execute(
        &mut self,
        handle: &Arc<SubscriptionHandle>,
        commands: &WorkerCommands,
    ) -> Result<(), RclrsError> {
        let mut evaluate = || {
            match self {
                NodeSubscriptionCallback::Regular(cb) => {
                    let (msg, _) = handle.take::<T>()?;
                    commands.run(cb(msg));
                }
                NodeSubscriptionCallback::RegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take::<T>()?;
                    commands.run(cb(msg, msg_info));
                }
                NodeSubscriptionCallback::Boxed(cb) => {
                    let (msg, _) = handle.take_boxed::<T>()?;
                    commands.run(cb(msg));
                }
                NodeSubscriptionCallback::BoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_boxed::<T>()?;
                    commands.run(cb(msg, msg_info));
                }
                NodeSubscriptionCallback::Loaned(cb) => {
                    let (msg, _) = handle.take_loaned::<T>()?;
                    commands.run(cb(msg));
                }
                NodeSubscriptionCallback::LoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_loaned::<T>()?;
                    commands.run(cb(msg, msg_info));
                }
            }
            Ok(())
        };

        evaluate().take_failed_ok()
    }
}
