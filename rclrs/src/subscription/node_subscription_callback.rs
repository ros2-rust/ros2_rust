use rosidl_runtime_rs::Message;

use super::{MessageInfo, SubscriptionHandle};
use crate::{RclrsError, RclrsErrorFilter, ReadOnlyLoanedMessage, WorkerCommands};

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
    // Sync variants — callback is called directly, no async wrapping.
    /// A synchronous callback with only the message as an argument.
    SyncRegular(Box<dyn FnMut(T) + Send>),
    /// A synchronous callback with the message and the message info as arguments.
    SyncRegularWithMessageInfo(Box<dyn FnMut(T, MessageInfo) + Send>),
    /// A synchronous callback with only the boxed message as an argument.
    SyncBoxed(Box<dyn FnMut(Box<T>) + Send>),
    /// A synchronous callback with the boxed message and the message info as arguments.
    SyncBoxedWithMessageInfo(Box<dyn FnMut(Box<T>, MessageInfo) + Send>),
    /// A synchronous callback with only the loaned message as an argument.
    SyncLoaned(Box<dyn FnMut(ReadOnlyLoanedMessage<T>) + Send>),
    /// A synchronous callback with the loaned message and the message info as arguments.
    SyncLoanedWithMessageInfo(Box<dyn FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) + Send>),
    // Async variants — callback returns a future, dispatched via the executor task queue.
    /// An async callback with only the message as an argument.
    AsyncRegular(Box<dyn FnMut(T) -> BoxFuture<'static, ()> + Send>),
    /// An async callback with the message and the message info as arguments.
    AsyncRegularWithMessageInfo(Box<dyn FnMut(T, MessageInfo) -> BoxFuture<'static, ()> + Send>),
    /// An async callback with only the boxed message as an argument.
    AsyncBoxed(Box<dyn FnMut(Box<T>) -> BoxFuture<'static, ()> + Send>),
    /// An async callback with the boxed message and the message info as arguments.
    AsyncBoxedWithMessageInfo(
        Box<dyn FnMut(Box<T>, MessageInfo) -> BoxFuture<'static, ()> + Send>,
    ),
    /// An async callback with only the loaned message as an argument.
    #[allow(clippy::type_complexity)]
    AsyncLoaned(Box<dyn FnMut(ReadOnlyLoanedMessage<T>) -> BoxFuture<'static, ()> + Send>),
    /// An async callback with the loaned message and the message info as arguments.
    #[allow(clippy::type_complexity)]
    AsyncLoanedWithMessageInfo(
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
                // Sync variants — call directly, no async overhead
                NodeSubscriptionCallback::SyncRegular(cb) => {
                    let (msg, _) = handle.take::<T>()?;
                    cb(msg);
                }
                NodeSubscriptionCallback::SyncRegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take::<T>()?;
                    cb(msg, msg_info);
                }
                NodeSubscriptionCallback::SyncBoxed(cb) => {
                    let (msg, _) = handle.take_boxed::<T>()?;
                    cb(msg);
                }
                NodeSubscriptionCallback::SyncBoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_boxed::<T>()?;
                    cb(msg, msg_info);
                }
                NodeSubscriptionCallback::SyncLoaned(cb) => {
                    let (msg, _) = handle.take_loaned::<T>()?;
                    cb(msg);
                }
                NodeSubscriptionCallback::SyncLoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_loaned::<T>()?;
                    cb(msg, msg_info);
                }
                // Async variants — dispatch through executor task queue
                NodeSubscriptionCallback::AsyncRegular(cb) => {
                    let (msg, _) = handle.take::<T>()?;
                    commands.run_async(cb(msg));
                }
                NodeSubscriptionCallback::AsyncRegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take::<T>()?;
                    commands.run_async(cb(msg, msg_info));
                }
                NodeSubscriptionCallback::AsyncBoxed(cb) => {
                    let (msg, _) = handle.take_boxed::<T>()?;
                    commands.run_async(cb(msg));
                }
                NodeSubscriptionCallback::AsyncBoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_boxed::<T>()?;
                    commands.run_async(cb(msg, msg_info));
                }
                NodeSubscriptionCallback::AsyncLoaned(cb) => {
                    let (msg, _) = handle.take_loaned::<T>()?;
                    commands.run_async(cb(msg));
                }
                NodeSubscriptionCallback::AsyncLoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_loaned::<T>()?;
                    commands.run_async(cb(msg, msg_info));
                }
            }
            Ok(())
        };

        evaluate().take_failed_ok()
    }
}
