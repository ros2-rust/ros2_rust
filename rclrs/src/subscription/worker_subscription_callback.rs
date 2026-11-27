use rosidl_runtime_rs::Message;

use crate::{
    subscription::SubscriptionHandle, MessageInfo, RclrsError, RclrsErrorFilter,
    ReadOnlyLoanedMessage,
};

use std::{any::Any, sync::Arc};

type RegularCallback<T, Payload> = Box<dyn FnMut(&mut Payload, T) + Send>;
type RegularWithInfoCallback<T, Payload> = Box<dyn FnMut(&mut Payload, T, MessageInfo) + Send>;
type BoxedCallback<T, Payload> = Box<dyn FnMut(&mut Payload, Box<T>) + Send>;
type BoxedWithInfoCallback<T, Payload> = Box<dyn FnMut(&mut Payload, Box<T>, MessageInfo) + Send>;
type LoanedCallback<T, Payload> = Box<dyn FnMut(&mut Payload, ReadOnlyLoanedMessage<T>) + Send>;
type LoanedWithInfoCallback<T, Payload> =
    Box<dyn FnMut(&mut Payload, ReadOnlyLoanedMessage<T>, MessageInfo) + Send>;

/// An enum capturing the various possible function signatures for subscription
/// callbacks that can be used by a [`Worker`][crate::Worker].
///
/// The correct enum variant is deduced by the [`IntoWorkerSubscriptionCallback`][1] trait.
///
/// [1]: crate::IntoWorkerSubscriptionCallback
pub enum WorkerSubscriptionCallback<T: Message, Payload> {
    /// A callback that only takes the payload and the message as arguments.
    Regular(RegularCallback<T, Payload>),
    /// A callback with the payload, message, and the message info as arguments.
    RegularWithMessageInfo(RegularWithInfoCallback<T, Payload>),
    /// A callback with only the payload and boxed message as arguments.
    Boxed(BoxedCallback<T, Payload>),
    /// A callback with the payload, boxed message, and the message info as arguments.
    BoxedWithMessageInfo(BoxedWithInfoCallback<T, Payload>),
    /// A callback with only the payload and loaned message as arguments.
    Loaned(LoanedCallback<T, Payload>),
    /// A callback with the payload, loaned message, and the message info as arguments.
    LoanedWithMessageInfo(LoanedWithInfoCallback<T, Payload>),
}

impl<T: Message, Payload: 'static> WorkerSubscriptionCallback<T, Payload> {
    pub(super) fn execute(
        &mut self,
        handle: &Arc<SubscriptionHandle>,
        any_payload: &mut dyn Any,
    ) -> Result<(), RclrsError> {
        let Some(payload) = any_payload.downcast_mut::<Payload>() else {
            return Err(RclrsError::InvalidPayload {
                expected: std::any::TypeId::of::<Payload>(),
                received: (*any_payload).type_id(),
            });
        };

        let mut evalute = || {
            match self {
                WorkerSubscriptionCallback::Regular(cb) => {
                    let (msg, _) = handle.take::<T>()?;
                    cb(payload, msg);
                }
                WorkerSubscriptionCallback::RegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take::<T>()?;
                    cb(payload, msg, msg_info);
                }
                WorkerSubscriptionCallback::Boxed(cb) => {
                    let (msg, _) = handle.take_boxed::<T>()?;
                    cb(payload, msg);
                }
                WorkerSubscriptionCallback::BoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_boxed::<T>()?;
                    cb(payload, msg, msg_info);
                }
                WorkerSubscriptionCallback::Loaned(cb) => {
                    let (msg, _) = handle.take_loaned::<T>()?;
                    cb(payload, msg);
                }
                WorkerSubscriptionCallback::LoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = handle.take_loaned::<T>()?;
                    cb(payload, msg, msg_info);
                }
            }
            Ok(())
        };

        evalute().take_failed_ok()
    }
}
