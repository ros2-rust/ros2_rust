use rosidl_runtime_rs::Message;

use crate::{
    subscription::SubscriptionHandle, RclrsError,
    ReadOnlyLoanedMessage, MessageInfo, AnySubscriptionCallback, RclrsErrorFilter,
};

use std::{
    any::Any,
    sync::Arc,
};

pub enum WorkerSubscriptionCallback<T: Message, Payload> {
    /// A callback that only takes the payload and the message as arguments.
    Regular(Box<dyn FnMut(&mut Payload, T) + Send>),
    /// A callback with the payload, message, and the message info as arguments.
    RegularWithMessageInfo(Box<dyn FnMut(&mut Payload, T, MessageInfo) + Send>),
    /// A callback with only the payload and boxed message as arguments.
    Boxed(Box<dyn FnMut(&mut Payload, Box<T>) + Send>),
    /// A callback with the payload, boxed message, and the message info as arguments.
    BoxedWithMessageInfo(Box<dyn FnMut(&mut Payload, Box<T>, MessageInfo) + Send>),
    /// A callback with only the payload and loaned message as arguments.
    Loaned(Box<dyn FnMut(&mut Payload, ReadOnlyLoanedMessage<T>) + Send>),
    /// A callback with the payload, loaned message, and the message info as arguments.
    LoanedWithMessageInfo(
        Box<dyn FnMut(&mut Payload, ReadOnlyLoanedMessage<T>, MessageInfo) + Send>,
    ),
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
                received: (*any_payload).type_id()
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

pub trait IntoWorkerSubscriptionCallback<T, Payload, Args>: Send + 'static
where
    T: Message,
    Payload: 'static,
{
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload>;
}

trait WorkerSubscriptionArgs<T, Payload, Func>
where
    T: Message,
    Payload: 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, Payload>;
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (T,)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(T) + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message| func(message));
        WorkerSubscriptionCallback::Regular(f).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Payload, T)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, T) + Send + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::Regular(Box::new(func)).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (T, MessageInfo)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(T, MessageInfo) + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message, info| func(message, info));
        WorkerSubscriptionCallback::RegularWithMessageInfo(f).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Payload, T, MessageInfo)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, T, MessageInfo) + Send + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::RegularWithMessageInfo(Box::new(func)).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Box<T>,)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(Box<T>) + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message| func(message));
        WorkerSubscriptionCallback::Boxed(f).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Payload, Box<T>)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, Box<T>) + Send + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::Boxed(Box::new(func)).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Box<T>, MessageInfo)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(Box<T>, MessageInfo) + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message, info| func(message, info));
        WorkerSubscriptionCallback::BoxedWithMessageInfo(f).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Payload, Box<T>, MessageInfo)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, Box<T>, MessageInfo) + Send + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::BoxedWithMessageInfo(Box::new(func)).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (ReadOnlyLoanedMessage<T>,)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(ReadOnlyLoanedMessage<T>) + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message| func(message));
        WorkerSubscriptionCallback::Loaned(f).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Payload, ReadOnlyLoanedMessage<T>)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, ReadOnlyLoanedMessage<T>) + Send + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::Loaned(Box::new(func)).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (ReadOnlyLoanedMessage<T>, MessageInfo)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message, info| func(message, info));
        WorkerSubscriptionCallback::LoanedWithMessageInfo(f).into()
    }
}

impl<T, Payload, Func> WorkerSubscriptionArgs<T, Payload, Func> for (Payload, ReadOnlyLoanedMessage<T>, MessageInfo)
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, ReadOnlyLoanedMessage<T>, MessageInfo) + Send + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::LoanedWithMessageInfo(Box::new(func)).into()
    }
}
