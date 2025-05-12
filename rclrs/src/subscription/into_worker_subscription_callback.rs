use rosidl_runtime_rs::Message;

use crate::{ReadOnlyLoanedMessage, MessageInfo, AnySubscriptionCallback, WorkerSubscriptionCallback};

/// A trait for callbacks of subscriptions that run on a worker.
//
// TODO(@mxgrey): Add a description of what callbacks signatures are supported
pub trait IntoWorkerSubscriptionCallback<T, Payload, Args>: Send + 'static
where
    T: Message,
    Payload: 'static,
{
    /// Converts a worker subscription callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload>;
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (T,)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(T) + Send + 'static,
{
    fn into_worker_subscription_callback(mut self) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message| self(message));
        WorkerSubscriptionCallback::Regular(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Payload, T)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, T) + Send + 'static,
{
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::Regular(Box::new(self)).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (T, MessageInfo)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(T, MessageInfo) + Send + 'static,
{
    fn into_worker_subscription_callback(mut self) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message, info| self(message, info));
        WorkerSubscriptionCallback::RegularWithMessageInfo(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Payload, T, MessageInfo)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, T, MessageInfo) + Send + 'static,
{
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::RegularWithMessageInfo(Box::new(self)).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Box<T>,)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(Box<T>) + Send + 'static,
{
    fn into_worker_subscription_callback(mut self) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message| self(message));
        WorkerSubscriptionCallback::Boxed(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Payload, Box<T>)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, Box<T>) + Send + 'static,
{
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::Boxed(Box::new(self)).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Box<T>, MessageInfo)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(Box<T>, MessageInfo) + Send + 'static,
{
    fn into_worker_subscription_callback(mut self) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message, info| self(message, info));
        WorkerSubscriptionCallback::BoxedWithMessageInfo(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Payload, Box<T>, MessageInfo)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, Box<T>, MessageInfo) + Send + 'static,
{
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::BoxedWithMessageInfo(Box::new(self)).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (ReadOnlyLoanedMessage<T>,)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(ReadOnlyLoanedMessage<T>) + Send + 'static,
{
    fn into_worker_subscription_callback(mut self) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message| self(message));
        WorkerSubscriptionCallback::Loaned(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Payload, ReadOnlyLoanedMessage<T>)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, ReadOnlyLoanedMessage<T>) + Send + 'static,
{
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::Loaned(Box::new(self)).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (ReadOnlyLoanedMessage<T>, MessageInfo)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) + Send + 'static,
{
    fn into_worker_subscription_callback(mut self) -> AnySubscriptionCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, message, info| self(message, info));
        WorkerSubscriptionCallback::LoanedWithMessageInfo(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerSubscriptionCallback<T, Payload, (Payload, ReadOnlyLoanedMessage<T>, MessageInfo)> for Func
where
    T: Message,
    Payload: 'static,
    Func: FnMut(&mut Payload, ReadOnlyLoanedMessage<T>, MessageInfo) + Send + 'static,
{
    fn into_worker_subscription_callback(self) -> AnySubscriptionCallback<T, Payload> {
        WorkerSubscriptionCallback::LoanedWithMessageInfo(Box::new(self)).into()
    }
}
