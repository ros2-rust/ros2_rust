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
