use rosidl_runtime_rs::Message;

use super::MessageInfo;
use crate::ReadOnlyLoanedMessage;

use futures::future::BoxFuture;
use std::future::Future;

/// A trait for allowed callbacks for subscriptions.
///
/// See [`AnySubscriptionCallback`] for a list of possible callback signatures.
pub trait SubscriptionCallback<T, Args>: Send + 'static
where
    T: Message,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_callback(self) -> AsyncSubscriptionCallback<T>;
}

/// An enum capturing the various possible function signatures for subscription callbacks.
///
/// The correct enum variant is deduced by the [`SubscriptionCallback`] trait.
pub enum AsyncSubscriptionCallback<T>
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

// We need one implementation per arity. This was inspired by Bevy's systems.
impl<T, F, A0, Func> SubscriptionCallback<T, (A0,)> for Func
where
    Func: FnMut(A0) -> F + Send + 'static,
    (A0,): ArgTuple<T, Func>,
    T: Message,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback(self) -> AsyncSubscriptionCallback<T> {
        <(A0,) as ArgTuple<T, Func>>::into_callback_with_args(self)
    }
}

impl<T, F, A0, A1, Func> SubscriptionCallback<T, (A0, A1)> for Func
where
    Func: FnMut(A0, A1) -> F + Send + 'static,
    (A0, A1): ArgTuple<T, Func>,
    T: Message,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback(self) -> AsyncSubscriptionCallback<T> {
        <(A0, A1) as ArgTuple<T, Func>>::into_callback_with_args(self)
    }
}

// Helper trait for SubscriptionCallback.
//
// For each tuple of args, it provides conversion from a function with
// these args to the correct enum variant.
trait ArgTuple<T, Func>
where
    T: Message,
{
    fn into_callback_with_args(func: Func) -> AsyncSubscriptionCallback<T>;
}

impl<T, F, Func> ArgTuple<T, Func> for (T,)
where
    T: Message,
    Func: FnMut(T) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback_with_args(mut func: Func) -> AsyncSubscriptionCallback<T> {
        AsyncSubscriptionCallback::Regular(Box::new(
            move |message| Box::pin(func(message))
        ))
    }
}

impl<T, F, Func> ArgTuple<T, Func> for (T, MessageInfo)
where
    T: Message,
    Func: FnMut(T, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback_with_args(mut func: Func) -> AsyncSubscriptionCallback<T> {
        AsyncSubscriptionCallback::RegularWithMessageInfo(Box::new(
            move |message, info| Box::pin(func(message, info))
        ))
    }
}

impl<T, F, Func> ArgTuple<T, Func> for (Box<T>,)
where
    T: Message,
    Func: FnMut(Box<T>) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback_with_args(mut func: Func) -> AsyncSubscriptionCallback<T> {
        AsyncSubscriptionCallback::Boxed(Box::new(
            move |message| Box::pin(func(message))
        ))
    }
}

impl<T, F, Func> ArgTuple<T, Func> for (Box<T>, MessageInfo)
where
    T: Message,
    Func: FnMut(Box<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback_with_args(mut func: Func) -> AsyncSubscriptionCallback<T> {
        AsyncSubscriptionCallback::BoxedWithMessageInfo(Box::new(
            move |message, info| Box::pin(func(message, info))
        ))
    }
}

impl<T, F, Func> ArgTuple<T, Func> for (ReadOnlyLoanedMessage<T>,)
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback_with_args(mut func: Func) -> AsyncSubscriptionCallback<T> {
        AsyncSubscriptionCallback::Loaned(Box::new(
            move |message| Box::pin(func(message))
        ))
    }
}

impl<T, F, Func> ArgTuple<T, Func> for (ReadOnlyLoanedMessage<T>, MessageInfo)
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_callback_with_args(mut func: Func) -> AsyncSubscriptionCallback<T> {
        AsyncSubscriptionCallback::LoanedWithMessageInfo(Box::new(
            move |message, info| Box::pin(func(message, info))
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn callback_conversion() {
        type Message = test_msgs::msg::BoundedSequences;
        let cb = |_msg: Message| { async { } };
        assert!(matches!(
            cb.into_callback(),
            AsyncSubscriptionCallback::<Message>::Regular(_)
        ));
        let cb = |_msg: Message, _info: MessageInfo| { async { } };
        assert!(matches!(
            cb.into_callback(),
            AsyncSubscriptionCallback::<Message>::RegularWithMessageInfo(_)
        ));
        let cb = |_msg: Box<Message>| { async { } };
        assert!(matches!(
            cb.into_callback(),
            AsyncSubscriptionCallback::<Message>::Boxed(_)
        ));
        let cb = |_msg: Box<Message>, _info: MessageInfo| { async { } };
        assert!(matches!(
            cb.into_callback(),
            AsyncSubscriptionCallback::<Message>::BoxedWithMessageInfo(_)
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<Message>| { async { } };
        assert!(matches!(
            cb.into_callback(),
            AsyncSubscriptionCallback::<Message>::Loaned(_)
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<Message>, _info: MessageInfo| { async { } };
        assert!(matches!(
            cb.into_callback(),
            AsyncSubscriptionCallback::<Message>::LoanedWithMessageInfo(_)
        ));
    }
}
