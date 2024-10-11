use rosidl_runtime_rs::Message;

use super::{
    MessageInfo,
    any_subscription_callback::AnySubscriptionCallback,
};
use crate::ReadOnlyLoanedMessage;

use std::future::Future;

/// A trait for async callbacks of subscriptions.
///
// TODO(@mxgrey): Add a description of what callback signatures are supported
pub trait SubscriptionAsyncCallback<T, Args>: Send + 'static
where
    T: Message,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_subscription_async_callback(self) -> AnySubscriptionCallback<T>;
}

// We need one implementation per arity. This was inspired by Bevy's systems.
impl<T, A0, Out, Func> SubscriptionAsyncCallback<T, (A0,)> for Func
where
    T: Message,
    (A0,): SubscriptionAsyncArgs<T, Func>,
    Out: Future<Output = ()> + Send + 'static,
    Func: FnMut(A0) -> Out + Send + 'static,
{
    fn into_subscription_async_callback(self) -> AnySubscriptionCallback<T> {
        <(A0,) as SubscriptionAsyncArgs<T, Func>>::into_any_callback(self)
    }
}

impl<T, A0, A1, Out, Func> SubscriptionAsyncCallback<T, (A0, A1)> for Func
where
    T: Message,
    (A0, A1): SubscriptionAsyncArgs<T, Func>,
    Out: Future<Output = ()> + Send + 'static,
    Func: FnMut(A0, A1) -> Out + Send + 'static,
{
    fn into_subscription_async_callback(self) -> AnySubscriptionCallback<T> {
        <(A0, A1) as SubscriptionAsyncArgs<T, Func>>::into_any_callback(self)
    }
}

/// Helper trait for SubscriptionCallback.
///
/// For each tuple of args, it provides conversion from a function with
/// these args to the correct enum variant.
trait SubscriptionAsyncArgs<T, Func>
where
    T: Message,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T>;
}

impl<T, Out, Func> SubscriptionAsyncArgs<T, Func> for (T,)
where
    T: Message,
    Func: FnMut(T) -> Out + Send + 'static,
    Out: Future<Output = ()> + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Regular(Box::new(
            move |message| Box::pin(func(message))
        ))
    }
}

impl<T, Out, Func> SubscriptionAsyncArgs<T, Func> for (T, MessageInfo)
where
    T: Message,
    Func: FnMut(T, MessageInfo) -> Out + Send + 'static,
    Out: Future<Output = ()> + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::RegularWithMessageInfo(Box::new(
            move |message, info| Box::pin(func(message, info))
        ))
    }
}

impl<T, Out, Func> SubscriptionAsyncArgs<T, Func> for (Box<T>,)
where
    T: Message,
    Func: FnMut(Box<T>) -> Out + Send + 'static,
    Out: Future<Output = ()> + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Boxed(Box::new(
            move |message| Box::pin(func(message))
        ))
    }
}

impl<T, F, Func> SubscriptionAsyncArgs<T, Func> for (Box<T>, MessageInfo)
where
    T: Message,
    Func: FnMut(Box<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::BoxedWithMessageInfo(Box::new(
            move |message, info| Box::pin(func(message, info))
        ))
    }
}

impl<T, F, Func> SubscriptionAsyncArgs<T, Func> for (ReadOnlyLoanedMessage<T>,)
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Loaned(Box::new(
            move |message| Box::pin(func(message))
        ))
    }
}

impl<T, F, Func> SubscriptionAsyncArgs<T, Func> for (ReadOnlyLoanedMessage<T>, MessageInfo)
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_any_callback(mut func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::LoanedWithMessageInfo(Box::new(
            move |message, info| Box::pin(func(message, info))
        ))
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    type TestMessage = test_msgs::msg::BoundedSequences;

    #[test]
    fn callback_conversion() {
        let cb = |_msg: TestMessage| { async { } };
        assert!(matches!(
            cb.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::Regular(_)
        ));
        let cb = |_msg: TestMessage, _info: MessageInfo| { async { } };
        assert!(matches!(
            cb.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_)
        ));
        let cb = |_msg: Box<TestMessage>| { async { } };
        assert!(matches!(
            cb.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::Boxed(_)
        ));
        let cb = |_msg: Box<TestMessage>, _info: MessageInfo| { async { } };
        assert!(matches!(
            cb.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_)
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>| { async { } };
        assert!(matches!(
            cb.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::Loaned(_)
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo| { async {}};
        assert!(matches!(
            cb.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_)
        ));

        assert!(matches!(
            test_regular.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::Regular(_),
        ));
        assert!(matches!(
            test_regular_with_info.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_),
        ));
        assert!(matches!(
            test_boxed.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::Boxed(_),
        ));
        assert!(matches!(
            test_boxed_with_info.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_),
        ));
        assert!(matches!(
            test_loaned.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::Loaned(_),
        ));
        assert!(matches!(
            test_loaned_with_info.into_subscription_async_callback(),
            AnySubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_),
        ));
    }

    async fn test_regular(_msg: TestMessage) {

    }

    async fn test_regular_with_info(_msg: TestMessage, _info: MessageInfo) {

    }

    async fn test_boxed(_msg: Box<TestMessage>) {

    }

    async fn test_boxed_with_info(_msg: Box<TestMessage>, _info: MessageInfo) {

    }

    async fn test_loaned(_msg: ReadOnlyLoanedMessage<TestMessage>) {

    }

    async fn test_loaned_with_info(_msg: ReadOnlyLoanedMessage<TestMessage>) {

    }
}
