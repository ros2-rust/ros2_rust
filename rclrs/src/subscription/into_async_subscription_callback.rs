use rosidl_runtime_rs::Message;

use super::{AnySubscriptionCallback, MessageInfo, NodeSubscriptionCallback};
use crate::ReadOnlyLoanedMessage;

use std::future::Future;

/// A trait for async callbacks of subscriptions.
///
/// Async subscription callbacks support six signatures:
/// - [`FnMut`] ( `Message` ) -> impl [`Future`]<Output=()>
/// - [`FnMut`] ( `Message`, [`MessageInfo`] ) -> impl [`Future`]<Output=()>
/// - [`FnMut`] ( [`Box`]<`Message`> ) -> impl [`Future`]<Output=()>
/// - [`FnMut`] ( [`Box`]<`Message`>, [`MessageInfo`] ) -> impl [`Future`]<Output=()>
/// - [`FnMut`] ( [`ReadOnlyLoanedMessage`]<`Message`> ) -> impl [`Future`]<Output=()>
/// - [`FnMut`] ( [`ReadOnlyLoanedMessage`]<`Message`>, [`MessageInfo`] ) -> impl [`Future`]<Output=()>
pub trait IntoAsyncSubscriptionCallback<T, Args>: Send + 'static
where
    T: Message,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_async_subscription_callback(self) -> AnySubscriptionCallback<T, ()>;
}

impl<T, Out, Func> IntoAsyncSubscriptionCallback<T, (T,)> for Func
where
    T: Message,
    Func: FnMut(T) -> Out + Send + 'static,
    Out: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::Regular(Box::new(move |message| Box::pin(self(message)))).into()
    }
}

impl<T, Out, Func> IntoAsyncSubscriptionCallback<T, (T, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(T, MessageInfo) -> Out + Send + 'static,
    Out: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::RegularWithMessageInfo(Box::new(move |message, info| {
            Box::pin(self(message, info))
        }))
        .into()
    }
}

impl<T, Out, Func> IntoAsyncSubscriptionCallback<T, (Box<T>,)> for Func
where
    T: Message,
    Func: FnMut(Box<T>) -> Out + Send + 'static,
    Out: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::Boxed(Box::new(move |message| Box::pin(self(message)))).into()
    }
}

impl<T, F, Func> IntoAsyncSubscriptionCallback<T, (Box<T>, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(Box<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::BoxedWithMessageInfo(Box::new(move |message, info| {
            Box::pin(self(message, info))
        }))
        .into()
    }
}

impl<T, F, Func> IntoAsyncSubscriptionCallback<T, (ReadOnlyLoanedMessage<T>,)> for Func
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::Loaned(Box::new(move |message| Box::pin(self(message)))).into()
    }
}

impl<T, F, Func> IntoAsyncSubscriptionCallback<T, (ReadOnlyLoanedMessage<T>, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::LoanedWithMessageInfo(Box::new(move |message, info| {
            Box::pin(self(message, info))
        }))
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestMessage = crate::vendor::test_msgs::msg::BoundedSequences;

    #[test]
    fn callback_conversion() {
        let cb = |_msg: TestMessage| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::Regular(_)),
        ));
        let cb = |_msg: TestMessage, _info: MessageInfo| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_)
            ),
        ));
        let cb = |_msg: Box<TestMessage>| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::Boxed(_)),
        ));
        let cb = |_msg: Box<TestMessage>, _info: MessageInfo| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_)
            ),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::Loaned(_)),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_)
            ),
        ));

        assert!(matches!(
            test_regular.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::Regular(_)),
        ));
        assert!(matches!(
            test_regular_with_info.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_boxed.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::Boxed(_)),
        ));
        assert!(matches!(
            test_boxed_with_info.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_loaned.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::Loaned(_)),
        ));
        assert!(matches!(
            test_loaned_with_info.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_)
            ),
        ));
    }

    async fn test_regular(_msg: TestMessage) {}

    async fn test_regular_with_info(_msg: TestMessage, _info: MessageInfo) {}

    async fn test_boxed(_msg: Box<TestMessage>) {}

    async fn test_boxed_with_info(_msg: Box<TestMessage>, _info: MessageInfo) {}

    async fn test_loaned(_msg: ReadOnlyLoanedMessage<TestMessage>) {}

    async fn test_loaned_with_info(_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo) {}
}
