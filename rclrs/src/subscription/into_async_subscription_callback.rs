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
        NodeSubscriptionCallback::AsyncRegular(Box::new(move |message| Box::pin(self(message))))
            .into()
    }
}

impl<T, Out, Func> IntoAsyncSubscriptionCallback<T, (T, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(T, MessageInfo) -> Out + Send + 'static,
    Out: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::AsyncRegularWithMessageInfo(Box::new(move |message, info| {
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
        NodeSubscriptionCallback::AsyncBoxed(Box::new(move |message| Box::pin(self(message))))
            .into()
    }
}

impl<T, F, Func> IntoAsyncSubscriptionCallback<T, (Box<T>, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(Box<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::AsyncBoxedWithMessageInfo(Box::new(move |message, info| {
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
        NodeSubscriptionCallback::AsyncLoaned(Box::new(move |message| Box::pin(self(message))))
            .into()
    }
}

impl<T, F, Func> IntoAsyncSubscriptionCallback<T, (ReadOnlyLoanedMessage<T>, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) -> F + Send + 'static,
    F: Future<Output = ()> + Send + 'static,
{
    fn into_async_subscription_callback(mut self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::AsyncLoanedWithMessageInfo(Box::new(move |message, info| {
            Box::pin(self(message, info))
        }))
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestMessage = ros_env::test_msgs::msg::BoundedSequences;

    #[test]
    fn callback_conversion() {
        let cb = |_msg: TestMessage| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::AsyncRegular(_)),
        ));
        let cb = |_msg: TestMessage, _info: MessageInfo| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::AsyncRegularWithMessageInfo(_)
            ),
        ));
        let cb = |_msg: Box<TestMessage>| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::AsyncBoxed(_)),
        ));
        let cb = |_msg: Box<TestMessage>, _info: MessageInfo| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::AsyncBoxedWithMessageInfo(_)
            ),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::AsyncLoaned(_)),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo| async {};
        assert!(matches!(
            cb.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::AsyncLoanedWithMessageInfo(_)
            ),
        ));

        assert!(matches!(
            test_regular.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::AsyncRegular(_)),
        ));
        assert!(matches!(
            test_regular_with_info.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::AsyncRegularWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_boxed.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::AsyncBoxed(_)),
        ));
        assert!(matches!(
            test_boxed_with_info.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::AsyncBoxedWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_loaned.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::AsyncLoaned(_)),
        ));
        assert!(matches!(
            test_loaned_with_info.into_async_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::AsyncLoanedWithMessageInfo(_)
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
