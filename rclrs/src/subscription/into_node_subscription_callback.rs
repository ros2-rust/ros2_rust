use rosidl_runtime_rs::Message;

use crate::{
    ReadOnlyLoanedMessage, AnySubscriptionCallback, MessageInfo, NodeSubscriptionCallback,
};

use std::sync::Arc;

/// A trait for regular callbacks of subscriptions.
//
// TODO(@mxgrey): Add a description of what callbacks signatures are supported
pub trait IntoNodeSubscriptionCallback<T, Args>: Send + 'static
where
    T: Message,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()>;
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (T,)> for Func
where
    T: Message,
    Func: Fn(T) + Send + Sync + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(self);
        NodeSubscriptionCallback::Regular(Box::new(move |message| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message);
            })
        }))
        .into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (T, MessageInfo)> for Func
where
    T: Message,
    Func: Fn(T, MessageInfo) + Send + Sync + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(self);
        NodeSubscriptionCallback::RegularWithMessageInfo(Box::new(move |message, info| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message, info);
            })
        }))
        .into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (Box<T>,)> for Func
where
    T: Message,
    Func: Fn(Box<T>) + Send + Sync + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(self);
        NodeSubscriptionCallback::Boxed(Box::new(move |message| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message);
            })
        }))
        .into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (Box<T>, MessageInfo)> for Func
where
    T: Message,
    Func: Fn(Box<T>, MessageInfo) + Send + Sync + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(self);
        NodeSubscriptionCallback::BoxedWithMessageInfo(Box::new(move |message, info| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message, info);
            })
        }))
        .into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (ReadOnlyLoanedMessage<T>,)> for Func
where
    T: Message,
    Func: Fn(ReadOnlyLoanedMessage<T>) + Send + Sync + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(self);
        NodeSubscriptionCallback::Loaned(Box::new(move |message| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message);
            })
        }))
        .into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (ReadOnlyLoanedMessage<T>, MessageInfo)> for Func
where
    T: Message,
    Func: Fn(ReadOnlyLoanedMessage<T>, MessageInfo) + Send + Sync + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(self);
        NodeSubscriptionCallback::LoanedWithMessageInfo(Box::new(move |message, info| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message, info);
            })
        }))
        .into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestMessage = test_msgs::msg::BoundedSequences;

    #[test]
    fn callback_conversion() {
        let cb = |_msg: TestMessage| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::Regular(_)
            ),
        ));
        let cb = |_msg: TestMessage, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_)
            )
        ));
        let cb = |_msg: Box<TestMessage>| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::Boxed(_)
            ),
        ));
        let cb = |_msg: Box<TestMessage>, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_)
            ),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::Loaned(_)
            ),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_)
            ),
        ));

        assert!(matches!(
            test_regular.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::Regular(_)
            ),
        ));
        assert!(matches!(
            test_regular_with_info.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_boxed.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::Boxed(_)
            ),
        ));
        assert!(matches!(
            test_boxed_with_info.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_loaned.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::Loaned(_)
            ),
        ));
        assert!(matches!(
            test_loaned_with_info.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_)
            ),
        ));
    }

    fn test_regular(_msg: TestMessage) {}

    fn test_regular_with_info(_msg: TestMessage, _info: MessageInfo) {}

    fn test_boxed(_msg: Box<TestMessage>) {}

    fn test_boxed_with_info(_msg: Box<TestMessage>, _info: MessageInfo) {}

    fn test_loaned(_msg: ReadOnlyLoanedMessage<TestMessage>) {}

    fn test_loaned_with_info(_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo) {}
}
