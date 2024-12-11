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

// We need one implementation per arity. This was inspired by Bevy's systems.
impl<T, A0, Func> IntoNodeSubscriptionCallback<T, (A0,)> for Func
where
    T: Message,
    (A0,): NodeSubscriptionArgs<T, Func>,
    Func: Fn(A0) + Send + Sync + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        <(A0,) as NodeSubscriptionArgs<T, Func>>::into_any_callback(self)
    }
}

impl<T, A0, A1, Func> IntoNodeSubscriptionCallback<T, (A0, A1)> for Func
where
    T: Message,
    (A0, A1): NodeSubscriptionArgs<T, Func>,
    Func: Fn(A0, A1) + Clone + Send + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        <(A0, A1) as NodeSubscriptionArgs<T, Func>>::into_any_callback(self)
    }
}

trait NodeSubscriptionArgs<T, Func>
where
    T: Message,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, ()>;
}

impl<T, Func> NodeSubscriptionArgs<T, Func> for (T,)
where
    T: Message,
    Func: Fn(T) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(func);
        NodeSubscriptionCallback::Regular(Box::new(move |message| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message);
            })
        }))
        .into()
    }
}

impl<T, Func> NodeSubscriptionArgs<T, Func> for (T, MessageInfo)
where
    T: Message,
    Func: Fn(T, MessageInfo) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(func);
        NodeSubscriptionCallback::RegularWithMessageInfo(Box::new(move |message, info| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message, info);
            })
        }))
        .into()
    }
}

impl<T, Func> NodeSubscriptionArgs<T, Func> for (Box<T>,)
where
    T: Message,
    Func: Fn(Box<T>) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(func);
        NodeSubscriptionCallback::Boxed(Box::new(move |message| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message);
            })
        }))
        .into()
    }
}

impl<T, Func> NodeSubscriptionArgs<T, Func> for (Box<T>, MessageInfo)
where
    T: Message,
    Func: Fn(Box<T>, MessageInfo) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(func);
        NodeSubscriptionCallback::BoxedWithMessageInfo(Box::new(move |message, info| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message, info);
            })
        }))
        .into()
    }
}

impl<T, Func> NodeSubscriptionArgs<T, Func> for (ReadOnlyLoanedMessage<T>,)
where
    T: Message,
    Func: Fn(ReadOnlyLoanedMessage<T>) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(func);
        NodeSubscriptionCallback::Loaned(Box::new(move |message| {
            let f = Arc::clone(&func);
            Box::pin(async move {
                f(message);
            })
        }))
        .into()
    }
}

impl<T, Func> NodeSubscriptionArgs<T, Func> for (ReadOnlyLoanedMessage<T>, MessageInfo)
where
    T: Message,
    Func: Fn(ReadOnlyLoanedMessage<T>, MessageInfo) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T, ()> {
        let func = Arc::new(func);
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
