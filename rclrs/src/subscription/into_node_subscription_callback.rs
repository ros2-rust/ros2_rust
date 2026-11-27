use rosidl_runtime_rs::Message;

use crate::{
    AnySubscriptionCallback, MessageInfo, NodeSubscriptionCallback, ReadOnlyLoanedMessage,
};

/// A trait for regular callbacks of subscriptions.
///
/// Subscription callbacks support six signatures:
/// - [`FnMut`] ( `Message` )
/// - [`FnMut`] ( `Message`, [`MessageInfo`] )
/// - [`FnMut`] ( [`Box`]<`Message`> )
/// - [`FnMut`] ( [`Box`]<`Message`>, [`MessageInfo`] )
/// - [`FnMut`] ( [`ReadOnlyLoanedMessage`]<`Message`> )
/// - [`FnMut`] ( [`ReadOnlyLoanedMessage`]<`Message`>, [`MessageInfo`] )
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
    Func: FnMut(T) + Send + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::SyncRegular(Box::new(self)).into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (T, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(T, MessageInfo) + Send + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::SyncRegularWithMessageInfo(Box::new(self)).into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (Box<T>,)> for Func
where
    T: Message,
    Func: FnMut(Box<T>) + Send + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::SyncBoxed(Box::new(self)).into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (Box<T>, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(Box<T>, MessageInfo) + Send + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::SyncBoxedWithMessageInfo(Box::new(self)).into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (ReadOnlyLoanedMessage<T>,)> for Func
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>) + Send + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::SyncLoaned(Box::new(self)).into()
    }
}

impl<T, Func> IntoNodeSubscriptionCallback<T, (ReadOnlyLoanedMessage<T>, MessageInfo)> for Func
where
    T: Message,
    Func: FnMut(ReadOnlyLoanedMessage<T>, MessageInfo) + Send + 'static,
{
    fn into_node_subscription_callback(self) -> AnySubscriptionCallback<T, ()> {
        NodeSubscriptionCallback::SyncLoanedWithMessageInfo(Box::new(self)).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestMessage = ros_env::test_msgs::msg::BoundedSequences;

    #[test]
    fn callback_conversion() {
        let cb = |_msg: TestMessage| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::SyncRegular(_)),
        ));
        let cb = |_msg: TestMessage, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::SyncRegularWithMessageInfo(_)
            )
        ));
        let cb = |_msg: Box<TestMessage>| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::SyncBoxed(_)),
        ));
        let cb = |_msg: Box<TestMessage>, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::SyncBoxedWithMessageInfo(_)
            ),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::SyncLoaned(_)),
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::SyncLoanedWithMessageInfo(_)
            ),
        ));

        assert!(matches!(
            test_regular.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::SyncRegular(_)),
        ));
        assert!(matches!(
            test_regular_with_info.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::SyncRegularWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_boxed.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::SyncBoxed(_)),
        ));
        assert!(matches!(
            test_boxed_with_info.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::SyncBoxedWithMessageInfo(_)
            ),
        ));
        assert!(matches!(
            test_loaned.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(NodeSubscriptionCallback::<TestMessage>::SyncLoaned(_)),
        ));
        assert!(matches!(
            test_loaned_with_info.into_node_subscription_callback(),
            AnySubscriptionCallback::Node(
                NodeSubscriptionCallback::<TestMessage>::SyncLoanedWithMessageInfo(_)
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
