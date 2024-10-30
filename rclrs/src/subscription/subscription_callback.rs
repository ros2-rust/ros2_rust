use rosidl_runtime_rs::Message;

use super::{
    MessageInfo,
    any_subscription_callback::AnySubscriptionCallback,
};
use crate::ReadOnlyLoanedMessage;

use std::sync::Arc;

/// A trait for regular callbacks of subscriptions.
///
// TODO(@mxgrey): Add a description of what callbacks signatures are supported
pub trait SubscriptionCallback<T, Args>: Send + 'static
where
    T: Message,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_subscription_callback(self) -> AnySubscriptionCallback<T>;
}

// We need one implementation per arity. This was inspired by Bevy's systems.
impl<T, A0, Func> SubscriptionCallback<T, (A0,)> for Func
where
    T: Message,
    (A0,): SubscriptionArgs<T, Func>,
    Func: Fn(A0) + Send + Sync + 'static,
{
    fn into_subscription_callback(self) -> AnySubscriptionCallback<T> {
        <(A0,) as SubscriptionArgs<T, Func>>::into_any_callback(self)
    }
}

impl<T, A0, A1, Func> SubscriptionCallback<T, (A0, A1)> for Func
where
    T: Message,
    (A0, A1): SubscriptionArgs<T, Func>,
    Func: Fn(A0, A1) + Clone + Send + 'static,
{
    fn into_subscription_callback(self) -> AnySubscriptionCallback<T> {
        <(A0, A1) as SubscriptionArgs<T, Func>>::into_any_callback(self)
    }
}

trait SubscriptionArgs<T, Func>
where
    T: Message,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T>;
}

impl<T, Func> SubscriptionArgs<T, Func> for (T,)
where
    T: Message,
    Func: Fn(T) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T> {
        let func = Arc::new(func);
        AnySubscriptionCallback::Regular(Box::new(
            move |message| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(message);
                })
            }
        ))
    }
}

impl<T, Func> SubscriptionArgs<T, Func> for (T, MessageInfo)
where
    T: Message,
    Func: Fn(T, MessageInfo) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T> {
        let func = Arc::new(func);
        AnySubscriptionCallback::RegularWithMessageInfo(Box::new(
            move |message, info| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(message, info);
                })
            }
        ))
    }
}

impl<T, Func> SubscriptionArgs<T, Func> for (Box<T>,)
where
    T: Message,
    Func: Fn(Box<T>) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T> {
        let func = Arc::new(func);
        AnySubscriptionCallback::Boxed(Box::new(
            move |message| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(message);
                })
            }
        ))
    }
}

impl<T, Func> SubscriptionArgs<T, Func> for (Box<T>, MessageInfo)
where
    T: Message,
    Func: Fn(Box<T>, MessageInfo) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T> {
        let func = Arc::new(func);
        AnySubscriptionCallback::BoxedWithMessageInfo(Box::new(
            move |message, info| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(message, info);
                })
            }
        ))
    }
}

impl<T, Func> SubscriptionArgs<T, Func> for (ReadOnlyLoanedMessage<T>,)
where
    T: Message,
    Func: Fn(ReadOnlyLoanedMessage<T>) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T> {
        let func = Arc::new(func);
        AnySubscriptionCallback::Loaned(Box::new(
            move |message| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(message);
                })
            }
        ))
    }
}

impl<T, Func> SubscriptionArgs<T, Func> for (ReadOnlyLoanedMessage<T>, MessageInfo)
where
    T: Message,
    Func: Fn(ReadOnlyLoanedMessage<T>, MessageInfo) + Send + Sync + 'static,
{
    fn into_any_callback(func: Func) -> AnySubscriptionCallback<T> {
        let func = Arc::new(func);
        AnySubscriptionCallback::LoanedWithMessageInfo(Box::new(
            move |message, info| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(message, info);
                })
            }
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestMessage = test_msgs::msg::BoundedSequences;

    #[test]
    fn callback_conversion() {
        let cb = |_msg: TestMessage| { };
        assert!(matches!(
            cb.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::Regular(_)
        ));
        let cb = |_msg: TestMessage, _info: MessageInfo| { };
        assert!(matches!(
            cb.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_)
        ));
        let cb = |_msg: Box<TestMessage>| { };
        assert!(matches!(
            cb.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::Boxed(_)
        ));
        let cb = |_msg: Box<TestMessage>, _info: MessageInfo| { };
        assert!(matches!(
            cb.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_)
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>| { };
        assert!(matches!(
            cb.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::Loaned(_)
        ));
        let cb = |_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo| { };
        assert!(matches!(
            cb.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_)
        ));

        assert!(matches!(
            test_regular.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::Regular(_),
        ));
        assert!(matches!(
            test_regular_with_info.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::RegularWithMessageInfo(_),
        ));
        assert!(matches!(
            test_boxed.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::Boxed(_),
        ));
        assert!(matches!(
            test_boxed_with_info.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::BoxedWithMessageInfo(_),
        ));
        assert!(matches!(
            test_loaned.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::Loaned(_),
        ));
        assert!(matches!(
            test_loaned_with_info.into_subscription_callback(),
            AnySubscriptionCallback::<TestMessage>::LoanedWithMessageInfo(_),
        ));
    }

    fn test_regular(_msg: TestMessage) {

    }

    fn test_regular_with_info(_msg: TestMessage, _info: MessageInfo) {

    }

    fn test_boxed(_msg: Box<TestMessage>) {

    }

    fn test_boxed_with_info(_msg: Box<TestMessage>, _info: MessageInfo) {

    }

    fn test_loaned(_msg: ReadOnlyLoanedMessage<TestMessage>) {

    }

    fn test_loaned_with_info(_msg: ReadOnlyLoanedMessage<TestMessage>, _info: MessageInfo) {

    }
}
