use rosidl_runtime_rs::Message;

use super::MessageInfo;
use crate::ReadOnlyLoanedMessage;

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
    fn into_callback(self) -> AnySubscriptionCallback<T>;
}

/// An enum capturing the various possible function signatures for subscription callbacks.
///
/// The correct enum variant is deduced by the [`SubscriptionCallback`] trait.
pub enum AnySubscriptionCallback<T>
where
    T: Message,
{
    /// A callback with only the message as an argument.
    Regular(Box<dyn FnMut(T) + Send>),
    /// A callback with the message and the message info as arguments.
    RegularWithMessageInfo(Box<dyn FnMut(T, MessageInfo) + Send>),
    /// A callback with only the boxed message as an argument.
    Boxed(Box<dyn FnMut(Box<T>) + Send>),
    /// A callback with the boxed message and the message info as arguments.
    BoxedWithMessageInfo(Box<dyn FnMut(Box<T>, MessageInfo) + Send>),
    /// A callback with only the loaned message as an argument.
    #[allow(clippy::type_complexity)]
    Loaned(Box<dyn for<'a> FnMut(ReadOnlyLoanedMessage<'a, T>) + Send>),
    /// A callback with the loaned message and the message info as arguments.
    #[allow(clippy::type_complexity)]
    LoanedWithMessageInfo(Box<dyn for<'a> FnMut(ReadOnlyLoanedMessage<'a, T>, MessageInfo) + Send>),
}

// We need one implementation per arity. This was inspired by Bevy's systems.
impl<T, A0, Func> SubscriptionCallback<T, (A0,)> for Func
where
    Func: FnMut(A0) + Send + 'static,
    (A0,): ArgTuple<T, Func>,
    T: Message,
{
    fn into_callback(self) -> AnySubscriptionCallback<T> {
        <(A0,) as ArgTuple<T, Func>>::into_callback_with_args(self)
    }
}

impl<T, A0, A1, Func> SubscriptionCallback<T, (A0, A1)> for Func
where
    Func: FnMut(A0, A1) + Send + 'static,
    (A0, A1): ArgTuple<T, Func>,
    T: Message,
{
    fn into_callback(self) -> AnySubscriptionCallback<T> {
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
    fn into_callback_with_args(func: Func) -> AnySubscriptionCallback<T>;
}

impl<T, Func> ArgTuple<T, Func> for (T,)
where
    T: Message,
    Func: FnMut(T) + Send + 'static,
{
    fn into_callback_with_args(func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Regular(Box::new(func))
    }
}

impl<T, Func> ArgTuple<T, Func> for (T, MessageInfo)
where
    T: Message,
    Func: FnMut(T, MessageInfo) + Send + 'static,
{
    fn into_callback_with_args(func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::RegularWithMessageInfo(Box::new(func))
    }
}

impl<T, Func> ArgTuple<T, Func> for (Box<T>,)
where
    T: Message,
    Func: FnMut(Box<T>) + Send + 'static,
{
    fn into_callback_with_args(func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Boxed(Box::new(func))
    }
}

impl<T, Func> ArgTuple<T, Func> for (Box<T>, MessageInfo)
where
    T: Message,
    Func: FnMut(Box<T>, MessageInfo) + Send + 'static,
{
    fn into_callback_with_args(func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::BoxedWithMessageInfo(Box::new(func))
    }
}

impl<T, Func> ArgTuple<T, Func> for (ReadOnlyLoanedMessage<'_, T>,)
where
    T: Message,
    Func: for<'b> FnMut(ReadOnlyLoanedMessage<'b, T>) + Send + 'static,
{
    fn into_callback_with_args(func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Loaned(Box::new(func))
    }
}

impl<T, Func> ArgTuple<T, Func> for (ReadOnlyLoanedMessage<'_, T>, MessageInfo)
where
    T: Message,
    Func: for<'b> FnMut(ReadOnlyLoanedMessage<'b, T>, MessageInfo) + Send + 'static,
{
    fn into_callback_with_args(func: Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::LoanedWithMessageInfo(Box::new(func))
    }
}
