use super::MessageInfo;

/// A trait for allowed callbacks for subscriptions.
///
/// See [`AnySubscriptionCallback`] for a list of possible callback signatures.
pub trait SubscriptionCallback<T, Args>: Send {
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_callback(self) -> AnySubscriptionCallback<T>;
}

/// An enum capturing the various possible function signatures for subscription callbacks.
///
/// The correct enum variant is deduced by the [`SubscriptionCallback`] trait.
pub enum AnySubscriptionCallback<T> {
    /// A callback with only the message as an argument.
    Regular(Box<dyn FnMut(T) + Send>),
    /// A callback with the message and the message info as arguments.
    RegularWithMessageInfo(Box<dyn FnMut(T, MessageInfo) + Send>),
    /// A callback with only the boxed message as an argument.
    Boxed(Box<dyn FnMut(Box<T>) + Send>),
    /// A callback with the boxed message and the message info as arguments.
    BoxedWithMessageInfo(Box<dyn FnMut(Box<T>, MessageInfo) + Send>),
}

// We need one implementation per arity. This was inspired by Bevy's systems.
impl<T, A0, Func> SubscriptionCallback<T, (A0,)> for Func
where
    Func: FnMut(A0) + Send + 'static,
    (A0,): ArgTuple<T, Func = Box<dyn FnMut(A0) + Send>>,
{
    fn into_callback(self) -> AnySubscriptionCallback<T> {
        <(A0,) as ArgTuple<T>>::into_callback_with_args(Box::new(self))
    }
}

impl<T, A0, A1, Func> SubscriptionCallback<T, (A0, A1)> for Func
where
    Func: FnMut(A0, A1) + Send + 'static,
    (A0, A1): ArgTuple<T, Func = Box<dyn FnMut(A0, A1) + Send>>,
{
    fn into_callback(self) -> AnySubscriptionCallback<T> {
        <(A0, A1) as ArgTuple<T>>::into_callback_with_args(Box::new(self))
    }
}

// Helper trait for SubscriptionCallback.
//
// For each tuple of args, it provides conversion from a function with
// these args to the correct enum variant.
trait ArgTuple<T> {
    type Func;
    fn into_callback_with_args(func: Self::Func) -> AnySubscriptionCallback<T>;
}

impl<T> ArgTuple<T> for (Box<T>,) {
    type Func = Box<dyn FnMut(Box<T>) + Send>;
    fn into_callback_with_args(func: Self::Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Boxed(func)
    }
}

impl<T> ArgTuple<T> for (Box<T>, MessageInfo) {
    type Func = Box<dyn FnMut(Box<T>, MessageInfo) + Send>;
    fn into_callback_with_args(func: Self::Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::BoxedWithMessageInfo(func)
    }
}

impl<T> ArgTuple<T> for (T,) {
    type Func = Box<dyn FnMut(T) + Send>;
    fn into_callback_with_args(func: Self::Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::Regular(func)
    }
}

impl<T> ArgTuple<T> for (T, MessageInfo) {
    type Func = Box<dyn FnMut(T, MessageInfo) + Send>;
    fn into_callback_with_args(func: Self::Func) -> AnySubscriptionCallback<T> {
        AnySubscriptionCallback::RegularWithMessageInfo(func)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn callback_conversion() {
        struct Message;
        let cb = |_msg: Message| {};
        assert!(matches!(
            cb.into_callback(),
            AnySubscriptionCallback::<Message>::Regular(_)
        ));
        let cb = |_msg: Message, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_callback(),
            AnySubscriptionCallback::<Message>::RegularWithMessageInfo(_)
        ));
        let cb = |_msg: Box<Message>| {};
        assert!(matches!(
            cb.into_callback(),
            AnySubscriptionCallback::<Message>::Boxed(_)
        ));
        let cb = |_msg: Box<Message>, _info: MessageInfo| {};
        assert!(matches!(
            cb.into_callback(),
            AnySubscriptionCallback::<Message>::BoxedWithMessageInfo(_)
        ));
    }
}
