use crate::{Node, Worker};

/// This is used to determine what kind of payload a subscription can accept, as
/// well as what kind of callbacks can be used with it. Users should not implement
/// this trait.
pub trait ServiceScope {
    /// What kind of payload should the worker hold for this scope.
    type Payload: 'static + Send;
}

impl ServiceScope for Node {
    type Payload = ();
}

impl<Payload: 'static + Send> ServiceScope for Worker<Payload> {
    type Payload = Payload;
}
