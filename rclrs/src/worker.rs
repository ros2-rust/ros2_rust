use std::sync::Arc;
use crate::{WorkerCommands};

/// A worker that carries a payload and synchronizes callbacks for subscriptions
/// and services.
///
/// The payload is any data type of your choosing. Each callback associated with
/// this worker will receive a mutable borrow (`&mut Payload`) of the payload,
/// allowing them to share the payload data for both viewing and modifying. The
/// worker only contains exactly one instance of the payload that is shared
/// across all callbacks.
///
/// You can also run ad hoc tasks on the worker to view or modify the payload
/// from callbacks that are not associated with this worker.
pub type Worker<Payload> = Arc<WorkerState<Payload>>;

/// The inner state of a [`Worker`].
///
/// This is public so that you can choose to create a [`Weak`][1] reference to it
/// if you want to be able to refer to a [`Worker`] in a non-owning way. It is
/// generally recommended to manage the `WorkerState` inside of an [`Arc`],
/// and [`Worker`] is provided as a convenience alias for that.
///
/// The public API of the [`Worker`] type is implemented via `WorkerState`.
///
/// [1]: std::sync::Weak
pub struct WorkerState<Payload> {
    commands: Arc<WorkerCommands>,
    _ignore: std::marker::PhantomData<Payload>,
}

impl<T> WorkerState<T> {
    pub(crate) fn new(commands: Arc<WorkerCommands>) -> Self {
        Self { commands, _ignore: Default::default() }
    }
}
