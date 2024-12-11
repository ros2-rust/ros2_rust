use rosidl_runtime_rs::{Message, Service as IdlService};
use std::{any::Any, sync::Arc};
use futures::channel::oneshot;
use crate::{
    WorkerCommands, NodeHandle, ToLogParams, Promise, log_fatal,
    IntoWorkerSubscriptionCallback, IntoWorkerServiceCallback,
    WorkerSubscription, SubscriptionState, WorkerService, ServiceState,
    SubscriptionOptions, ServiceOptions, RclrsError,
};

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
    /// The node that this worker is associated with
    node: Arc<NodeHandle>,
    /// The commands to communicate with the runtime of the worker
    commands: Arc<WorkerCommands>,
    _ignore: std::marker::PhantomData<Payload>,
}

impl<Payload: 'static + Send> WorkerState<Payload> {
    /// Run a task on this worker. This allows you to view and modify the payload
    /// of the worker. You will receive a [`Promise`] which you can use to view
    /// what happened inside the callback.
    pub fn run<Out, F>(&self, f: F) -> Promise<Out>
    where
        F: FnOnce(&mut Payload) -> Out + 'static + Send,
        Out: 'static + Send,
    {
        let (sender, receiver) = oneshot::channel();
        self.commands.run_on_payload(Box::new(move |any_payload: &mut dyn Any| {
            let Some(payload) = any_payload.downcast_mut::<Payload>() else {
                log_fatal!(
                    "rclrs.worker",
                    "Received invalid payload from worker. Expected: {:?}, received: {:?}. \
                    This should never happen. Please report this to the maintainers of rclrs \
                    with a minimal reproducible example.",
                    std::any::TypeId::of::<Payload>(),
                    any_payload,
                );
                return;
            };

            let out = f(payload);
            sender.send(out).ok();
        }));
        receiver
    }

    /// Creates a [`WorkerSubscription`].
    pub fn create_subscription<'a, T, Args>(
        &self,
        options: impl Into<SubscriptionOptions<'a>>,
        callback: impl IntoWorkerSubscriptionCallback<T, Payload, Args>,
    ) -> Result<WorkerSubscription<T, Payload>, RclrsError>
    where
        T: Message,
    {
        SubscriptionState::<T, Worker<Payload>>::create(
            options,
            callback.into_worker_subscription_callback(),
            &self.node,
            &self.commands,
        )
    }

    /// Creates a [`WorkerService`].
    pub fn create_service<'a, T, Args>(
        &self,
        options: impl Into<ServiceOptions<'a>>,
        callback: impl IntoWorkerServiceCallback<T, Payload, Args>,
    ) -> Result<WorkerService<T, Payload>, RclrsError>
    where
        T: IdlService,
    {
        ServiceState::<T, Worker<Payload>>::create(
            options,
            callback.into_worker_service_callback(),
            &self.node,
            &self.commands,
        )
    }

    /// Used by [`Node`][crate::Node] to create a `WorkerState`. Users should
    /// call [`Node::create_worker`][crate::NodeState::create_worker] instead of
    /// this.
    pub(crate) fn create(
        node: Arc<NodeHandle>,
        commands: Arc<WorkerCommands>,
    ) -> Arc<Self> {
        Arc::new(Self { node, commands, _ignore: Default::default() })
    }
}

/// Options used while creating a new [`Worker`].
pub struct WorkerOptions<Payload> {
    /// The value of the initial payload for the [`Worker`].
    pub payload: Payload,
}

impl<Payload> WorkerOptions<Payload> {
    /// Create a new `WorkerOptions`.
    pub fn new(payload: Payload) -> Self {
        Self { payload }
    }
}

/// Implicitly convert something into [`WorkerOptions`].
pub trait IntoWorkerOptions<Payload> {
    /// Convert an object into [`WorkerOptions`]. Users do not need to call this.
    fn into_worker_options(self) -> WorkerOptions<Payload>;
}

impl<Payload> IntoWorkerOptions<Payload> for Payload {
    fn into_worker_options(self) -> WorkerOptions<Payload> {
        WorkerOptions::new(self)
    }
}
