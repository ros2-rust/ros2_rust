use rosidl_runtime_rs::{Message, Service as IdlService};
use std::{any::Any, sync::{Arc, Mutex, Weak}};
use futures::channel::oneshot;
use crate::{
    WorkerCommands, ToLogParams, Promise, log_fatal,
    IntoWorkerSubscriptionCallback, IntoWorkerServiceCallback,
    WorkerSubscription, SubscriptionState, WorkerService, ServiceState,
    SubscriptionOptions, ServiceOptions, RclrsError, Node,
    IntoWorkerTimerRepeatingCallback, IntoWorkerTimerOneshotCallback,
    IntoTimerOptions, AnyTimerCallback, WorkerTimer, TimerState,
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
    node: Node,
    /// The commands to communicate with the runtime of the worker
    commands: Arc<WorkerCommands>,
    _ignore: std::marker::PhantomData<Payload>,
}

impl<Payload: 'static + Send + Sync> WorkerState<Payload> {
    /// Run a task on this worker. This allows you to view and modify the payload
    /// of the worker. You will receive a [`Promise`] which you can use to view
    /// what happened inside the callback.
    pub fn run<Out, F>(&self, f: F) -> Promise<Out>
    where
        F: FnOnce(&mut Payload) -> Out + 'static + Send,
        Out: 'static + Send,
    {
        let (sender, receiver) = oneshot::channel();
        self.commands.run_on_payload(Box::new(move |any_payload| {
            let Some(payload) = any_payload.downcast_mut::<Payload>() else {
                log_fatal!("rclrs.worker", "{}", Self::conversion_failure_message(any_payload));
                return;
            };

            let out = f(payload);
            sender.send(out).ok();
        }));
        receiver
    }

    /// Same as [`Self::run`] but you will additionally receive a notification
    /// promise that will be triggered after the output is available. The
    /// notification is suitable to be passed into [`until_promise_resolved`][1].
    ///
    /// [1]: SpinOptions::until_promise_resolved
    pub fn run_with_notice<Out, F>(&self, f: F) -> (Promise<Out>, Promise<()>)
    where
        F: FnOnce(&mut Payload) -> Out + 'static + Send,
        Out: 'static + Send,
    {
        let (notice_sender, notice_receiver) = oneshot::channel();
        let (sender, receiver) = oneshot::channel();
        self.commands.run_on_payload(Box::new(move |any_payload| {
            let Some(payload) = any_payload.downcast_mut::<Payload>() else {
                log_fatal!("rclrs.worker", "{}", Self::conversion_failure_message(any_payload));
                return;
            };

            let out = f(payload);
            sender.send(out).ok();
            notice_sender.send(()).ok();
        }));
        (receiver, notice_receiver)
    }

    /// Listen to activity that happens with this worker's primitives.
    pub fn listen<F>(&self, mut f: F) -> ActivityListener<Payload>
    where
        F: FnMut(&mut Payload) + 'static + Send + Sync,
        Payload: Sync,
    {
        let f = Box::new(move |any_payload: &mut dyn Any| {
            let Some(payload) = any_payload.downcast_mut::<Payload>() else {
                log_fatal!("rclrs.worker", "{}", Self::conversion_failure_message(any_payload));
                return;
            };

            f(payload);
        });

        let listener = ActivityListener::new(ActivityListenerCallback::Listen(f));
        self.commands.add_activity_listener(Arc::downgrade(&listener.callback));
        listener
    }

    /// This will be triggered each time the worker has some activity until it
    /// returns [`Some<Out>`] or until the [`Promise`] is droped.
    ///
    /// As long as the [`Promise`] is alive and the callback returns [`None`],
    /// the listener will keep running.
    pub fn listen_until<Out, F>(&self, mut f: F) -> Promise<Out>
    where
        F: FnMut(&mut Payload) -> Option<Out> + 'static + Send + Sync,
        Out: 'static + Send + Sync,
        Payload: Sync,
    {
        let (sender, receiver) = oneshot::channel();
        let mut captured_sender = Some(sender);
        let listener = ActivityListener::new(ActivityListenerCallback::Inert);

        // We capture the listener so that its lifecycle is tied to the success
        // of the callback.
        let mut _captured_listener = Some(listener.clone());
        listener.set_callback(move |payload| {
            if let Some(sender) = captured_sender.take() {
                if sender.is_canceled() {
                    _captured_listener = None;
                    return;
                }

                if let Some(out) = f(payload) {
                    sender.send(out).ok();
                    _captured_listener = None;
                } else {
                    captured_sender = Some(sender);
                }
            } else {
                _captured_listener = None;
            }
        });

        self.commands.add_activity_listener(Arc::downgrade(&listener.callback));

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
            self.node.handle(),
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
            self.node.handle(),
            &self.commands,
        )
    }

    /// Create a [`WorkerTimer`] with a repeating callback.
    ///
    /// See also:
    /// * [`Self::create_timer_oneshot`]
    /// * [`Self::create_timer_inert`]
    pub fn create_timer_repeating<'a, Args>(
        &self,
        options: impl IntoTimerOptions<'a>,
        callback: impl IntoWorkerTimerRepeatingCallback<Worker<Payload>, Args>,
    ) -> Result<WorkerTimer<Payload>, RclrsError> {
        self.create_timer(options, callback.into_worker_timer_repeating_callback())
    }

    /// Create a [`WorkerTimer`] whose callback will be triggered once after the
    /// period of the timer has elapsed. After that you will need to use
    /// [`WorkerTimer::set_worker_oneshot`] or [`WorkerTimer::set_worker_repeating`]
    /// or else nothing will happen the following times that the `Timer` elapses.
    ///
    /// See also:
    /// * [`Self::create_timer_repeating`]
    /// * [`Self::create_time_inert`]
    pub fn create_timer_oneshot<'a, Args>(
        &self,
        options: impl IntoTimerOptions<'a>,
        callback: impl IntoWorkerTimerOneshotCallback<Worker<Payload>, Args>,
    ) -> Result<WorkerTimer<Payload>, RclrsError> {
        self.create_timer(options, callback.into_worker_timer_oneshot_callback())
    }

    /// Create a [`WorkerTimer`] without a callback. Nothing will happen when this
    /// `WorkerTimer` elapses until you use [`WorkerTimer::set_worker_repeating`]
    /// or [`WorkerTimer::set_worker_oneshot`].
    ///
    /// See also:
    /// * [`Self::create_timer_repeating`]
    /// * [`Self::create_timer_oneshot`]
    pub fn create_timer_inert<'a>(
        &self,
        options: impl IntoTimerOptions<'a>,
    ) -> Result<WorkerTimer<Payload>, RclrsError> {
        self.create_timer(options, AnyTimerCallback::Inert)
    }

    fn create_timer<'a>(
        &self,
        options: impl IntoTimerOptions<'a>,
        callback: AnyTimerCallback<Worker<Payload>>,
    ) -> Result<WorkerTimer<Payload>, RclrsError> {
        let options = options.into_timer_options();
        let clock = options.clock.as_clock(&*self.node);
        TimerState::create(
            options.period,
            clock,
            callback,
            &self.commands,
            &self.node.handle().context_handle,
        )
    }

    /// Used by [`Node`][crate::Node] to create a `WorkerState`. Users should
    /// call [`Node::create_worker`][crate::NodeState::create_worker] instead of
    /// this.
    pub(crate) fn create(
        node: Node,
        commands: Arc<WorkerCommands>,
    ) -> Arc<Self> {
        Arc::new(Self { node, commands, _ignore: Default::default() })
    }

    fn conversion_failure_message(any_payload: &mut dyn Any) -> String {
        format!(
            "Received invalid payload from worker. Expected: {:?}, received: {:?}. \
            This should never happen. Please report this to the maintainers of rclrs \
            with a minimal reproducible example.",
            std::any::TypeId::of::<Payload>(),
            any_payload,
        )
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

/// `ActivityListener` is used to watch the activity of the primitives of a [`Worker`].
///
/// Each listener will be triggered each time a primitive is active on the `Worker`.
/// Listeners will not be triggered when something other than a primitive modifies
/// the payload, which can happen through [interior mutability][1] or by one of
/// the other listeners, so this is not a sure-fire way to track changes in the
/// payload.
///
/// [1]: https://doc.rust-lang.org/book/ch15-05-interior-mutability.html
pub struct ActivityListener<Payload> {
    callback: Arc<Mutex<Option<ActivityListenerCallback>>>,
    _ignore: std::marker::PhantomData<Payload>,
}

impl<Payload> Clone for ActivityListener<Payload> {
    fn clone(&self) -> Self {
        Self {
            callback: Arc::clone(&self.callback),
            _ignore: Default::default(),
        }
    }
}

impl<Payload: 'static + Send + Sync> ActivityListener<Payload> {
    /// Change the callback for this listener.
    pub fn set_callback<F>(&self, mut f: F)
    where
        F: FnMut(&mut Payload) + 'static + Send + Sync,
    {
        let f = Box::new(move |any_payload: &mut dyn Any| {
            let Some(payload) = any_payload.downcast_mut::<Payload>() else {
                let msg = WorkerState::<Payload>::conversion_failure_message(any_payload);
                log_fatal!("rclrs.worker", "{msg}");
                return;
            };

            f(payload);
        });

        *self.callback.lock().unwrap() = Some(ActivityListenerCallback::Listen(f));
    }

    /// Change the listener to be inert but remain alive.
    pub fn set_inert(&self) {
        *self.callback.lock().unwrap() = Some(ActivityListenerCallback::Inert);
    }

    fn new(callback: ActivityListenerCallback) -> Self {
        let callback = Arc::new(Mutex::new(Some(callback)));
        Self { callback, _ignore: Default::default() }
    }
}

/// This type is used by executor runtimes to keep track of listeners.
pub type WeakActivityListener = Weak<Mutex<Option<ActivityListenerCallback>>>;

/// Enum for the different types of callbacks that a listener may have
pub enum ActivityListenerCallback {
    /// The listener is listening
    Listen(Box<dyn FnMut(&mut dyn Any) + 'static + Send>),
    /// The listener is inert
    Inert,
}

/// This is used to determine what kind of payload a callback can accept, as
/// well as what kind of callbacks can be used with it. Users should not implement
/// this trait.
pub trait WorkScope: 'static + Send + Sync {
    /// What kind of payload should the worker hold for this scope.
    type Payload: 'static + Send;
}

impl WorkScope for Node {
    type Payload = ();
}

impl<Payload: 'static + Send + Sync> WorkScope for Worker<Payload> {
    type Payload = Payload;
}

#[cfg(test)]
mod tests {
    use crate::*;
    use test_msgs::{
        msg::Empty as EmptyMsg,
        srv::{Empty as EmptySrv, Empty_Request, Empty_Response},
    };
    use std::time::Duration;

    #[derive(Default, Clone, Copy, Debug)]
    struct TestPayload {
        subscription_count: usize,
        service_count: usize,
    }

    #[test]
    fn test_worker() {
        let mut executor = Context::default().create_basic_executor();
        let node = executor.create_node("test_worker_node").unwrap();
        let worker = node.create_worker(TestPayload::default());
        let _count_sub = worker.create_subscription(
            "test_worker_topic",
            |payload: &mut TestPayload, _msg: EmptyMsg| {
                payload.subscription_count += 1;
            }
        );

        let _count_srv = worker.create_service::<EmptySrv, _>(
            "test_worker_service",
            |payload: &mut TestPayload, _req: Empty_Request| {
                payload.service_count += 1;
                Empty_Response::default()
            }
        );

        let promise = worker.listen_until(move |payload| {
            if payload.service_count > 0 && payload.subscription_count > 0 {
                Some(*payload)
            } else {
                None
            }
        });

        let publisher = node.create_publisher("test_worker_topic").unwrap();
        publisher.publish(EmptyMsg::default()).unwrap();

        let client = node.create_client::<EmptySrv>("test_worker_service").unwrap();
        let _: Promise<Empty_Response> = client.call(Empty_Request::default()).unwrap();

        let (mut promise, notice) = executor.commands().create_notice(promise);

        executor.spin(
            SpinOptions::new()
            .until_promise_resolved(notice)
            .timeout(Duration::from_millis(500))
        )
            .first_error()
            .unwrap();

        let payload = promise.try_recv().unwrap().unwrap();
        assert_eq!(payload.subscription_count, 1);
        assert_eq!(payload.service_count, 1);
    }
}