use crate::{
    log_fatal, AnyTimerCallback, IntoTimerOptions, IntoWorkerServiceCallback,
    IntoWorkerSubscriptionCallback, IntoWorkerTimerOneshotCallback,
    IntoWorkerTimerRepeatingCallback, Node, Promise, RclrsError, ServiceOptions, ServiceState,
    SubscriptionOptions, SubscriptionState, TimerState, WorkerCommands, WorkerService,
    WorkerSubscription, WorkerTimer,
};
use futures::channel::oneshot;
use rosidl_runtime_rs::{Message, Service as ServiceIDL};
use std::{
    any::Any,
    sync::{Arc, Mutex, Weak},
};

/// A worker that carries a payload and synchronizes callbacks for subscriptions
/// and services. Workers share much in common with "callback groups" from rclcpp,
/// with the addition of holding a data payload to share between the callbacks.
///
/// The payload is any data type of your choosing. Each callback associated with
/// this worker will receive a mutable borrow (`&mut Payload`) of the payload,
/// allowing them to share the payload data for both viewing and modifying. The
/// worker only contains exactly one instance of the payload that is shared
/// across all callbacks.
///
/// You can also run ad hoc tasks on the worker to view or modify the payload
/// from callbacks that are not associated with this worker.
///
/// The API for the worker is provided through [`WorkerState`].
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
                log_fatal!(
                    "rclrs.worker",
                    "{}",
                    Self::conversion_failure_message(any_payload)
                );
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
    /// [1]: crate::SpinOptions::until_promise_resolved
    pub fn run_with_notice<Out, F>(&self, f: F) -> (Promise<Out>, Promise<()>)
    where
        F: FnOnce(&mut Payload) -> Out + 'static + Send,
        Out: 'static + Send,
    {
        let (notice_sender, notice_receiver) = oneshot::channel();
        let (sender, receiver) = oneshot::channel();
        self.commands.run_on_payload(Box::new(move |any_payload| {
            let Some(payload) = any_payload.downcast_mut::<Payload>() else {
                log_fatal!(
                    "rclrs.worker",
                    "{}",
                    Self::conversion_failure_message(any_payload)
                );
                return;
            };

            let out = f(payload);
            sender.send(out).ok();
            notice_sender.send(()).ok();
        }));
        (receiver, notice_receiver)
    }

    /// Listen to activity that happens with this worker's primitives. The
    /// listening will continue until the [`ActivityListener`] is dropped.
    pub fn listen<F>(&self, mut f: F) -> ActivityListener<Payload>
    where
        F: FnMut(&mut Payload) + 'static + Send + Sync,
        Payload: Sync,
    {
        let f = Box::new(move |any_payload: &mut dyn Any| {
            let Some(payload) = any_payload.downcast_mut::<Payload>() else {
                log_fatal!(
                    "rclrs.worker",
                    "{}",
                    Self::conversion_failure_message(any_payload)
                );
                return;
            };

            f(payload);
        });

        let listener = ActivityListener::new(ActivityListenerCallback::Listen(f));
        self.commands
            .add_activity_listener(Arc::downgrade(&listener.callback));
        listener
    }

    /// The callback that you provide will be triggered each time the worker has
    /// some activity until it returns [`Some<Out>`] or until the [`Promise`] is
    /// dropped.
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

        self.commands
            .add_activity_listener(Arc::downgrade(&listener.callback));

        receiver
    }

    /// Creates a [`WorkerSubscription`].
    ///
    /// Unlike subscriptions created from a [`Node`], the callbacks for these
    /// subscriptions can have an additional argument to get a mutable borrow of
    /// the [`Worker`]'s payload. This allows the subscription callback to operate
    /// on state data that gets shared with other callbacks.
    ///
    /// # Behavior
    ///
    /// While the callback of this subscription is running, no other callbacks
    /// associated with the [`Worker`] will be able to run. This is necessary to
    /// guarantee that the mutable borrow of the payload is safe.
    ///
    /// Since the callback of this subscription may block other callbacks from
    /// being able to run, it is strongly recommended to ensure that the
    /// callback returns quickly. If the callback needs to trigger long-running
    /// behavior then you can consider using `std::thread::spawn`, or for async
    /// behaviors you can capture an [`ExecutorCommands`][1] in your callback
    /// and use [`ExecutorCommands::run`][2] to issue a task for the executor
    /// to run in its async task pool.
    ///
    /// [1]: crate::ExecutorCommands
    /// [2]: crate::ExecutorCommands::run
    ///
    /// # Subscription Options
    ///
    /// See [`NodeState::create_subscription`][3] for examples of setting the
    /// subscription options.
    ///
    /// [3]: crate::NodeState::create_subscription
    ///
    /// # Worker Subscription Callbacks
    ///
    /// Worker Subscription callbacks support twelve signatures:
    /// - [`FnMut`] ( `Message` )
    /// - [`FnMut`] ( `Message`, [`MessageInfo`][4] )
    /// - [`FnMut`] ( `&mut Payload`, `Message` )
    /// - [`FnMut`] ( `&mut Payload`, `Message`, [`MessageInfo`][4] )
    /// - [`FnMut`] ( [`Box`]<`Message`> )
    /// - [`FnMut`] ( [`Box`]<`Message`>, [`MessageInfo`][4] )
    /// - [`FnMut`] ( `&mut Payload`, [`Box`]<`Message`> )
    /// - [`FnMut`] ( `&mut Payload`, [`Box`]<`Message`>, [`MessageInfo`][4] )
    /// - [`FnMut`] ( [`ReadOnlyLoanedMessage`][5]<`Message`> )
    /// - [`FnMut`] ( [`ReadOnlyLoanedMessage`][5]<`Message`>, [`MessageInfo`][4] )
    /// - [`FnMut`] ( `&mut Payload`, [`ReadOnlyLoanedMessage`][5]<`Message`> )
    /// - [`FnMut`] ( `&mut Payload`, [`ReadOnlyLoanedMessage`][5]<`Message`>, [`MessageInfo`][4] )
    ///
    /// [4]: crate::MessageInfo
    /// [5]: crate::ReadOnlyLoanedMessage
    ///
    /// Note that these signatures all use [`FnMut`] which means, unlike node
    /// subscriptions, the callback can have mutable internal state without
    /// needing to use [`Mutex`]. This is possible because the [`Worker`]
    /// ensures the callback cannot run multiple times simultaneously.
    ///
    /// Additionally your callback can get mutable access to the worker's
    /// payload by setting the first argument of the callback to `&mut Payload`.
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// // The worker's payload is data that we want to share with other callbacks.
    /// let worker = node.create_worker::<Option<String>>(None);
    ///
    /// // This variable will be the mutable internal state of the subscription
    /// // callback.
    /// let mut count = 0_usize;
    ///
    /// let subscription = worker.create_subscription::<example_interfaces::msg::String, _>(
    ///     "topic",
    ///     move |data: &mut Option<String>, msg: example_interfaces::msg::String| {
    ///         count += 1;
    ///         println!("#{count} | I heard: '{}'", msg.data);
    ///
    ///         *data = Some(msg.data);
    ///     },
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
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
    ///
    /// Unlike services created from a [`Node`], the callbacks for these services
    /// can have an additional argument to get a mutable borrow of the [`Worker`]'s
    /// payload. This allows the service callback to operate on state data that
    /// gets shared with other callbacks.
    ///
    /// # Behavior
    ///
    /// While the callback of this service is running, no other callbacks
    /// associated with the [`Worker`] will be able to run. This is necessary to
    /// guarantee that the mutable borrow of the payload is safe.
    ///
    /// Since the callback of this service may block other callbacks from being
    /// able to run, it is strongly recommended to ensure that the callback
    /// returns quickly. If the callback needs to trigger some long-running
    /// behavior before returning, then you should probably create the service
    /// using [`NodeState::create_async_service`][1] since the service callback
    /// of a worker is required to return the response of the service as its
    /// output value.
    ///
    /// To access the payload of a worker from an async service, see examples in
    /// the documentation of [`NodeState::create_async_service`][1].
    ///
    /// [1]: crate::NodeState::create_async_service
    ///
    /// # Service Options
    ///
    /// See the documentation of [`NodeState::create_service`][2] for examples
    /// of setting the service options.
    ///
    /// [2]: crate::NodeState::create_service
    ///
    /// # Worker Service Callbacks
    ///
    /// Worker service callbacks support six signatures:
    /// - [`FnMut`] ( `Request` ) -> `Response`
    /// - [`FnMut`] ( `Request`, [`RequestId`][3] ) -> `Response`
    /// - [`FnMut`] ( `Request`, [`ServiceInfo`][4] ) -> `Response`
    /// - [`FnMut`] ( `&mut Payload`, `Request` ) -> `Response`
    /// - [`FnMut`] ( `&mut Payload`, `Request`,  [`RequestId`][3] ) -> `Response`
    /// - [`FnMut`] ( `&mut Payload`, `Request`, [`ServiceInfo`][4] ) -> `Response`
    ///
    /// [3]: crate::RequestId
    /// [4]: crate::ServiceInfo
    ///
    /// Note that these signatures all use [`FnMut`] which means, unlike node
    /// services, the callback can have mutable internal state without needing
    /// to use [`Mutex`]. This is possible because the [`Worker`] ensures the
    /// callback cannot run multiple times simultaneously.
    ///
    /// Additionally your callback can get mutable access to the worker's
    /// payload by setting the first argument of the callback to `&mut Payload`.
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::example_interfaces;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    ///
    /// /// Store the operands of the service request for later reference
    /// #[derive(Default)]
    /// struct Operands {
    ///     a: i64,
    ///     b: i64,
    /// }
    ///
    /// // The worker's payload is data that we want to share with other callbacks.
    /// let worker = node.create_worker::<Operands>(Operands::default());
    ///
    /// // This variable will be the mutable internal state of the service
    /// // callback.
    /// let mut count = 0_usize;
    ///
    /// let service = worker.create_service::<example_interfaces::srv::AddTwoInts, _>(
    ///     "add",
    ///     move |payload: &mut Operands, request: example_interfaces::srv::AddTwoInts_Request| {
    ///         count += 1;
    ///         let example_interfaces::srv::AddTwoInts_Request { a, b } = request;
    ///         let sum = a + b;
    ///         println!("#{count} | {a} + {b} = {sum}");
    ///
    ///         *payload = Operands { a, b };
    ///
    ///         example_interfaces::srv::AddTwoInts_Response { sum }
    ///     }
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn create_service<'a, T, Args>(
        &self,
        options: impl Into<ServiceOptions<'a>>,
        callback: impl IntoWorkerServiceCallback<T, Payload, Args>,
    ) -> Result<WorkerService<T, Payload>, RclrsError>
    where
        T: ServiceIDL,
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
    /// Unlike timers created from a [`Node`], the callbacks for these timers
    /// can have an additional argument to get a mutable borrow of the [`Worker`]'s
    /// payload. This allows the timer callback to operate on state data that gets
    /// shared with other callbacks.
    ///
    /// See also:
    /// * [`Self::create_timer_oneshot`]
    /// * [`Self::create_timer_inert`]
    ///
    /// # Behavior
    ///
    /// While the callback of this timer is running, no other callbacks associated
    /// with the [`Worker`] will be able to run. This is necessary to guarantee
    /// that the mutable borrow of the payload is safe.
    ///
    /// Since the callback of this timer may block other callbacks from being
    /// able to run, it is strongly recommended to ensure that the callback
    /// returns quickly. If the callback needs to trigger long-running behavior
    /// then you can condier using [`std::thread::spawn`], or for async behaviors
    /// you can capture an [`ExecutorCommands`][1] in your callback and use
    /// [`ExecutorCommands::run`][2] to issue a task for the executor to run in
    /// its async task pool.
    ///
    /// # Timer Options
    ///
    /// See [`NodeState::create_timer_repeating`][3] for examples of setting the
    /// timer options.
    ///
    /// # Worker Timer Repeating Callbacks
    ///
    /// Worker Timer repeating callbacks support four signatures:
    /// - <code>[FnMut] ()</code>
    /// - <code>[FnMut] ( &mut Payload )</code>
    /// - <code>[FnMut] ( &mut Payload, [Time][4] )</code>
    /// - <code>[FnMut] ( &mut Payload, &[WorkerTimer]&lt;Payload&gt; )</code>
    ///
    /// You can choose to access the payload of the worker. You can additionally
    /// choose to receive the current time when the callback is being triggered.
    ///
    /// Or instead of the current time, you can get a borrow of the [`WorkerTimer`]
    /// itself, that way if you need to access it from inside the callback, you
    /// do not need to worry about capturing a [`Weak`] and then locking it. This
    /// is useful if you need to change the callback of the timer from inside the
    /// callback of the timer.
    ///
    /// For an [`FnOnce`] callback instead of [`FnMut`], use [`Self::create_timer_oneshot`].
    ///
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use std::time::Duration;
    ///
    /// let worker = node.create_worker::<usize>(0);
    /// let timer = worker.create_timer_repeating(
    ///     Duration::from_secs(1),
    ///     |count: &mut usize, time: Time| {
    ///         *count += 1;
    ///         println!("Drinking my {}th 🧉 at {:?}.", *count, time);
    ///     },
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::ExecutorCommands
    /// [2]: crate::ExecutorCommands::run
    /// [3]: crate::NodeState::create_timer_repeating
    /// [4]: crate::Time
    pub fn create_timer_repeating<'a, Args>(
        &self,
        options: impl IntoTimerOptions<'a>,
        callback: impl IntoWorkerTimerRepeatingCallback<Worker<Payload>, Args>,
    ) -> Result<WorkerTimer<Payload>, RclrsError> {
        self.create_timer(options, callback.into_worker_timer_repeating_callback())
    }

    /// Create a [`WorkerTimer`] whose callback will be triggered once after the
    /// period of the timer has elapsed. After that you will need to use
    /// [`TimerState::set_worker_oneshot`] or [`TimerState::set_worker_repeating`]
    /// or else nothing will happen the following times that the `Timer` elapses.
    ///
    /// See also:
    /// * [`Self::create_timer_repeating`]
    /// * [`Self::create_timer_inert`]
    ///
    /// # Behavior
    ///
    /// While the callback of this timer is running, no other callbacks associated
    /// with the [`Worker`] will be able to run. This is necessary to guarantee
    /// that the mutable borrow of the payload is safe.
    ///
    /// Since the callback of this timer may block other callbacks from being
    /// able to run, it is strongly recommended to ensure that the callback
    /// returns quickly. If the callback needs to trigger long-running behavior
    /// then you can condier using `std::thread::spawn`, or for async behaviors
    /// you can capture an [`ExecutorCommands`][1] in your callback and use
    /// [`ExecutorCommands::run`][2] to issue a task for the executor to run in
    /// its async task pool.
    ///
    /// # Timer Options
    ///
    /// See [`NodeSate::create_timer_repeating`][3] for examples of setting the
    /// timer options.
    ///
    /// # Worker Timer Oneshot Callbacks
    ///
    /// Worker Timer oneshot callbacks support four signatures:
    /// - <code>[FnOnce] ()</code>
    /// - <code>[FnOnce] ( &mut Payload )</code>
    /// - <code>[FnOnce] ( &mut Payload, [Time][4] )</code>
    /// - <code>[FnOnce] ( &mut Payload, &[WorkerTimer]&lt;Payload&gt; )</code>
    ///
    /// You can choose to access the payload of the worker. You can additionally
    /// choose to receive the current time when the callback is being triggered.
    ///
    /// Or instead of the current time, you can get a borrow of the [`WorkerTimer`]
    /// itself, that way if you need to access it from inside the callback, you
    /// do not need to worry about capturing a [`Weak`] and then locking it. This
    /// is useful if you need to change the callback of the timer from inside the
    /// callback of the timer, which may be needed often for oneshot callbacks.
    ///
    /// The callback will only be triggered once. After that, this will effectively
    /// be an [inert][5] timer.
    ///
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// # let worker = node.create_worker::<()>(());
    /// use std::time::Duration;
    ///
    /// let timer = worker.create_timer_oneshot(
    ///     Duration::from_secs(1),
    ///     || {
    ///         println!("This will only fire once");
    ///     }
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// [1]: crate::ExecutorCommands
    /// [2]: crate::ExecutorCommands::run
    /// [3]: crate::NodeState::create_timer_repeating
    /// [4]: crate::Time
    /// [5]: Self::create_timer_inert
    pub fn create_timer_oneshot<'a, Args>(
        &self,
        options: impl IntoTimerOptions<'a>,
        callback: impl IntoWorkerTimerOneshotCallback<Worker<Payload>, Args>,
    ) -> Result<WorkerTimer<Payload>, RclrsError> {
        self.create_timer(options, callback.into_worker_timer_oneshot_callback())
    }

    /// Create a [`WorkerTimer`] without a callback. Nothing will happen when this
    /// `WorkerTimer` elapses until you use [`TimerState::set_worker_repeating`]
    /// or [`TimerState::set_worker_oneshot`].
    ///
    /// This function is not usually what you want. An inert timer is usually
    /// just a follow-up state to a oneshot timer which is waiting to be given
    /// a new callback to run. However, you could use this method to declare a
    /// timer whose callbacks you will start to feed in at a later.
    ///
    /// There is no equivalent to this function in `rclcpp`.
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
        let node = options.clock.is_node_time().then(|| Arc::clone(&self.node));
        TimerState::create(
            options.period,
            clock,
            callback,
            &self.commands,
            &self.node.handle().context_handle,
            node,
        )
    }

    /// Used by [`Node`][crate::Node] to create a `WorkerState`. Users should
    /// call [`Node::create_worker`][crate::NodeState::create_worker] instead of
    /// this.
    pub(crate) fn create(node: Node, commands: Arc<WorkerCommands>) -> Arc<Self> {
        Arc::new(Self {
            node,
            commands,
            _ignore: Default::default(),
        })
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

impl<T> From<T> for WorkerOptions<T> {
    fn from(value: T) -> Self {
        WorkerOptions::new(value)
    }
}

/// `ActivityListener` is used to watch the activity of the primitives of a [`Worker`].
///
/// Each listener will be triggered each time a primitive is active on the `Worker`.
/// Listener callbacks will not be triggered when something other than a primitive
/// modifies the payload, which can happen through [interior mutability][1] or by
/// one of the other listeners, so this is not a sure-fire way to track changes in
/// the payload.
///
/// Note that the listener callback will be triggered if any primitive has run
/// since the payload *may* have been modified. We do not track changes on the
/// payload, so it is possible that its data was not changed by the primitive at
/// all, but the listener will still be triggered.
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
        Self {
            callback,
            _ignore: Default::default(),
        }
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
    use crate::{
        vendor::test_msgs::{
            msg::Empty as EmptyMsg,
            srv::{Empty as EmptySrv, Empty_Request, Empty_Response},
        },
        *,
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
            },
        );

        let _count_srv = worker.create_service::<EmptySrv, _>(
            "test_worker_service",
            |payload: &mut TestPayload, _req: Empty_Request| {
                payload.service_count += 1;
                Empty_Response::default()
            },
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

        let client = node
            .create_client::<EmptySrv>("test_worker_service")
            .unwrap();
        let _: Promise<Empty_Response> = client.call(Empty_Request::default()).unwrap();

        let (mut promise, notice) = executor.commands().create_notice(promise);

        executor
            .spin(
                SpinOptions::new()
                    .until_promise_resolved(notice)
                    .timeout(Duration::from_millis(500)),
            )
            .first_error()
            .unwrap();

        let payload = promise.try_recv().unwrap().unwrap();
        assert_eq!(payload.subscription_count, 1);
        assert_eq!(payload.service_count, 1);
    }
}
