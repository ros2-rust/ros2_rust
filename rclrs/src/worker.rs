#[cfg(feature = "dyn_msg")]
use crate::{
    dynamic_message::{
        DynamicMessage, DynamicSubscriptionState, MessageTypeName, WorkerDynamicSubscription,
        WorkerDynamicSubscriptionCallback,
    },
    MessageInfo,
};
use crate::{
    log_fatal, IntoWorkerServiceCallback, IntoWorkerSubscriptionCallback, Node, Promise,
    RclrsError, ServiceOptions, ServiceState, SubscriptionOptions, SubscriptionState,
    WorkerCommands, WorkerService, WorkerSubscription,
};
use futures::channel::oneshot;
use rosidl_runtime_rs::{Message, Service as IdlService};
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

    /// Creates a [`WorkerDynamicSubscription`], whose message type is only know at runtime.
    ///
    /// Refer to ['Worker::create_subscription`] for the API and behavior except two key
    /// differences:
    ///
    ///   - The message type is determined at runtime through the `topic_type` function parameter.
    ///   - Only one type of callback is supported (returning both [`crate::DynamicMessage`] and
    ///   [`crate::MessageInfo`]).
    ///
    /// ```
    /// # use rclrs::*;
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// // The worker's payload is data that we want to share with other callbacks.
    /// let worker = node.create_worker::<Option<String>>(None);
    ///
    /// // This variable will be the mutable internal state of the subscription
    /// // callback.
    /// let mut count = 0_usize;
    ///
    /// let subscription = worker.create_dynamic_subscription(
    ///     "example_interfaces/msg/String".try_into()?,
    ///     "topic",
    ///     move |data: &mut Option<String>, msg: DynamicMessage, _msg_info: MessageInfo| {
    ///         count += 1;
    ///         let value = msg.get("data").unwrap();
    ///         let Value::Simple(value) = value else {
    ///             panic!("Unexpected value type, expected Simple value");
    ///         };
    ///         let SimpleValue::String(value) = value else {
    ///             panic!("Unexpected value type, expected String");
    ///         };
    ///         println!("#{count} | I heard: '{}'", value);
    ///
    ///         *data = Some(value.to_string());
    ///     },
    /// )?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    #[cfg(feature = "dyn_msg")]
    pub fn create_dynamic_subscription<'a, F>(
        &self,
        topic_type: MessageTypeName,
        options: impl Into<SubscriptionOptions<'a>>,
        callback: F,
    ) -> Result<WorkerDynamicSubscription<Payload>, RclrsError>
    where
        F: FnMut(&mut Payload, DynamicMessage, MessageInfo) + Send + Sync + 'static,
    {
        DynamicSubscriptionState::<Worker<Payload>>::create(
            topic_type,
            options,
            WorkerDynamicSubscriptionCallback::new(callback),
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
    /// # let executor = Context::default().create_basic_executor();
    /// # let node = executor.create_node("my_node").unwrap();
    /// use example_interfaces::srv::*;
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
    /// let service = worker.create_service::<AddTwoInts, _>(
    ///     "add",
    ///     move |payload: &mut Operands, request: AddTwoInts_Request| {
    ///         count += 1;
    ///         let AddTwoInts_Request { a, b } = request;
    ///         let sum = a + b;
    ///         println!("#{count} | {a} + {b} = {sum}");
    ///
    ///         *payload = Operands { a, b };
    ///
    ///         AddTwoInts_Response { sum }
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
        T: IdlService,
    {
        ServiceState::<T, Worker<Payload>>::create(
            options,
            callback.into_worker_service_callback(),
            self.node.handle(),
            &self.commands,
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
    use crate::*;
    use std::time::Duration;
    use test_msgs::{
        msg::Empty as EmptyMsg,
        srv::{Empty as EmptySrv, Empty_Request, Empty_Response},
    };

    #[derive(Default, Clone, Copy, Debug)]
    struct TestPayload {
        subscription_count: usize,
        service_count: usize,
        #[cfg(feature = "dyn_msg")]
        dynamic_subscription_count: usize,
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

        #[cfg(feature = "dyn_msg")]
        let _count_dynamic_sub = worker.create_dynamic_subscription(
            "test_msgs/msg/Empty".try_into().unwrap(),
            "test_worker_topic",
            |payload: &mut TestPayload, _, _| {
                payload.dynamic_subscription_count += 1;
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
                #[cfg(feature = "dyn_msg")]
                if payload.dynamic_subscription_count > 0 {
                    Some(*payload)
                } else {
                    None
                }
                #[cfg(not(feature = "dyn_msg"))]
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
