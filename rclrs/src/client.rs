use std::{
    any::Any,
    collections::HashMap,
    ffi::{CStr, CString},
    sync::{Arc, Mutex, MutexGuard},
};

use rosidl_runtime_rs::Message;

use crate::{
    error::ToResult, log_fatal, rcl_bindings::*, IntoPrimitiveOptions, MessageCow, Node, Promise,
    QoSProfile, RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, RclReturnCode, RclrsError,
    ReadyKind, ServiceInfo, Waitable, WaitableLifecycle, ENTITY_LIFECYCLE_MUTEX,
};

mod client_async_callback;
pub use client_async_callback::*;

mod client_callback;
pub use client_callback::*;

mod client_output;
pub use client_output::*;

/// Main class responsible for sending requests to a ROS service.
///
/// Create a client using [`Node::create_client`][1].
///
/// Receiving responses requires the node's executor to [spin][2].
///
/// [1]: crate::NodeState::create_client
/// [2]: crate::Executor::spin
pub type Client<T> = Arc<ClientState<T>>;

/// The inner state of a [`Client`].
///
/// This is public so that you can choose to create a [`Weak`][1] reference to it
/// if you want to be able to refer to a [`Client`] in a non-owning way. It is
/// generally recommended to manage the `ClientState` inside of an [`Arc`],
/// and [`Client`] is provided as a convenience alias for that.
///
/// The public API of the [`Client`] type is implemented via `ClientState`.
///
/// [1]: std::sync::Weak
pub struct ClientState<T>
where
    T: rosidl_runtime_rs::Service,
{
    handle: Arc<ClientHandle>,
    board: Arc<Mutex<ClientRequestBoard<T>>>,
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
}

impl<T> ClientState<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Send out a request for this service client.
    ///
    /// If the call to rcl succeeds, you will receive a [`Promise`] of the
    /// service response. You can choose what kind of metadata you receive. The
    /// promise can provide any of the following:
    /// - `Response`
    /// - `(Response, `[`RequestId`][1]`)`
    /// - `(Response, `[`ServiceInfo`][2]`)`
    ///
    /// Dropping the [`Promise`] that this returns will not cancel the request.
    /// Once this function is called, the service provider will receive the
    /// request and respond to it no matter what.
    ///
    /// [1]: crate::RequestId
    /// [2]: crate::ServiceInfo
    pub fn call<'a, Req, Out>(&self, request: Req) -> Result<Promise<Out>, RclrsError>
    where
        Req: MessageCow<'a, T::Request>,
        Out: ClientOutput<T::Response>,
    {
        let (sender, promise) = Out::create_channel();
        let rmw_message = T::Request::into_rmw_message(request.into_cow());
        let mut sequence_number = -1;
        unsafe {
            // SAFETY: The client handle ensures the rcl_client is valid and
            // our generic system ensures it has the correct type.
            rcl_send_request(
                &*self.handle.lock() as *const _,
                rmw_message.as_ref() as *const <T::Request as Message>::RmwMsg as *mut _,
                &mut sequence_number,
            )
        }
        .ok()?;

        self.board
            .lock()
            .map_err(|_| RclrsError::PoisonedMutex)?
            .new_request(sequence_number, sender);

        Ok(promise)
    }

    /// Call this service and then handle its response with a regular callback.
    ///
    /// You do not need to retain the [`Promise`] that this returns, even if the
    /// compiler warns you that you need to. You can use the [`Promise`] to know
    /// when the response is finished being processed, but otherwise you can
    /// safely discard it.
    ///
    /// # Client Callbacks
    ///
    /// Three callback signatures are supported:
    /// - [`FnOnce`] ( `Response` )
    /// - [`FnOnce`] ( `Response`, [`RequestId`][1] )
    /// - [`FnOnce`] ( `Response`, [`ServiceInfo`] )
    ///
    /// [1]: crate::RequestId
    ///
    /// Note that all of these are [`FnOnce`] which grants the greatest amount
    /// of freedom for what kind of operations you can perform within the
    /// callback. Just remember that this also means the callbacks are strictly
    /// one-time-use.
    pub fn call_then<'a, Req, Args>(
        &self,
        request: Req,
        callback: impl ClientCallback<T, Args>,
    ) -> Result<Promise<()>, RclrsError>
    where
        Req: MessageCow<'a, T::Request>,
    {
        let callback = move |response, info| async {
            callback.run_client_callback(response, info);
        };
        self.call_then_async(request, callback)
    }

    /// Call this service and then handle its response with an async callback.
    ///
    /// You do not need to retain the [`Promise`] that this returns, even if the
    /// compiler warns you that you need to. You can use the [`Promise`] to know
    /// when the response is finished being processed, but otherwise you can
    /// safely discard it.
    ///
    /// # Async Client Callbacks
    ///
    /// Three callback signatures are supported:
    /// - [`FnOnce`] ( `Response` ) -> impl [`Future`][1]<Output=()>
    /// - [`FnOnce`] ( `Response`, [`RequestId`][2] ) -> impl [`Future`][1]<Output=()>
    /// - [`FnOnce`] ( `Response`, [`ServiceInfo`] ) -> impl [`Future`][1]<Output=()>
    ///
    /// [1]: std::future::Future
    /// [2]: crate::RequestId
    ///
    /// Since this method is to help implement async behaviors, the callback that
    /// you pass to it must return a [`Future`][1]. There are two ways to create
    /// a `Future` in Rust:
    ///
    /// ## 1. `async fn`
    ///
    /// Define an `async fn` whose arguments are compatible with one of the above
    /// signatures and which returns a `()` (a.k.a. nothing).
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::test_msgs;
    /// # let node = Context::default()
    /// #   .create_basic_executor()
    /// #   .create_node("test_node")?;
    ///
    /// async fn print_hello(_response: test_msgs::srv::Empty_Response) {
    ///     print!("Hello!");
    /// }
    ///
    /// let client = node.create_client::<test_msgs::srv::Empty>("my_service")?;
    /// let request = test_msgs::srv::Empty_Request::default();
    /// let promise = client.call_then_async(&request, print_hello)?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// ## 2. Function that returns an `async { ... }`
    ///
    /// You can pass in a callback that returns an `async` block. `async` blocks
    /// have an important advantage over `async fn`: You can use `async move { ... }`
    /// to capture data into the async block. This allows you to embed some state
    /// data into your callback.
    ///
    /// You can do this with either a regular `fn` or with a closure.
    ///
    /// ### `fn`
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::test_msgs;
    /// # use std::future::Future;
    /// # let node = Context::default()
    /// #   .create_basic_executor()
    /// #   .create_node("test_node")?;
    ///
    /// fn print_greeting(_response: test_msgs::srv::Empty_Response) -> impl Future<Output=()> {
    ///     let greeting = "Hello!";
    ///     async move {
    ///         print!("Hello!");
    ///     }
    /// }
    ///
    /// let client = node.create_client::<test_msgs::srv::Empty>("my_service")?;
    /// let request = test_msgs::srv::Empty_Request::default();
    /// let promise = client.call_then_async(
    ///     &request,
    ///     print_greeting)?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    ///
    /// ### Closure
    ///
    /// A closure will allow you to capture data into the callback from the
    /// surrounding context. While the syntax for this is more complicated, it
    /// is also the most powerful option.
    ///
    /// ```
    /// # use rclrs::*;
    /// # use crate::rclrs::vendor::test_msgs;
    /// # let node = Context::default()
    /// #   .create_basic_executor()
    /// #   .create_node("test_node")?;
    ///
    /// let greeting = "Hello!";
    /// let client = node.create_client::<test_msgs::srv::Empty>("my_service")?;
    /// let request = test_msgs::srv::Empty_Request::default();
    /// let promise = client.call_then_async(
    ///     &request,
    ///     move |response: test_msgs::srv::Empty_Response| {
    ///         async move {
    ///             print!("{greeting}");
    ///         }
    ///     })?;
    /// # Ok::<(), RclrsError>(())
    /// ```
    pub fn call_then_async<'a, Req, Args>(
        &self,
        request: Req,
        callback: impl ClientAsyncCallback<T, Args>,
    ) -> Result<Promise<()>, RclrsError>
    where
        Req: MessageCow<'a, T::Request>,
    {
        let response: Promise<(T::Response, ServiceInfo)> = self.call(request)?;
        let promise = self.handle.node.commands().run(async move {
            match response.await {
                Ok((response, info)) => {
                    callback.run_client_async_callback(response, info).await;
                }
                Err(_) => {
                    log_fatal!(
                        "rclrs.client.call_then_async",
                        "Request promise has been dropped by the executor",
                    );
                }
            }
        });

        Ok(promise)
    }

    /// Check if a service server is available.
    ///
    /// Will return true if there is a service server available, false if unavailable.
    ///
    /// Consider using [`Self::notify_on_service_ready`] if you want to wait
    /// until a service for this client is ready.
    pub fn service_is_ready(&self) -> Result<bool, RclrsError> {
        let mut is_ready = false;
        let client = &mut *self.handle.rcl_client.lock().unwrap();
        let node = &mut *self.handle.node.handle().rcl_node.lock().unwrap();

        unsafe {
            // SAFETY both node and client are guaranteed to be valid here
            // client is guaranteed to have been generated with node
            rcl_service_server_is_available(node as *const _, client as *const _, &mut is_ready)
        }
        .ok()?;
        Ok(is_ready)
    }

    /// Get a promise that will be fulfilled when a service is ready for this
    /// client. You can `.await` the promise in an async function or use it for
    /// `until_promise_resolved` in [`SpinOptions`][crate::SpinOptions].
    pub fn notify_on_service_ready(self: &Arc<Self>) -> Promise<()> {
        let client = Arc::clone(self);
        self.handle
            .node
            .notify_on_graph_change(move || client.service_is_ready().is_ok_and(|r| r))
    }

    /// Get the name of the service that this client intends to call.
    pub fn service_name(&self) -> String {
        unsafe {
            let char_ptr = rcl_client_get_service_name(&*self.handle.lock() as *const _);
            debug_assert!(!char_ptr.is_null());
            CStr::from_ptr(char_ptr).to_string_lossy().into_owned()
        }
    }

    /// Creates a new client.
    pub(crate) fn create<'a>(
        options: impl Into<ClientOptions<'a>>,
        node: &Node,
    ) -> Result<Arc<Self>, RclrsError>
    // This uses pub(crate) visibility to avoid instantiating this struct outside
    // [`Node::create_client`], see the struct's documentation for the rationale
    where
        T: rosidl_runtime_rs::Service,
    {
        let ClientOptions { service_name, qos } = options.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_client = unsafe { rcl_get_zero_initialized_client() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string =
            CString::new(service_name).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: service_name.into(),
            })?;

        // SAFETY: No preconditions for this function.
        let mut client_options = unsafe { rcl_client_get_default_options() };
        client_options.qos = qos.into();

        {
            let rcl_node = node.handle().rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();

            // SAFETY:
            // * The rcl_client was zero-initialized as expected by this function.
            // * The rcl_node is kept alive by the NodeHandle because it is a dependency of the client.
            // * The topic name and the options are copied by this function, so they can be dropped
            //   afterwards.
            // * The entity lifecycle mutex is locked to protect against the risk of global
            //   variables in the rmw implementation being unsafely modified during initialization.
            unsafe {
                rcl_client_init(
                    &mut rcl_client,
                    &*rcl_node,
                    type_support,
                    topic_c_string.as_ptr(),
                    &client_options,
                )
                .ok()?;
            }
        }

        let commands = node.commands().async_worker_commands();
        let handle = Arc::new(ClientHandle {
            rcl_client: Mutex::new(rcl_client),
            node: Arc::clone(&node),
        });

        let board = Arc::new(Mutex::new(ClientRequestBoard::new()));

        let (waitable, lifecycle) = Waitable::new(
            Box::new(ClientExecutable {
                handle: Arc::clone(&handle),
                board: Arc::clone(&board),
            }),
            Some(Arc::clone(&commands.get_guard_condition())),
        );
        commands.add_to_wait_set(waitable);

        Ok(Arc::new(Self {
            handle,
            board,
            lifecycle,
        }))
    }
}

/// `ClientOptions` are used by [`Node::create_client`][1] to initialize a
/// [`Client`] for a service.
///
/// [1]: crate::NodeState::create_client
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct ClientOptions<'a> {
    /// The name of the service that this client will send requests to
    pub service_name: &'a str,
    /// The quality of the service profile for this client
    pub qos: QoSProfile,
}

impl<'a> ClientOptions<'a> {
    /// Initialize a new [`ClientOptions`] with default settings.
    pub fn new(service_name: &'a str) -> Self {
        Self {
            service_name,
            qos: QoSProfile::services_default(),
        }
    }
}

impl<'a, T: IntoPrimitiveOptions<'a>> From<T> for ClientOptions<'a> {
    fn from(value: T) -> Self {
        let primitive = value.into_primitive_options();
        let mut options = Self::new(primitive.name);
        primitive.apply_to(&mut options.qos);
        options
    }
}

struct ClientExecutable<T>
where
    T: rosidl_runtime_rs::Service,
{
    handle: Arc<ClientHandle>,
    board: Arc<Mutex<ClientRequestBoard<T>>>,
}

impl<T> RclPrimitive for ClientExecutable<T>
where
    T: rosidl_runtime_rs::Service,
{
    unsafe fn execute(&mut self, ready: ReadyKind, _: &mut dyn Any) -> Result<(), RclrsError> {
        ready.for_basic()?;
        self.board.lock().unwrap().execute(&self.handle)
    }

    fn handle(&self) -> RclPrimitiveHandle {
        RclPrimitiveHandle::Client(self.handle.lock())
    }

    fn kind(&self) -> RclPrimitiveKind {
        RclPrimitiveKind::Client
    }
}

type SequenceNumber = i64;

/// This is used internally to monitor the state of active requests, as well as
/// responses that have arrived without a known request.
struct ClientRequestBoard<T>
where
    T: rosidl_runtime_rs::Service,
{
    // This stores all active requests that have not received a response yet
    active_requests: HashMap<SequenceNumber, AnyClientOutputSender<T::Response>>,
    // This holds responses that came in when no active request matched the
    // sequence number. This could happen if take_response is triggered before
    // the new_request for the same sequence number. That is extremely unlikely
    // to ever happen but is theoretically possible on systems that may exhibit
    // very strange CPU scheduling patterns, so we should account for it.
    loose_responses: HashMap<SequenceNumber, (T::Response, rmw_service_info_t)>,
}

impl<T> ClientRequestBoard<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn new() -> Self {
        Self {
            active_requests: Default::default(),
            loose_responses: Default::default(),
        }
    }

    fn new_request(
        &mut self,
        sequence_number: SequenceNumber,
        sender: AnyClientOutputSender<T::Response>,
    ) {
        if let Some((response, info)) = self.loose_responses.remove(&sequence_number) {
            // Weirdly the response for this request already arrived, so we'll
            // send it off immediately.
            sender.send_response(response, info);
        } else {
            self.active_requests.insert(sequence_number, sender);
        }
    }

    fn execute(&mut self, handle: &Arc<ClientHandle>) -> Result<(), RclrsError> {
        match self.take_response(handle) {
            Ok((response, info)) => {
                let seq = info.request_id.sequence_number;
                if let Some(sender) = self.active_requests.remove(&seq) {
                    // The active request is available, so send this response off
                    sender.send_response(response, info);
                } else {
                    // Weirdly there isn't an active request for this, so save
                    // it in the loose responses map.
                    self.loose_responses.insert(seq, (response, info));
                }
            }
            Err(err) => {
                match err {
                    RclrsError::RclError {
                        code: RclReturnCode::ClientTakeFailed,
                        ..
                    } => {
                        // This is okay, it means a spurious wakeup happened
                    }
                    err => {
                        log_fatal!(
                            "rclrs.client.execute",
                            "Error while taking a response for a client: {err}",
                        );
                    }
                }
            }
        }
        Ok(())
    }

    fn take_response(
        &self,
        handle: &Arc<ClientHandle>,
    ) -> Result<(T::Response, rmw_service_info_t), RclrsError> {
        let mut service_info_out = ServiceInfo::zero_initialized_rmw();
        let mut response_out = <T::Response as Message>::RmwMsg::default();
        let handle = &*handle.lock();
        unsafe {
            // SAFETY: The three pointers are all kept valid by the handle
            rcl_take_response_with_info(
                handle,
                &mut service_info_out,
                &mut response_out as *mut <T::Response as Message>::RmwMsg as *mut _,
            )
        }
        .ok()
        .map(|_| {
            (
                T::Response::from_rmw_message(response_out),
                service_info_out,
            )
        })
    }
}

/// Manage the lifecycle of an `rcl_client_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_client_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
struct ClientHandle {
    rcl_client: Mutex<rcl_client_t>,
    /// We store the whole node here because we use some of its user-facing API
    /// in some of the Client methods.
    node: Node,
}

impl ClientHandle {
    fn lock(&self) -> MutexGuard<rcl_client_t> {
        self.rcl_client.lock().unwrap()
    }
}

impl Drop for ClientHandle {
    fn drop(&mut self) {
        let rcl_client = self.rcl_client.get_mut().unwrap();
        let mut rcl_node = self.node.handle().rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_client_fini(rcl_client, &mut *rcl_node);
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_client_t {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{test_helpers::*, vendor::test_msgs};

    #[test]
    fn traits() {
        assert_send::<Client<test_msgs::srv::Arrays>>();
        assert_sync::<Client<test_msgs::srv::Arrays>>();
    }

    #[test]
    fn test_clients() -> Result<(), RclrsError> {
        let namespace = "/test_clients_graph";
        let graph = construct_test_graph(namespace)?;
        let _node_2_empty_client = graph
            .node2
            .create_client::<test_msgs::srv::Empty>("graph_test_topic_4")?;

        std::thread::sleep(std::time::Duration::from_millis(200));

        let client_names_and_types = graph
            .node2
            .get_client_names_and_types_by_node(&graph.node2.name(), &graph.node2.namespace())?;
        let types = client_names_and_types
            .get("/test_clients_graph/graph_test_topic_4")
            .unwrap();

        assert!(types.contains(&"test_msgs/srv/Empty".to_string()));

        Ok(())
    }
}
