use std::{
    ffi::CString,
    sync::{Arc, Mutex, MutexGuard},
    collections::HashMap,
};

use rosidl_runtime_rs::Message;

use crate::{
    error::ToResult,
    rcl_bindings::*,
    MessageCow, Node, RclrsError, RclReturnCode, Promise, ENTITY_LIFECYCLE_MUTEX,
    RclPrimitive, QoSProfile, Waitable, WaitableLifecycle,
    RclPrimitiveHandle, RclPrimitiveKind, ServiceInfo,
};

mod client_async_callback;
pub use client_async_callback::*;

mod client_callback;
pub use client_callback::*;

mod client_output;
pub use client_output::*;

/// Main class responsible for sending requests to a ROS service.
///
/// The only available way to instantiate clients is via [`Node::create_client`][1], this is to
/// ensure that [`Node`][2]s can track all the clients that have been created.
///
/// [1]: crate::Node::create_client
/// [2]: crate::Node
pub struct Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    handle: Arc<ClientHandle>,
    board: Arc<Mutex<ClientRequestBoard<T>>>,
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
}

impl<T> Client<T>
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
    pub fn call<'a, Req, Out>(
        &self,
        request: Req,
    ) -> Result<Promise<Out>, RclrsError>
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

        // TODO(@mxgrey): Log errors here when logging becomes available.
        self.board.lock().unwrap().new_request(sequence_number, sender);

        Ok(promise)
    }

    /// Call this service and then handle its response with a regular callback.
    ///
    /// You do not need to retain the [`Promise`] that this returns, even if the
    /// compiler warns you that you need to. You can use the [`Promise`] to know
    /// when the response is finished being processed, but otherwise you can
    /// safely discard it.
    //
    // TODO(@mxgrey): Add documentation to show what callback signatures are supported
    pub fn call_then<'a, Req, Args>(
        &self,
        request: Req,
        callback: impl ClientCallback<T, Args>,
    ) -> Result<Promise<()>, RclrsError>
    where
        Req: MessageCow<'a, T::Request>,
    {
        let callback = move |response, info| {
            async { callback.run_client_callback(response, info); }
        };
        self.call_then_async(request, callback)
    }

    /// Call this service and then handle its response with an async callback.
    ///
    /// You do not need to retain the [`Promise`] that this returns, even if the
    /// compiler warns you that you need to. You can use the [`Promise`] to know
    /// when the response is finished being processed, but otherwise you can
    /// safely discard it.
    //
    // TODO(@mxgrey): Add documentation to show what callback signatures are supported
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
                    // TODO(@mxgrey): Log this error when logging becomes available
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
        self.handle.node.notify_on_graph_change(
            // TODO(@mxgrey): Log any errors here once logging is available
            move || client.service_is_ready().is_ok_and(|r| r)
        )
    }

    /// Creates a new client.
    pub(crate) fn create(
        topic: &str,
        qos: QoSProfile,
        node: &Arc<Node>,
    ) -> Result<Arc<Self>, RclrsError>
    // This uses pub(crate) visibility to avoid instantiating this struct outside
    // [`Node::create_client`], see the struct's documentation for the rationale
    where
        T: rosidl_runtime_rs::Service,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_client = unsafe { rcl_get_zero_initialized_client() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
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

        let handle = Arc::new(ClientHandle {
            rcl_client: Mutex::new(rcl_client),
            node: Arc::clone(&node),
        });

        let commands = node.commands();
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

struct ClientExecutable<T>
where
    T: rosidl_runtime_rs::Service,
{
    handle: Arc<ClientHandle>,
    board: Arc<Mutex<ClientRequestBoard<T>>>
}

impl<T> RclPrimitive for ClientExecutable<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn execute(&mut self) -> Result<(), RclrsError> {
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
                    RclrsError::RclError { code: RclReturnCode::ClientTakeFailed, .. } => {
                        // This is okay, it means a spurious wakeup happened
                    }
                    err => {
                        // TODO(@mxgrey): Log the error here once logging is available
                        eprintln!("Error while taking a response for a client: {err}");
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
        .map(|_| (
            T::Response::from_rmw_message(response_out),
            service_info_out,
        ))
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
    node: Arc<Node>,
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
    use crate::test_helpers::*;
    use test_msgs::srv;

    #[test]
    fn traits() {
        assert_send::<Client<srv::Arrays>>();
        assert_sync::<Client<srv::Arrays>>();
    }

    #[test]
    fn test_clients() -> Result<(), RclrsError> {
        let namespace = "/test_clients_graph";
        let graph = construct_test_graph(namespace)?;
        let _node_2_empty_client = graph
            .node2
            .create_client::<srv::Empty>("graph_test_topic_4", QoSProfile::services_default())?;

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
