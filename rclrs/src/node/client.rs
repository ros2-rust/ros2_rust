use futures::channel::oneshot;
use std::boxed::Box;
use std::collections::HashMap;
use std::ffi::CString;
use std::sync::Arc;

use crate::rcl_bindings::*;
use crate::{MessageCow, Node, RclReturnCode, RclrsError, ToResult, WaitSet, Waitable};

use parking_lot::Mutex;
use rosidl_runtime_rs::Message;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_client_t {}

type RequestValue<Response> = Box<dyn FnOnce(Response) + 'static + Send>;

type RequestId = i64;

/// Main class responsible for sending requests to a ROS service.
pub struct Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    rcl_client_mtx: Mutex<rcl_client_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    requests: Mutex<HashMap<RequestId, RequestValue<T::Response>>>,
    futures: Arc<Mutex<HashMap<RequestId, oneshot::Sender<T::Response>>>>,
}

impl<T> Drop for Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn drop(&mut self) {
        let rcl_client = self.rcl_client_mtx.get_mut();
        let rcl_node = &mut *self.rcl_node_mtx.lock();
        // SAFETY: No preconditions for this function
        unsafe {
            rcl_client_fini(rcl_client, rcl_node);
        }
    }
}

impl<T> Waitable for Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    unsafe fn add_to_wait_set(self: Arc<Self>, wait_set: &mut WaitSet) -> Result<(), RclrsError> {
        // SAFETY: I'm not sure if it's required, but the client pointer will remain valid
        // for as long as the wait set exists, because it's stored in self.clients.
        // Passing in a null pointer for the third argument is explicitly allowed.
        rcl_wait_set_add_client(
            &mut wait_set.rcl_wait_set,
            &*self.rcl_client_mtx.lock(),
            std::ptr::null_mut(),
        )
        .ok()?;
        wait_set.clients.push(self);
        Ok(())
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let (res, req_id) = match self.take_response() {
            Ok((res, req_id)) => (res, req_id),
            Err(RclrsError::RclError {
                code: RclReturnCode::ClientTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // client was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let requests = &mut *self.requests.lock();
        let futures = &mut *self.futures.lock();
        if let Some(callback) = requests.remove(&req_id.sequence_number) {
            callback(res);
        } else if let Some(future) = futures.remove(&req_id.sequence_number) {
            let _ = future.send(res);
        }
        Ok(())
    }
}

/// A marker trait to distinguish `Client` waitables from other [`Waitable`]s.
pub(crate) trait ClientWaitable: Waitable {}

impl<T> ClientWaitable for Client<T> where T: rosidl_runtime_rs::Service {}

impl<T> Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Creates a new client.
    pub fn new(node: &Node, topic: &str) -> Result<Self, RclrsError>
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
        let rcl_node = { &mut *node.rcl_node_mtx.lock() };

        // SAFETY: No preconditions for this function.
        let client_options = unsafe { rcl_client_get_default_options() };

        unsafe {
            // SAFETY: The rcl_client is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the client.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            rcl_client_init(
                &mut rcl_client,
                rcl_node,
                type_support,
                topic_c_string.as_ptr(),
                &client_options,
            )
            .ok()?;
        }

        Ok(Self {
            rcl_client_mtx: Mutex::new(rcl_client),
            rcl_node_mtx: Arc::clone(&node.rcl_node_mtx),
            requests: Mutex::new(HashMap::new()),
            futures: Arc::new(Mutex::new(
                HashMap::<RequestId, oneshot::Sender<T::Response>>::new(),
            )),
        })
    }

    /// Sends a request with a callback to be called with the response.
    ///
    /// The [`MessageCow`] trait is implemented by any
    /// [`Message`] as well as any reference to a `Message`.
    ///
    /// The reason for allowing owned messages is that publishing owned messages can be more
    /// efficient in the case of idiomatic messages[^note].
    ///
    /// [^note]: See the [`Message`] trait for an explanation of "idiomatic".
    ///
    /// Hence, when a message will not be needed anymore after publishing, pass it by value.
    /// When a message will be needed again after publishing, pass it by reference, instead of cloning and passing by value.
    pub fn async_send_request_with_callback<'a, M: MessageCow<'a, T::Request>, F>(
        &self,
        message: M,
        callback: F,
    ) -> Result<(), RclrsError>
    where
        F: FnOnce(T::Response) + 'static + Send,
    {
        let rmw_message = T::Request::into_rmw_message(message.into_cow());
        let mut sequence_number = -1;
        unsafe {
            // SAFETY: The request type is guaranteed to match the client type by the type system.
            rcl_send_request(
                &*self.rcl_client_mtx.lock() as *const _,
                rmw_message.as_ref() as *const <T::Request as Message>::RmwMsg as *mut _,
                &mut sequence_number,
            )
        }
        .ok()?;
        let requests = &mut *self.requests.lock();
        requests.insert(sequence_number, Box::new(callback));
        Ok(())
    }

    /// Sends a request and returns the response as a `Future`.
    ///
    /// The [`MessageCow`] trait is implemented by any
    /// [`Message`] as well as any reference to a `Message`.
    ///
    /// The reason for allowing owned messages is that publishing owned messages can be more
    /// efficient in the case of idiomatic messages[^note].
    ///
    /// [^note]: See the [`Message`] trait for an explanation of "idiomatic".
    ///
    /// Hence, when a message will not be needed anymore after publishing, pass it by value.
    /// When a message will be needed again after publishing, pass it by reference, instead of cloning and passing by value.
    pub async fn call_async<'a, R: MessageCow<'a, T::Request>>(
        &self,
        request: R,
    ) -> Result<T::Response, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        let rmw_message = T::Request::into_rmw_message(request.into_cow());
        let mut sequence_number = -1;
        unsafe {
            // SAFETY: The request type is guaranteed to match the client type by the type system.
            rcl_send_request(
                &*self.rcl_client_mtx.lock() as *const _,
                rmw_message.as_ref() as *const <T::Request as Message>::RmwMsg as *mut _,
                &mut sequence_number,
            )
        }
        .ok()?;
        let (tx, rx) = oneshot::channel::<T::Response>();
        self.futures.lock().insert(sequence_number, tx);
        // It is safe to call unwrap() here since the `Canceled` error will only happen when the
        // `Sender` is dropped
        // https://docs.rs/futures/latest/futures/channel/oneshot/struct.Canceled.html
        Ok(rx.await.unwrap())
    }

    /// Fetches a new response.
    ///
    /// When there is no new message, this will return a
    /// [`ClientTakeFailed`][1].
    ///
    /// [1]: crate::RclrsError
    //
    // ```text
    // +----------------------+
    // | rclrs::take_response |
    // +----------+-----------+
    //            |
    //            |
    // +----------v-----------+
    // |   rcl_take_response  |
    // +----------+-----------+
    //            |
    //            |
    // +----------v----------+
    // |      rmw_take       |
    // +---------------------+
    // ```
    pub fn take_response(&self) -> Result<(T::Response, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwMsg<T> =
            <<T as rosidl_runtime_rs::Service>::Response as rosidl_runtime_rs::Message>::RmwMsg;
        let mut response_out = RmwMsg::<T>::default();
        let rcl_client = &*self.rcl_client_mtx.lock();
        unsafe {
            // SAFETY: The three pointers are valid/initialized
            rcl_take_response(
                rcl_client,
                &mut request_id_out,
                &mut response_out as *mut RmwMsg<T> as *mut _,
            )
        }
        .ok()?;
        Ok((T::Response::from_rmw_message(response_out), request_id_out))
    }
}
