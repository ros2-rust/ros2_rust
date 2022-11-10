use std::boxed::Box;
use std::collections::HashMap;
use std::ffi::CString;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex, MutexGuard};

use futures::channel::oneshot;
use rosidl_runtime_rs::Message;

use crate::error::{RclReturnCode, ToResult};
use crate::MessageCow;
use crate::{rcl_bindings::*, RclrsError};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_client_t {}

/// Internal struct used by clients.
pub struct ClientHandle {
    rcl_client_mtx: Mutex<rcl_client_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ClientHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_client_t> {
        self.rcl_client_mtx.lock().unwrap()
    }
}

impl Drop for ClientHandle {
    fn drop(&mut self) {
        let rcl_client = self.rcl_client_mtx.get_mut().unwrap();
        let rcl_node_mtx = &mut *self.rcl_node_mtx.lock().unwrap();
        // SAFETY: No preconditions for this function
        unsafe {
            rcl_client_fini(rcl_client, rcl_node_mtx);
        }
    }
}

/// Trait to be implemented by concrete Client structs.
///
/// See [`Client<T>`] for an example.
pub trait ClientBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ClientHandle;
    /// Tries to take a new response and run the callback or future with it.
    fn execute(&self) -> Result<(), RclrsError>;
}

type RequestValue<Response> = Box<dyn FnOnce(Response) + 'static + Send>;

type RequestId = i64;

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
    pub(crate) handle: Arc<ClientHandle>,
    requests: Mutex<HashMap<RequestId, RequestValue<T::Response>>>,
    futures: Arc<Mutex<HashMap<RequestId, oneshot::Sender<T::Response>>>>,
}

impl<T> Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Creates a new client.
    pub(crate) fn new(rcl_node_mtx: Arc<Mutex<rcl_node_t>>, topic: &str) -> Result<Self, RclrsError>
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
        let client_options = unsafe { rcl_client_get_default_options() };

        unsafe {
            // SAFETY: The rcl_client is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the client.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            rcl_client_init(
                &mut rcl_client,
                &*rcl_node_mtx.lock().unwrap(),
                type_support,
                topic_c_string.as_ptr(),
                &client_options,
            )
            .ok()?;
        }

        let handle = Arc::new(ClientHandle {
            rcl_client_mtx: Mutex::new(rcl_client),
            rcl_node_mtx,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            handle,
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
                &*self.handle.lock() as *const _,
                rmw_message.as_ref() as *const <T::Request as Message>::RmwMsg as *mut _,
                &mut sequence_number,
            )
        }
        .ok()?;
        let requests = &mut *self.requests.lock().unwrap();
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
                &*self.handle.lock() as *const _,
                rmw_message.as_ref() as *const <T::Request as Message>::RmwMsg as *mut _,
                &mut sequence_number,
            )
        }
        .ok()?;
        let (tx, rx) = oneshot::channel::<T::Response>();
        self.futures.lock().unwrap().insert(sequence_number, tx);
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
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The three pointers are valid/initialized
            rcl_take_response(
                handle,
                &mut request_id_out,
                &mut response_out as *mut RmwMsg<T> as *mut _,
            )
        }
        .ok()?;
        Ok((T::Response::from_rmw_message(response_out), request_id_out))
    }
}

impl<T> ClientBase for Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn handle(&self) -> &ClientHandle {
        &self.handle
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
        let requests = &mut *self.requests.lock().unwrap();
        let futures = &mut *self.futures.lock().unwrap();
        if let Some(callback) = requests.remove(&req_id.sequence_number) {
            callback(res);
        } else if let Some(future) = futures.remove(&req_id.sequence_number) {
            let _ = future.send(res);
        }
        Ok(())
    }
}
