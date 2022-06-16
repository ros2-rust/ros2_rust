use crate::node::client::oneshot::Canceled;
use futures::channel::oneshot;
use std::borrow::Borrow;
use std::boxed::Box;
use std::collections::HashMap;
use std::ffi::CString;
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::Arc;

use crate::error::{RclReturnCode, ToResult};
use crate::MessageCow;
use crate::Node;
use crate::{rcl_bindings::*, RclrsError};

use parking_lot::{Mutex, MutexGuard};
use rosidl_runtime_rs::Message;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_client_t {}

/// Internal struct used by clients.
pub struct ClientHandle {
    rcl_client_mtx: Mutex<rcl_client_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
}

impl ClientHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_client_t> {
        self.rcl_client_mtx.lock()
    }
}

impl Drop for ClientHandle {
    fn drop(&mut self) {
        let handle = self.rcl_client_mtx.get_mut();
        let rcl_node_mtx = &mut *self.rcl_node_mtx.lock();
        unsafe {
            rcl_client_fini(handle as *mut _, rcl_node_mtx as *mut _);
        }
    }
}

impl From<Canceled> for RclrsError {
    fn from(_: Canceled) -> Self {
        RclrsError::RclError {
            code: RclReturnCode::Error,
            msg: None,
        }
    }
}

/// Trait to be implemented by concrete Client structs
/// See [`Client<T>`] for an example
pub trait ClientBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ClientHandle;
    /// Tries to take a new response and run the callback or future with it.
    fn execute(&self) -> Result<(), RclrsError>;
}

type RequestValue<Response> = Box<dyn FnOnce(&Response) + 'static + Send>;

/// Main class responsible for publishing data to ROS topics
pub struct Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    pub(crate) handle: Arc<ClientHandle>,
    requests: Mutex<HashMap<i64, RequestValue<T::Response>>>,
    futures: Arc<Mutex<HashMap<i64, oneshot::Sender<T::Response>>>>,
    sequence_number: AtomicI64,
}

impl<T> Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Creates a new client.
    pub fn new(node: &Node, topic: &str) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
    {
        let mut rcl_client = unsafe { rcl_get_zero_initialized_client() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let rcl_node = &mut *node.rcl_node_mtx.lock();

        unsafe {
            let client_options = rcl_client_get_default_options();

            rcl_client_init(
                &mut rcl_client,
                rcl_node,
                type_support,
                topic_c_string.as_ptr(),
                &client_options,
            )
            .ok()?;
        }

        let handle = Arc::new(ClientHandle {
            rcl_client_mtx: Mutex::new(rcl_client),
            rcl_node_mtx: node.rcl_node_mtx.clone(),
        });

        Ok(Self {
            handle,
            requests: Mutex::new(HashMap::new()),
            futures: Arc::new(Mutex::new(
                HashMap::<i64, oneshot::Sender<T::Response>>::new(),
            )),
            sequence_number: AtomicI64::new(0),
        })
    }

    /// Send a requests with a callback as a parameter.
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
        F: FnOnce(&T::Response) + 'static + Send,
    {
        let rmw_message = T::Request::into_rmw_message(message.into_cow());
        let mut sequence_number = self.sequence_number.load(Ordering::SeqCst);
        let ret = unsafe {
            rcl_send_request(
                &*self.handle.lock() as *const _,
                rmw_message.as_ref() as *const <T::Request as Message>::RmwMsg as *mut _,
                &mut sequence_number,
            )
        };
        let requests = &mut *self.requests.lock();
        requests.insert(sequence_number, Box::new(callback));
        self.sequence_number.swap(sequence_number, Ordering::SeqCst);
        ret.ok()
    }

    /// Send a request with a callback as a parameter.
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
        T: rosidl_runtime_rs::Service + 'static,
    {
        let rmw_message = T::Request::into_rmw_message(request.into_cow());
        let mut sequence_number = self.sequence_number.load(Ordering::SeqCst);
        let ret = unsafe {
            rcl_send_request(
                &*self.handle.lock() as *const _,
                rmw_message.as_ref() as *const <T::Request as Message>::RmwMsg as *mut _,
                &mut sequence_number,
            )
        };
        debug_assert_eq!(ret, 0);
        let (tx, rx) = oneshot::channel::<T::Response>();
        self.futures.lock().insert(sequence_number, tx);
        Ok(rx.await?)
    }

    /// Ask RMW for the data
    ///
    /// +----------------------+
    /// | rclrs::take_response |
    /// +----------+-----------+
    ///            |
    ///            |
    /// +----------v-----------+
    /// |   rcl_take_response  |
    /// +----------+-----------+
    ///            |
    ///            |
    /// +----------v----------+
    /// |      rmw_take       |
    /// +---------------------+
    pub fn take_response(&self) -> Result<(T::Response, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            sequence_number: 0,
        };
        type RmwMsg<T> =
            <<T as rosidl_runtime_rs::Service>::Response as rosidl_runtime_rs::Message>::RmwMsg;
        let mut response_out = RmwMsg::<T>::default();
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_take_response(
                handle as *const _,
                &mut request_id_out,
                &mut response_out as *mut RmwMsg<T> as *mut _,
            )
        };
        ret.ok()?;
        Ok((T::Response::from_rmw_message(response_out), request_id_out))
    }
}

impl<T> ClientBase for Client<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn handle(&self) -> &ClientHandle {
        self.handle.borrow()
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let (res, req_id) = match self.take_response() {
            Ok((res, req_id)) => (res, req_id),
            Err(RclrsError::RclError {
                code: RclReturnCode::ClientTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // subscription was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let requests = &mut *self.requests.lock();
        let futures = &mut *self.futures.lock();
        if requests.contains_key(&req_id.sequence_number) {
            requests.remove(&req_id.sequence_number).unwrap()(&res);
        } else if futures.contains_key(&req_id.sequence_number) {
            futures
                .remove(&req_id.sequence_number)
                .unwrap()
                .send(res)
                .unwrap_or_else(|_| panic!("fail to send response via channel in Client::execute"));
        }
        Ok(())
    }
}