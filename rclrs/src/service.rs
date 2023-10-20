use std::boxed::Box;
use std::ffi::CString;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Weak, Mutex, MutexGuard};

use rosidl_runtime_rs::Message;

use crate::error::{RclReturnCode, ToResult};
use crate::{rcl_bindings::*, MessageCow, RclrsError};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_service_t {}

/// Internal struct used by services.
pub struct ServiceHandle {
    rcl_service_mtx: Mutex<rcl_service_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ServiceHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_service_t> {
        self.rcl_service_mtx.lock().unwrap()
    }
}

impl Drop for ServiceHandle {
    fn drop(&mut self) {
        let rcl_service = self.rcl_service_mtx.get_mut().unwrap();
        let rcl_node = &mut *self.rcl_node_mtx.lock().unwrap();
        // SAFETY: No preconditions for this function
        unsafe {
            rcl_service_fini(rcl_service, rcl_node);
        }
    }
}

/// Trait to be implemented by concrete Service structs.
///
/// See [`Service<T>`] for an example
pub trait ServiceBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ServiceHandle;
    /// Tries to take a new request and run the callback with it.
    fn execute(&self) -> Result<(), RclrsError>;
}

/// Main class responsible for responding to requests sent by ROS clients.
///
/// The only available way to instantiate services is via [`Node::create_service()`][1]
/// or [`Node::create_deferred_service()`][2], this is to ensure that [`Node`][3]s
/// can track all the services that have been created.
///
/// [1]: crate::Node::create_service
/// [2]: crate::Node::create_deferred_service
/// [3]: crate::Node
pub struct Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    pub(crate) handle: Arc<ServiceHandle>,
    /// The user-defined responder for this service
    pub responder: Mutex<Responder<T>>,
}

/// Defines how a service responds to requests.
pub enum Responder<T: rosidl_runtime_rs::Service> {
    /// The responder will immediately return a response
    Immediate(ImmediateResponder<T>),
    /// The response may be deferred until later
    Deferred(DeferredResponder<T>),
}

/// A responder that blocks execution until it has provided a response.
pub struct ImmediateResponder<T: rosidl_runtime_rs::Service> {
    /// The callback function that runs when a request was received.
    pub callback: ImmediateServiceCallback<T::Request, T::Response>,
}

/// Storage for the callback used by an [`ImmediateResponder`].
type ImmediateServiceCallback<Request, Response> =
    Box<dyn Fn(&rmw_request_id_t, Request) -> Response + 'static + Send>;

/// A responder that allows a response to be provided later.
pub struct DeferredResponder<T: rosidl_runtime_rs::Service> {
    /// The callback that will be triggered to handle incoming requests.
    pub callback: DeferredServiceCallback<T::Request, T::Response>,
    /// The default callback that will be triggered if a [`ResponseSender`] is
    /// dropped without sending any response. This can be overriden using
    /// [`ResponseSender::on_drop()`].
    pub default_on_drop: Option<DeferredServiceDrop<T::Response>>,
}

/// Used by deferred services to send a response when it is ready.
pub struct ResponseSender<Response: rosidl_runtime_rs::Message> {
    request_id: rmw_request_id_s,
    handle: Weak<ServiceHandle>,
    on_drop: Option<DeferredServiceDrop<Response>>,
    sent: bool,
}

impl<Response: rosidl_runtime_rs::Message> ResponseSender<Response> {
    /// Send a response for an earlier request.
    pub fn send(mut self, response: Response) -> Result<(), ResponseSendError> {
        self.sent = true;
        let Some(handle) = self.handle.upgrade() else {
            return Err(ResponseSendError::ServiceDropped);
        };
        send_response(&mut self.request_id, response, &handle)
            .map_err(ResponseSendError::Rclrs)
    }

    /// Set a callback that will be triggered if this [`ResponseSender`] gets
    /// dropped without sending a response. If you do not set this, then the
    /// service's default on-drop behavior will be used.
    ///
    /// The provided callback can choose to return `Some(response)` to provide
    /// a fallback response for the service. If the callback returns `None`,
    /// then no response will ever be delivered for the request.
    pub fn on_drop<F>(&mut self, f: F)
    where
        F: Fn(&rmw_request_id_s) -> Option<Response> + 'static + Send + Sync,
    {
        self.on_drop = Some(Arc::new(f));
    }

    /// Do nothing if this [`ResponseSender`] gets dropped without sending a
    /// response. This will clear out the service's default on-drop behavior.
    pub fn ignore_drop(&mut self) {
        self.on_drop = None;
    }

    /// Get information about the request that this [`ResponseSender`] is
    /// associated with.
    pub fn request_id(&self) -> &rmw_request_id_s {
        &self.request_id
    }

    /// Check if the service that this [`ResponseSender`] is associated with is
    /// still alive (has not been dropped). If the service is not alive anymore,
    /// then the response can never be sent.
    pub fn is_service_alive(&self) -> bool {
        self.handle.strong_count() > 0
    }
}

/// An error that can happen when sending a response for a deferred service.
#[derive(Debug, PartialEq, Eq)]
pub enum ResponseSendError {
    /// The service associated with this request has been dropped, so the
    /// response can no longer be sent.
    ServiceDropped,
    ///
    Rclrs(RclrsError),
}

/// Storage for the kind of callback used by deferred services.
type DeferredServiceCallback<Request, Response> =
    Box<dyn Fn(Request, ResponseSender<Response>) + 'static + Send>;

/// Storage for the kind of callback triggered when a [`ResponseSender`] gets dropped.
type DeferredServiceDrop<Response> =
    Arc<dyn Fn(&rmw_request_id_s) -> Option<Response> + 'static + Send + Sync>;

impl<Response: rosidl_runtime_rs::Message> Drop for ResponseSender<Response> {
    fn drop(&mut self) {
        if self.sent {
            return;
        }
        let Some(handle) = self.handle.upgrade() else {
            return;
        };
        if let Some(on_drop) = &self.on_drop {
            if let Some(response) = on_drop(&self.request_id) {
                send_response(&mut self.request_id, response, &handle).ok();
            }
        }
    }
}

impl<T> Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Creates a new service. Consider using [`Service::immediate()`] or
    /// [`Service::deferred()`] instead.
    pub(crate) fn new(
        rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
        topic: &str,
        responder: Responder<T>,
    ) -> Result<Self, RclrsError>
    // This uses pub(crate) visibility to avoid instantiating this struct outside
    // [`Node::create_service`], see the struct's documentation for the rationale
    where
        T: rosidl_runtime_rs::Service,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_service = unsafe { rcl_get_zero_initialized_service() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let service_options = unsafe { rcl_service_get_default_options() };

        unsafe {
            // SAFETY: The rcl_service is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the service.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            rcl_service_init(
                &mut rcl_service,
                &*rcl_node_mtx.lock().unwrap(),
                type_support,
                topic_c_string.as_ptr(),
                &service_options as *const _,
            )
            .ok()?;
        }

        let handle = Arc::new(ServiceHandle {
            rcl_service_mtx: Mutex::new(rcl_service),
            rcl_node_mtx,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self { handle, responder: Mutex::new(responder) })
    }

    /// Create an immediate responder service.
    pub(crate) fn immediate<F>(
        rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
        topic: &str,
        f: F,
    ) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
        F: Fn(&rmw_request_id_t, T::Request) -> T::Response + 'static + Send,
    {
        let responder = Responder::Immediate(ImmediateResponder {
            callback: Box::new(f),
        });
        Self::new(rcl_node_mtx, topic, responder)
    }

    /// Create a deferred responder service.
    pub(crate) fn deferred<F>(
        rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
        topic: &str,
        f: F,
    ) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
        F: Fn(T::Request, ResponseSender<T::Response>) + 'static + Send,
    {
        let responder = Responder::Deferred(DeferredResponder {
            callback: Box::new(f),
            default_on_drop: None
        });
        Self::new(rcl_node_mtx, topic, responder)
    }

    /// Fetches a new request.
    ///
    /// When there is no new message, this will return a
    /// [`ServiceTakeFailed`][1].
    ///
    /// [1]: crate::RclrsError
    //
    // ```text
    // +---------------------+
    // | rclrs::take_request |
    // +----------+----------+
    //            |
    //            |
    // +----------v----------+
    // |  rcl_take_request   |
    // +----------+----------+
    //            |
    //            |
    // +----------v----------+
    // |      rmw_take       |
    // +---------------------+
    // ```
    pub fn take_request(&self) -> Result<(T::Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            sequence_number: 0,
        };
        type RmwMsg<T> =
            <<T as rosidl_runtime_rs::Service>::Request as rosidl_runtime_rs::Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The three pointers are valid/initialized
            rcl_take_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            )
        }
        .ok()?;
        Ok((T::Request::from_rmw_message(request_out), request_id_out))
    }
}

impl<T> ServiceBase for Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn handle(&self) -> &ServiceHandle {
        &self.handle
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let (req, mut req_id) = match self.take_request() {
            Ok((req, req_id)) => (req, req_id),
            Err(RclrsError::RclError {
                code: RclReturnCode::ServiceTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // service was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let responder = self.responder.lock().unwrap();
        match &*responder {
            Responder::Immediate(immediate) => {
                let res = (*immediate.callback)(&req_id, req);
                send_response(&mut req_id, res, &self.handle)
            }
            Responder::Deferred(deferred) => {
                let sender = ResponseSender {
                    request_id: req_id,
                    handle: Arc::downgrade(&self.handle),
                    on_drop: deferred.default_on_drop.clone(),
                    sent: false,
                };
                (*deferred.callback)(req, sender);
                Ok(())
            }
        }
    }
}

fn send_response<Response: rosidl_runtime_rs::Message>(
    request_id: &mut rmw_request_id_s,
    response: Response,
    handle: &ServiceHandle,
) -> Result<(), RclrsError> {
    let rmw_message = Response::into_rmw_message(response.into_cow());
    let raw_handle = &*handle.lock();
    unsafe {
        // SAFETY: The response type is guaranteed to match the service type by the type system.
        rcl_send_response(
            raw_handle,
            request_id,
            rmw_message.as_ref() as *const Response::RmwMsg as *mut _,
        )
    }
    .ok()
}
