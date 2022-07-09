use std::boxed::Box;
use std::ffi::CString;
use std::sync::Arc;

use crate::rcl_bindings::*;
use crate::{Node, RclReturnCode, RclrsError, ToResult, WaitSet, Waitable};

use rosidl_runtime_rs::Message;

use crate::node::publisher::MessageCow;

use parking_lot::Mutex;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_service_t {}

type ServiceCallback<Request, Response> =
    Box<dyn Fn(&rmw_request_id_t, Request) -> Response + 'static + Send>;

/// Main class responsible for responding to requests sent by ROS clients.
pub struct Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    rcl_service_mtx: Mutex<rcl_service_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    /// The callback function that runs when a request was received.
    pub callback: Mutex<ServiceCallback<T::Request, T::Response>>,
}

impl<T> Drop for Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn drop(&mut self) {
        let rcl_service = self.rcl_service_mtx.get_mut();
        let rcl_node = &mut *self.rcl_node_mtx.lock();
        // SAFETY: No preconditions for this function
        unsafe {
            rcl_service_fini(rcl_service, rcl_node);
        }
    }
}

impl<T> Waitable for Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    unsafe fn add_to_wait_set(self: Arc<Self>, wait_set: &mut WaitSet) -> Result<(), RclrsError> {
        // SAFETY: I'm not sure if it's required, but the service pointer will remain valid
        // for as long as the wait set exists, because it's stored in self.clients.
        // Passing in a null pointer for the third argument is explicitly allowed.
        rcl_wait_set_add_service(
            &mut wait_set.rcl_wait_set,
            &*self.rcl_service_mtx.lock(),
            std::ptr::null_mut(),
        )
        .ok()?;
        wait_set.services.push(self);
        Ok(())
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
        let res = (*self.callback.lock())(&req_id, req);
        let rmw_message = <T::Response as Message>::into_rmw_message(res.into_cow());
        let rcl_service = &*self.rcl_service_mtx.lock();
        unsafe {
            // SAFETY: The response type is guaranteed to match the service type by the type system.
            rcl_send_response(
                rcl_service,
                &mut req_id,
                rmw_message.as_ref() as *const <T::Response as Message>::RmwMsg as *mut _,
            )
        }
        .ok()
    }
}

/// A marker trait to distinguish `Service` waitables from other [`Waitable`]s.
pub trait ServiceWaitable: Waitable {}

impl<T> ServiceWaitable for Service<T> where T: rosidl_runtime_rs::Service {}

impl<T> Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Creates a new service.
    pub fn new<F>(node: &Node, topic: &str, callback: F) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
        F: Fn(&rmw_request_id_t, T::Request) -> T::Response + 'static + Send,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut service_handle = unsafe { rcl_get_zero_initialized_service() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;
        let node_handle = &mut *node.rcl_node_mtx.lock();

        // SAFETY: No preconditions for this function.
        let service_options = unsafe { rcl_service_get_default_options() };

        unsafe {
            // SAFETY: The rcl_service is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the service.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            rcl_service_init(
                &mut service_handle as *mut _,
                node_handle as *mut _,
                type_support,
                topic_c_string.as_ptr(),
                &service_options as *const _,
            )
            .ok()?;
        }

        Ok(Self {
            rcl_service_mtx: Mutex::new(service_handle),
            rcl_node_mtx: Arc::clone(&node.rcl_node_mtx),
            callback: Mutex::new(Box::new(callback)),
        })
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
        let rcl_service = &*self.rcl_service_mtx.lock();
        unsafe {
            // SAFETY: The three pointers are valid/initialized
            rcl_take_request(
                rcl_service,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            )
        }
        .ok()?;
        Ok((T::Request::from_rmw_message(request_out), request_id_out))
    }
}
