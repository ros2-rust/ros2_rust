use std::borrow::Borrow;
use std::boxed::Box;
use std::ffi::CString;
use std::sync::Arc;

use crate::error::{RclReturnCode, ToResult};
use crate::Node;
use crate::{rcl_bindings::*, RclrsError};

use rosidl_runtime_rs::Message;

use crate::node::publisher::MessageCow;

use parking_lot::{Mutex, MutexGuard};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_service_t {}

pub struct ServiceHandle {
    handle: Mutex<rcl_service_t>,
    node_handle: Arc<Mutex<rcl_node_t>>,
}

impl ServiceHandle {
    pub fn lock(&self) -> MutexGuard<rcl_service_t> {
        self.handle.lock()
    }
}

impl Drop for ServiceHandle {
    fn drop(&mut self) {
        let handle = self.handle.get_mut();
        let node_handle = &mut *self.node_handle.lock();
        unsafe {
            rcl_service_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Trait to be implemented by concrete Service structs
/// See [`Service<T>`] for an example
pub trait ServiceBase: Send + Sync {
    fn handle(&self) -> &ServiceHandle;
    fn execute(&self) -> Result<(), RclrsError>;
}

/// Main class responsible for subscribing to topics and receiving data over IPC in ROS
pub struct Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    pub handle: Arc<ServiceHandle>,
    // The callback's lifetime should last as long as we need it to
    pub callback:
        Mutex<Box<dyn FnMut(&rmw_request_id_t, &T::Request, &mut T::Response) + 'static + Send>>,
}

impl<T> Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    pub fn new<F>(node: &Node, topic: &str, callback: F) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Service,
        F: FnMut(&rmw_request_id_t, &T::Request, &mut T::Response) + 'static + Send,
    {
        let mut service_handle = unsafe { rcl_get_zero_initialized_service() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

        unsafe {
            let service_options = rcl_service_get_default_options();

            rcl_service_init(
                &mut service_handle as *mut _,
                node_handle as *mut _,
                type_support,
                topic_c_string.as_ptr(),
                &service_options as *const _,
            )
            .ok()?;
        }

        let handle = Arc::new(ServiceHandle {
            handle: Mutex::new(service_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(Box::new(callback)),
        })
    }

    /// Ask RMW for the data
    ///
    /// +---------------------+
    /// | rclrs::take_request |
    /// +----------+----------+
    ///            |
    ///            |
    /// +----------v----------+
    /// |  rcl_take_request   |
    /// +----------+----------+
    ///            |
    ///            |
    /// +----------v----------+
    /// |      rmw_take       |
    /// +---------------------+
    pub fn take_request(&self) -> Result<(T::Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            sequence_number: 0,
        };
        type RmwMsg<T> =
            <<T as rosidl_runtime_rs::Service>::Request as rosidl_runtime_rs::Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_take_request(
                handle as *const _,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            )
        };
        ret.ok()?;
        Ok((T::Request::from_rmw_message(request_out), request_id_out))
    }
}

impl<T> ServiceBase for Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn handle(&self) -> &ServiceHandle {
        self.handle.borrow()
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let (req, mut req_id) = match self.take_request() {
            Ok((req, req_id)) => (req, req_id),
            Err(RclrsError {
                code: RclReturnCode::ServiceTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // subscription was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let mut res = T::Response::default();
        (*self.callback.lock())(&req_id, &req, &mut res);
        let rmw_message = <T::Response as Message>::into_rmw_message(res.into_cow());
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_send_response(
                handle as *mut _,
                &mut req_id,
                rmw_message.as_ref() as *const <T::Response as Message>::RmwMsg as *mut _,
            )
        };
        ret.ok()
    }
}
