use rosidl_runtime_rs::{Service, Message};

use crate::{
    error::ToResult,
    rcl_bindings::{
        rmw_request_id_t, rmw_service_info_t, rcl_take_request, rcl_take_request_with_info,
        rcl_send_response,
    },
    RequestId, ServiceInfo, ServiceHandle, ExecutorCommands,
    RclrsError, RclReturnCode, MessageCow,
};

use futures::future::BoxFuture;

use std::sync::Arc;

/// An enum capturing the various possible function signatures for service callbacks.
pub enum AnyServiceCallback<T>
where
    T: Service,
{
    /// A callback that only takes in the request value
    OnlyRequest(Box<dyn FnMut(T::Request) -> BoxFuture<'static, T::Response> + Send>),
    /// A callback that takes in the request value and the ID of the request
    WithId(Box<dyn FnMut(T::Request, RequestId) -> BoxFuture<'static, T::Response> + Send>),
    /// A callback that takes in the request value and all available
    WithInfo(Box<dyn FnMut(T::Request, ServiceInfo) -> BoxFuture<'static, T::Response> + Send>),
}

impl<T: Service> AnyServiceCallback<T> {
    pub(super) fn execute(
        &mut self,
        handle: &Arc<ServiceHandle>,
        commands: &Arc<ExecutorCommands>,
    ) -> Result<(), RclrsError> {
        let mut evaluate = || {
            match self {
                AnyServiceCallback::OnlyRequest(cb) => {
                    let (msg, mut rmw_request_id) = Self::take_request(handle)?;
                    let handle = Arc::clone(&handle);
                    let response = cb(msg);
                    commands.run_detached(async move {
                        // TODO(@mxgrey): Log any errors here when logging is available
                        Self::send_response(&handle, &mut rmw_request_id, response.await).ok();
                    });
                }
                AnyServiceCallback::WithId(cb) => {
                    let (msg, mut rmw_request_id) = Self::take_request(handle)?;
                    let request_id = RequestId::from_rmw_request_id(&rmw_request_id);
                    let handle = Arc::clone(&handle);
                    let response = cb(msg, request_id);
                    commands.run_detached(async move {
                        // TODO(@mxgrey): Log any errors here when logging is available
                        Self::send_response(&handle, &mut rmw_request_id, response.await).ok();
                    });
                }
                AnyServiceCallback::WithInfo(cb) => {
                    let (msg, rmw_service_info) = Self::take_request_with_info(handle)?;
                    let mut rmw_request_id = rmw_request_id_t {
                        writer_guid: rmw_service_info.request_id.writer_guid,
                        sequence_number: rmw_service_info.request_id.sequence_number,
                    };
                    let service_info = ServiceInfo::from_rmw_service_info(&rmw_service_info);
                    let handle = Arc::clone(&handle);
                    let response = cb(msg, service_info);
                    commands.run_detached(async move {
                        // TODO(@mxgrey): Log any errors here when logging is available
                        Self::send_response(&handle, &mut rmw_request_id, response.await).ok();
                    });
                }
            }

            Ok(())
        };

        match evaluate() {
            Err(RclrsError::RclError {
                code: RclReturnCode::ServiceTakeFailed,
                ..
            }) => {
                // Spurious wakeup - this may happen even when a waitlist indicated that this
                // subscription was ready, so it shouldn't be an error.
                Ok(())
            }
            other => other,
        }
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
    fn take_request(handle: &ServiceHandle) -> Result<(T::Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = RequestId::zero_initialized_rmw();
        type RmwMsg<T> = <<T as Service>::Request as Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*handle.lock();
        unsafe {
            // SAFETY: The three pointers are valid and initialized
            rcl_take_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            )
        }
        .ok()?;
        Ok((T::Request::from_rmw_message(request_out), request_id_out))
    }

    /// Same as [`Self::take_request`] but includes additional info about the service
    fn take_request_with_info(handle: &ServiceHandle) -> Result<(T::Request, rmw_service_info_t), RclrsError> {
        let mut service_info_out = ServiceInfo::zero_initialized_rmw();
        type RmwMsg<T> = <<T as Service>::Request as Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*handle.lock();
        unsafe {
            // SAFETY: The three pointers are valid and initialized
            rcl_take_request_with_info(
                handle,
                &mut service_info_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            )
        }
        .ok()?;
        Ok((T::Request::from_rmw_message(request_out), service_info_out))
    }

    fn send_response(
        handle: &Arc<ServiceHandle>,
        request_id: &mut rmw_request_id_t,
        response: T::Response,
    ) -> Result<(), RclrsError> {
        let rmw_message = <T::Response as Message>::into_rmw_message(response.into_cow());
        let handle = &*handle.lock();
        unsafe {
            rcl_send_response(
                handle,
                request_id,
                rmw_message.as_ref() as *const <T::Response as Message>::RmwMsg as *mut _,
            )
        }
        .ok()
    }
}
