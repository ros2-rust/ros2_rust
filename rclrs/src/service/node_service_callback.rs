use rosidl_runtime_rs::Service;

use crate::{
    log_error, rcl_bindings::rmw_request_id_t, RclrsError, RclrsErrorFilter, RequestId,
    ServiceHandle, ServiceInfo, WorkerCommands,
};

use futures::future::BoxFuture;

use std::sync::Arc;

/// An enum capturing the various possible function signatures for service callbacks.
pub enum NodeServiceCallback<T>
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

impl<T: Service> NodeServiceCallback<T> {
    pub(super) fn execute(
        &mut self,
        handle: Arc<ServiceHandle>,
        commands: &Arc<WorkerCommands>,
    ) -> Result<(), RclrsError> {
        let evaluate = || {
            match self {
                NodeServiceCallback::OnlyRequest(cb) => {
                    let (msg, mut rmw_request_id) = handle.take_request::<T>()?;
                    let response = cb(msg);
                    commands.run_async(async move {
                        if let Err(err) =
                            handle.send_response::<T>(&mut rmw_request_id, response.await)
                        {
                            log_service_send_error(&handle, rmw_request_id, err);
                        }
                    });
                }
                NodeServiceCallback::WithId(cb) => {
                    let (msg, mut rmw_request_id) = handle.take_request::<T>()?;
                    let request_id = RequestId::from_rmw_request_id(&rmw_request_id);
                    let response = cb(msg, request_id);
                    commands.run_async(async move {
                        if let Err(err) =
                            handle.send_response::<T>(&mut rmw_request_id, response.await)
                        {
                            log_service_send_error(&handle, rmw_request_id, err);
                        }
                    });
                }
                NodeServiceCallback::WithInfo(cb) => {
                    let (msg, rmw_service_info) = handle.take_request_with_info::<T>()?;
                    let mut rmw_request_id = rmw_service_info.rmw_request_id();
                    let service_info = ServiceInfo::from_rmw_service_info(&rmw_service_info);
                    let response = cb(msg, service_info);
                    commands.run_async(async move {
                        if let Err(err) =
                            handle.send_response::<T>(&mut rmw_request_id, response.await)
                        {
                            log_service_send_error(&handle, rmw_request_id, err);
                        }
                    });
                }
            }

            Ok(())
        };

        evaluate().take_failed_ok()
    }
}

fn log_service_send_error(
    handle: &ServiceHandle,
    rmw_request_id: rmw_request_id_t,
    err: RclrsError,
) {
    let service_name = handle.service_name();
    log_error!(
        &service_name,
        "Error while sending service response for {rmw_request_id:?}: {err}",
    );
}
