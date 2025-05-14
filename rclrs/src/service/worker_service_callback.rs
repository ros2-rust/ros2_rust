use rosidl_runtime_rs::Service;

use crate::{RclrsError, RclrsErrorFilter, RequestId, ServiceHandle, ServiceInfo};

use std::{any::Any, sync::Arc};

/// An enum capturing the various possible function signatures for service
/// callbacks that can be used by a [`Worker`][crate::Worker].
///
/// The correct enum variant is deduced by the [`IntoWorkerServiceCallback`][1] trait.
///
/// [1]: crate::IntoWorkerServiceCallback
pub enum WorkerServiceCallback<T, Payload>
where
    T: Service,
    Payload: 'static + Send,
{
    /// A callback that only takes in the request value
    OnlyRequest(Box<dyn FnMut(&mut Payload, T::Request) -> T::Response + Send>),
    /// A callback that takes in the request value and the ID of the request
    WithId(Box<dyn FnMut(&mut Payload, T::Request, RequestId) -> T::Response + Send>),
    /// A callback that takes in the request value and all available
    WithInfo(Box<dyn FnMut(&mut Payload, T::Request, ServiceInfo) -> T::Response + Send>),
}

impl<T, Payload> WorkerServiceCallback<T, Payload>
where
    T: Service,
    Payload: 'static + Send,
{
    pub(super) fn execute(
        &mut self,
        handle: &Arc<ServiceHandle>,
        any_payload: &mut dyn Any,
    ) -> Result<(), RclrsError> {
        let Some(payload) = any_payload.downcast_mut::<Payload>() else {
            return Err(RclrsError::InvalidPayload {
                expected: std::any::TypeId::of::<Payload>(),
                received: (*any_payload).type_id(),
            });
        };

        let mut evaluate = || {
            match self {
                WorkerServiceCallback::OnlyRequest(cb) => {
                    let (msg, mut rmw_request_id) = handle.take_request::<T>()?;
                    let response = cb(payload, msg);
                    handle.send_response::<T>(&mut rmw_request_id, response)?;
                }
                WorkerServiceCallback::WithId(cb) => {
                    let (msg, mut rmw_request_id) = handle.take_request::<T>()?;
                    let request_id = RequestId::from_rmw_request_id(&rmw_request_id);
                    let response = cb(payload, msg, request_id);
                    handle.send_response::<T>(&mut rmw_request_id, response)?;
                }
                WorkerServiceCallback::WithInfo(cb) => {
                    let (msg, rmw_service_info) = handle.take_request_with_info::<T>()?;
                    let mut rmw_request_id = rmw_service_info.rmw_request_id();
                    let service_info = ServiceInfo::from_rmw_service_info(&rmw_service_info);
                    let response = cb(payload, msg, service_info);
                    handle.send_response::<T>(&mut rmw_request_id, response)?;
                }
            }

            Ok(())
        };

        evaluate().take_failed_ok()
    }
}
