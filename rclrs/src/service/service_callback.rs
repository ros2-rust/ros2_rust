use rosidl_runtime_rs::Service;

use super::{
    ServiceInfo, RequestId,
    any_service_callback::AnyServiceCallback,
};

use std::sync::Arc;

/// A trait for regular callbacks of services.
///
// TODO(@mxgrey): Add a description of what callbacks signatures are supported
pub trait ServiceCallback<T, Args>: Send + 'static
where
    T: Service,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_service_callback(self) -> AnyServiceCallback<T>;
}

impl<T, Func> ServiceCallback<T, ()> for Func
where
    T: Service,
    Func: Fn(T::Request) -> T::Response + Send + Sync + 'static,
{
    fn into_service_callback(self) -> AnyServiceCallback<T> {
        let func = Arc::new(self);
        AnyServiceCallback::OnlyRequest(Box::new(
            move |request| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(request)
                })
            }
        ))
    }
}

impl<T, Func> ServiceCallback<T, RequestId> for Func
where
    T: Service,
    Func: Fn(T::Request, RequestId) -> T::Response + Send + Sync + 'static,
{
    fn into_service_callback(self) -> AnyServiceCallback<T> {
        let func = Arc::new(self);
        AnyServiceCallback::WithId(Box::new(
            move |request, request_id| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(request, request_id)
                })
            }
        ))
    }
}

impl<T, Func> ServiceCallback<T, ServiceInfo> for Func
where
    T: Service,
    Func: Fn(T::Request, ServiceInfo) -> T::Response + Send + Sync + 'static,
{
    fn into_service_callback(self) -> AnyServiceCallback<T> {
        let func = Arc::new(self);
        AnyServiceCallback::WithInfo(Box::new(
            move |request, service_info| {
                let f = Arc::clone(&func);
                Box::pin(async move {
                    f(request, service_info)
                })
            }
        ))
    }
}
