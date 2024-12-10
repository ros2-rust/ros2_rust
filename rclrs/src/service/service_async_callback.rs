use rosidl_runtime_rs::Service;

use super::{any_service_callback::AnyServiceCallback, RequestId, ServiceInfo};

use std::future::Future;

/// A trait for async callbacks of services.
///
// TODO(@mxgrey): Add a description of what callbacks signatures are supported
pub trait ServiceAsyncCallback<T, Args>: Send + 'static
where
    T: Service,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_service_async_callback(self) -> AnyServiceCallback<T>;
}

impl<T, F, Func> ServiceAsyncCallback<T, ()> for Func
where
    T: Service,
    Func: FnMut(T::Request) -> F + Send + 'static,
    F: Future<Output = T::Response> + Send + 'static,
{
    fn into_service_async_callback(mut self) -> AnyServiceCallback<T> {
        AnyServiceCallback::OnlyRequest(Box::new(move |request| Box::pin(self(request))))
    }
}

impl<T, F, Func> ServiceAsyncCallback<T, RequestId> for Func
where
    T: Service,
    Func: FnMut(T::Request, RequestId) -> F + Send + 'static,
    F: Future<Output = T::Response> + Send + 'static,
{
    fn into_service_async_callback(mut self) -> AnyServiceCallback<T> {
        AnyServiceCallback::WithId(Box::new(move |request, request_id| {
            Box::pin(self(request, request_id))
        }))
    }
}

impl<T, F, Func> ServiceAsyncCallback<T, ServiceInfo> for Func
where
    T: Service,
    Func: FnMut(T::Request, ServiceInfo) -> F + Send + 'static,
    F: Future<Output = T::Response> + Send + 'static,
{
    fn into_service_async_callback(mut self) -> AnyServiceCallback<T> {
        AnyServiceCallback::WithInfo(Box::new(move |request, service_info| {
            Box::pin(self(request, service_info))
        }))
    }
}
