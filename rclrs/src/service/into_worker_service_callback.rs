use rosidl_runtime_rs::Service;

use crate::{
    AnyServiceCallback, WorkerServiceCallback, RequestId, ServiceInfo,
    Worker,
};

/// A trait used to deduce callbacks for services that run on a worker.
///
/// Users of rclrs never need to use this trait directly.
///
/// Worker service callbacks support six signatures:
/// - [`FnMut`] ( `Request` ) -> `Response`
/// - [`FnMut`] ( `Request`, [`RequestId`] ) -> `Response`
/// - [`FnMut`] ( `Request`, [`ServiceInfo`] ) -> `Response`
/// - [`FnMut`] ( `&mut Payload`, `Request` ) -> `Response`
/// - [`FnMut`] ( `&mut Payload`, `Request`,  [`RequestId`] ) -> `Response`
/// - [`FnMut`] ( `&mut Payload`, `Request`, [`ServiceInfo`] ) -> `Response`
pub trait IntoWorkerServiceCallback<T, Payload, Args>: Send + 'static
where
    T: Service,
    Payload: 'static + Send,
{
    /// Converts the callback into an enum
    ///
    /// User code never needs to call this function.
    fn into_worker_service_callback(self) -> AnyServiceCallback<T, Payload>;
}

impl<T, Payload, Func> IntoWorkerServiceCallback<T, Payload, ()> for Func
where
    T: Service,
    Payload: 'static + Send,
    Func: FnMut(T::Request) -> T::Response + Send + 'static,
{
    fn into_worker_service_callback(mut self) -> AnyServiceCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, request| self(request));
        WorkerServiceCallback::OnlyRequest(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerServiceCallback<T, Payload, Worker<Payload>> for Func
where
    T: Service,
    Payload: 'static + Send,
    Func: FnMut(&mut Payload, T::Request) -> T::Response + Send + 'static,
{
    fn into_worker_service_callback(self) -> AnyServiceCallback<T, Payload> {
        WorkerServiceCallback::OnlyRequest(Box::new(self)).into()
    }
}

impl<T, Payload, Func> IntoWorkerServiceCallback<T, Payload, RequestId> for Func
where
    T: Service,
    Payload: 'static + Send,
    Func: FnMut(T::Request, RequestId) -> T::Response + Send + 'static,
{
    fn into_worker_service_callback(mut self) -> AnyServiceCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, request, request_id| {
            self(request, request_id)
        });
        WorkerServiceCallback::WithId(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerServiceCallback<T, Payload, (Worker<Payload>, RequestId)> for Func
where
    T: Service,
    Payload: 'static + Send,
    Func: FnMut(&mut Payload, T::Request, RequestId) -> T::Response + Send + 'static,
{
    fn into_worker_service_callback(self) -> AnyServiceCallback<T, Payload> {
        WorkerServiceCallback::WithId(Box::new(self)).into()
    }
}

impl<T, Payload, Func> IntoWorkerServiceCallback<T, Payload, ServiceInfo> for Func
where
    T: Service,
    Payload: 'static + Send,
    Func: FnMut(T::Request, ServiceInfo) -> T::Response + Send + 'static,
{
    fn into_worker_service_callback(mut self) -> AnyServiceCallback<T, Payload> {
        let f = Box::new(move |_: &mut Payload, request, info| {
            self(request, info)
        });
        WorkerServiceCallback::WithInfo(f).into()
    }
}

impl<T, Payload, Func> IntoWorkerServiceCallback<T, Payload, (Worker<T>, ServiceInfo)> for Func
where
    T: Service,
    Payload: 'static + Send,
    Func: FnMut(&mut Payload, T::Request, ServiceInfo) -> T::Response + Send + 'static,
{
    fn into_worker_service_callback(self) -> AnyServiceCallback<T, Payload> {
        WorkerServiceCallback::WithInfo(Box::new(self)).into()
    }
}
