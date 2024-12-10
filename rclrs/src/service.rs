use std::{
    boxed::Box,
    ffi::{CStr, CString},
    sync::{Arc, Mutex, MutexGuard},
};

use crate::{
    error::ToResult, rcl_bindings::*, ExecutorCommands, IntoPrimitiveOptions, NodeHandle,
    QoSProfile, RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, RclrsError, Waitable,
    WaitableLifecycle, ENTITY_LIFECYCLE_MUTEX,
};

mod any_service_callback;
pub use any_service_callback::*;

mod service_async_callback;
pub use service_async_callback::*;

mod service_callback;
pub use service_callback::*;

mod service_info;
pub use service_info::*;

/// Provide a service that can respond to requests sent by ROS service clients.
///
/// Create a service using [`Node::create_service`][1]
/// or [`Node::create_async_service`][2].
///
/// ROS only supports having one service provider for any given fully-qualified
/// service name. "Fully-qualified" means the namespace is also taken into account
/// for uniqueness. A clone of a `Service` will refer to the same service provider
/// instance as the original. The underlying instance is tied to [`ServiceState`]
/// which implements the [`Service`] API.
///
/// Responding to requests requires the node's executor to [spin][3].
///
/// [1]: crate::Node::create_service
/// [2]: crate::Node::create_async_service
/// [3]: crate::Executor::spin
///
pub type Service<T> = Arc<ServiceState<T>>;

/// The inner state of a [`Service`].
///
/// This is public so that you can choose to create a [`Weak`][1] reference to it
/// if you want to be able to refer to a [`Service`] in a non-owning way. It is
/// generally recommended to manage the `ServiceState` inside of an [`Arc`],
/// and [`Service`] is provided as a convenience alias for that.
///
/// The public API of the [`Service`] type is implemented via `ServiceState`.
///
/// [1]: std::sync::Weak
pub struct ServiceState<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// This handle is used to access the data that rcl holds for this service.
    handle: Arc<ServiceHandle>,
    /// This is the callback that will be executed each time a request arrives.
    callback: Arc<Mutex<AnyServiceCallback<T>>>,
    /// Holding onto this keeps the waiter for this service alive in the wait
    /// set of the executor.
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
}

impl<T> ServiceState<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Returns the name of the service.
    ///
    /// This returns the service name after remapping, so it is not necessarily the
    /// service name which was used when creating the service.
    pub fn service_name(&self) -> String {
        // SAFETY: The service handle is valid because its lifecycle is managed by an Arc.
        // The unsafe variables get converted to safe types before being returned
        unsafe {
            let raw_service_pointer = rcl_service_get_service_name(&*self.handle.lock());
            CStr::from_ptr(raw_service_pointer)
        }
        .to_string_lossy()
        .into_owned()
    }

    /// Set the callback of this service, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the service previously used an async callback.
    pub fn set_callback<Args>(&self, callback: impl ServiceCallback<T, Args>) {
        let callback = callback.into_service_callback();
        // TODO(@mxgrey): Log any errors here when logging becomes available
        *self.callback.lock().unwrap() = callback;
    }

    /// Set the callback of this service, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the service previously used a non-async callback.
    pub fn set_async_callback<Args>(&self, callback: impl ServiceAsyncCallback<T, Args>) {
        let callback = callback.into_service_async_callback();
        *self.callback.lock().unwrap() = callback;
    }

    /// Used by [`Node`][crate::Node] to create a new service
    pub(crate) fn create<'a>(
        options: impl Into<ServiceOptions<'a>>,
        callback: AnyServiceCallback<T>,
        node_handle: &Arc<NodeHandle>,
        commands: &Arc<ExecutorCommands>,
    ) -> Result<Arc<Self>, RclrsError> {
        let ServiceOptions { name, qos } = options.into();
        let callback = Arc::new(Mutex::new(callback));
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_service = unsafe { rcl_get_zero_initialized_service() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(name).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: name.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let mut service_options = unsafe { rcl_service_get_default_options() };
        service_options.qos = qos.into();

        {
            let rcl_node = node_handle.rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            unsafe {
                // SAFETY:
                // * The rcl_service is zero-initialized as mandated by this function.
                // * The rcl_node is kept alive by the NodeHandle it is a dependency of the service.
                // * The topic name and the options are copied by this function, so they can be dropped
                //   afterwards.
                // * The entity lifecycle mutex is locked to protect against the risk of global
                //   variables in the rmw implementation being unsafely modified during initialization.
                rcl_service_init(
                    &mut rcl_service,
                    &*rcl_node,
                    type_support,
                    topic_c_string.as_ptr(),
                    &service_options as *const _,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(ServiceHandle {
            rcl_service: Mutex::new(rcl_service),
            node_handle: Arc::clone(&node_handle),
        });

        let (waitable, lifecycle) = Waitable::new(
            Box::new(ServiceExecutable {
                handle: Arc::clone(&handle),
                callback: Arc::clone(&callback),
                commands: Arc::clone(&commands),
            }),
            Some(Arc::clone(commands.get_guard_condition())),
        );

        let service = Arc::new(Self {
            handle,
            callback,
            lifecycle,
        });
        commands.add_to_wait_set(waitable);

        Ok(service)
    }
}

/// `ServiceOptions are used by [`Node::create_service`][1] to initialize a
/// [`Service`] provider.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct ServiceOptions<'a> {
    /// The name for the service
    pub name: &'a str,
    /// The quality of service profile for the service.
    pub qos: QoSProfile,
}

impl<'a> ServiceOptions<'a> {
    /// Initialize a new [`ServiceOptions`] with default settings.
    pub fn new(name: &'a str) -> Self {
        Self {
            name,
            qos: QoSProfile::services_default(),
        }
    }
}

impl<'a, T: IntoPrimitiveOptions<'a>> From<T> for ServiceOptions<'a> {
    fn from(value: T) -> Self {
        let primitive = value.into_primitive_options();
        let mut options = Self::new(primitive.name);
        primitive.apply(&mut options.qos);
        options
    }
}

struct ServiceExecutable<T: rosidl_runtime_rs::Service> {
    handle: Arc<ServiceHandle>,
    callback: Arc<Mutex<AnyServiceCallback<T>>>,
    commands: Arc<ExecutorCommands>,
}

impl<T> RclPrimitive for ServiceExecutable<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn execute(&mut self) -> Result<(), RclrsError> {
        if let Err(err) = self
            .callback
            .lock()
            .unwrap()
            .execute(&self.handle, &self.commands)
        {
            // TODO(@mxgrey): Log the error here once logging is implemented
            eprintln!("Error while executing a service callback: {err}");
        }
        Ok(())
    }

    fn kind(&self) -> crate::RclPrimitiveKind {
        RclPrimitiveKind::Service
    }

    fn handle(&self) -> RclPrimitiveHandle {
        RclPrimitiveHandle::Service(self.handle.lock())
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_service_t {}

/// Manage the lifecycle of an `rcl_service_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_service_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct ServiceHandle {
    rcl_service: Mutex<rcl_service_t>,
    node_handle: Arc<NodeHandle>,
}

impl ServiceHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_service_t> {
        self.rcl_service.lock().unwrap()
    }
}

impl Drop for ServiceHandle {
    fn drop(&mut self) {
        let rcl_service = self.rcl_service.get_mut().unwrap();
        let mut rcl_node = self.node_handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_service_fini(rcl_service, &mut *rcl_node);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn traits() {
        assert_send::<Service<test_msgs::srv::Arrays>>();
        assert_sync::<Service<test_msgs::srv::Arrays>>();
    }

    #[test]
    fn test_services() -> Result<(), RclrsError> {
        use crate::TopicNamesAndTypes;
        use test_msgs::srv;

        let namespace = "/test_services_graph";
        let graph = construct_test_graph(namespace)?;
        let check_names_and_types = |names_and_types: TopicNamesAndTypes| {
            let types = names_and_types
                .get("/test_services_graph/graph_test_topic_4")
                .unwrap();
            assert!(types.contains(&"test_msgs/srv/Empty".to_string()));
        };

        let _node_1_empty_service = graph.node1.create_service::<srv::Empty, _>(
            "graph_test_topic_4",
            |_: srv::Empty_Request| srv::Empty_Response {
                structure_needs_at_least_one_member: 0,
            },
        )?;
        let _node_2_empty_client = graph
            .node2
            .create_client::<srv::Empty>("graph_test_topic_4")?;

        std::thread::sleep(std::time::Duration::from_millis(100));

        let service_names_and_types = graph.node1.get_service_names_and_types()?;
        check_names_and_types(service_names_and_types);

        let service_names_and_types = graph
            .node1
            .get_service_names_and_types_by_node(&graph.node1.name(), namespace)?;
        check_names_and_types(service_names_and_types);

        Ok(())
    }
}
