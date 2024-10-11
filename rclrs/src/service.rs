use std::{
    boxed::Box,
    ffi::{CStr, CString},
    sync::{Arc, Mutex, MutexGuard},
};

use futures::channel::mpsc::{unbounded, UnboundedSender, TrySendError};

use crate::{
    error::ToResult,
    rcl_bindings::*,
    NodeHandle, RclrsError, Waiter, WaiterLifecycle, GuardCondition, QoSProfile,
    Executable, Waitable, WaitableKind, ENTITY_LIFECYCLE_MUTEX, ExecutorCommands,
};

mod any_service_callback;
pub use any_service_callback::*;

mod service_async_callback;
pub use service_async_callback::*;

mod service_callback;
pub use service_callback::*;

mod service_info;
pub use service_info::*;

mod service_task;
use service_task::*;

/// Struct for responding to requests sent by ROS service clients.
///
/// The only way to instantiate a service is via [`Node::create_service()`][1]
/// or [`Node::create_async_service`][2].
///
/// Services should be unique per service name (including namespace) throughout
/// an entire ROS system. ROS does not currently support multiple services that
/// use the same name (including namespace).
///
/// [1]: crate::Node::create_service
/// [2]: crate::Node::create_async_service
pub struct Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// This handle is used to access the data that rcl holds for this service.
    handle: Arc<ServiceHandle>,
    /// This allows us to replace the callback in the service task.
    ///
    /// Holding onto this sender will keep the service task alive. Once this
    /// sender is dropped, the service task will end itself.
    action: UnboundedSender<ServiceAction<T>>,
    /// Holding onto this keeps the waiter for this service alive in the wait
    /// set of the executor.
    lifecycle: WaiterLifecycle,
}

impl<T> Service<T>
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
    pub fn set_callback<Args>(
        &self,
        callback: impl ServiceCallback<T, Args>,
    ) -> Result<(), TrySendError<ServiceAction<T>>> {
        let callback = callback.into_service_callback();
        self.action.unbounded_send(ServiceAction::SetCallback(callback))
    }

    /// Set the callback of this service, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the service previously used a non-async callback.
    pub fn set_async_callback<Args>(
        &self,
        callback: impl ServiceAsyncCallback<T, Args>,
    ) -> Result<(), TrySendError<ServiceAction<T>>> {
        let callback = callback.into_service_async_callback();
        self.action.unbounded_send(ServiceAction::SetCallback(callback))
    }

    /// Used by [`Node`][crate::Node] to create a new service
    pub(crate) fn create(
        topic: &str,
        qos: QoSProfile,
        callback: AnyServiceCallback<T>,
        node_handle: &Arc<NodeHandle>,
        commands: &Arc<ExecutorCommands>,
    ) -> Result<Arc<Self>, RclrsError> {
        let (sender, receiver) = unbounded();
        let (service, waiter) = Self::new(
            topic,
            qos,
            sender,
            Arc::clone(&node_handle),
            Arc::clone(commands.get_guard_condition()),
        )?;

        commands.run_detached(service_task(
            callback,
            receiver,
            Arc::clone(&service.handle),
            Arc::clone(commands),
        ));

        commands.add_to_wait_set(waiter);

        Ok(service)
    }

    /// Instantiate the service.
    fn new(
        topic: &str,
        qos: QoSProfile,
        action: UnboundedSender<ServiceAction<T>>,
        node_handle: Arc<NodeHandle>,
        guard_condition: Arc<GuardCondition>,
    ) -> Result<(Arc<Self>, Waiter), RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_service = unsafe { rcl_get_zero_initialized_service() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
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
            node_handle,
        });

        let (waiter, lifecycle) = Waiter::new(
            Box::new(ServiceWaitable {
                handle: Arc::clone(&handle),
                action: action.clone(),
            }),
            Some(guard_condition),
        );

        let service = Arc::new(Self { handle, action, lifecycle });

        Ok((service, waiter))
    }
}

struct ServiceWaitable<T: rosidl_runtime_rs::Service> {
    handle: Arc<ServiceHandle>,
    action: UnboundedSender<ServiceAction<T>>,
}

impl<T> Executable for ServiceWaitable<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn execute(&mut self) -> Result<(), RclrsError> {
        self.action.unbounded_send(ServiceAction::Execute).ok();
        Ok(())
    }

    fn kind(&self) -> WaitableKind {
        WaitableKind::Service
    }
}

impl<T> Waitable for ServiceWaitable<T>
where
    T: rosidl_runtime_rs::Service,
{
    unsafe fn add_to_wait_set(
        &mut self,
        wait_set: &mut rcl_wait_set_t,
    ) -> Result<usize, RclrsError> {
        let mut index: usize = 0;
        unsafe {
            // SAFETY: We are holding an Arc of the handle, so the service is
            // still valid.
            rcl_wait_set_add_service(
                wait_set,
                &*self.handle.lock(),
                &mut index,
            )
        }
        .ok()?;

        Ok(index)
    }

    unsafe fn is_ready(
        &self,
        wait_set: &rcl_wait_set_t,
        index: usize,
    ) -> bool {
        let entity = unsafe {
            // SAFETY: The `services` field is an array of pointers, and this
            // dereferencing is equivalent to getting the element of the array
            // at `index`.
            *wait_set.services.add(index)
        };

        !entity.is_null()
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

        let _node_1_empty_service =
            graph
                .node1
                .create_service::<srv::Empty, _>(
                    "graph_test_topic_4",
                    QoSProfile::services_default(),
                    |_| {
                        srv::Empty_Response {
                            structure_needs_at_least_one_member: 0,
                        }
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
