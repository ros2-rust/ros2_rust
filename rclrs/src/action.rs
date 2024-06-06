use crate::{error::ToResult, rcl_bindings::*, Node, RclrsError, ENTITY_LIFECYCLE_MUTEX};
use std::{
    ffi::CString,
    sync::{atomic::AtomicBool, Arc, Mutex, MutexGuard},
};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}

unsafe impl Sync for rcl_action_goal_handle_t {}

use std::marker::PhantomData;

pub type GoalUUID = [u8; RCL_ACTION_UUID_SIZE];

pub enum GoalResponse {
    Reject = 1,
    AcceptAndExecute = 2,
    AcceptAndDefer = 3,
}

pub enum CancelResponse {
    Reject = 1,
    Accept = 2,
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_client_t {}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_server_t {}

/// Manage the lifecycle of an `rcl_action_client_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_action_client_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct ActionClientHandle {
    rcl_action_client: Mutex<rcl_action_client_t>,
    node: Node,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ActionClientHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_client_t> {
        self.rcl_action_client.lock().unwrap()
    }
}

impl Drop for ActionClientHandle {
    fn drop(&mut self) {
        let rcl_action_client = self.rcl_action_client.get_mut().unwrap();
        let mut rcl_node = self.node.handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_action_client_fini(rcl_action_client, &mut *rcl_node);
        }
    }
}

/// Manage the lifecycle of an `rcl_action_server_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_action_server_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct ActionServerHandle {
    rcl_action_server: Mutex<rcl_action_server_t>,
    node: Node,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ActionServerHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_server_t> {
        self.rcl_action_server.lock().unwrap()
    }
}

impl Drop for ActionServerHandle {
    fn drop(&mut self) {
        let rcl_action_server = self.rcl_action_server.get_mut().unwrap();
        let mut rcl_node = self.node.handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_action_server_fini(rcl_action_server, &mut *rcl_node);
        }
    }
}

/// Trait to be implemented by concrete ActionClient structs.
///
/// See [`ActionClient<T>`] for an example
pub trait ActionClientBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ActionClientHandle;
    // /// Tries to take a new request and run the callback with it.
    // fn execute(&self) -> Result<(), RclrsError>;
}

/// Trait to be implemented by concrete ActionServer structs.
///
/// See [`ActionServer<T>`] for an example
pub trait ActionServerBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ActionServerHandle;
    // /// Tries to take a new request and run the callback with it.
    // fn execute(&self) -> Result<(), RclrsError>;
}

pub struct ActionClient<T>
where
    T: rosidl_runtime_rs::Action,
{
    _marker: PhantomData<fn() -> T>,
    pub(crate) handle: Arc<ActionClientHandle>,
}

impl<T> ActionClient<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new action client.
    pub(crate) fn new(node: &Node, topic: &str) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_client = unsafe { rcl_action_get_zero_initialized_client() };
        let type_support = <T as rosidl_runtime_rs::Action>::get_type_support()
            as *const rosidl_action_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let action_client_options = unsafe { rcl_action_client_get_default_options() };

        {
            let mut rcl_node = node.handle.rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();

            // SAFETY:
            // * The rcl_client was zero-initialized as expected by this function.
            // * The rcl_node is kept alive by the NodeHandle because it is a dependency of the client.
            // * The topic name and the options are copied by this function, so they can be dropped
            //   afterwards.
            // * The entity lifecycle mutex is locked to protect against the risk of global
            //   variables in the rmw implementation being unsafely modified during initialization.
            unsafe {
                rcl_action_client_init(
                    &mut rcl_action_client,
                    &mut *rcl_node,
                    type_support,
                    topic_c_string.as_ptr(),
                    &action_client_options,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(ActionClientHandle {
            rcl_action_client: Mutex::new(rcl_action_client),
            node: node.clone(),
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            _marker: Default::default(),
            handle,
        })
    }
}

impl<T> ActionClientBase for ActionClient<T>
where
    T: rosidl_runtime_rs::Action,
{
    fn handle(&self) -> &ActionClientHandle {
        &self.handle
    }
}

pub struct ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    _marker: PhantomData<fn() -> T>,
    pub(crate) handle: Arc<ActionServerHandle>,
    // goal_callback: (),
    // cancel_callback: (),
    // accepted_callback: (),
}

impl<T> ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new action server.
    pub(crate) fn new(node: &Node, topic: &str) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_server = unsafe { rcl_action_get_zero_initialized_server() };
        let type_support = <T as rosidl_runtime_rs::Action>::get_type_support()
            as *const rosidl_action_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let action_server_options = unsafe { rcl_action_server_get_default_options() };

        {
            let mut rcl_node = node.handle.rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            unsafe {
                // SAFETY:
                // * The rcl_action_server is zero-initialized as mandated by this function.
                // * The rcl_node is kept alive by the NodeHandle it is a dependency of the action server.
                // * The topic name and the options are copied by this function, so they can be dropped
                //   afterwards.
                // * The entity lifecycle mutex is locked to protect against the risk of global
                //   variables in the rmw implementation being unsafely modified during initialization.
                rcl_action_server_init(
                    &mut rcl_action_server,
                    &mut *rcl_node,
                    todo!("pass in a rcl_clock_t"),
                    type_support,
                    topic_c_string.as_ptr(),
                    &action_server_options,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(ActionServerHandle {
            rcl_action_server: Mutex::new(rcl_action_server),
            node: node.clone(),
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            _marker: Default::default(),
            handle,
        })
    }
}

impl<T> ActionServerBase for ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    fn handle(&self) -> &ActionServerHandle {
        &self.handle
    }
}

pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    rcl_handle: Arc<rcl_action_goal_handle_t>,
    goal_request: Arc<T>,
    _marker: PhantomData<T>,
}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub fn new(rcl_handle: Arc<rcl_action_goal_handle_t>, goal_request: Arc<T>) -> Self {
        Self {
            rcl_handle,
            goal_request: Arc::clone(&goal_request),
            _marker: Default::default(),
        }
    }

    pub fn is_canceling(&self) -> bool {
        false
    }

    pub fn is_active(&self) -> bool {
        false
    }

    pub fn is_executing(&self) -> bool {
        false
    }

    pub fn succeed(&self, result: &T::Result) -> Result<(), RclrsError> {
        Ok(())
    }

    pub fn canceled(&self, result: &T::Result) -> Result<(), RclrsError> {
        Ok(())
    }
}
