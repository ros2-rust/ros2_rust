use crate::rcl_bindings as ffi;
use crate::Node;
use crate::NodeOptions;
use crate::NodeError;
use std::convert::TryInto;
use std::env;
use std::ffi::CString;
use std::os::raw::c_char;
use std::sync::Mutex;
use thiserror::Error;

struct ContextOptions {
    handle: ffi::rcl_init_options_t,
}

#[derive(Error, Debug)]
pub enum ContextOptionsError {
    #[error("The given init_options has already been initialized")]
    AlreadyInitialized,
    #[error("One of the provided arguments is invalid")]
    InvalidArgument,
    #[error("Allocating memory failed")]
    BadAllocation,
    #[error("Unspecified error occured")]
    UnspecifiedError,
}

impl ContextOptions {
    fn new() -> Result<Self, ContextOptionsError> {
        let allocator = unsafe { ffi::rcutils_get_default_allocator() };
        let mut init_options = unsafe { ffi::rcl_get_zero_initialized_init_options() };

        match unsafe { ffi::rcl_init_options_init(&mut init_options, allocator) } as u32 {
            ffi::RCL_RET_OK => Ok(Self {
                handle: init_options,
            }),
            ffi::RCL_RET_ALREADY_INIT => Err(ContextOptionsError::AlreadyInitialized),
            ffi::RCL_RET_INVALID_ARGUMENT => Err(ContextOptionsError::InvalidArgument),
            ffi::RCL_RET_BAD_ALLOC => Err(ContextOptionsError::BadAllocation),
            _ => Err(ContextOptionsError::UnspecifiedError),
        }
    }
}

impl Drop for ContextOptions {
    // Dropping the `ContextOptions` could hypothetically return an
    // "invalid argument" error, but this will never occur since the
    // argument will always be a valid instance of `rcl_init_options_t`.
    fn drop(&mut self) {
        unsafe {
            ffi::rcl_init_options_fini(&mut self.handle);
        }
    }
}

#[derive(Error, Debug)]
pub enum ContextError {
    #[error("Failed to initialize context options")]
    FailedToInitializeOptions(#[from] ContextOptionsError),
    #[error("An invalid amount of arguments was supplied")]
    InvalidAmountOfArguments(#[from] std::num::TryFromIntError),
    #[error("The given init_options has already been initialized")]
    AlreadyInitialized,
    #[error("One of the provided arguments is invalid")]
    InvalidArgument,
    #[error("Allocating memory failed")]
    BadAllocation,
    #[error("Unspecified error occured")]
    UnspecifiedError,
}

pub struct Context {
    pub(crate) handle: Mutex<ffi::rcl_context_t>,
}

impl Context {
    /// Create a new context in which this client of ROS 2 is executed.
    /// # Example
    /// ```
    /// # use rclrs::Context;
    /// let ros = Context::new().unwrap();
    /// ```
    pub fn new() -> Result<Self, ContextError> {
        let args: Vec<CString> = env::args()
            .filter_map(|arg| CString::new(arg).ok())
            .collect();
        let c_args: Vec<*const c_char> = args.iter().map(|arg| arg.as_ptr()).collect();

        // Reserve some memory to store the RCL context in.
        let mut context = unsafe { ffi::rcl_get_zero_initialized_context() };
        let options = ContextOptions::new()?;

        // These arguments are safe to use, because
        // - `c_args` are only parsed during `rcl_init` and are not used after
        //      this function returns.
        // - `&options.handle` is copied into the context. Therefore, the options
        //      can safely be dropped at the end of this function without creating
        //      a dangling pointer.
        // - `&mut context` is a pointer to `context`-sized zero-initialized memory.
        let init_result = unsafe {
            ffi::rcl_init(
                c_args.len().try_into()?,
                c_args.as_ptr(),
                &options.handle,
                &mut context,
            )
        } as u32;

        match init_result {
            ffi::RCL_RET_OK => Ok(Self { handle: Mutex::new(context) }),
            ffi::RCL_RET_ALREADY_INIT => Err(ContextError::AlreadyInitialized),
            ffi::RCL_RET_INVALID_ARGUMENT => Err(ContextError::InvalidArgument),
            ffi::RCL_RET_BAD_ALLOC => Err(ContextError::BadAllocation),
            _ => Err(ContextError::UnspecifiedError),
        }
    }

    fn is_valid(&self) -> bool {
        // This cast to a mutable variable is necessary
        // until https://github.com/ros2/rcl/pull/872 is merged.
        let mut_handle = &self.handle as *const _ as *mut _;
        unsafe { ffi::rcl_context_is_valid(mut_handle) }
    }

    /// Create a ROS Node within this Context. A Node has to have a lifetime that
    /// is at most as long as the Context it is created in.
    ///
    /// # Arguments
    /// * `name` - The name of the node. Node names must not be empty, only
    ///            contain alphanumeric characters and underscores and
    ///            cannot start with a number.
    /// * `namespace` - The namespace of the node. Namespaces must comply
    ///            with [these rules](https://design.ros2.org/articles/topic_and_service_names.html#namespaces).
    /// * `options` - Options for the Node.
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, NodeOptions};
    /// let ros = Context::new().unwrap();
    /// let node = ros.create_node("hello", "world", &NodeOptions::default()).unwrap();
    /// ```
    pub fn create_node(&self, name: &str, namespace: &str, options: &NodeOptions) -> Result<Node, NodeError> {
        Ok(Node::new(&self, name, namespace, options)?)
    }
}

impl Drop for Context {
    // Dropping the `Context` could hypothetically throw three types of errors.
    // 1. Invalid argument - This cannot happen in the case of
    //     `rcl_shutdown`, because a `Context` can only be constructed by
    //     a succesful `Context::new()` and that function will only create
    //     a valid instances of `Context`.
    //     This cannot error can therefore not happen in in the `rcl_context_fini`,
    //     because the handle is valid, not NULL and `rcl_shutdown` is called
    //     before `rcl_context_fini`.
    // 2. Already shutdown - This cannot happen, because `drop` is called
    //     once per instance.
    // 3. Unspecified error - This could happen, but no precaution can be
    //     taken to prevent this.
    //
    // Since these specified errors cannot occur, it is safe to ignore them
    // in the implementation of `drop`.
    fn drop(&mut self) {
        let mut context = self.handle.lock().unwrap();
        unsafe {
            ffi::rcl_shutdown(&mut *context);
            ffi::rcl_context_fini(&mut *context);
        }
    }
}
