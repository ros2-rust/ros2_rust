use crate::rcl_bindings::*;
use crate::{Node, RclrsError, ToResult};

use std::ffi::CString;
use std::os::raw::c_char;
use std::string::String;
use std::sync::Arc;
use std::vec::Vec;

use parking_lot::Mutex;

impl Drop for rcl_context_t {
    fn drop(&mut self) {
        unsafe {
            // The context may be invalid when rcl_init failed, e.g. because of invalid command
            // line arguments.
            // SAFETY: No preconditions for this function.
            if rcl_context_is_valid(self) {
                // SAFETY: These functions have no preconditions besides a valid handle
                rcl_shutdown(self);
                rcl_context_fini(self);
            }
        }
    }
}

/// Shared state between nodes and similar entities.
///
/// It is possible, but not usually necessary, to have several contexts in an application.
///
/// Ownership of the context is shared by the `Context` itself and all nodes created from it.
///
/// # Details
/// A context stores, among other things
/// - command line arguments (used for e.g. name remapping)
/// - middleware-specific data, e.g. the domain participant in DDS
/// - the allocator used (left as the default by `rclrs`)
///
pub struct Context {
    pub(crate) handle: Arc<Mutex<rcl_context_t>>,
}

impl Context {
    /// Creates a new context.
    ///
    /// Usually, this would be called with `std::env::args()`, analogously to `rclcpp::init()`.
    /// See also the official "Passing ROS arguments to nodes via the command-line" tutorial.
    ///
    /// Creating a context can fail in case the args contain invalid ROS arguments.
    ///
    /// # Example
    /// ```
    /// # use rclrs::Context;
    /// assert!(Context::new([]).is_ok());
    /// let invalid_remapping = ["--ros-args", "-r", ":=:*/]"].map(String::from);
    /// assert!(Context::new(invalid_remapping).is_err());
    /// ```
    ///
    /// # Panics
    /// When there is an interior null byte in any of the args.
    pub fn new(args: impl IntoIterator<Item = String>) -> Result<Self, RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe
        let mut rcl_context = unsafe { rcl_get_zero_initialized_context() };
        let cstring_args: Vec<CString> = args
            .into_iter()
            .map(|arg| CString::new(arg).unwrap())
            .collect();
        // Vector of pointers into cstring_args
        let c_args: Vec<*const c_char> = cstring_args.iter().map(|arg| arg.as_ptr()).collect();
        unsafe {
            // SAFETY: No preconditions for this function.
            let allocator = rcutils_get_default_allocator();
            // SAFETY: Getting a zero-initialized value is always safe.
            let mut init_options = rcl_get_zero_initialized_init_options();
            // SAFETY: Passing in a zero-initialized value is expected.
            // In the case where this returns not ok, there's nothing to clean up.
            rcl_init_options_init(&mut init_options, allocator).ok()?;
            // SAFETY: This function does not store the ephemeral init_options and c_args
            // pointers. Passing in a zero-initialized handle is expected.
            let ret = rcl_init(
                c_args.len() as i32,
                if c_args.is_empty() {
                    std::ptr::null()
                } else {
                    c_args.as_ptr()
                },
                &init_options,
                &mut rcl_context,
            )
            .ok();
            // SAFETY: It's safe to pass in an initialized object.
            // Early return will not leak memory, because this is the last fini function.
            rcl_init_options_fini(&mut init_options).ok()?;
            // Move the check after the last fini()
            ret?;
        }
        Ok(Self {
            handle: Arc::new(Mutex::new(rcl_context)),
        })
    }

    /// Creates a new node in the empty namespace.
    ///
    /// Convenience function equivalent to [`Node::new`][1].
    /// Please see that function's documentation.
    ///
    /// [1]: crate::Node::new
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclReturnCode};
    /// let ctx = Context::new([])?;
    /// let node = ctx.create_node("my_node");
    /// assert!(node.is_ok());
    /// # Ok::<(), RclReturnCode>(())
    /// ```
    pub fn create_node(&self, node_name: &str) -> Result<Node, RclrsError> {
        Node::new(node_name, self)
    }

    /// Creates a new node in a namespace.
    ///
    /// Convenience function equivalent to [`Node::new_with_namespace`][1].
    /// Please see that function's documentation.
    ///
    /// [1]: crate::Node::new_with_namespace
    ///
    /// # Example
    /// ```
    /// # use rclrs::{Context, RclReturnCode};
    /// let ctx = Context::new([])?;
    /// let node = ctx.create_node_with_namespace("/my/nested/namespace", "my_node");
    /// assert!(node.is_ok());
    /// # Ok::<(), RclReturnCode>(())
    /// ```
    pub fn create_node_with_namespace(
        &self,
        node_namespace: &str,
        node_name: &str,
    ) -> Result<Node, RclrsError> {
        Node::new_with_namespace(node_namespace, node_name, self)
    }

    /// Checks if the context is still valid.
    ///
    /// This will return `false` when a signal has caused the context to shut down (currently
    /// unimplemented).
    pub fn ok(&self) -> bool {
        // This will currently always return true, but once we have a signal handler, the signal
        // handler could call `rcl_shutdown()`, hence making the context invalid.
        let handle = &mut *self.handle.lock();
        // SAFETY: No preconditions for this function.
        unsafe { rcl_context_is_valid(handle) }
    }
}
