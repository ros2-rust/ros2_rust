use crate::rcl_bindings::*;
use crate::{Node, RclReturnCode, ToResult, WaitSet};
use alloc::sync::Arc;
use alloc::vec::Vec;
use std::string::String;
use std::time::Duration;

#[cfg(not(feature = "std"))]
use cstr_core::{c_char, CString};
#[cfg(not(feature = "std"))]
use spin::Mutex;

#[cfg(feature = "std")]
use cty::c_char;
#[cfg(feature = "std")]
use parking_lot::Mutex;
#[cfg(feature = "std")]
use std::ffi::CString;

impl Drop for rcl_context_t {
    fn drop(&mut self) {
        unsafe {
            rcl_shutdown(self as *mut _);
            rcl_context_fini(self as *mut _);
        }
    }
}

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
    pub fn new(args: impl IntoIterator<Item = String>) -> Result<Self, RclReturnCode> {
        let context = Self {
            // SAFETY: Getting a zero-initialized value is always safe
            handle: Arc::new(Mutex::new(unsafe { rcl_get_zero_initialized_context() })),
        };
        let cstring_args: Vec<CString> = args
            .into_iter()
            .map(|arg| CString::new(arg).unwrap())
            .collect();
        // Vector of pointers into cstring_args
        let c_args: Vec<*const c_char> = cstring_args.iter().map(|arg| arg.as_ptr()).collect();
        // Scope for the handle
        {
            let handle = &mut *context.handle.lock();
            unsafe {
                // SAFETY: No preconditions for this function.
                let allocator = rcutils_get_default_allocator();
                // SAFETY: Getting a zero-initialized value is always safe.
                let mut init_options = rcl_get_zero_initialized_init_options();
                // SAFETY: Passing in a zero-initialized value is expected.
                // In the case where this returns not ok, there's nothing to clean up.
                rcl_init_options_init(&mut init_options as *mut _, allocator).ok()?;
                // SAFETY: This function does not store the ephemeral init_options and c_args
                // pointers. Passing in a zero-initialized handle is expected.
                let ret = rcl_init(
                    c_args.len() as i32,
                    if c_args.is_empty() {
                        std::ptr::null()
                    } else {
                        c_args.as_ptr()
                    },
                    &init_options as *const _,
                    handle as *mut _,
                );
                // SAFETY: It's safe to pass in an initialized object.
                // Early return will not leak memory, because this is the last fini function.
                rcl_init_options_fini(&mut init_options as *mut _).ok()?;
                // Move the check after the last fini()
                ret.ok()?;
            }
        }
        Ok(context)
    }

    pub fn create_node(&self, node_name: &str) -> Result<Node, RclReturnCode> {
        Node::new(node_name, self)
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
        unsafe { rcl_context_is_valid(handle as *mut _) }
    }

    /// Polls the node for new messages and executes the corresponding callbacks.
    ///
    /// See [`WaitSet::wait`] for the meaning of the `timeout` parameter.
    ///
    /// This may under some circumstances return
    /// [`SubscriptionTakeFailed`][1] when the wait set spuriously wakes up.
    /// This can usually be ignored.
    ///
    /// [1]: crate::SubscriberErrorCode
    pub fn spin_once(&self, node: &Node, timeout: Option<Duration>) -> Result<(), RclReturnCode> {
        let live_subscriptions = node.live_subscriptions();

        let mut wait_set = WaitSet::new(live_subscriptions.len(), self)?;

        for live_subscription in &live_subscriptions {
            wait_set.add_subscription(live_subscription.clone())?;
        }

        let ready_entities = wait_set.wait(timeout)?;
        for ready_subscription in ready_entities.subscriptions {
            ready_subscription.execute()?;
        }

        Ok(())
    }

    /// Convenience function for calling [`Context::spin_once`] in a loop.
    ///
    /// This function additionally checks that the context is still valid.
    pub fn spin(&self, node: &Node) -> Result<(), RclReturnCode> {
        while unsafe { rcl_context_is_valid(&mut *node.context.lock() as *mut _) } {
            if let Some(error) = self.spin_once(node, None).err() {
                match error {
                    RclReturnCode::Timeout => continue,
                    error => return Err(error),
                };
            }
        }

        Ok(())
    }
}
