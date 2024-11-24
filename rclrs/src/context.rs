use std::{
    ffi::CString,
    os::raw::c_char,
    string::String,
    sync::{Arc, Mutex},
    vec::Vec,
};

use crate::{rcl_bindings::*, LoggingLifecycle, RclrsError, ToResult};

/// This is locked whenever initializing or dropping any middleware entity
/// because we have found issues in RCL and some RMW implementations that
/// make it unsafe to simultaneously initialize and/or drop middleware
/// entities such as `rcl_context_t` and `rcl_node_t` as well middleware
/// primitives such as `rcl_publisher_t`, `rcl_subscription_t`, etc.
/// It seems these C and C++ based libraries will regularly use
/// unprotected global variables in their object initialization and cleanup.
///
/// Further discussion with the RCL team may help to improve the RCL
/// documentation to specifically call out where these risks are present. For
/// now we lock this mutex for any RCL function that carries reasonable suspicion
/// of a risk.
pub(crate) static ENTITY_LIFECYCLE_MUTEX: Mutex<()> = Mutex::new(());

impl Drop for rcl_context_t {
    fn drop(&mut self) {
        unsafe {
            // The context may be invalid when rcl_init failed, e.g. because of invalid command
            // line arguments.

            // SAFETY: No preconditions for rcl_context_is_valid.
            if rcl_context_is_valid(self) {
                let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
                // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
                // global variables in the rmw implementation being unsafely modified during cleanup.
                rcl_shutdown(self);
                rcl_context_fini(self);
            }
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_context_t {}

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
/// The context also configures the rcl_logging_* layer to allow publication to /rosout
/// (as well as the terminal).  TODO: This behaviour should be configurable using an
/// "auto logging initialise" flag as per rclcpp and rclpy.
///
pub struct Context {
    pub(crate) handle: Arc<ContextHandle>,
}

/// This struct manages the lifetime and access to the `rcl_context_t`. It will also
/// account for the lifetimes of any dependencies, if we need to add
/// dependencies in the future (currently there are none). It is not strictly
/// necessary to decompose `Context` and `ContextHandle` like this, but we are
/// doing it to be consistent with the lifecycle management of other rcl
/// bindings in this library.
pub(crate) struct ContextHandle {
    pub(crate) rcl_context: Mutex<rcl_context_t>,
    /// This ensures that logging does not get cleaned up until after this ContextHandle
    /// has dropped.
    #[allow(unused)]
    logging: Arc<LoggingLifecycle>,
}

impl Context {
    /// Creates a new context.
    ///
    /// Usually this would be called with `std::env::args()`, analogously to `rclcpp::init()`.
    /// See also the official "Passing ROS arguments to nodes via the command-line" tutorial.
    ///
    /// Creating a context will fail if the args contain invalid ROS arguments.
    ///
    /// # Example
    /// ```
    /// # use rclrs::Context;
    /// assert!(Context::new([]).is_ok());
    /// let invalid_remapping = ["--ros-args", "-r", ":=:*/]"].map(String::from);
    /// assert!(Context::new(invalid_remapping).is_err());
    /// ```
    pub fn new(args: impl IntoIterator<Item = String>) -> Result<Self, RclrsError> {
        Self::new_with_options(args, InitOptions::new())
    }

    /// Same as [`Context::new`] except you can additionally provide initialization options.
    ///
    /// # Example
    /// ```
    /// use rclrs::{Context, InitOptions};
    /// let context = Context::new_with_options([], InitOptions::new().with_domain_id(Some(5))).unwrap();
    /// assert_eq!(context.domain_id(), 5);
    /// ````
    pub fn new_with_options(
        args: impl IntoIterator<Item = String>,
        options: InitOptions,
    ) -> Result<Self, RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe
        let mut rcl_context = unsafe { rcl_get_zero_initialized_context() };
        let cstring_args: Vec<CString> = args
            .into_iter()
            .map(|arg| {
                CString::new(arg.as_str()).map_err(|err| RclrsError::StringContainsNul {
                    err,
                    s: arg.clone(),
                })
            })
            .collect::<Result<_, _>>()?;
        // Vector of pointers into cstring_args
        let c_args: Vec<*const c_char> = cstring_args.iter().map(|arg| arg.as_ptr()).collect();
        unsafe {
            // SAFETY: No preconditions for this function.
            let allocator = rcutils_get_default_allocator();
            let mut rcl_init_options = options.into_rcl(allocator)?;
            // SAFETY:
            // * This function does not store the ephemeral init_options and c_args pointers.
            // * Passing in a zero-initialized rcl_context is mandatory.
            // * The entity lifecycle mutex is locked to protect against the risk of global variables
            //   in the rmw implementation being unsafely modified during initialization.
            let ret = {
                let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
                rcl_init(
                    c_args.len() as i32,
                    if c_args.is_empty() {
                        std::ptr::null()
                    } else {
                        c_args.as_ptr()
                    },
                    &rcl_init_options,
                    &mut rcl_context,
                )
                .ok()
            };
            // SAFETY: It's safe to pass in an initialized object.
            // Early return will not leak memory, because this is the last fini function.
            rcl_init_options_fini(&mut rcl_init_options).ok()?;
            // Move the check after the last fini()
            ret?;
        }

        // TODO: "Auto set-up logging" is forced but should be configurable as per rclcpp and rclpy
        // SAFETY: We created this context a moment ago and verified that it is valid.
        // No other conditions are needed.
        let logging = unsafe { LoggingLifecycle::configure(&rcl_context)? };

        Ok(Self {
            handle: Arc::new(ContextHandle {
                rcl_context: Mutex::new(rcl_context),
                logging,
            }),
        })
    }

    /// Returns the ROS domain ID that the context is using.
    ///
    /// The domain ID controls which nodes can send messages to each other, see the [ROS 2 concept article][1].
    /// It can be set through the `ROS_DOMAIN_ID` environment variable.
    ///
    /// [1]: https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html
    pub fn domain_id(&self) -> usize {
        let mut domain_id: usize = 0;
        let ret = unsafe {
            rcl_context_get_domain_id(
                &mut *self.handle.rcl_context.lock().unwrap(),
                &mut domain_id,
            )
        };

        debug_assert_eq!(ret, 0);
        domain_id
    }

    /// Checks if the context is still valid.
    ///
    /// This will return `false` when a signal has caused the context to shut down (currently
    /// unimplemented).
    pub fn ok(&self) -> bool {
        // This will currently always return true, but once we have a signal handler, the signal
        // handler could call `rcl_shutdown()`, hence making the context invalid.
        let rcl_context = &mut *self.handle.rcl_context.lock().unwrap();
        // SAFETY: No preconditions for this function.
        unsafe { rcl_context_is_valid(rcl_context) }
    }
}

/// Additional options for initializing the Context.
#[derive(Default, Clone)]
pub struct InitOptions {
    /// The domain ID that should be used by the Context. Set to None to ask for
    /// the default behavior, which is to set the domain ID according to the
    /// [ROS_DOMAIN_ID][1] environment variable.
    ///
    /// [1]: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html#the-ros-domain-id
    domain_id: Option<usize>,
}

impl InitOptions {
    /// Create a new InitOptions with all default values.
    pub fn new() -> InitOptions {
        Self::default()
    }

    /// Transform an InitOptions into a new one with a certain domain_id
    pub fn with_domain_id(mut self, domain_id: Option<usize>) -> InitOptions {
        self.domain_id = domain_id;
        self
    }

    /// Set the domain_id of an InitOptions, or reset it to the default behavior
    /// (determined by environment variables) by providing None.
    pub fn set_domain_id(&mut self, domain_id: Option<usize>) {
        self.domain_id = domain_id;
    }

    /// Get the domain_id that will be provided by these InitOptions.
    pub fn domain_id(&self) -> Option<usize> {
        self.domain_id
    }

    fn into_rcl(self, allocator: rcutils_allocator_s) -> Result<rcl_init_options_t, RclrsError> {
        unsafe {
            // SAFETY: Getting a zero-initialized value is always safe.
            let mut rcl_init_options = rcl_get_zero_initialized_init_options();
            // SAFETY: Passing in a zero-initialized value is expected.
            // In the case where this returns not ok, there's nothing to clean up.
            rcl_init_options_init(&mut rcl_init_options, allocator).ok()?;

            // We only need to set the domain_id if the user asked for something
            // other than None. When the user asks for None, that is equivalent
            // to the default value in rcl_init_options.
            if let Some(domain_id) = self.domain_id {
                rcl_init_options_set_domain_id(&mut rcl_init_options, domain_id);
            }
            Ok(rcl_init_options)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn traits() {
        use crate::test_helpers::*;

        assert_send::<Context>();
        assert_sync::<Context>();
    }

    #[test]
    fn test_create_context() -> Result<(), RclrsError> {
        // If the context fails to be created, this will cause a panic
        let _ = Context::new(vec![])?;
        Ok(())
    }

    #[test]
    fn test_context_ok() -> Result<(), RclrsError> {
        // If the context fails to be created, this will cause a panic
        let created_context = Context::new(vec![]).unwrap();
        assert!(created_context.ok());

        Ok(())
    }
}
