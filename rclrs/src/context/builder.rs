use std::ffi::CString;
use std::os::raw::c_char;
use std::sync::{Arc, Mutex};

use crate::rcl_bindings::*;
use crate::{Context, RclrsError, ToResult};

/// A builder for creating a [`Context`][1].
///
/// The builder pattern allows selectively setting some fields, and leaving all others at their default values.
/// This struct instance can be created via [`Context::builder()`][2].
///
/// # Example
/// ```
/// # use rclrs::{Context, ContextBuilder, RclrsError};
/// // Building a context in a single expression
/// let args = ["ROS 1 ROS 2"].map(String::from);
/// let context = ContextBuilder::new(args.clone()).domain_id(1).build()?;
/// assert_eq!(context.domain_id(), 1);
/// // Building a context via Context::builder()
/// let context = Context::builder(args.clone()).domain_id(2).build()?;
/// assert_eq!(context.domain_id(), 2);
/// // Building a context step-by-step
/// let mut builder = Context::builder(args.clone());
/// builder = builder.domain_id(3);
/// let context = builder.build()?;
/// assert_eq!(context.domain_id(), 3);
/// # Ok::<(), RclrsError>(())
/// ```
///
/// [1]: crate::Context
/// [2]: crate::Context::builder
pub struct ContextBuilder {
    arguments: Vec<String>,
    domain_id: usize,
}

impl ContextBuilder {
    /// Creates a builder for a context with arguments.
    ///
    /// Usually, this would be called with `std::env::args()`, analogously to `rclcpp::init()`.
    /// See also the official "Passing ROS arguments to nodes via the command-line" tutorial.
    ///
    /// Creating a context can fail in case the args contain invalid ROS arguments.
    ///
    /// # Example
    /// ```
    /// # use rclrs::ContextBuilder;
    /// let invalid_remapping = ["--ros-args", "-r", ":=:*/]"].map(String::from);
    /// assert!(ContextBuilder::new(invalid_remapping).build().is_err());
    /// let valid_remapping = ["--ros-args", "--remap", "__node:=my_node"].map(String::from);
    /// assert!(ContextBuilder::new(valid_remapping).build().is_ok());
    /// ```
    pub fn new(args: impl IntoIterator<Item = String>) -> ContextBuilder {
        let mut domain_id = 0;
        // SAFETY: Getting the default domain ID, based on the environment
        let ret = unsafe { rcl_get_default_domain_id(&mut domain_id) };
        debug_assert_eq!(ret, 0);

        ContextBuilder {
            arguments: args.into_iter().collect(),
            domain_id,
        }
    }

    /// Sets the context domain id.
    ///
    /// The domain ID controls which nodes can send messages to each other, see the [ROS 2 concept article][1].
    ///
    /// [1]: https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html
    #[cfg(not(ros_distro = "foxy"))]
    pub fn domain_id(mut self, domain_id: usize) -> Self {
        self.domain_id = domain_id;
        self
    }

    /// Builds the context instance and `rcl_init_options` in order to initialize rcl
    ///
    /// For example usage, see the [`ContextBuilder`][1] docs.
    ///
    /// [1]: crate::ContextBuilder
    pub fn build(&self) -> Result<Context, RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe
        let mut rcl_context = unsafe { rcl_get_zero_initialized_context() };
        let cstring_args: Vec<CString> = self
            .arguments
            .iter()
            .map(|arg| {
                CString::new(arg.as_str()).map_err(|err| RclrsError::StringContainsNul {
                    err,
                    s: arg.clone(),
                })
            })
            .collect::<Result<_, _>>()?;
        // Vector of pointers into cstring_args
        let c_args: Vec<*const c_char> = cstring_args.iter().map(|arg| arg.as_ptr()).collect();
        let rcl_init_options = self.create_rcl_init_options()?;
        unsafe {
            // SAFETY: This function does not store the ephemeral init_options and c_args
            // pointers. Passing in a zero-initialized rcl_context is expected.
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
            .ok()?;
        }
        Ok(Context {
            rcl_context_mtx: Arc::new(Mutex::new(rcl_context)),
        })
    }

    /// Creates a rcl_init_options_t struct from this builder.
    ///
    /// domain id validation is performed in this method.
    fn create_rcl_init_options(&self) -> Result<rcl_init_options_t, RclrsError> {
        unsafe {
            // SAFETY: No preconditions for this function.
            let allocator = rcutils_get_default_allocator();
            // SAFETY: Getting a zero-initialized value is always safe.
            let mut rcl_init_options = rcl_get_zero_initialized_init_options();
            // SAFETY: Passing in a zero-initialized value is expected.
            // In the case where this returns not ok, there's nothing to clean up.
            rcl_init_options_init(&mut rcl_init_options, allocator).ok()?;
            // SAFETY: Setting domain id in the init options provided.
            // In the case where this returns not ok, the domain id is invalid.
            rcl_init_options_set_domain_id(&mut rcl_init_options, self.domain_id).ok()?;

            Ok(rcl_init_options)
        }
    }
}

impl Drop for rcl_init_options_t {
    fn drop(&mut self) {
        // SAFETY: Do not finish this struct except here.
        unsafe {
            rcl_init_options_fini(self).ok().unwrap();
        }
    }
}
