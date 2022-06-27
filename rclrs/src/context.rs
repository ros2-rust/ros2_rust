use crate::rcl_bindings::*;
use crate::RclReturnCode::InvalidArgument;
use crate::{RclrsError, ToResult};

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr::null_mut;
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
                // SAFETY: These functions have no preconditions besides a valid rcl_context
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
pub struct Context {
    pub(crate) rcl_context_mtx: Arc<Mutex<rcl_context_t>>,
    pub(crate) non_ros_arguments: Vec<String>,
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
    pub fn new(args: impl IntoIterator<Item = String>) -> Result<Self, RclrsError> {
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
            // SAFETY: Getting a zero-initialized value is always safe.
            let mut rcl_init_options = rcl_get_zero_initialized_init_options();
            // SAFETY: Passing in a zero-initialized value is expected.
            // In the case where this returns not ok, there's nothing to clean up.
            rcl_init_options_init(&mut rcl_init_options, allocator).ok()?;
            // SAFETY: This function does not store the ephemeral init_options and c_args
            // pointers. Passing in a zero-initialized rcl_context is expected.
            let ret = rcl_init(
                c_args.len() as i32,
                if c_args.is_empty() {
                    std::ptr::null()
                } else {
                    c_args.as_ptr()
                },
                &rcl_init_options,
                &mut rcl_context,
            )
            .ok();
            // SAFETY: It's safe to pass in an initialized object.
            // Early return will not leak memory, because this is the last fini function.
            rcl_init_options_fini(&mut rcl_init_options).ok()?;
            // Move the check after the last fini()
            ret?;
        }
        let non_ros_args = get_non_ros_arguments(&rcl_context.global_arguments, &c_args)?;
        Ok(Self {
            rcl_context_mtx: Arc::new(Mutex::new(rcl_context)),
            non_ros_arguments: non_ros_args,
        })
    }

    /// Returns non-ROS arguments detected during context initialization.
    pub fn non_ros_arguments(&self) -> &Vec<String> {
        &self.non_ros_arguments
    }
    /// Checks if the context is still valid.
    ///
    /// This will return `false` when a signal has caused the context to shut down (currently
    /// unimplemented).
    pub fn ok(&self) -> bool {
        // This will currently always return true, but once we have a signal handler, the signal
        // handler could call `rcl_shutdown()`, hence making the context invalid.
        let rcl_context = &mut *self.rcl_context_mtx.lock();
        // SAFETY: No preconditions for this function.
        unsafe { rcl_context_is_valid(rcl_context) }
    }
}

/// Returns non-ROS arguments held by context.
///
/// This function should be called after rcl_init. `rcl_arguments` should be `global_arguments`
/// field of initialized context. `args` is array of input arguments passed to node.
fn get_non_ros_arguments(
    rcl_arguments: *const rcl_arguments_t,
    args: &[*const c_char],
) -> Result<Vec<String>, RclrsError> {
    // SAFETY: In case of error -1 will be returned and handled a line below.
    let args_status = unsafe { rcl_arguments_get_count_unparsed(rcl_arguments) };
    if args_status == -1 {
        return Err(RclrsError::RclError {
            code: InvalidArgument,
            msg: None,
        });
    }
    // SAFETY: All possible negative args_status values handled above.
    let args_num = args_status as usize;
    if args_num == 0 {
        return Ok(Vec::new());
    }
    let mut out_ptr: *mut i32 = null_mut();
    unsafe {
        // SAFETY: No preconditions for this function.
        let allocator = rcutils_get_default_allocator();
        // SAFETY: Error will be returned in case of broken rcl_arguments, no precondition for the rest.
        rcl_arguments_get_unparsed(rcl_arguments, allocator, &mut out_ptr).ok()?;
    }
    // SAFETY: out_ptr won't be allocated in case of error and this code won't be reached.
    // Indices of non-ROS args in input args array
    let indices: Vec<i32> = unsafe { Vec::from_raw_parts(out_ptr, args_num, args_num) };
    let mut non_ros_args: Vec<String> = Vec::with_capacity(args_num);

    for i in indices.iter() {
        // SAFETY: args are expected to have been transformed from String. Indices are expected to be positive.
        let c_str = unsafe { CStr::from_ptr(args[*i as usize]).to_str().unwrap() };
        non_ros_args.push(c_str.to_string());
    }
    Ok(non_ros_args)
}

#[cfg(test)]
mod tests {
    use super::*;

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
