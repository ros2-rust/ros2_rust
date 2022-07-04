use crate::rcl_bindings::*;
use crate::{RclrsError, ToResult};

use libc::c_void;
use std::ffi::CString;
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
        Ok(Self {
            rcl_context_mtx: Arc::new(Mutex::new(rcl_context)),
        })
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

    /// Get non-ROS arguments
    ///
    /// `input_args` is array of arguments passed to launched node.
    pub fn non_ros_arguments(&self, input_args: &[String]) -> Result<Vec<String>, RclrsError> {
        let rcl_ctx = self.rcl_context_mtx.lock();
        let rcl_args = &rcl_ctx.global_arguments;
        get_non_ros_arguments(rcl_args, input_args)
    }
}

/// Returns non-ROS arguments held by rcl arguments.
///
/// This function should be called after rcl_init. `rcl_arguments` should be `global_arguments`
/// field of initialized context. `args` is array of input arguments passed to node.
pub(crate) fn get_non_ros_arguments(
    rcl_arguments: *const rcl_arguments_t,
    args: &[String],
) -> Result<Vec<String>, RclrsError> {
    // SAFETY: No preconditions for this function.
    let args_count = unsafe { rcl_arguments_get_count_unparsed(rcl_arguments) };
    debug_assert!(args_count != -1);
    // All possible negative args_count values were handled above.
    let args_count = args_count as usize;
    if args_count == 0 {
        return Ok(Vec::new());
    }
    let mut non_ros_args: Vec<String> = Vec::with_capacity(args_count);
    unsafe {
        let mut indices_ptr: *mut i32 = null_mut();
        // SAFETY: No preconditions for next 2 functions.
        let allocator = rcutils_get_default_allocator();
        rcl_arguments_get_unparsed(rcl_arguments, allocator, &mut indices_ptr).ok()?;

        for i in 0..args_count {
            // If rcl_arguments_get_unparsed finishes with success, indices_ptr is valid
            // and is allocated with size equal to one returned by rcl_arguments_get_count_unparsed.
            // SAFETY: No preconditions for this function.
            let index = *(indices_ptr.add(i));
            non_ros_args.push(args.get(index as usize).unwrap().clone());
        }
        // SAFETY: No preconditions for next 2 functions.
        let allocator = rcutils_get_default_allocator();
        allocator.deallocate.unwrap()(indices_ptr as *mut c_void, null_mut());
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

    #[test]
    fn test_non_ros_arguments() -> Result<(), String> {
        // ROS args are expected to be between '--ros-args' and '--'. Everything beside that is 'non-ROS'.
        let input_args: Vec<String> = vec![
            "non-ros1",
            "--ros-args",
            "ros-args",
            "--",
            "non-ros2",
            "non-ros3",
        ]
            .into_iter()
            .map(|x| x.to_string())
            .collect();
        let context = Context::new(input_args.clone()).unwrap();

        let non_ros_args: Vec<String> = context.non_ros_arguments(input_args.as_slice()).unwrap();
        let expected = vec!["non-ros1", "non-ros2", "non-ros3"];

        if non_ros_args.len() != expected.len() {
            Err(format!(
                "Expected vector size: {}, actual: {}",
                expected.len(),
                non_ros_args.len()
            ))
        } else {
            for i in 0..non_ros_args.len() {
                if non_ros_args[i] != expected[i] {
                    let msg = format!(
                        "Mismatching elements at position: {}. Expected: {}, got: {}",
                        i, expected[i], non_ros_args[i]
                    );
                    return Err(msg);
                }
            }
            Ok(())
        }
    }
}
