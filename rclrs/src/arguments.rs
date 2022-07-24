use crate::error::*;
use crate::rcl_bindings::*;
use crate::rcl_utils::get_rcl_arguments;
use std::ffi::CString;
use std::os::raw::c_char;

/// Extract non-ROS arguments from program's input arguments.
///
/// `args` is expected to be the input arguments of the program (e.g. [`std::env::args()`]),
/// which are expected to contain at least one element - the executable name.
/// 
/// ROS arguments are arguments between `--ros-args` and `--`, with the final `--` being optional.
/// Everything else is considered as non-ROS arguments and will be left unparsed by 
/// [`Context::new()`][1] etc.
/// Extracted non-ROS arguments are returned in the order that they appear by this function.
///
/// # Example
/// ```
/// # use rclrs::RclrsError;
/// let input_args = [
///     "arg1", "--ros-args", "some", "args", "--", "arg2"
/// ].map(|x| x.to_string());
/// let non_ros_args = rclrs::extract_non_ros_args(input_args)?;
/// assert_eq!(non_ros_args.len(), 2);
/// assert_eq!(non_ros_args[0], "arg1");
/// assert_eq!(non_ros_args[1], "arg2");
/// # Ok::<(), RclrsError>(())
/// ```
/// 
/// [1]: crate::Context::new
pub fn extract_non_ros_args(
    args: impl IntoIterator<Item = String>,
) -> Result<Vec<String>, RclrsError> {
    // SAFETY: Getting a zero-initialized value is always safe.
    let mut rcl_arguments = unsafe { rcl_get_zero_initialized_arguments() };

    let (args, cstring_args): (Vec<String>, Vec<Result<CString, RclrsError>>) = args
        .into_iter()
        .map(|arg| {
            let cstring_arg =
                CString::new(arg.as_str()).map_err(|err| RclrsError::StringContainsNul {
                    err,
                    s: arg.clone(),
                });
            (arg, cstring_arg)
        })
        .unzip();
    let cstring_args: Vec<CString> = cstring_args
        .into_iter()
        .collect::<Result<Vec<CString>, RclrsError>>()?;
    // Vector of pointers into cstring_args
    let c_args: Vec<*const c_char> = cstring_args.iter().map(|arg| arg.as_ptr()).collect();

    let argv = if c_args.is_empty() {
        std::ptr::null()
    } else {
        c_args.as_ptr()
    };

    unsafe {
        // SAFETY: Getting a default value is always safe.
        let allocator = rcutils_get_default_allocator();
        // SAFETY: No preconditions for this function.
        rcl_parse_arguments(c_args.len() as i32, argv, allocator, &mut rcl_arguments).ok()?;
    }

    let ret = get_rcl_arguments(
        rcl_arguments_get_count_unparsed,
        rcl_arguments_get_unparsed,
        &rcl_arguments,
        &args,
    );
    unsafe {
        // SAFETY: No preconditions for this function.
        rcl_arguments_fini(&mut rcl_arguments).ok()?;
    }
    ret
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_non_ros_arguments() -> Result<(), String> {
        // ROS args are expected to be between '--ros-args' and '--'. Everything beside that is 'non-ROS'.
        let input_args: [String; 6] = [
            "non-ros1",
            "--ros-args",
            "ros-args",
            "--",
            "non-ros2",
            "non-ros3",
        ]
        .map(|x| x.to_string());

        let non_ros_args: Vec<String> = extract_non_ros_args(input_args).unwrap();
        let expected = vec!["non-ros1", "non-ros2", "non-ros3"];

        if non_ros_args.len() != expected.len() {
            return Err(format!(
                "Expected vector size: {}, actual: {}",
                expected.len(),
                non_ros_args.len()
            ));
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
        }

        Ok(())
    }

    #[test]
    fn test_empty_non_ros_arguments() -> Result<(), RclrsError> {
        let empty_non_ros_args = extract_non_ros_args(vec![])?;
        assert_eq!(empty_non_ros_args.len(), 0);

        Ok(())
    }
}
