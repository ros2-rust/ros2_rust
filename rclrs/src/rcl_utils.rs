use crate::rcl_bindings::*;
use crate::{RclrsError, ToResult};

use libc::c_void;
use std::ptr::null_mut;

/// Trait used for obtaining information from rcl_arguments_t.
pub(crate) trait RclGetArg {
    /// Get count function from rcl library (e.g. rcl_arguments_get_count_unparsed)
    unsafe fn get_count(rcl_args: *const rcl_arguments_t) -> ::std::os::raw::c_int;

    /// Get function from rcl library (e.g. rcl_arguments_get_unparsed)
    unsafe fn get_indices(
        rcl_args: *const rcl_arguments_t,
        allocator: rcl_allocator_t,
        out_ptr: *mut *mut ::std::os::raw::c_int,
    ) -> rcl_ret_t;
}

/// Get ROS unparsed arguments tag.
///
/// Tag designated to be used as generic argument of [`get_rcl_arguments`].
pub(crate) struct UnparsedRos;

/// Get non-ROS unparsed arguments tag.
///
/// Tag designated to be used as generic argument of [`get_rcl_arguments`].
pub(crate) struct UnparsedNonRos;

/// Macro to avoid boilerplate code for above tags.
///
/// tag - one of above tags
/// get_count_function - function for getting count of given arguments type
/// get_indices_function - function for getting indices of given arguments type
macro_rules! impl_get_arg {
    ($tag:ty, $get_count_function:ident, $get_indices_function:ident) => {
        impl RclGetArg for $tag {
            unsafe fn get_count(rcl_args: *const rcl_arguments_t) -> ::std::os::raw::c_int {
                $get_count_function(rcl_args)
            }

            unsafe fn get_indices(
                rcl_args: *const rcl_arguments_t,
                allocator: rcl_allocator_t,
                out_ptr: *mut *mut ::std::os::raw::c_int,
            ) -> rcl_ret_t {
                $get_indices_function(rcl_args, allocator, out_ptr)
            }
        }
    };
}

impl_get_arg!(
    UnparsedRos,
    rcl_arguments_get_count_unparsed_ros,
    rcl_arguments_get_unparsed_ros
);
impl_get_arg!(
    UnparsedNonRos,
    rcl_arguments_get_count_unparsed,
    rcl_arguments_get_unparsed
);

/// Returns arguments type indicated by [`ArgsGetter`] held by rcl arguments.
///
/// This function should be called after rcl_init. `rcl_arguments` should be `global_arguments`
/// field of initialized rcl_context. `args` is array of input arguments passed to node.
pub(crate) fn get_rcl_arguments<ArgGetter: RclGetArg>(
    rcl_arguments: *const rcl_arguments_t,
    args: &[String],
) -> Result<Vec<String>, RclrsError> {
    // SAFETY: No preconditions for this function.
    let args_count = unsafe { ArgGetter::get_count(rcl_arguments) };
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
        // Indices will be set to NULL in case of no arguments handled, but then this code won't be
        // reached because of if statement above.
        ArgGetter::get_indices(rcl_arguments, allocator, &mut indices_ptr).ok()?;

        for i in 0..args_count {
            // If get_indices finishes with success, indices_ptr is valid
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
