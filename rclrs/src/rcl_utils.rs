use crate::rcl_bindings::*;
use crate::{RclrsError, ToResult};

use crate::RclrsError::IndexOutOfRange;
use libc::c_void;
use std::ptr::null_mut;

/// Trait used for obtaining information from rcl_arguments_t.
pub(crate) trait RclGetArg {
    /// Get count function from rcl library (e.g. rcl_arguments_get_count_unparsed).
    ///
    /// SAFETY: [`get_indices()`] implementation for this trait must use corresponding `rcl_arguments_get` function.
    unsafe fn get_count(rcl_args: *const rcl_arguments_t) -> ::std::os::raw::c_int;

    /// Get function from rcl library (e.g. rcl_arguments_get_unparsed).
    ///
    /// SAFETY: [`get_count()`] implementation for this trait must use corresponding `rcl_arguments_get_count` function.
    unsafe fn get_indices(
        rcl_args: *const rcl_arguments_t,
        allocator: rcl_allocator_t,
        out_ptr: *mut *mut ::std::os::raw::c_int,
    ) -> rcl_ret_t;
}

/// Get non-ROS unparsed arguments tag.
///
/// Tag designated to be used as generic argument of [`get_rcl_arguments`].
pub(crate) struct UnparsedNonRos;

/// Get ROS unparsed arguments tag.
///
/// Tag designated to be used as generic argument of [`get_rcl_arguments`].
pub(crate) struct UnparsedRos;

/// Implementation is safe, since both functions corresponds to each other as rcl documentation states.
impl RclGetArg for UnparsedNonRos {
    /// Get count of unparsed non-ROS arguments from `rcl_arguments_t` struct.
    ///
    /// SAFETY: [`get_indices()`] for this implementation must use `rcl_arguments_get_unparsed` function.
    unsafe fn get_count(rcl_args: *const rcl_arguments_t) -> ::std::os::raw::c_int {
        rcl_arguments_get_count_unparsed(rcl_args)
    }

    /// Obtain indices of unparsed non-ROS arguments. Array of indices is stored in `out_ptr` (must be deallocated with passed `allocator`).
    ///
    /// SAFETY: [`get_count()`] for this implementation must use `rcl_arguments_get_count_unparsed` function.
    unsafe fn get_indices(
        rcl_args: *const rcl_arguments_t,
        allocator: rcl_allocator_t,
        out_ptr: *mut *mut ::std::os::raw::c_int,
    ) -> rcl_ret_t {
        rcl_arguments_get_unparsed(rcl_args, allocator, out_ptr)
    }
}

/// Implementation is safe since both functions corresponds to each other as rcl documentation states.
impl RclGetArg for UnparsedRos {
    /// Get count of unparsed ROS specific arguments from `rcl_arguments_t` struct.
    ///
    /// SAFETY: [`get_indices()`] for this implementation must use `rcl_arguments_get_unparsed_ros` function.
    unsafe fn get_count(rcl_args: *const rcl_arguments_t) -> ::std::os::raw::c_int {
        rcl_arguments_get_count_unparsed_ros(rcl_args)
    }

    /// Obtain indices of unparsed ROS specific arguments. Array of indices is stored in `out_ptr` (must be deallocated with passed `allocator`).
    ///
    /// SAFETY: [`get_count()`] for this implementation must use `rcl_get_count_unparsed_ros` function.
    unsafe fn get_indices(
        rcl_args: *const rcl_arguments_t,
        allocator: rcl_allocator_t,
        out_ptr: *mut *mut ::std::os::raw::c_int,
    ) -> rcl_ret_t {
        rcl_arguments_get_unparsed_ros(rcl_args, allocator, out_ptr)
    }
}

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
    let mut extracted_args: Vec<String> = Vec::with_capacity(args_count);
    unsafe {
        let mut indices_ptr: *mut i32 = null_mut();
        // SAFETY: No preconditions for this function.
        let allocator = rcutils_get_default_allocator();
        // Indices will be set to NULL in case of no arguments handled, but then this code won't be
        // reached because of if statement above.
        // SAFETY: No preconditions for this function.
        ArgGetter::get_indices(rcl_arguments, allocator, &mut indices_ptr).ok()?;

        for i in 0..args_count {
            // If get_indices finishes with success, indices_ptr is valid
            // and is allocated with size equal to one returned by rcl_arguments_get_count_unparsed.
            // SAFETY: No preconditions for this function.
            let index = *(indices_ptr.add(i));
            let arg = args.get(index as usize).ok_or(IndexOutOfRange {
                wrong_index: index as usize,
                max_index: args.len() - 1,
            })?;
            extracted_args.push(arg.clone());
        }
        // SAFETY: No preconditions for this function.
        let allocator = rcutils_get_default_allocator();
        // SAFETY: No preconditions for this function.
        allocator.deallocate.unwrap()(indices_ptr as *mut c_void, null_mut());
    }
    Ok(extracted_args)
}
