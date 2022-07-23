use crate::rcl_bindings::*;
use crate::{RclrsError, ToResult};

use crate::RclrsError::IndexOutOfRange;
use libc::c_void;
use std::ptr::null_mut;

/// Returns arguments type held by `rcl_arguments` basing on `rcl_get_count` and `rcl_get_indices` function pointers.
///
/// This function must be called after `rcl_arguments` was initialized. `args` must be array of input arguments passed to node/program.
///
/// SAFETY: `rcl_get_count` and `rcl_get_indices` has to be corresponding rcl API functions, e.g.:
/// `rcl_arguments_get_count_unparsed` -> `rcl_arguments_get_unparsed`
/// `rcl_arguments_get_count_unparsed_ros` -> `rcl_arguments_get_count_ros`
/// ...
pub(crate) fn get_rcl_arguments(
    rcl_get_count: unsafe extern "C" fn(*const rcl_arguments_t) -> std::os::raw::c_int,
    rcl_get_indices: unsafe extern "C" fn(
        *const rcl_arguments_t,
        rcl_allocator_t,
        *mut *mut ::std::os::raw::c_int,
    ) -> rcl_ret_t,
    rcl_arguments: *const rcl_arguments_t,
    args: &[String],
) -> Result<Vec<String>, RclrsError> {
    // SAFETY: No preconditions for this function.
    let args_count = unsafe { rcl_get_count(rcl_arguments) };
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
        // SAFETY: The indices_ptr is an output parameter, so it is expected that it contains null.
        // The indices_ptr will need to be freed by the caller, which happens later in this function.
        rcl_get_indices(rcl_arguments, allocator, &mut indices_ptr).ok()?;

        for i in 0..args_count {
            // SAFETY: get_indices finished with success, and the length of arguments was not zero,
            // so indices_ptr is a valid array pointer with size equal to one returned by 
            // rcl_arguments_get_count_unparsed. Therefore this array indexing is safe.
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
