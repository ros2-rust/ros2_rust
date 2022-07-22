use crate::error::ToResult;
use crate::rcl_bindings::*;
use crate::{Context, RclrsError};
use std::ffi::CString;
use std::sync::Arc;

use parking_lot::Mutex;
use std::os::raw::c_char;

pub struct ContextBuilder {
    cstring_args: Vec<CString>,
    init_options_mtx: Mutex<rcl_init_options_t>,
}

impl ContextBuilder {
    /// Build a new ContextBuilder instance
    pub fn new(args: impl IntoIterator<Item = String>) -> Result<ContextBuilder, RclrsError> {
        Ok(ContextBuilder {
            cstring_args: args
                .into_iter()
                .map(|arg| {
                    CString::new(arg.as_str()).map_err(|err| RclrsError::StringContainsNul {
                        err,
                        s: arg.clone(),
                    })
                })
                .collect::<Result<_, _>>()?,
            // SAFETY: Getting a zero-initialized value is always safe.
            init_options_mtx: unsafe { Mutex::new(rcl_get_zero_initialized_init_options()) },
        })
    }

    /// Function to build the Context instance
    pub fn build(&self) -> Result<Context, RclrsError> {
        let mut rcl_init_options = self.init_options_mtx.lock();

        let c_args: Vec<*const c_char> = self.cstring_args.iter().map(|arg| arg.as_ptr()).collect();
        unsafe {
            // SAFETY: Getting a zero-initialized value is always safe
            let mut rcl_context: rcl_context_t = rcl_get_zero_initialized_context();
            // SAFETY: No preconditions for this function.
            let allocator: rcutils_allocator_t = rcutils_get_default_allocator();

            // SAFETY: Passing in a zero-initialized value is expected.
            // In the case where this returns not ok, there's nothing to clean up.
            rcl_init_options_init(&mut *rcl_init_options, allocator).ok()?;
            // SAFETY: This function does not store the ephemeral init_options and c_args
            // pointers. Passing in a zero-initialized rcl_context is expected.

            let ret = rcl_init(
                c_args.len() as i32,
                if c_args.is_empty() {
                    std::ptr::null()
                } else {
                    c_args.as_ptr()
                },
                &*rcl_init_options,
                &mut rcl_context,
            )
            .ok();
            // SAFETY: It's safe to pass in an initialized object.
            // Early return will not leak memory, because this is the last fini function.
            rcl_init_options_fini(&mut *rcl_init_options).ok()?;
            // Move the check after the last fini()
            ret?;
            Ok(Context {
                rcl_context_mtx: Arc::new(Mutex::new(rcl_context)),
            })
        }
    }
}
