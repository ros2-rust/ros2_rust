use crate::rcl_bindings::*;
use crate::{Context, RclrsError};
use std::ffi::CString;
use std::os::raw::c_char;
use parking_lot::Mutex;

pub struct ContextBuilder {
    cstring_args: Vec<CString>,
    c_args: Option<Vec<*const c_char>>,
    context_mtx: Mutex<rcl_context_t>,
    allocator: rcl_allocator_t,
    init_options_mtx: Mutex<rcl_init_options_t>,
}

impl ContextBuilder {

    /// Build a new ContextBuilder instance
    pub fn new(args: impl IntoIterator<Item = String>) -> Result<ContextBuilder, RclrsError> {
        unsafe {
            Ok(ContextBuilder {
                cstring_args: args.into_iter().map(|arg| {
                    CString::new(arg.as_str()).map_err(|err| RclrsError::StringContainsNul{
                        err,
                        s: arg.clone(),
                    })
                }).collect::<Result<_, _>>()?,
                c_args: None, // to be built in the build function
                context_mtx: Mutex::new(rcl_get_zero_initialized_context()),
                allocator: rcutils_get_default_allocator(),
                init_options_mtx: Mutex::new(rcl_get_zero_initialized_init_options())
            })
        }
    }

    pub fn build(&self) -> Result<Context, RclrsError> {
        todo!("call build here");
    }
}
