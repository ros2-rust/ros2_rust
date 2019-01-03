use super::Node;
use crate::error::{RclResult, ToRclResult};
use rcl_sys::*;
use std::env;
use std::ffi::CString;
use std::os::raw::c_char;
use std::sync::{Arc, RwLock};

pub struct Context {
    pub(crate) inner: Arc<RwLock<rcl_context_t>>,
}

impl Context {
    fn init(&mut self) -> RclResult {
        let args: Vec<CString> = env::args()
            .filter_map(|arg| CString::new(arg).ok())
            .collect();

        let c_args: Vec<*const c_char> = args.iter().map(|arg| arg.as_ptr()).collect();

        unsafe {
            let allocator = rcutils_get_default_allocator();
            let mut init_options = rcl_get_zero_initialized_init_options();
            rcl_init_options_init(&mut init_options as *mut _, allocator);
            rcl_init(
                c_args.len() as i32,
                c_args.as_ptr(),
                &init_options as *const _,
                &mut *self.inner.write().unwrap() as *mut _,
            )
            .ok()?;
            rcl_init_options_fini(&mut init_options as *mut _).ok()?;
        }

        Ok(())
    }

    fn shutdown(&mut self) -> RclResult {
        unsafe { rcl_shutdown(&mut *self.inner.write().unwrap() as *mut _) }.ok()
    }

    pub fn ok(&self) -> bool {
        unsafe { rcl_context_is_valid(&mut *self.inner.write().unwrap() as *mut _) }
    }

    pub fn new_node(&self, node_name: &str) -> RclResult<Node<'_>> {
        Ok(Node::new(node_name, self)?)
    }
}

impl Default for Context {
    fn default() -> Self {
        let mut context = Self {
            inner: Arc::new(RwLock::new(unsafe { rcl_get_zero_initialized_context() })),
        };
        context.init().unwrap();
        context
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        self.shutdown().unwrap();
    }
}
