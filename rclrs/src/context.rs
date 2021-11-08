use crate::error::ToResult;
use crate::rcl_bindings::*;
use crate::Node;
use alloc::sync::Arc;
use alloc::vec::Vec;
use cstr_core::{c_char, CString};
use rclrs_common::error::RclReturnCode;

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

#[derive(Debug)]
pub struct ContextHandle(Mutex<rcl_context_t>);

impl ContextHandle {
    pub fn get_mut(&mut self) -> &mut rcl_context_t {
        self.0.get_mut()
    }

    pub fn lock(&self) -> MutexGuard<rcl_context_t> {
        self.0.lock()
    }

    pub fn try_lock(&self) -> Option<MutexGuard<rcl_context_t>> {
        self.0.try_lock()
    }
}

impl Drop for ContextHandle {
    fn drop(&mut self) {
        unsafe {
            rcl_shutdown(&mut *self.get_mut() as *mut _);
        }
    }
}

#[derive(Debug)]
pub struct Context {
    pub handle: Arc<ContextHandle>,
}

impl Context {
    fn init(&self, context_env_args: Vec<CString>) -> Result<(), RclReturnCode> {
        let c_args: Vec<*const c_char> = context_env_args.iter().map(|arg| arg.as_ptr()).collect();
        let handle = &mut *self.handle.lock();

        unsafe {
            let allocator = rcutils_get_default_allocator();
            let mut init_options = rcl_get_zero_initialized_init_options();
            rcl_init_options_init(&mut init_options as *mut _, allocator);
            rcl_init(
                c_args.len() as i32,
                c_args.as_ptr(),
                &init_options as *const _,
                handle as *mut _,
            )
            .ok()?;
            rcl_init_options_fini(&mut init_options as *mut _).ok()?;
        }

        Ok(())
    }

    pub fn default(args: Vec<CString>) -> Self {
        let context = Self {
            handle: Arc::new(ContextHandle(Mutex::new(unsafe {
                rcl_get_zero_initialized_context()
            }))),
        };
        context.init(args).unwrap(); // If we can't initialize the context, ROS 2 cannot function
        context
    }

    pub fn ok(&self) -> Result<bool, RclReturnCode> {
        let handle = &mut *self.handle.lock();
        unsafe { Ok(rcl_context_is_valid(handle as *mut _)) }
    }

    pub fn create_node(&self, node_name: &str) -> Result<Node, RclReturnCode> {
        Ok(Node::new(node_name, self)?)
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use std::{env, println};

    fn default_context() -> Context {
        let args: Vec<CString> = env::args()
            .filter_map(|arg| CString::new(arg).ok())
            .collect();
        println!("<test_create_context> Context args: {:?}", args);
        Context::default(args)
    }

    #[test]
    fn test_create_context() {
        // If the context fails to be created, this will cause a panic
        let created_context = default_context();
        println!("<test_create_context> Created Context: {:?}", created_context);
    }

    #[test]
    fn test_context_ok() {
        // If the context fails to be created, this will cause a panic
        let created_context = default_context();
        let ctxt_ok = created_context.ok();
        match ctxt_ok {
            Ok(is_ok) => assert!(is_ok),
            Err(err_code) => panic!("<test_context_ok> RCL Error occured during test: {:?}", err_code),
        }
    }

    #[test]
    fn test_create_node() -> Result<(), RclReturnCode> {
        // If the context fails to be created, this will cause a panic
        let created_context = default_context();
        created_context.create_node("Bob").map(|_x| ())
    }

    #[test]
    fn text_context_init() {
        // If the context fails to be created, this will cause a panic
        let args: Vec<CString> = env::args()
            .filter_map(|arg| CString::new(arg).ok())
            .collect();
        let context = Context {
            handle: Arc::new(ContextHandle(Mutex::new(unsafe {
                rcl_get_zero_initialized_context()
            }))),
        };
        context.init(args).unwrap();
    }

}