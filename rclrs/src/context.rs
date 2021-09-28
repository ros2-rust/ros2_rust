use crate::error::{RclError, ToResult};
use crate::{Handle, Node};
use crate::rcl_bindings::*;
use std::cell::{Ref, RefCell, RefMut};
use std::env;
use std::ffi::CString;
use std::os::raw::c_char;
use std::rc::Rc;
use anyhow::Result;

pub struct ContextHandle(RefCell<rcl_context_t>);

impl<'a> Handle<rcl_context_t> for &'a ContextHandle {
    type DerefT = Ref<'a, rcl_context_t>;
    type DerefMutT = RefMut<'a, rcl_context_t>;

    fn get(self) -> Self::DerefT {
        self.0.borrow()
    }

    fn get_mut(self) -> Self::DerefMutT {
        self.0.borrow_mut()
    }
}

impl Drop for ContextHandle {
    fn drop(&mut self) {
        let handle = &mut *self.get_mut();
        unsafe {
            rcl_shutdown(handle as *mut _);
        }
    }
}

pub struct Context {
    pub handle: Rc<ContextHandle>,
}

impl Context {
    fn init(&mut self) -> Result<(), RclError> {
        let args: Vec<CString> = env::args()
            .filter_map(|arg| CString::new(arg).ok())
            .collect();

        let c_args: Vec<*const c_char> = args.iter().map(|arg| arg.as_ptr()).collect();
        let handle = &mut *self.handle.get_mut();

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

    pub fn ok(&self) -> bool {
        let handle = &mut *self.handle.get_mut();
        unsafe { rcl_context_is_valid(handle as *mut _) }
    }

    pub fn create_node(&self, node_name: &str) -> Result<Node, RclError> {
        Ok(Node::new(node_name, self)?)
    }
}

impl Default for Context {
    fn default() -> Self {
        let mut context = Self {
            handle: Rc::new(ContextHandle(RefCell::new(unsafe {
                rcl_get_zero_initialized_context()
            }))),
        };
        context.init().unwrap();
        context
    }
}
