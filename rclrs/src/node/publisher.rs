use crate::error::{RclResult, ToRclResult};
use crate::qos::QoSProfile;
use crate::{Handle, Node, NodeHandle};
use crate::rcl_bindings::*;
use std::borrow::Borrow;
use std::cell::{Ref, RefCell, RefMut};
use std::ffi::CString;
use std::marker::PhantomData;
use std::rc::Rc;

pub struct PublisherHandle {
    handle: RefCell<rcl_publisher_t>,
    node_handle: Rc<NodeHandle>,
}

impl PublisherHandle {
    fn node_handle(&self) -> &NodeHandle {
        self.node_handle.borrow()
    }
}

impl<'a> Handle<rcl_publisher_t> for &'a PublisherHandle {
    type DerefT = Ref<'a, rcl_publisher_t>;
    type DerefMutT = RefMut<'a, rcl_publisher_t>;

    fn get(self) -> Self::DerefT {
        self.handle.borrow()
    }

    fn get_mut(self) -> Self::DerefMutT {
        self.handle.borrow_mut()
    }
}

impl Drop for PublisherHandle {
    fn drop(&mut self) {
        let handle = &mut *self.get_mut();
        let node_handle = &mut *self.node_handle().get_mut();
        unsafe {
            rcl_publisher_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

pub struct Publisher<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub handle: Rc<PublisherHandle>,
    message: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub fn new(node: &Node, topic: &str, qos: QoSProfile) -> RclResult<Self>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support = T::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.get_mut();

        unsafe {
            let mut publisher_options = rcl_publisher_get_default_options();
            publisher_options.qos = qos.into();

            rcl_publisher_init(
                &mut publisher_handle as *mut _,
                node_handle as *mut _,
                type_support,
                topic_c_string.as_ptr(),
                &publisher_options as *const _,
            )
            .ok()?;
        }

        let handle = Rc::new(PublisherHandle {
            handle: RefCell::new(publisher_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            message: PhantomData,
        })
    }

    pub fn publish(&self, message: &T) -> RclResult {
        let native_message_ptr = message.get_native_message();
        let handle = &mut *self.handle.get_mut();
        let ret = unsafe { rcl_publish(handle as *mut _, native_message_ptr as *mut _, std::ptr::null_mut()) };
        message.destroy_native_message(native_message_ptr);
        ret.ok()
    }
}
