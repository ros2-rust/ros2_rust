use crate::error::{RclReturnCode, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::{Node, NodeHandle};
use alloc::sync::Arc;
use core::marker::PhantomData;
use cstr_core::CString;
use rosidl_runtime_rs::{Message, RmwMessage};
use std::borrow::Cow;

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

pub(crate) struct PublisherHandle {
    handle: Mutex<rcl_publisher_t>,
    node_handle: Arc<NodeHandle>,
}

impl PublisherHandle {
    fn lock(&self) -> MutexGuard<rcl_publisher_t> {
        self.handle.lock()
    }
}

impl Drop for PublisherHandle {
    fn drop(&mut self) {
        let handle = self.handle.get_mut();
        let node_handle = &mut *self.node_handle.lock();
        unsafe {
            rcl_publisher_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Main class responsible for publishing data to ROS topics
pub struct Publisher<T>
where
    T: Message,
{
    pub(crate) handle: Arc<PublisherHandle>,
    message: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: Message,
{
    pub fn new(node: &Node, topic: &str, qos: QoSProfile) -> Result<Self, RclReturnCode>
    where
        T: Message,
    {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

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

        let handle = Arc::new(PublisherHandle {
            handle: Mutex::new(publisher_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            message: PhantomData,
        })
    }

    /// Publishes a message.
    ///
    /// The [`MessageCow`] trait is implemented by any
    /// [`Message`] as well as any reference to a `Message`.
    ///
    /// The reason for allowing owned messages is that publishing owned messages can be more
    /// efficient in the case of idiomatic messages[^note].
    ///
    /// [^note]: See the [`Message`] trait for an explanation of "idiomatic".
    ///
    /// Hence, when a message will not be needed anymore after publishing, pass it by value.
    /// When a message will be needed again after publishing, pass it by reference, instead of cloning and passing by value.
    pub fn publish<'a, M: MessageCow<'a, T>>(&self, message: M) -> Result<(), RclReturnCode> {
        let rmw_message = T::into_rmw_message(message.into_cow());
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_publish(
                handle as *mut _,
                rmw_message.as_ref() as *const <T as Message>::RmwMsg as *mut _,
                core::ptr::null_mut(),
            )
        };
        ret.ok()
    }
}

/// Convenience trait for [`Publisher::publish`].
pub trait MessageCow<'a, T: Message> {
    fn into_cow(self) -> Cow<'a, T>;
}

impl<'a, T: Message> MessageCow<'a, T> for T {
    fn into_cow(self) -> Cow<'a, T> {
        Cow::Owned(self)
    }
}

impl<'a, T: Message> MessageCow<'a, T> for &'a T {
    fn into_cow(self) -> Cow<'a, T> {
        Cow::Borrowed(self)
    }
}
