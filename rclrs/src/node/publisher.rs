use crate::error::{RclrsError, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::Node;

use std::borrow::Cow;
use std::ffi::CString;
use std::marker::PhantomData;
use std::sync::Arc;

use parking_lot::{Mutex, MutexGuard};

use rosidl_runtime_rs::{Message, RmwMessage};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_publisher_t {}

pub(crate) struct PublisherHandle {
    handle: Mutex<rcl_publisher_t>,
    node_handle: Arc<Mutex<rcl_node_t>>,
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
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_publisher_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Struct for sending messages of type `T`.
///
/// Multiple publishers can be created for the same topic, in different nodes or the same node.
///
/// The underlying RMW will decide on the concrete delivery mechanism (network stack, shared
/// memory, or intraprocess).
///
/// Sending messages does not require calling [`spin`][1] on the publisher's node.
///
/// [1]: crate::spin
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
    /// Creates a new `Publisher`.
    ///
    /// Node and namespace changes are always applied _before_ topic remapping.
    ///
    /// # Panics
    /// When the topic contains interior null bytes.
    pub fn new(node: &Node, topic: &str, qos: QoSProfile) -> Result<Self, RclrsError>
    where
        T: Message,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

        // SAFETY: No preconditions for this function.
        let mut publisher_options = unsafe { rcl_publisher_get_default_options() };
        publisher_options.qos = qos.into();
        unsafe {
            // SAFETY: The publisher handle is zero-initialized as expected by this function.
            // The node handle is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            // TODO: type support?
            rcl_publisher_init(
                &mut publisher_handle,
                node_handle,
                type_support,
                topic_c_string.as_ptr(),
                &publisher_options,
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
    ///
    /// Calling `publish()` is a potentially blocking call, see [this issue][1] for details.
    ///
    /// [1]: https://github.com/ros2/ros2/issues/255
    pub fn publish<'a, M: MessageCow<'a, T>>(&self, message: M) -> Result<(), RclrsError> {
        let rmw_message = T::into_rmw_message(message.into_cow());
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            // SAFETY: The message type is guaranteed to match the publisher type by the type system.
            // The message does not need to be valid beyond the duration of this function call.
            // The third argument is explictly allowed to be NULL.
            rcl_publish(
                handle,
                rmw_message.as_ref() as *const <T as Message>::RmwMsg as *mut _,
                std::ptr::null_mut(),
            )
        };
        ret.ok()
    }
}

/// Convenience trait for [`Publisher::publish`].
pub trait MessageCow<'a, T: Message> {
    /// Wrap the owned or borrowed message in a `Cow`.
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
