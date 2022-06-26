use crate::error::{RclrsError, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::Node;

use std::borrow::Cow;
use std::ffi::CStr;
use std::ffi::CString;
use std::marker::PhantomData;
use std::sync::Arc;

use parking_lot::Mutex;

use rosidl_runtime_rs::{Message, RmwMessage};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_publisher_t {}

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
    rcl_publisher_mtx: Mutex<rcl_publisher_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    message: PhantomData<T>,
}

impl<T> Drop for Publisher<T>
where
    T: Message,
{
    fn drop(&mut self) {
        unsafe {
            // SAFETY: No preconditions for this function (besides the arguments being valid).
            rcl_publisher_fini(
                self.rcl_publisher_mtx.get_mut(),
                &mut *self.rcl_node_mtx.lock(),
            );
        }
    }
}

impl<T> Publisher<T>
where
    T: Message,
{
    /// Creates a new `Publisher`.
    ///
    /// Node and namespace changes are always applied _before_ topic remapping.
    pub fn new(node: &Node, topic: &str, qos: QoSProfile) -> Result<Self, RclrsError>
    where
        T: Message,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_publisher = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;
        let rcl_node = &mut *node.rcl_node_mtx.lock();

        // SAFETY: No preconditions for this function.
        let mut publisher_options = unsafe { rcl_publisher_get_default_options() };
        publisher_options.qos = qos.into();
        unsafe {
            // SAFETY: The rcl_publisher is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            // TODO: type support?
            rcl_publisher_init(
                &mut rcl_publisher,
                rcl_node,
                type_support,
                topic_c_string.as_ptr(),
                &publisher_options,
            )
            .ok()?;
        }

        Ok(Self {
            rcl_publisher_mtx: Mutex::new(rcl_publisher),
            rcl_node_mtx: Arc::clone(&node.rcl_node_mtx),
            message: PhantomData,
        })
    }

    /// Returns the topic name of the publisher.
    ///
    /// This returns the topic name after remapping, so it is not necessarily the
    /// topic name which was used when creating the publisher.
    pub fn topic_name(&self) -> String {
        // SAFETY: No preconditions for the functions called.
        // The unsafe variables created get converted to safe types before being returned
        unsafe {
            let raw_topic_pointer = rcl_publisher_get_topic_name(&*self.rcl_publisher_mtx.lock());
            CStr::from_ptr(raw_topic_pointer)
                .to_string_lossy()
                .into_owned()
        }
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
        let rcl_publisher = &mut *self.rcl_publisher_mtx.lock();
        let ret = unsafe {
            // SAFETY: The message type is guaranteed to match the publisher type by the type system.
            // The message does not need to be valid beyond the duration of this function call.
            // The third argument is explictly allowed to be NULL.
            rcl_publish(
                rcl_publisher,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{create_node, Context, Publisher, QOS_PROFILE_DEFAULT};

    fn create_fixture(name: &str) -> (Context, Node) {
        let context =
            Context::new(vec![]).expect("Context instantiation is expected to be a success");
        let node =
            create_node(&context, name).expect("Node instantiation is expected to be a success");

        (context, node)
    }

    #[test]
    fn test_new_publisher() -> Result<(), RclrsError> {
        let (_, node) = create_fixture("test_new_publisher");
        let _ = Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT)?;

        Ok(())
    }

    #[test]
    fn test_publish_message() -> Result<(), RclrsError> {
        let (_, node) = create_fixture("test_publish_message");
        let publisher =
            Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT)?;
        let message = std_msgs::msg::String {
            data: "Hello world!".to_owned(),
        };
        publisher.publish(&message)
    }
}
