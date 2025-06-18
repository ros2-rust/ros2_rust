use std::ffi::CStr;
use std::ffi::CString;
use std::sync::{Arc, Mutex};

use super::{
    get_type_support_handle, get_type_support_library, DynamicMessage, DynamicMessageError,
    DynamicMessageMetadata,
};
use crate::error::{RclrsError, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::NodeHandle;

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
pub struct DynamicPublisher {
    rcl_publisher_mtx: Mutex<rcl_publisher_t>,
    node_handle: Arc<NodeHandle>,
    metadata: DynamicMessageMetadata,
    // This is the regular type support library, not the introspection one.
    #[allow(dead_code)]
    type_support_library: Arc<libloading::Library>,
}

impl Drop for DynamicPublisher {
    fn drop(&mut self) {
        let mut rcl_node = self.node_handle.rcl_node.lock().unwrap();
        unsafe {
            // SAFETY: No preconditions for this function (besides the arguments being valid).
            rcl_publisher_fini(
                self.rcl_publisher_mtx.get_mut().unwrap(),
                &mut *rcl_node,
            );
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
// unsafe impl Send for DynamicPublisher {}
// SAFETY: The type_support_ptr prevents the default Sync impl.
// rosidl_message_type_support_t is a read-only type without interior mutability.
// unsafe impl Sync for DynamicPublisher {}

impl DynamicPublisher {
    /// Creates a new `DynamicPublisher`.
    ///
    /// Node and namespace changes are always applied _before_ topic remapping.
    pub fn new(
        node_handle: &Arc<NodeHandle>,
        topic: &str,
        topic_type: &str,
        qos: QoSProfile,
    ) -> Result<Self, RclrsError> {
        // This loads the introspection type support library.
        let metadata = DynamicMessageMetadata::new(topic_type)?;
        // However, we also need the regular type support library –
        // the rosidl_typesupport_c one.
        let message_type = &metadata.message_type;
        let type_support_library =
            get_type_support_library(&message_type.package_name, "rosidl_typesupport_c")?;
        // SAFETY: The symbol type of the type support getter function can be trusted
        // assuming the install dir hasn't been tampered with.
        // The pointer returned by this function is kept valid by keeping the library loaded.
        let type_support_ptr = unsafe {
            get_type_support_handle(
                type_support_library.as_ref(),
                "rosidl_typesupport_c",
                message_type,
            )?
        };

        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_publisher = unsafe { rcl_get_zero_initialized_publisher() };
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;
        let rcl_node = node_handle.rcl_node.lock().unwrap();

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
                &*rcl_node,
                type_support_ptr,
                topic_c_string.as_ptr(),
                &publisher_options,
            )
            .ok()?;
        }

        Ok(Self {
            rcl_publisher_mtx: Mutex::new(rcl_publisher),
            node_handle: node_handle.clone(),
            metadata,
            type_support_library,
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
            let raw_topic_pointer =
                rcl_publisher_get_topic_name(&*self.rcl_publisher_mtx.lock().unwrap());
            CStr::from_ptr(raw_topic_pointer)
                .to_string_lossy()
                .into_owned()
        }
    }

    /// Publishes a message.
    ///
    /// Calling `publish()` is a potentially blocking call, see [this issue][1] for details.
    ///
    /// [1]: https://github.com/ros2/ros2/issues/255
    pub fn publish(&self, mut message: DynamicMessage) -> Result<(), RclrsError> {
        if message.metadata.message_type != self.metadata.message_type {
            return Err(DynamicMessageError::MessageTypeMismatch.into());
        }
        let rcl_publisher = &mut *self.rcl_publisher_mtx.lock().unwrap();
        unsafe {
            // SAFETY: The message type is guaranteed to match the publisher type by the type system.
            // The message does not need to be valid beyond the duration of this function call.
            // The third argument is explictly allowed to be NULL.
            rcl_publish(
                rcl_publisher,
                message.storage.as_mut_ptr() as *mut _,
                std::ptr::null_mut(),
            )
            .ok()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_send<T: Send>() {}
    fn assert_sync<T: Sync>() {}

    #[test]
    fn dynamic_publisher_is_sync_and_send() {
        assert_send::<DynamicPublisher>();
        assert_sync::<DynamicPublisher>();
    }
}
