use std::boxed::Box;
use std::ffi::{CStr, CString};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use super::{
    get_type_support_handle, get_type_support_library, DynamicMessage, DynamicMessageMetadata,
    MessageStructure,
};
use crate::rcl_bindings::*;
use crate::{
    Node, QoSProfile, RclReturnCode, RclrsError, SubscriptionBase, SubscriptionHandle, ToResult,
};

/// Struct for receiving messages whose type is only known at runtime.
pub struct DynamicSubscription {
    pub(crate) handle: Arc<SubscriptionHandle>,
    /// The callback function that runs when a message was received.
    pub callback: Mutex<Box<dyn FnMut(DynamicMessage) + 'static + Send>>,
    metadata: DynamicMessageMetadata,
    // This is the regular type support library, not the introspection one.
    #[allow(dead_code)]
    type_support_library: Arc<libloading::Library>,
}

impl DynamicSubscription {
    /// Creates a new dynamic subscription.
    ///
    /// This is not a public function, by the same rationale as `Subscription::new()`.
    pub(crate) fn new<F>(
        node: &Node,
        topic: &str,
        topic_type: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Self, RclrsError>
    where
        F: FnMut(DynamicMessage) + 'static + Send,
    {
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

        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;
        let rcl_node = &mut *node.rcl_node_mtx.lock().unwrap();

        // SAFETY: No preconditions for this function.
        let mut subscription_options = unsafe { rcl_subscription_get_default_options() };
        subscription_options.qos = qos.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        unsafe {
            // SAFETY: The rcl_subscription is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            // TODO: type support?
            rcl_subscription_init(
                &mut rcl_subscription,
                rcl_node,
                type_support_ptr,
                topic_c_string.as_ptr(),
                &subscription_options,
            )
            .ok()?;
        }

        let handle = Arc::new(SubscriptionHandle {
            rcl_subscription_mtx: Mutex::new(rcl_subscription),
            rcl_node_mtx: node.rcl_node_mtx.clone(),
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(Box::new(callback)),
            metadata,
            type_support_library,
        })
    }

    /// Returns the topic name of the subscription.
    ///
    /// This returns the topic name after remapping, so it is not necessarily the
    /// topic name which was used when creating the subscription.
    pub fn topic_name(&self) -> String {
        // SAFETY: No preconditions for the function used
        // The unsafe variables get converted to safe types before being returned
        unsafe {
            let raw_topic_pointer = rcl_subscription_get_topic_name(&*self.handle.lock());
            CStr::from_ptr(raw_topic_pointer)
                .to_string_lossy()
                .into_owned()
        }
    }

    /// Returns a description of the message structure.
    pub fn structure(&self) -> &MessageStructure {
        &self.metadata.structure
    }

    /// Fetches a new message.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1].
    ///
    /// [1]: crate::RclrsError
    //
    // ```text
    // +-------------+
    // | rclrs::take |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rcl_take   |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rmw_take   |
    // +-------------+
    // ```
    pub fn take(&self) -> Result<DynamicMessage, RclrsError> {
        let mut dynamic_message = self.metadata.create()?;
        let rmw_message = dynamic_message.storage.as_mut_ptr();
        let rcl_subscription = &mut *self.handle.lock();
        unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                rcl_subscription,
                rmw_message as *mut _,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
            .ok()?
        };
        Ok(dynamic_message)
    }
}

impl SubscriptionBase for DynamicSubscription {
    fn handle(&self) -> &SubscriptionHandle {
        &self.handle
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let msg = match self.take() {
            Ok(msg) => msg,
            Err(RclrsError::RclError {
                code: RclReturnCode::SubscriptionTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // subscription was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        (*self.callback.lock().unwrap())(msg);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_send<T: Send>() {}
    fn assert_sync<T: Sync>() {}

    #[test]
    fn dynamic_subscription_is_sync_and_send() {
        assert_send::<DynamicSubscription>();
        assert_sync::<DynamicSubscription>();
    }
}
