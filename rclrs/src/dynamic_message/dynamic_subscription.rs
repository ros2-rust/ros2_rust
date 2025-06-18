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
    ENTITY_LIFECYCLE_MUTEX, Waitable,
    Node, QoSProfile, RclReturnCode, RclrsError, ToResult, NodeHandle, WorkerCommands, WaitableLifecycle, SubscriptionHandle,
};

/// Struct for receiving messages whose type is only known at runtime.
pub struct DynamicSubscription {
    /// This handle is used to access the data that rcl holds for this subscription.
    handle: Arc<SubscriptionHandle>,
    /// This allows us to replace the callback in the subscription task.
    ///
    /// Holding onto this sender will keep the subscription task alive. Once
    /// this sender is dropped, the subscription task will end itself.
    pub callback: Arc<Mutex<AnySubscriptionCallback<T, Scope::Payload>>>,
    // pub callback: Mutex<Box<dyn FnMut(DynamicMessage) + 'static + Send>>,
    /// Holding onto this keeps the waiter for this subscription alive in the
    /// wait set of the executor.
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
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
        topic: &str,
        topic_type: &str,
        qos: QoSProfile,
        callback: F,
        node_handle: &Arc<NodeHandle>,
        commands: &Arc<WorkerCommands>,
    ) -> Result<Arc<Self>, RclrsError>
    where
        F: FnMut(DynamicMessage) + 'static + Send,
    {
        // TODO(luca) a lot of duplication with nomral, refactor
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

        // SAFETY: No preconditions for this function.
        let mut rcl_subscription_options = unsafe { rcl_subscription_get_default_options() };
        rcl_subscription_options.qos = qos.into();
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        {
            let rcl_node = node_handle.rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            unsafe {
                // SAFETY:
                // * The rcl_subscription is zero-initialized as mandated by this function.
                // * The rcl_node is kept alive by the NodeHandle because it is a dependency of the subscription.
                // * The topic name and the options are copied by this function, so they can be dropped afterwards.
                // * The entity lifecycle mutex is locked to protect against the risk of global
                //   variables in the rmw implementation being unsafely modified during cleanup.
                rcl_subscription_init(
                    &mut rcl_subscription,
                    &*rcl_node,
                    type_support_ptr,
                    topic_c_string.as_ptr(),
                    &rcl_subscription_options,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(SubscriptionHandle {
            rcl_subscription: Mutex::new(rcl_subscription),
            node_handle: Arc::clone(node_handle),
        });

        let callback = Arc::new(Mutex::new(callback));

        let (waitable, lifecycle) = Waitable::new(
            Box::new(SubscriptionExecutable {
                handle: Arc::clone(&handle),
                callback: Arc::clone(&callback),
                commands: Arc::clone(commands),
            }),
            Some(Arc::clone(commands.get_guard_condition())),
        );
        commands.add_to_wait_set(waitable);

        Ok(Arc::new(Self {
            handle,
            callback,
            lifecycle,
            metadata,
            type_support_library,
        }))

        /*
        Ok(Self {
            handle,
            callback: Mutex::new(Box::new(callback)),
            metadata,
            type_support_library,
        })
        */
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
