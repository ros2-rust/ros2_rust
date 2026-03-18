use std::{
    any::Any,
    ffi::CString,
    sync::{Arc, Mutex},
};

use crate::{
    dynamic_message::{get_type_support_handle, get_type_support_library, MessageTypeName},
    rcl_bindings::*,
    MessageInfo, NodeHandle, RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, RclrsError,
    RclrsErrorFilter, ReadyKind, SubscriptionHandle, SubscriptionOptions, ToResult, Waitable,
    WaitableLifecycle, WorkerCommands, ENTITY_LIFECYCLE_MUTEX,
};

/// Struct for receiving serialized ROS 2 messages as raw bytes.
///
/// Create a serialized subscription using [`NodeState::create_serialized_subscription`].
pub type SerializedSubscription = Arc<SerializedSubscriptionState>;

struct SerializedSubscriptionExecutable {
    handle: Arc<SubscriptionHandle>,
    callback: Arc<Mutex<NodeSerializedSubscriptionCallback>>,
}

struct SerializedMessageBuffer {
    inner: rcl_serialized_message_t,
}

pub(crate) struct NodeSerializedSubscriptionCallback(
    Box<dyn Fn(Vec<u8>, MessageInfo) + Send + Sync>,
);

impl NodeSerializedSubscriptionCallback {
    pub(crate) fn new(f: impl Fn(Vec<u8>, MessageInfo) + Send + Sync + 'static) -> Self {
        Self(Box::new(f))
    }
}

impl SerializedMessageBuffer {
    fn new() -> Self {
        Self {
            inner: unsafe { rcutils_get_zero_initialized_uint8_array() },
        }
    }

    fn take_bytes(&self) -> Vec<u8> {
        unsafe { std::slice::from_raw_parts(self.inner.buffer, self.inner.buffer_length) }.to_vec()
    }
}

impl Drop for SerializedMessageBuffer {
    fn drop(&mut self) {
        if self.inner.allocator.allocate.is_some() {
            unsafe {
                rcutils_uint8_array_fini(&mut self.inner);
            }
        }
    }
}

impl SerializedSubscriptionExecutable {
    fn take(&self) -> Result<(Vec<u8>, MessageInfo), RclrsError> {
        let mut serialized_message = SerializedMessageBuffer::new();
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        let rcl_subscription = &mut *self.handle.lock();

        unsafe {
            rcl_take_serialized_message(
                rcl_subscription,
                &mut serialized_message.inner,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?;
        };

        Ok((
            serialized_message.take_bytes(),
            MessageInfo::from_rmw_message_info(&message_info),
        ))
    }
}

impl RclPrimitive for SerializedSubscriptionExecutable {
    unsafe fn execute(
        &mut self,
        ready: ReadyKind,
        _payload: &mut dyn Any,
    ) -> Result<(), RclrsError> {
        ready.for_basic()?;
        let evaluate = || {
            let (msg, msg_info) = self.take()?;
            (self.callback.lock().unwrap().0)(msg, msg_info);
            Ok(())
        };

        evaluate().take_failed_ok()
    }

    fn kind(&self) -> RclPrimitiveKind {
        RclPrimitiveKind::Subscription
    }

    fn handle(&self) -> RclPrimitiveHandle<'_> {
        RclPrimitiveHandle::Subscription(self.handle.lock())
    }
}

/// Inner state of a [`SerializedSubscription`].
pub struct SerializedSubscriptionState {
    handle: Arc<SubscriptionHandle>,
    #[allow(unused)]
    callback: Arc<Mutex<NodeSerializedSubscriptionCallback>>,
    #[allow(unused)]
    lifecycle: WaitableLifecycle,
    #[allow(dead_code)]
    type_support_library: Arc<libloading::Library>,
}

impl SerializedSubscriptionState {
    pub(crate) fn create<'a>(
        topic_type: MessageTypeName,
        options: impl Into<SubscriptionOptions<'a>>,
        callback: NodeSerializedSubscriptionCallback,
        node_handle: &Arc<NodeHandle>,
        commands: &Arc<WorkerCommands>,
    ) -> Result<Arc<Self>, RclrsError> {
        let SubscriptionOptions { topic, qos } = options.into();
        let type_support_library =
            get_type_support_library(&topic_type.package_name, "rosidl_typesupport_c")?;
        let type_support_ptr = unsafe {
            get_type_support_handle(
                type_support_library.as_ref(),
                "rosidl_typesupport_c",
                &topic_type,
            )?
        };

        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        let mut rcl_subscription_options = unsafe { rcl_subscription_get_default_options() };
        rcl_subscription_options.qos = qos.into();
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        {
            let rcl_node = node_handle.rcl_node.lock()?;
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock()?;
            unsafe {
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
            Box::new(SerializedSubscriptionExecutable {
                handle: Arc::clone(&handle),
                callback: Arc::clone(&callback),
            }),
            Some(Arc::clone(commands.get_guard_condition())),
        );
        commands.add_to_wait_set(waitable);

        Ok(Arc::new(Self {
            handle,
            callback,
            lifecycle,
            type_support_library,
        }))
    }

    /// Returns the topic name of the subscription.
    pub fn topic_name(&self) -> String {
        self.handle.topic_name()
    }
}
