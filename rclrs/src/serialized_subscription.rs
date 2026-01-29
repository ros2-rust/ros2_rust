use crate::{node::NodeHandle, rcl_bindings::*, MessageInfo, RclrsError, ENTITY_LIFECYCLE_MUTEX};
use std::{ptr, sync::Arc};

use crate::serialized_message::SerializedMessage;

/// A subscription which receives serialized ROS messages.
pub struct SerializedSubscription {
    pub(crate) handle: Arc<NodeHandle>,
    pub(crate) sub: rcl_subscription_t,
}

unsafe impl Send for SerializedSubscription {}
unsafe impl Sync for SerializedSubscription {}

impl Drop for SerializedSubscription {
    fn drop(&mut self) {
        let _context_lock = self.handle.context_handle.rcl_context.lock().unwrap();
        let mut node = self.handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        unsafe {
            let _ = rcl_subscription_fini(&mut self.sub, &mut *node);
        }
    }
}

impl SerializedSubscription {
    /// Take a serialized (CDR) message.
    ///
    /// Returns `Ok(None)` when no message is available.
    pub fn take(&self, buf: &mut SerializedMessage) -> Result<Option<MessageInfo>, RclrsError> {
        unsafe {
            let mut info: rmw_message_info_t = std::mem::zeroed();
            let rc =
                rcl_take_serialized_message(&self.sub, &mut buf.msg, &mut info, ptr::null_mut());
            if rc != 0 {
                // No message available or error. The rmw/rcl API uses negative codes for "take failed".
                return Ok(None);
            }
            Ok(Some(MessageInfo::from_rmw_message_info(&info)))
        }
    }
}
