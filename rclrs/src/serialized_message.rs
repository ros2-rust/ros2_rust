use crate::{rcl_bindings::*, RclrsError, ToResult};

/// A growable serialized message buffer.
///
/// This wraps `rcl_serialized_message_t` (aka `rmw_serialized_message_t`).
pub struct SerializedMessage {
    pub(crate) msg: rcl_serialized_message_t,
}

unsafe impl Send for SerializedMessage {}

impl SerializedMessage {
    /// Create a new serialized message buffer with the given capacity in bytes.
    pub fn new(capacity: usize) -> Result<Self, RclrsError> {
        unsafe {
            let mut msg = rcutils_get_zero_initialized_uint8_array();
            let allocator = rcutils_get_default_allocator();
            rcutils_uint8_array_init(&mut msg, capacity, &allocator).ok()?;
            Ok(Self { msg })
        }
    }

    /// Return the current serialized payload.
    pub fn as_bytes(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.msg.buffer, self.msg.buffer_length) }
    }

    /// Reset the length to 0 without changing capacity.
    pub fn clear(&mut self) {
        self.msg.buffer_length = 0;
    }
}

impl Drop for SerializedMessage {
    fn drop(&mut self) {
        unsafe {
            let _ = rcutils_uint8_array_fini(&mut self.msg);
        }
    }
}
