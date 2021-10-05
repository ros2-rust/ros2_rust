use crate::rcl_bindings::*;
pub use rclrs_common::error::{to_rcl_result, RclError};

pub(crate) trait ToResult {
    fn ok(&self) -> Result<(), RclError>;

    fn unwrap(&self) {
        self.ok().unwrap();
    }
}

impl ToResult for rcl_ret_t {
    fn ok(&self) -> Result<(), RclError> {
        to_rcl_result(*self as i32)
    }
}
