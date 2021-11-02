use crate::rcl_bindings::*;
pub use rclrs_common::error::{to_rcl_result, RclReturnCode};

pub(crate) trait ToResult {
    fn ok(&self) -> Result<(), RclReturnCode>;

    fn unwrap(&self) {
        self.ok().unwrap();
    }
}

impl ToResult for rcl_ret_t {
    fn ok(&self) -> Result<(), RclReturnCode> {
        to_rcl_result(*self as i32)
    }
}
