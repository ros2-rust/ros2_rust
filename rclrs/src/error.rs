use anyhow::Result;

use crate::rcl_bindings::*;
pub use rclrs_common::error::RclError;

pub(crate) trait ToResult {
    fn ok(&self) -> Result<(), RclError>;

    fn unwrap(&self) {
        self.ok().unwrap();
    }
}

impl ToResult for rcl_ret_t {
    fn ok(&self) -> Result<(), RclError> {
        if *self as u32 == RCL_RET_OK {
            Ok(())
        } else {
            Err(RclError::from(*self))
        }
    }
}
