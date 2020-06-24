use crate::rcl_bindings::*;
pub use rclrs_common::error::RCLStatusCode as RclError;

/// Error code that ROS nodes written in Rust should be returning from `main`
pub type RclResult<T = ()> = Result<T, RclError>;

pub(crate) trait ToRclResult {
    fn ok(&self) -> RclResult<()>;

    fn unwrap(&self) {
        self.ok().unwrap();
    }
}

impl ToRclResult for rcl_ret_t {
    fn ok(&self) -> RclResult<()> {
        if *self as u32 == RCL_RET_OK {
            Ok(())
        } else {
            Err(RclError::from(*self))
        }
    }
}
