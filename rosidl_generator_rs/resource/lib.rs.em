extern crate rclrs_common;
extern crate libc;

@[if len(msg_specs) > 0]@
pub mod msg;
@[end if]@

@[if len(srv_specs) > 0]@
pub mod srv;
@[end if]@