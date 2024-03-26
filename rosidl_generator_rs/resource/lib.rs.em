#![allow(non_camel_case_types)]

@[if len(msg_specs) > 0]@
pub mod msg;
@[end if]@

@[if len(srv_specs) > 0]@
pub mod srv;
@[end if]@

@[if len(action_specs) > 0]@
pub mod action;
@[end if]@
