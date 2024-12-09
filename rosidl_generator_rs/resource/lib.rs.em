#![allow(non_camel_case_types)]
#![allow(clippy::derive_partial_eq_without_eq)]
#![allow(clippy::upper_case_acronyms)]

@[if len(msg_specs) > 0]@
pub mod msg;
@[end if]@

@[if len(srv_specs) > 0]@
pub mod srv;
@[end if]@

@[if len(action_specs) > 0]@
pub mod action;
@[end if]@
