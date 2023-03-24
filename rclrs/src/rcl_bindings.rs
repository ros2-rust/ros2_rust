#![allow(dead_code)]
#![allow(deref_nullptr)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(clippy::all)]
#![allow(missing_docs)]

include!(concat!(env!("OUT_DIR"), "/rcl_bindings_generated.rs"));

pub const RMW_GID_STORAGE_SIZE: usize = rmw_gid_storage_size_constant;
