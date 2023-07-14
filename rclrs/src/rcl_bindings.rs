#![allow(dead_code)]
#![allow(deref_nullptr)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
// Added to avoid clippy complaining about u128 types not being FFI safe
// Caused by https://github.com/ros2/ros2/issues/1374 in iron and newer
// See https://github.com/ros2-rust/ros2_rust/issues/320
#![allow(improper_ctypes)]
#![allow(clippy::all)]
#![allow(missing_docs)]

include!(concat!(env!("OUT_DIR"), "/rcl_bindings_generated.rs"));

pub const RMW_GID_STORAGE_SIZE: usize = rmw_gid_storage_size_constant;
