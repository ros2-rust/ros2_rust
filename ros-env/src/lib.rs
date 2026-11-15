// Without the shim, this crate is generated code, so lints do not apply.
#![cfg_attr(not(feature = "use_ros_shim"), allow(clippy::all, rustdoc::all))]

#[cfg(feature = "use_ros_shim")]
#[allow(missing_docs)]
mod shim;
#[cfg(feature = "use_ros_shim")]
pub use shim::*;

#[cfg(not(feature = "use_ros_shim"))]
include!(concat!(env!("OUT_DIR"), "/interfaces.rs"));
