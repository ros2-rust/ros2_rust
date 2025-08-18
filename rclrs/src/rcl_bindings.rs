#![allow(dead_code)]
#![allow(deref_nullptr)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
// Added to avoid clippy complaining about u128 types not being FFI safe
// Caused by https://github.com/ros2/ros2/issues/1374 in iron and newer
// See https://github.com/ros2-rust/ros2_rust/issues/320
#![allow(improper_ctypes)]
#![allow(improper_ctypes_definitions)]
#![allow(clippy::all)]
#![allow(missing_docs)]

cfg_if::cfg_if! {
    if #[cfg(feature="use_ros_shim")] {
        include!(
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/rcl_bindings_generated_",
                "rolling", // rolling will always be a valid ROS distro
                ".rs",
            )
        );

    } else {
        include!(
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/src/rcl_bindings_generated_",
                env!("ROS_DISTRO"),
                ".rs",
            )
        );

        pub const RMW_GID_STORAGE_SIZE: usize = rmw_gid_storage_size_constant;
    }
}

/// Wrapper around [`std::slice::from_raw_parts`] that accommodates the rcl
/// convention of providing a null pointer to represent empty arrays. This
/// violates the safety requirements of [`std::slice::from_raw_parts`].
///
/// # Safety
///
/// Behavior is undefined in all the same scenarios as [`slice::from_raw_parts`]
/// (see its safety section) except that null pointers are allowed and will
/// return a slice to an empty array.
pub(crate) unsafe fn rcl_from_raw_parts<'a, T>(data: *const T, len: usize) -> &'a [T] {
    if data.is_null() {
        &[]
    } else {
        // SAFETY: The user of this function is instructed to abide by all the
        // safety requirements of slice::from_raw_parts except for null pointer
        // values, which are checked above.
        unsafe { std::slice::from_raw_parts(data, len) }
    }
}
