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
    if #[cfg(feature="generate_docs")] {
        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_allocator_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_arguments_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_client_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_clock_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_clock_type_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_context_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_guard_condition_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_names_and_types_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_node_options_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_node_params_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_node_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_params_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_publisher_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_ret_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_service_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_subscription_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_topic_endpoint_info_array_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_variant_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_wait_set_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcutils_string_array_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_message_info_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_names_and_types_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_qos_durability_policy_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_qos_history_policy_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_qos_liveliness_policy_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_qos_profile_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_qos_reliability_policy_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_request_id_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_time_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_topic_endpoint_info_array_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rosidl_message_type_support_t;

        pub const RMW_GID_STORAGE_SIZE: usize = 24;

        extern "C" {
            pub fn rcl_context_is_valid(context: *const rcl_context_t) -> bool;
        }
    } else {
        include!(concat!(env!("OUT_DIR"), "/rcl_bindings_generated.rs"));

        pub const RMW_GID_STORAGE_SIZE: usize = rmw_gid_storage_size_constant;
    }
}
