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
        pub struct rcl_wait_set_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_clock_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_node_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_context_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_guard_condition_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_subscription_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_request_id_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_service_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_client_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_publisher_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_time_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_arguments_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_allocator_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_ret_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_clock_type_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_node_options_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_names_and_types_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_topic_endpoint_info_array_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcutils_string_array_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_names_and_types_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_topic_endpoint_info_array_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_node_params_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_variant_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_params_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rmw_message_info_t;

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
        pub struct rosidl_message_type_support_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rosidl_service_type_support_t;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rosidl_typesupport_introspection_c__MessageMembers_s;

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcl_jump_callback_info_t;

        pub const RMW_GID_STORAGE_SIZE: usize = 24;

        extern "C" {
            pub fn rcl_context_is_valid(context: *const rcl_context_t) -> bool;
        }

        extern "C" {
            pub fn rcl_arguments_get_count_unparsed(args: *const rcl_arguments_t) -> ::std::os::raw::c_int;
        }

        extern "C" {
            pub fn rcl_arguments_get_unparsed(
                args: *const rcl_arguments_t,
                allocator: rcl_allocator_t,
                output_unparsed_indices: *mut *mut ::std::os::raw::c_int,
            ) -> rcl_ret_t;
        }

        extern "C" {
            pub fn rcl_get_service_names_and_types_by_node(
                node: *const rcl_node_t,
                allocator: *mut rcl_allocator_t,
                node_name: *const ::std::os::raw::c_char,
                node_namespace: *const ::std::os::raw::c_char,
                service_names_and_types: *mut rcl_names_and_types_t,
            ) -> rcl_ret_t;
        }
        extern "C" {
            pub fn rcl_get_client_names_and_types_by_node(
                node: *const rcl_node_t,
                allocator: *mut rcl_allocator_t,
                node_name: *const ::std::os::raw::c_char,
                node_namespace: *const ::std::os::raw::c_char,
                service_names_and_types: *mut rcl_names_and_types_t,
            ) -> rcl_ret_t;
        }
        extern "C" {
            pub fn rcl_get_publishers_info_by_topic(
                node: *const rcl_node_t,
                allocator: *mut rcutils_allocator_t,
                topic_name: *const ::std::os::raw::c_char,
                no_mangle: bool,
                publishers_info: *mut rcl_topic_endpoint_info_array_t,
            ) -> rcl_ret_t;
        }
        extern "C" {
            pub fn rcl_get_subscriptions_info_by_topic(
                node: *const rcl_node_t,
                allocator: *mut rcutils_allocator_t,
                topic_name: *const ::std::os::raw::c_char,
                no_mangle: bool,
                subscriptions_info: *mut rcl_topic_endpoint_info_array_t,
            ) -> rcl_ret_t;
        }
        extern "C" {
            pub fn rcl_node_get_name(node: *const rcl_node_t) -> *const ::std::os::raw::c_char;
        }
        extern "C" {
            pub fn rcl_node_get_namespace(node: *const rcl_node_t) -> *const ::std::os::raw::c_char;
        }
        extern "C" {
            pub fn rcl_node_get_fully_qualified_name(
                node: *const rcl_node_t,
            ) -> *const ::std::os::raw::c_char;
        }

        #[repr(C)]
        #[derive(Debug)]
        pub struct rcutils_allocator_t;
    } else {
        include!(concat!(env!("OUT_DIR"), "/rcl_bindings_generated.rs"));

        pub const RMW_GID_STORAGE_SIZE: usize = rmw_gid_storage_size_constant;
    }
}
