pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__FloatingPointRange(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__FloatingPointRange__init(msg: *mut FloatingPointRange) -> bool;
        fn rcl_interfaces__msg__FloatingPointRange__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<FloatingPointRange>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__FloatingPointRange__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<FloatingPointRange>,
        );
        fn rcl_interfaces__msg__FloatingPointRange__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<FloatingPointRange>,
            out_seq: *mut rosidl_runtime_rs::Sequence<FloatingPointRange>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__FloatingPointRange
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct FloatingPointRange {
        pub from_value: f64,
        pub to_value: f64,
        pub step: f64,
    }

    impl Default for FloatingPointRange {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__FloatingPointRange__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__FloatingPointRange__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for FloatingPointRange {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__FloatingPointRange__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__FloatingPointRange__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__FloatingPointRange__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for FloatingPointRange {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for FloatingPointRange
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/FloatingPointRange";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__FloatingPointRange()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__IntegerRange(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__IntegerRange__init(msg: *mut IntegerRange) -> bool;
        fn rcl_interfaces__msg__IntegerRange__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<IntegerRange>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__IntegerRange__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<IntegerRange>,
        );
        fn rcl_interfaces__msg__IntegerRange__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<IntegerRange>,
            out_seq: *mut rosidl_runtime_rs::Sequence<IntegerRange>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__IntegerRange
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct IntegerRange {
        pub from_value: i64,
        pub to_value: i64,
        pub step: u64,
    }

    impl Default for IntegerRange {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__IntegerRange__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__IntegerRange__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for IntegerRange {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__IntegerRange__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__IntegerRange__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__IntegerRange__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for IntegerRange {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for IntegerRange
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/IntegerRange";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__IntegerRange()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ListParametersResult(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__ListParametersResult__init(msg: *mut ListParametersResult) -> bool;
        fn rcl_interfaces__msg__ListParametersResult__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<ListParametersResult>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__ListParametersResult__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<ListParametersResult>,
        );
        fn rcl_interfaces__msg__ListParametersResult__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<ListParametersResult>,
            out_seq: *mut rosidl_runtime_rs::Sequence<ListParametersResult>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__ListParametersResult
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct ListParametersResult {
        pub names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
        pub prefixes: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    }

    impl Default for ListParametersResult {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__ListParametersResult__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__ListParametersResult__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for ListParametersResult {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__ListParametersResult__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ListParametersResult__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__ListParametersResult__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for ListParametersResult {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for ListParametersResult
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/ListParametersResult";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ListParametersResult()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__Log(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__Log__init(msg: *mut Log) -> bool;
        fn rcl_interfaces__msg__Log__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Log>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__Log__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Log>);
        fn rcl_interfaces__msg__Log__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Log>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Log>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__Log
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Log {
        pub stamp: crate::vendor::builtin_interfaces::msg::rmw::Time,
        pub level: u8,
        pub name: rosidl_runtime_rs::String,
        pub msg: rosidl_runtime_rs::String,
        pub file: rosidl_runtime_rs::String,
        pub function: rosidl_runtime_rs::String,
        pub line: u32,
    }

    impl Log {
        /// Debug is for pedantic information, which is useful when debugging issues.
        pub const DEBUG: u8 = 10;
        /// Info is the standard informational level and is used to report expected
        /// information.
        pub const INFO: u8 = 20;
        /// Warning is for information that may potentially cause issues or possibly unexpected
        /// behavior.
        pub const WARN: u8 = 30;
        /// Error is for information that this node cannot resolve.
        pub const ERROR: u8 = 40;
        /// Information about a impending node shutdown.
        pub const FATAL: u8 = 50;
    }

    impl Default for Log {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__Log__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__Log__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Log {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__Log__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__Log__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__Log__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Log {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for Log
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/Log";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__Log()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterDescriptor(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__ParameterDescriptor__init(msg: *mut ParameterDescriptor) -> bool;
        fn rcl_interfaces__msg__ParameterDescriptor__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterDescriptor>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__ParameterDescriptor__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterDescriptor>,
        );
        fn rcl_interfaces__msg__ParameterDescriptor__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<ParameterDescriptor>,
            out_seq: *mut rosidl_runtime_rs::Sequence<ParameterDescriptor>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__ParameterDescriptor
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct ParameterDescriptor {
        pub name: rosidl_runtime_rs::String,
        pub type_: u8,
        pub description: rosidl_runtime_rs::String,
        pub additional_constraints: rosidl_runtime_rs::String,
        pub read_only: bool,
        pub dynamic_typing: bool,
        pub floating_point_range: rosidl_runtime_rs::BoundedSequence<
            crate::vendor::rcl_interfaces::msg::rmw::FloatingPointRange,
            1,
        >,
        pub integer_range: rosidl_runtime_rs::BoundedSequence<
            crate::vendor::rcl_interfaces::msg::rmw::IntegerRange,
            1,
        >,
    }

    impl Default for ParameterDescriptor {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__ParameterDescriptor__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__ParameterDescriptor__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for ParameterDescriptor {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterDescriptor__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterDescriptor__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__ParameterDescriptor__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for ParameterDescriptor {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for ParameterDescriptor
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/ParameterDescriptor";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterDescriptor()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterEventDescriptors(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__ParameterEventDescriptors__init(
            msg: *mut ParameterEventDescriptors,
        ) -> bool;
        fn rcl_interfaces__msg__ParameterEventDescriptors__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterEventDescriptors>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__ParameterEventDescriptors__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterEventDescriptors>,
        );
        fn rcl_interfaces__msg__ParameterEventDescriptors__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<ParameterEventDescriptors>,
            out_seq: *mut rosidl_runtime_rs::Sequence<ParameterEventDescriptors>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__ParameterEventDescriptors
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct ParameterEventDescriptors {
        pub new_parameters: rosidl_runtime_rs::Sequence<
            crate::vendor::rcl_interfaces::msg::rmw::ParameterDescriptor,
        >,
        pub changed_parameters: rosidl_runtime_rs::Sequence<
            crate::vendor::rcl_interfaces::msg::rmw::ParameterDescriptor,
        >,
        pub deleted_parameters: rosidl_runtime_rs::Sequence<
            crate::vendor::rcl_interfaces::msg::rmw::ParameterDescriptor,
        >,
    }

    impl Default for ParameterEventDescriptors {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__ParameterEventDescriptors__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__ParameterEventDescriptors__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for ParameterEventDescriptors {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__ParameterEventDescriptors__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterEventDescriptors__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__ParameterEventDescriptors__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for ParameterEventDescriptors {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for ParameterEventDescriptors
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/ParameterEventDescriptors";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterEventDescriptors()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterEvent(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__ParameterEvent__init(msg: *mut ParameterEvent) -> bool;
        fn rcl_interfaces__msg__ParameterEvent__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterEvent>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__ParameterEvent__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterEvent>,
        );
        fn rcl_interfaces__msg__ParameterEvent__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<ParameterEvent>,
            out_seq: *mut rosidl_runtime_rs::Sequence<ParameterEvent>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__ParameterEvent
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct ParameterEvent {
        pub stamp: crate::vendor::builtin_interfaces::msg::rmw::Time,
        pub node: rosidl_runtime_rs::String,
        pub new_parameters:
            rosidl_runtime_rs::Sequence<crate::vendor::rcl_interfaces::msg::rmw::Parameter>,
        pub changed_parameters:
            rosidl_runtime_rs::Sequence<crate::vendor::rcl_interfaces::msg::rmw::Parameter>,
        pub deleted_parameters:
            rosidl_runtime_rs::Sequence<crate::vendor::rcl_interfaces::msg::rmw::Parameter>,
    }

    impl Default for ParameterEvent {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__ParameterEvent__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__ParameterEvent__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for ParameterEvent {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterEvent__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterEvent__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__ParameterEvent__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for ParameterEvent {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for ParameterEvent
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/ParameterEvent";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterEvent()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__Parameter(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__Parameter__init(msg: *mut Parameter) -> bool;
        fn rcl_interfaces__msg__Parameter__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Parameter>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__Parameter__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Parameter>,
        );
        fn rcl_interfaces__msg__Parameter__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Parameter>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Parameter>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__Parameter
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Parameter {
        pub name: rosidl_runtime_rs::String,
        pub value: crate::vendor::rcl_interfaces::msg::rmw::ParameterValue,
    }

    impl Default for Parameter {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__Parameter__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__Parameter__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Parameter {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__Parameter__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__Parameter__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__Parameter__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Parameter {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for Parameter
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/Parameter";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__Parameter()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterType(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__ParameterType__init(msg: *mut ParameterType) -> bool;
        fn rcl_interfaces__msg__ParameterType__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterType>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__ParameterType__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterType>,
        );
        fn rcl_interfaces__msg__ParameterType__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<ParameterType>,
            out_seq: *mut rosidl_runtime_rs::Sequence<ParameterType>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__ParameterType
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct ParameterType {
        pub structure_needs_at_least_one_member: u8,
    }

    impl ParameterType {
        /// Default value, which implies this is not a valid parameter.
        pub const PARAMETER_NOT_SET: u8 = 0;
        pub const PARAMETER_BOOL: u8 = 1;
        pub const PARAMETER_INTEGER: u8 = 2;
        pub const PARAMETER_DOUBLE: u8 = 3;
        pub const PARAMETER_STRING: u8 = 4;
        pub const PARAMETER_BYTE_ARRAY: u8 = 5;
        pub const PARAMETER_BOOL_ARRAY: u8 = 6;
        pub const PARAMETER_INTEGER_ARRAY: u8 = 7;
        pub const PARAMETER_DOUBLE_ARRAY: u8 = 8;
        pub const PARAMETER_STRING_ARRAY: u8 = 9;
    }

    impl Default for ParameterType {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__ParameterType__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__ParameterType__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for ParameterType {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterType__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterType__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterType__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for ParameterType {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for ParameterType
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/ParameterType";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterType()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterValue(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__ParameterValue__init(msg: *mut ParameterValue) -> bool;
        fn rcl_interfaces__msg__ParameterValue__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterValue>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__ParameterValue__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<ParameterValue>,
        );
        fn rcl_interfaces__msg__ParameterValue__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<ParameterValue>,
            out_seq: *mut rosidl_runtime_rs::Sequence<ParameterValue>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__ParameterValue
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct ParameterValue {
        pub type_: u8,
        pub bool_value: bool,
        pub integer_value: i64,
        pub double_value: f64,
        pub string_value: rosidl_runtime_rs::String,
        pub byte_array_value: rosidl_runtime_rs::Sequence<u8>,
        pub bool_array_value: rosidl_runtime_rs::Sequence<bool>,
        pub integer_array_value: rosidl_runtime_rs::Sequence<i64>,
        pub double_array_value: rosidl_runtime_rs::Sequence<f64>,
        pub string_array_value: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    }

    impl Default for ParameterValue {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__ParameterValue__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__ParameterValue__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for ParameterValue {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterValue__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__ParameterValue__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__ParameterValue__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for ParameterValue {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for ParameterValue
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/ParameterValue";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterValue()
            }
        }
    }

    #[link(name = "rcl_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__SetParametersResult(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "rcl_interfaces__rosidl_generator_c")]
    extern "C" {
        fn rcl_interfaces__msg__SetParametersResult__init(msg: *mut SetParametersResult) -> bool;
        fn rcl_interfaces__msg__SetParametersResult__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<SetParametersResult>,
            size: usize,
        ) -> bool;
        fn rcl_interfaces__msg__SetParametersResult__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<SetParametersResult>,
        );
        fn rcl_interfaces__msg__SetParametersResult__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<SetParametersResult>,
            out_seq: *mut rosidl_runtime_rs::Sequence<SetParametersResult>,
        ) -> bool;
    }

    // Corresponds to rcl_interfaces__msg__SetParametersResult
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct SetParametersResult {
        pub successful: bool,
        pub reason: rosidl_runtime_rs::String,
    }

    impl Default for SetParametersResult {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !rcl_interfaces__msg__SetParametersResult__init(&mut msg as *mut _) {
                    panic!("Call to rcl_interfaces__msg__SetParametersResult__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for SetParametersResult {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__SetParametersResult__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { rcl_interfaces__msg__SetParametersResult__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                rcl_interfaces__msg__SetParametersResult__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for SetParametersResult {
        type RmwMsg = Self;
        fn into_rmw_message(
            msg_cow: std::borrow::Cow<'_, Self>,
        ) -> std::borrow::Cow<'_, Self::RmwMsg> {
            msg_cow
        }
        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg
        }
    }

    impl rosidl_runtime_rs::RmwMessage for SetParametersResult
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "rcl_interfaces/msg/SetParametersResult";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__SetParametersResult()
            }
        }
    }
} // mod rmw

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FloatingPointRange {
    pub from_value: f64,
    pub to_value: f64,
    pub step: f64,
}

impl Default for FloatingPointRange {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::FloatingPointRange::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for FloatingPointRange {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::FloatingPointRange;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                from_value: msg.from_value,
                to_value: msg.to_value,
                step: msg.step,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                from_value: msg.from_value,
                to_value: msg.to_value,
                step: msg.step,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            from_value: msg.from_value,
            to_value: msg.to_value,
            step: msg.step,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IntegerRange {
    pub from_value: i64,
    pub to_value: i64,
    pub step: u64,
}

impl Default for IntegerRange {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::IntegerRange::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for IntegerRange {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::IntegerRange;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                from_value: msg.from_value,
                to_value: msg.to_value,
                step: msg.step,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                from_value: msg.from_value,
                to_value: msg.to_value,
                step: msg.step,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            from_value: msg.from_value,
            to_value: msg.to_value,
            step: msg.step,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ListParametersResult {
    pub names: Vec<std::string::String>,
    pub prefixes: Vec<std::string::String>,
}

impl Default for ListParametersResult {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::ListParametersResult::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for ListParametersResult {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::ListParametersResult;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                names: msg
                    .names
                    .into_iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
                prefixes: msg
                    .prefixes
                    .into_iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                names: msg.names.iter().map(|elem| elem.as_str().into()).collect(),
                prefixes: msg
                    .prefixes
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            names: msg.names.into_iter().map(|elem| elem.to_string()).collect(),
            prefixes: msg
                .prefixes
                .into_iter()
                .map(|elem| elem.to_string())
                .collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Log {
    pub stamp: crate::vendor::builtin_interfaces::msg::Time,
    pub level: u8,
    pub name: std::string::String,
    pub msg: std::string::String,
    pub file: std::string::String,
    pub function: std::string::String,
    pub line: u32,
}

impl Log {
    /// Debug is for pedantic information, which is useful when debugging issues.
    pub const DEBUG: u8 = 10;
    /// Info is the standard informational level and is used to report expected
    /// information.
    pub const INFO: u8 = 20;
    /// Warning is for information that may potentially cause issues or possibly unexpected
    /// behavior.
    pub const WARN: u8 = 30;
    /// Error is for information that this node cannot resolve.
    pub const ERROR: u8 = 40;
    /// Information about a impending node shutdown.
    pub const FATAL: u8 = 50;
}

impl Default for Log {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::Log::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Log {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::Log;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Owned(msg.stamp),
                )
                .into_owned(),
                level: msg.level,
                name: msg.name.as_str().into(),
                msg: msg.msg.as_str().into(),
                file: msg.file.as_str().into(),
                function: msg.function.as_str().into(),
                line: msg.line,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.stamp),
                )
                .into_owned(),
                level: msg.level,
                name: msg.name.as_str().into(),
                msg: msg.msg.as_str().into(),
                file: msg.file.as_str().into(),
                function: msg.function.as_str().into(),
                line: msg.line,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            stamp: crate::vendor::builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
            level: msg.level,
            name: msg.name.to_string(),
            msg: msg.msg.to_string(),
            file: msg.file.to_string(),
            function: msg.function.to_string(),
            line: msg.line,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParameterDescriptor {
    pub name: std::string::String,
    pub type_: u8,
    pub description: std::string::String,
    pub additional_constraints: std::string::String,
    pub read_only: bool,
    pub dynamic_typing: bool,
    pub floating_point_range: rosidl_runtime_rs::BoundedSequence<
        crate::vendor::rcl_interfaces::msg::rmw::FloatingPointRange,
        1,
    >,
    pub integer_range: rosidl_runtime_rs::BoundedSequence<
        crate::vendor::rcl_interfaces::msg::rmw::IntegerRange,
        1,
    >,
}

impl Default for ParameterDescriptor {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::ParameterDescriptor::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for ParameterDescriptor {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::ParameterDescriptor;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                name: msg.name.as_str().into(),
                type_: msg.type_,
                description: msg.description.as_str().into(),
                additional_constraints: msg.additional_constraints.as_str().into(),
                read_only: msg.read_only,
                dynamic_typing: msg.dynamic_typing,
                floating_point_range: msg.floating_point_range,
                integer_range: msg.integer_range,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                name: msg.name.as_str().into(),
                type_: msg.type_,
                description: msg.description.as_str().into(),
                additional_constraints: msg.additional_constraints.as_str().into(),
                read_only: msg.read_only,
                dynamic_typing: msg.dynamic_typing,
                floating_point_range: msg.floating_point_range.clone(),
                integer_range: msg.integer_range.clone(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            name: msg.name.to_string(),
            type_: msg.type_,
            description: msg.description.to_string(),
            additional_constraints: msg.additional_constraints.to_string(),
            read_only: msg.read_only,
            dynamic_typing: msg.dynamic_typing,
            floating_point_range: msg.floating_point_range,
            integer_range: msg.integer_range,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParameterEventDescriptors {
    pub new_parameters: Vec<crate::vendor::rcl_interfaces::msg::ParameterDescriptor>,
    pub changed_parameters: Vec<crate::vendor::rcl_interfaces::msg::ParameterDescriptor>,
    pub deleted_parameters: Vec<crate::vendor::rcl_interfaces::msg::ParameterDescriptor>,
}

impl Default for ParameterEventDescriptors {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::ParameterEventDescriptors::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for ParameterEventDescriptors {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::ParameterEventDescriptors;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                new_parameters: msg
                    .new_parameters
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::ParameterDescriptor::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                changed_parameters: msg
                    .changed_parameters
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::ParameterDescriptor::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                deleted_parameters: msg
                    .deleted_parameters
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::ParameterDescriptor::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                new_parameters: msg
                    .new_parameters
                    .iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::ParameterDescriptor::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                changed_parameters: msg
                    .changed_parameters
                    .iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::ParameterDescriptor::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                deleted_parameters: msg
                    .deleted_parameters
                    .iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::ParameterDescriptor::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            new_parameters: msg
                .new_parameters
                .into_iter()
                .map(crate::vendor::rcl_interfaces::msg::ParameterDescriptor::from_rmw_message)
                .collect(),
            changed_parameters: msg
                .changed_parameters
                .into_iter()
                .map(crate::vendor::rcl_interfaces::msg::ParameterDescriptor::from_rmw_message)
                .collect(),
            deleted_parameters: msg
                .deleted_parameters
                .into_iter()
                .map(crate::vendor::rcl_interfaces::msg::ParameterDescriptor::from_rmw_message)
                .collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParameterEvent {
    pub stamp: crate::vendor::builtin_interfaces::msg::Time,
    pub node: std::string::String,
    pub new_parameters: Vec<crate::vendor::rcl_interfaces::msg::Parameter>,
    pub changed_parameters: Vec<crate::vendor::rcl_interfaces::msg::Parameter>,
    pub deleted_parameters: Vec<crate::vendor::rcl_interfaces::msg::Parameter>,
}

impl Default for ParameterEvent {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::ParameterEvent::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for ParameterEvent {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::ParameterEvent;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Owned(msg.stamp),
                )
                .into_owned(),
                node: msg.node.as_str().into(),
                new_parameters: msg
                    .new_parameters
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::Parameter::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                changed_parameters: msg
                    .changed_parameters
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::Parameter::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                deleted_parameters: msg
                    .deleted_parameters
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::Parameter::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.stamp),
                )
                .into_owned(),
                node: msg.node.as_str().into(),
                new_parameters: msg
                    .new_parameters
                    .iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::Parameter::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                changed_parameters: msg
                    .changed_parameters
                    .iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::Parameter::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                deleted_parameters: msg
                    .deleted_parameters
                    .iter()
                    .map(|elem| {
                        crate::vendor::rcl_interfaces::msg::Parameter::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            stamp: crate::vendor::builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
            node: msg.node.to_string(),
            new_parameters: msg
                .new_parameters
                .into_iter()
                .map(crate::vendor::rcl_interfaces::msg::Parameter::from_rmw_message)
                .collect(),
            changed_parameters: msg
                .changed_parameters
                .into_iter()
                .map(crate::vendor::rcl_interfaces::msg::Parameter::from_rmw_message)
                .collect(),
            deleted_parameters: msg
                .deleted_parameters
                .into_iter()
                .map(crate::vendor::rcl_interfaces::msg::Parameter::from_rmw_message)
                .collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Parameter {
    pub name: std::string::String,
    pub value: crate::vendor::rcl_interfaces::msg::ParameterValue,
}

impl Default for Parameter {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::Parameter::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Parameter {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::Parameter;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                name: msg.name.as_str().into(),
                value: crate::vendor::rcl_interfaces::msg::ParameterValue::into_rmw_message(
                    std::borrow::Cow::Owned(msg.value),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                name: msg.name.as_str().into(),
                value: crate::vendor::rcl_interfaces::msg::ParameterValue::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.value),
                )
                .into_owned(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            name: msg.name.to_string(),
            value: crate::vendor::rcl_interfaces::msg::ParameterValue::from_rmw_message(msg.value),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParameterType {
    pub structure_needs_at_least_one_member: u8,
}

impl ParameterType {
    /// Default value, which implies this is not a valid parameter.
    pub const PARAMETER_NOT_SET: u8 = 0;
    pub const PARAMETER_BOOL: u8 = 1;
    pub const PARAMETER_INTEGER: u8 = 2;
    pub const PARAMETER_DOUBLE: u8 = 3;
    pub const PARAMETER_STRING: u8 = 4;
    pub const PARAMETER_BYTE_ARRAY: u8 = 5;
    pub const PARAMETER_BOOL_ARRAY: u8 = 6;
    pub const PARAMETER_INTEGER_ARRAY: u8 = 7;
    pub const PARAMETER_DOUBLE_ARRAY: u8 = 8;
    pub const PARAMETER_STRING_ARRAY: u8 = 9;
}

impl Default for ParameterType {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::ParameterType::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for ParameterType {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::ParameterType;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ParameterValue {
    pub type_: u8,
    pub bool_value: bool,
    pub integer_value: i64,
    pub double_value: f64,
    pub string_value: std::string::String,
    pub byte_array_value: Vec<u8>,
    pub bool_array_value: Vec<bool>,
    pub integer_array_value: Vec<i64>,
    pub double_array_value: Vec<f64>,
    pub string_array_value: Vec<std::string::String>,
}

impl Default for ParameterValue {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::ParameterValue::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for ParameterValue {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::ParameterValue;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                type_: msg.type_,
                bool_value: msg.bool_value,
                integer_value: msg.integer_value,
                double_value: msg.double_value,
                string_value: msg.string_value.as_str().into(),
                byte_array_value: msg.byte_array_value.into(),
                bool_array_value: msg.bool_array_value.into(),
                integer_array_value: msg.integer_array_value.into(),
                double_array_value: msg.double_array_value.into(),
                string_array_value: msg
                    .string_array_value
                    .into_iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                type_: msg.type_,
                bool_value: msg.bool_value,
                integer_value: msg.integer_value,
                double_value: msg.double_value,
                string_value: msg.string_value.as_str().into(),
                byte_array_value: msg.byte_array_value.as_slice().into(),
                bool_array_value: msg.bool_array_value.as_slice().into(),
                integer_array_value: msg.integer_array_value.as_slice().into(),
                double_array_value: msg.double_array_value.as_slice().into(),
                string_array_value: msg
                    .string_array_value
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            type_: msg.type_,
            bool_value: msg.bool_value,
            integer_value: msg.integer_value,
            double_value: msg.double_value,
            string_value: msg.string_value.to_string(),
            byte_array_value: msg.byte_array_value.into_iter().collect(),
            bool_array_value: msg.bool_array_value.into_iter().collect(),
            integer_array_value: msg.integer_array_value.into_iter().collect(),
            double_array_value: msg.double_array_value.into_iter().collect(),
            string_array_value: msg
                .string_array_value
                .into_iter()
                .map(|elem| elem.to_string())
                .collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetParametersResult {
    pub successful: bool,
    pub reason: std::string::String,
}

impl Default for SetParametersResult {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::rcl_interfaces::msg::rmw::SetParametersResult::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for SetParametersResult {
    type RmwMsg = crate::vendor::rcl_interfaces::msg::rmw::SetParametersResult;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                successful: msg.successful,
                reason: msg.reason.as_str().into(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                successful: msg.successful,
                reason: msg.reason.as_str().into(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            successful: msg.successful,
            reason: msg.reason.to_string(),
        }
    }
}
