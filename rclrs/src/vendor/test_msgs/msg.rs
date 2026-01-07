pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__KeyedLong(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__KeyedLong__init(msg: *mut KeyedLong) -> bool;
        fn test_msgs__msg__KeyedLong__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<KeyedLong>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__KeyedLong__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<KeyedLong>,
        );
        fn test_msgs__msg__KeyedLong__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<KeyedLong>,
            out_seq: *mut rosidl_runtime_rs::Sequence<KeyedLong>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__KeyedLong
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct KeyedLong {
        pub key: i32,
        pub value: i32,
    }

    impl Default for KeyedLong {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__KeyedLong__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__KeyedLong__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for KeyedLong {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__KeyedLong__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__KeyedLong__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__KeyedLong__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for KeyedLong {
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

    impl rosidl_runtime_rs::RmwMessage for KeyedLong
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/KeyedLong";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__KeyedLong()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__KeyedString(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__KeyedString__init(msg: *mut KeyedString) -> bool;
        fn test_msgs__msg__KeyedString__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<KeyedString>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__KeyedString__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<KeyedString>,
        );
        fn test_msgs__msg__KeyedString__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<KeyedString>,
            out_seq: *mut rosidl_runtime_rs::Sequence<KeyedString>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__KeyedString
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct KeyedString {
        pub key: rosidl_runtime_rs::String,
        pub value: rosidl_runtime_rs::String,
    }

    impl Default for KeyedString {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__KeyedString__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__KeyedString__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for KeyedString {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__KeyedString__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__KeyedString__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__KeyedString__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for KeyedString {
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

    impl rosidl_runtime_rs::RmwMessage for KeyedString
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/KeyedString";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__KeyedString()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__NonKeyedWithNestedKey(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__NonKeyedWithNestedKey__init(msg: *mut NonKeyedWithNestedKey) -> bool;
        fn test_msgs__msg__NonKeyedWithNestedKey__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<NonKeyedWithNestedKey>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__NonKeyedWithNestedKey__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<NonKeyedWithNestedKey>,
        );
        fn test_msgs__msg__NonKeyedWithNestedKey__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<NonKeyedWithNestedKey>,
            out_seq: *mut rosidl_runtime_rs::Sequence<NonKeyedWithNestedKey>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__NonKeyedWithNestedKey
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct NonKeyedWithNestedKey {
        pub nested_data: crate::vendor::test_msgs::msg::rmw::KeyedString,
        pub some_int: i32,
    }

    impl Default for NonKeyedWithNestedKey {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__NonKeyedWithNestedKey__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__NonKeyedWithNestedKey__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for NonKeyedWithNestedKey {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__NonKeyedWithNestedKey__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__NonKeyedWithNestedKey__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                test_msgs__msg__NonKeyedWithNestedKey__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for NonKeyedWithNestedKey {
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

    impl rosidl_runtime_rs::RmwMessage for NonKeyedWithNestedKey
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/NonKeyedWithNestedKey";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__NonKeyedWithNestedKey()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__ComplexNestedKey(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__ComplexNestedKey__init(msg: *mut ComplexNestedKey) -> bool;
        fn test_msgs__msg__ComplexNestedKey__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<ComplexNestedKey>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__ComplexNestedKey__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<ComplexNestedKey>,
        );
        fn test_msgs__msg__ComplexNestedKey__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<ComplexNestedKey>,
            out_seq: *mut rosidl_runtime_rs::Sequence<ComplexNestedKey>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__ComplexNestedKey
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct ComplexNestedKey {
        pub uint32_key: u32,
        pub nested_keys: crate::vendor::test_msgs::msg::rmw::NonKeyedWithNestedKey,
        pub float64_value: f64,
    }

    impl Default for ComplexNestedKey {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__ComplexNestedKey__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__ComplexNestedKey__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for ComplexNestedKey {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__ComplexNestedKey__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__ComplexNestedKey__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__ComplexNestedKey__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for ComplexNestedKey {
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

    impl rosidl_runtime_rs::RmwMessage for ComplexNestedKey
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/ComplexNestedKey";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__ComplexNestedKey()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Arrays(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__Arrays__init(msg: *mut Arrays) -> bool;
        fn test_msgs__msg__Arrays__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Arrays>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__Arrays__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Arrays>);
        fn test_msgs__msg__Arrays__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Arrays>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Arrays>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__Arrays
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Arrays {
        pub bool_values: [bool; 3],
        pub byte_values: [u8; 3],
        pub char_values: [u8; 3],
        pub float32_values: [f32; 3],
        pub float64_values: [f64; 3],
        pub int8_values: [i8; 3],
        pub uint8_values: [u8; 3],
        pub int16_values: [i16; 3],
        pub uint16_values: [u16; 3],
        pub int32_values: [i32; 3],
        pub uint32_values: [u32; 3],
        pub int64_values: [i64; 3],
        pub uint64_values: [u64; 3],
        pub string_values: [rosidl_runtime_rs::String; 3],
        pub basic_types_values: [crate::vendor::test_msgs::msg::rmw::BasicTypes; 3],
        pub constants_values: [crate::vendor::test_msgs::msg::rmw::Constants; 3],
        pub defaults_values: [crate::vendor::test_msgs::msg::rmw::Defaults; 3],
        pub bool_values_default: [bool; 3],
        pub byte_values_default: [u8; 3],
        pub char_values_default: [u8; 3],
        pub float32_values_default: [f32; 3],
        pub float64_values_default: [f64; 3],
        pub int8_values_default: [i8; 3],
        pub uint8_values_default: [u8; 3],
        pub int16_values_default: [i16; 3],
        pub uint16_values_default: [u16; 3],
        pub int32_values_default: [i32; 3],
        pub uint32_values_default: [u32; 3],
        pub int64_values_default: [i64; 3],
        pub uint64_values_default: [u64; 3],
        pub string_values_default: [rosidl_runtime_rs::String; 3],
        pub alignment_check: i32,
    }

    impl Default for Arrays {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__Arrays__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__Arrays__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Arrays {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Arrays__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Arrays__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Arrays__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Arrays {
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

    impl rosidl_runtime_rs::RmwMessage for Arrays
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/Arrays";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Arrays()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__BasicTypes(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__BasicTypes__init(msg: *mut BasicTypes) -> bool;
        fn test_msgs__msg__BasicTypes__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<BasicTypes>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__BasicTypes__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<BasicTypes>,
        );
        fn test_msgs__msg__BasicTypes__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<BasicTypes>,
            out_seq: *mut rosidl_runtime_rs::Sequence<BasicTypes>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__BasicTypes
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct BasicTypes {
        pub bool_value: bool,
        pub byte_value: u8,
        pub char_value: u8,
        pub float32_value: f32,
        pub float64_value: f64,
        pub int8_value: i8,
        pub uint8_value: u8,
        pub int16_value: i16,
        pub uint16_value: u16,
        pub int32_value: i32,
        pub uint32_value: u32,
        pub int64_value: i64,
        pub uint64_value: u64,
    }

    impl Default for BasicTypes {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__BasicTypes__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__BasicTypes__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for BasicTypes {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BasicTypes__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BasicTypes__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BasicTypes__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for BasicTypes {
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

    impl rosidl_runtime_rs::RmwMessage for BasicTypes
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/BasicTypes";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__BasicTypes()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__BoundedPlainSequences(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__BoundedPlainSequences__init(msg: *mut BoundedPlainSequences) -> bool;
        fn test_msgs__msg__BoundedPlainSequences__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<BoundedPlainSequences>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__BoundedPlainSequences__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<BoundedPlainSequences>,
        );
        fn test_msgs__msg__BoundedPlainSequences__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<BoundedPlainSequences>,
            out_seq: *mut rosidl_runtime_rs::Sequence<BoundedPlainSequences>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__BoundedPlainSequences
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct BoundedPlainSequences {
        pub bool_values: rosidl_runtime_rs::BoundedSequence<bool, 3>,
        pub byte_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub char_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub float32_values: rosidl_runtime_rs::BoundedSequence<f32, 3>,
        pub float64_values: rosidl_runtime_rs::BoundedSequence<f64, 3>,
        pub int8_values: rosidl_runtime_rs::BoundedSequence<i8, 3>,
        pub uint8_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub int16_values: rosidl_runtime_rs::BoundedSequence<i16, 3>,
        pub uint16_values: rosidl_runtime_rs::BoundedSequence<u16, 3>,
        pub int32_values: rosidl_runtime_rs::BoundedSequence<i32, 3>,
        pub uint32_values: rosidl_runtime_rs::BoundedSequence<u32, 3>,
        pub int64_values: rosidl_runtime_rs::BoundedSequence<i64, 3>,
        pub uint64_values: rosidl_runtime_rs::BoundedSequence<u64, 3>,
        pub basic_types_values:
            rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::BasicTypes, 3>,
        pub constants_values:
            rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Constants, 3>,
        pub defaults_values:
            rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Defaults, 3>,
        pub bool_values_default: rosidl_runtime_rs::BoundedSequence<bool, 3>,
        pub byte_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub char_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub float32_values_default: rosidl_runtime_rs::BoundedSequence<f32, 3>,
        pub float64_values_default: rosidl_runtime_rs::BoundedSequence<f64, 3>,
        pub int8_values_default: rosidl_runtime_rs::BoundedSequence<i8, 3>,
        pub uint8_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub int16_values_default: rosidl_runtime_rs::BoundedSequence<i16, 3>,
        pub uint16_values_default: rosidl_runtime_rs::BoundedSequence<u16, 3>,
        pub int32_values_default: rosidl_runtime_rs::BoundedSequence<i32, 3>,
        pub uint32_values_default: rosidl_runtime_rs::BoundedSequence<u32, 3>,
        pub int64_values_default: rosidl_runtime_rs::BoundedSequence<i64, 3>,
        pub uint64_values_default: rosidl_runtime_rs::BoundedSequence<u64, 3>,
        pub alignment_check: i32,
    }

    impl Default for BoundedPlainSequences {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__BoundedPlainSequences__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__BoundedPlainSequences__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for BoundedPlainSequences {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BoundedPlainSequences__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BoundedPlainSequences__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                test_msgs__msg__BoundedPlainSequences__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for BoundedPlainSequences {
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

    impl rosidl_runtime_rs::RmwMessage for BoundedPlainSequences
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/BoundedPlainSequences";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__BoundedPlainSequences()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__BoundedSequences(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__BoundedSequences__init(msg: *mut BoundedSequences) -> bool;
        fn test_msgs__msg__BoundedSequences__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<BoundedSequences>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__BoundedSequences__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<BoundedSequences>,
        );
        fn test_msgs__msg__BoundedSequences__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<BoundedSequences>,
            out_seq: *mut rosidl_runtime_rs::Sequence<BoundedSequences>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__BoundedSequences
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct BoundedSequences {
        pub bool_values: rosidl_runtime_rs::BoundedSequence<bool, 3>,
        pub byte_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub char_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub float32_values: rosidl_runtime_rs::BoundedSequence<f32, 3>,
        pub float64_values: rosidl_runtime_rs::BoundedSequence<f64, 3>,
        pub int8_values: rosidl_runtime_rs::BoundedSequence<i8, 3>,
        pub uint8_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub int16_values: rosidl_runtime_rs::BoundedSequence<i16, 3>,
        pub uint16_values: rosidl_runtime_rs::BoundedSequence<u16, 3>,
        pub int32_values: rosidl_runtime_rs::BoundedSequence<i32, 3>,
        pub uint32_values: rosidl_runtime_rs::BoundedSequence<u32, 3>,
        pub int64_values: rosidl_runtime_rs::BoundedSequence<i64, 3>,
        pub uint64_values: rosidl_runtime_rs::BoundedSequence<u64, 3>,
        pub string_values: rosidl_runtime_rs::BoundedSequence<rosidl_runtime_rs::String, 3>,
        pub basic_types_values:
            rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::BasicTypes, 3>,
        pub constants_values:
            rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Constants, 3>,
        pub defaults_values:
            rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Defaults, 3>,
        pub bool_values_default: rosidl_runtime_rs::BoundedSequence<bool, 3>,
        pub byte_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub char_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub float32_values_default: rosidl_runtime_rs::BoundedSequence<f32, 3>,
        pub float64_values_default: rosidl_runtime_rs::BoundedSequence<f64, 3>,
        pub int8_values_default: rosidl_runtime_rs::BoundedSequence<i8, 3>,
        pub uint8_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
        pub int16_values_default: rosidl_runtime_rs::BoundedSequence<i16, 3>,
        pub uint16_values_default: rosidl_runtime_rs::BoundedSequence<u16, 3>,
        pub int32_values_default: rosidl_runtime_rs::BoundedSequence<i32, 3>,
        pub uint32_values_default: rosidl_runtime_rs::BoundedSequence<u32, 3>,
        pub int64_values_default: rosidl_runtime_rs::BoundedSequence<i64, 3>,
        pub uint64_values_default: rosidl_runtime_rs::BoundedSequence<u64, 3>,
        pub string_values_default: rosidl_runtime_rs::BoundedSequence<rosidl_runtime_rs::String, 3>,
        pub alignment_check: i32,
    }

    impl Default for BoundedSequences {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__BoundedSequences__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__BoundedSequences__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for BoundedSequences {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BoundedSequences__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BoundedSequences__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__BoundedSequences__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for BoundedSequences {
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

    impl rosidl_runtime_rs::RmwMessage for BoundedSequences
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/BoundedSequences";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__BoundedSequences()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Constants(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__Constants__init(msg: *mut Constants) -> bool;
        fn test_msgs__msg__Constants__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Constants>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__Constants__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Constants>,
        );
        fn test_msgs__msg__Constants__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Constants>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Constants>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__Constants
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Constants {
        pub structure_needs_at_least_one_member: u8,
    }

    impl Constants {
        pub const BOOL_CONST: bool = true;
        pub const BYTE_CONST: u8 = 50;
        pub const CHAR_CONST: u8 = 100;
        pub const FLOAT32_CONST: f32 = 1.125;
        pub const FLOAT64_CONST: f64 = 1.125;
        pub const INT8_CONST: i8 = -50;
        pub const UINT8_CONST: u8 = 200;
        pub const INT16_CONST: i16 = -1000;
        pub const UINT16_CONST: u16 = 2000;
        pub const INT32_CONST: i32 = -30000;
        pub const UINT32_CONST: u32 = 60000;
        pub const INT64_CONST: i64 = -40000000;
        pub const UINT64_CONST: u64 = 50000000;
    }

    impl Default for Constants {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__Constants__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__Constants__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Constants {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Constants__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Constants__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Constants__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Constants {
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

    impl rosidl_runtime_rs::RmwMessage for Constants
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/Constants";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Constants()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Defaults(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__Defaults__init(msg: *mut Defaults) -> bool;
        fn test_msgs__msg__Defaults__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Defaults>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__Defaults__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Defaults>,
        );
        fn test_msgs__msg__Defaults__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Defaults>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Defaults>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__Defaults
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Defaults {
        pub bool_value: bool,
        pub byte_value: u8,
        pub char_value: u8,
        pub float32_value: f32,
        pub float64_value: f64,
        pub int8_value: i8,
        pub uint8_value: u8,
        pub int16_value: i16,
        pub uint16_value: u16,
        pub int32_value: i32,
        pub uint32_value: u32,
        pub int64_value: i64,
        pub uint64_value: u64,
    }

    impl Default for Defaults {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__Defaults__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__Defaults__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Defaults {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Defaults__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Defaults__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Defaults__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Defaults {
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

    impl rosidl_runtime_rs::RmwMessage for Defaults
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/Defaults";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Defaults()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Empty(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__Empty__init(msg: *mut Empty) -> bool;
        fn test_msgs__msg__Empty__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Empty>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__Empty__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Empty>);
        fn test_msgs__msg__Empty__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Empty>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Empty>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__Empty
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Empty {
        pub structure_needs_at_least_one_member: u8,
    }

    impl Default for Empty {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__Empty__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__Empty__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Empty {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Empty__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Empty__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Empty__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Empty {
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

    impl rosidl_runtime_rs::RmwMessage for Empty
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/Empty";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Empty()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__MultiNested(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__MultiNested__init(msg: *mut MultiNested) -> bool;
        fn test_msgs__msg__MultiNested__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<MultiNested>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__MultiNested__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<MultiNested>,
        );
        fn test_msgs__msg__MultiNested__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<MultiNested>,
            out_seq: *mut rosidl_runtime_rs::Sequence<MultiNested>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__MultiNested
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct MultiNested {
        pub array_of_arrays: [crate::vendor::test_msgs::msg::rmw::Arrays; 3],
        pub array_of_bounded_sequences: [crate::vendor::test_msgs::msg::rmw::BoundedSequences; 3],
        pub array_of_unbounded_sequences:
            [crate::vendor::test_msgs::msg::rmw::UnboundedSequences; 3],
        pub bounded_sequence_of_arrays:
            rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Arrays, 3>,
        pub bounded_sequence_of_bounded_sequences: rosidl_runtime_rs::BoundedSequence<
            crate::vendor::test_msgs::msg::rmw::BoundedSequences,
            3,
        >,
        pub bounded_sequence_of_unbounded_sequences: rosidl_runtime_rs::BoundedSequence<
            crate::vendor::test_msgs::msg::rmw::UnboundedSequences,
            3,
        >,
        pub unbounded_sequence_of_arrays:
            rosidl_runtime_rs::Sequence<crate::vendor::test_msgs::msg::rmw::Arrays>,
        pub unbounded_sequence_of_bounded_sequences:
            rosidl_runtime_rs::Sequence<crate::vendor::test_msgs::msg::rmw::BoundedSequences>,
        pub unbounded_sequence_of_unbounded_sequences:
            rosidl_runtime_rs::Sequence<crate::vendor::test_msgs::msg::rmw::UnboundedSequences>,
    }

    impl Default for MultiNested {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__MultiNested__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__MultiNested__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for MultiNested {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__MultiNested__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__MultiNested__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__MultiNested__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for MultiNested {
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

    impl rosidl_runtime_rs::RmwMessage for MultiNested
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/MultiNested";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__MultiNested()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Nested(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__Nested__init(msg: *mut Nested) -> bool;
        fn test_msgs__msg__Nested__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Nested>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__Nested__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Nested>);
        fn test_msgs__msg__Nested__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Nested>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Nested>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__Nested
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Nested {
        pub basic_types_value: crate::vendor::test_msgs::msg::rmw::BasicTypes,
    }

    impl Default for Nested {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__Nested__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__Nested__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Nested {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Nested__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Nested__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Nested__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Nested {
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

    impl rosidl_runtime_rs::RmwMessage for Nested
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/Nested";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Nested()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Strings(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__Strings__init(msg: *mut Strings) -> bool;
        fn test_msgs__msg__Strings__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Strings>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__Strings__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Strings>);
        fn test_msgs__msg__Strings__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Strings>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Strings>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__Strings
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Strings {
        pub string_value: rosidl_runtime_rs::String,
        pub string_value_default1: rosidl_runtime_rs::String,
        pub string_value_default2: rosidl_runtime_rs::String,
        pub string_value_default3: rosidl_runtime_rs::String,
        pub string_value_default4: rosidl_runtime_rs::String,
        pub string_value_default5: rosidl_runtime_rs::String,
        pub bounded_string_value: rosidl_runtime_rs::BoundedString<22>,
        pub bounded_string_value_default1: rosidl_runtime_rs::BoundedString<22>,
        pub bounded_string_value_default2: rosidl_runtime_rs::BoundedString<22>,
        pub bounded_string_value_default3: rosidl_runtime_rs::BoundedString<22>,
        pub bounded_string_value_default4: rosidl_runtime_rs::BoundedString<22>,
        pub bounded_string_value_default5: rosidl_runtime_rs::BoundedString<22>,
    }

    impl Strings {
        pub const STRING_CONST: &'static str = "Hello world!";
    }

    impl Default for Strings {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__Strings__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__Strings__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Strings {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Strings__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Strings__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Strings__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Strings {
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

    impl rosidl_runtime_rs::RmwMessage for Strings
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/Strings";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Strings()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__UnboundedSequences(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__UnboundedSequences__init(msg: *mut UnboundedSequences) -> bool;
        fn test_msgs__msg__UnboundedSequences__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<UnboundedSequences>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__UnboundedSequences__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<UnboundedSequences>,
        );
        fn test_msgs__msg__UnboundedSequences__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<UnboundedSequences>,
            out_seq: *mut rosidl_runtime_rs::Sequence<UnboundedSequences>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__UnboundedSequences
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct UnboundedSequences {
        pub bool_values: rosidl_runtime_rs::Sequence<bool>,
        pub byte_values: rosidl_runtime_rs::Sequence<u8>,
        pub char_values: rosidl_runtime_rs::Sequence<u8>,
        pub float32_values: rosidl_runtime_rs::Sequence<f32>,
        pub float64_values: rosidl_runtime_rs::Sequence<f64>,
        pub int8_values: rosidl_runtime_rs::Sequence<i8>,
        pub uint8_values: rosidl_runtime_rs::Sequence<u8>,
        pub int16_values: rosidl_runtime_rs::Sequence<i16>,
        pub uint16_values: rosidl_runtime_rs::Sequence<u16>,
        pub int32_values: rosidl_runtime_rs::Sequence<i32>,
        pub uint32_values: rosidl_runtime_rs::Sequence<u32>,
        pub int64_values: rosidl_runtime_rs::Sequence<i64>,
        pub uint64_values: rosidl_runtime_rs::Sequence<u64>,
        pub string_values: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
        pub basic_types_values:
            rosidl_runtime_rs::Sequence<crate::vendor::test_msgs::msg::rmw::BasicTypes>,
        pub constants_values:
            rosidl_runtime_rs::Sequence<crate::vendor::test_msgs::msg::rmw::Constants>,
        pub defaults_values:
            rosidl_runtime_rs::Sequence<crate::vendor::test_msgs::msg::rmw::Defaults>,
        pub bool_values_default: rosidl_runtime_rs::Sequence<bool>,
        pub byte_values_default: rosidl_runtime_rs::Sequence<u8>,
        pub char_values_default: rosidl_runtime_rs::Sequence<u8>,
        pub float32_values_default: rosidl_runtime_rs::Sequence<f32>,
        pub float64_values_default: rosidl_runtime_rs::Sequence<f64>,
        pub int8_values_default: rosidl_runtime_rs::Sequence<i8>,
        pub uint8_values_default: rosidl_runtime_rs::Sequence<u8>,
        pub int16_values_default: rosidl_runtime_rs::Sequence<i16>,
        pub uint16_values_default: rosidl_runtime_rs::Sequence<u16>,
        pub int32_values_default: rosidl_runtime_rs::Sequence<i32>,
        pub uint32_values_default: rosidl_runtime_rs::Sequence<u32>,
        pub int64_values_default: rosidl_runtime_rs::Sequence<i64>,
        pub uint64_values_default: rosidl_runtime_rs::Sequence<u64>,
        pub string_values_default: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
        pub alignment_check: i32,
    }

    impl Default for UnboundedSequences {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__UnboundedSequences__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__UnboundedSequences__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for UnboundedSequences {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__UnboundedSequences__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__UnboundedSequences__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__UnboundedSequences__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for UnboundedSequences {
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

    impl rosidl_runtime_rs::RmwMessage for UnboundedSequences
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/UnboundedSequences";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__UnboundedSequences()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__WStrings(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__WStrings__init(msg: *mut WStrings) -> bool;
        fn test_msgs__msg__WStrings__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<WStrings>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__WStrings__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<WStrings>,
        );
        fn test_msgs__msg__WStrings__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<WStrings>,
            out_seq: *mut rosidl_runtime_rs::Sequence<WStrings>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__WStrings
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct WStrings {
        pub wstring_value: rosidl_runtime_rs::WString,
        pub wstring_value_default1: rosidl_runtime_rs::WString,
        pub wstring_value_default2: rosidl_runtime_rs::WString,
        pub wstring_value_default3: rosidl_runtime_rs::WString,
        pub array_of_wstrings: [rosidl_runtime_rs::WString; 3],
        pub bounded_sequence_of_wstrings:
            rosidl_runtime_rs::BoundedSequence<rosidl_runtime_rs::WString, 3>,
        pub unbounded_sequence_of_wstrings: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::WString>,
    }

    impl Default for WStrings {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__WStrings__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__WStrings__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for WStrings {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__WStrings__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__WStrings__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__WStrings__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for WStrings {
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

    impl rosidl_runtime_rs::RmwMessage for WStrings
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/WStrings";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__WStrings()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Builtins(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__msg__Builtins__init(msg: *mut Builtins) -> bool;
        fn test_msgs__msg__Builtins__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Builtins>,
            size: usize,
        ) -> bool;
        fn test_msgs__msg__Builtins__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Builtins>,
        );
        fn test_msgs__msg__Builtins__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Builtins>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Builtins>,
        ) -> bool;
    }

    // Corresponds to test_msgs__msg__Builtins
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Builtins {
        pub duration_value: crate::vendor::builtin_interfaces::msg::rmw::Duration,
        pub time_value: crate::vendor::builtin_interfaces::msg::rmw::Time,
    }

    impl Default for Builtins {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__msg__Builtins__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__msg__Builtins__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Builtins {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Builtins__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Builtins__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__msg__Builtins__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Builtins {
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

    impl rosidl_runtime_rs::RmwMessage for Builtins
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/msg/Builtins";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__msg__Builtins()
            }
        }
    }
} // mod rmw

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyedLong {
    pub key: i32,
    pub value: i32,
}

impl Default for KeyedLong {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::KeyedLong::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for KeyedLong {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::KeyedLong;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                key: msg.key,
                value: msg.value,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                key: msg.key,
                value: msg.value,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            key: msg.key,
            value: msg.value,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyedString {
    pub key: std::string::String,
    pub value: std::string::String,
}

impl Default for KeyedString {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::KeyedString::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for KeyedString {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::KeyedString;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                key: msg.key.as_str().into(),
                value: msg.value.as_str().into(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                key: msg.key.as_str().into(),
                value: msg.value.as_str().into(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            key: msg.key.to_string(),
            value: msg.value.to_string(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NonKeyedWithNestedKey {
    pub nested_data: crate::vendor::test_msgs::msg::KeyedString,
    pub some_int: i32,
}

impl Default for NonKeyedWithNestedKey {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::NonKeyedWithNestedKey::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for NonKeyedWithNestedKey {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::NonKeyedWithNestedKey;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                nested_data: crate::vendor::test_msgs::msg::KeyedString::into_rmw_message(
                    std::borrow::Cow::Owned(msg.nested_data),
                )
                .into_owned(),
                some_int: msg.some_int,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                nested_data: crate::vendor::test_msgs::msg::KeyedString::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.nested_data),
                )
                .into_owned(),
                some_int: msg.some_int,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            nested_data: crate::vendor::test_msgs::msg::KeyedString::from_rmw_message(
                msg.nested_data,
            ),
            some_int: msg.some_int,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ComplexNestedKey {
    pub uint32_key: u32,
    pub nested_keys: crate::vendor::test_msgs::msg::NonKeyedWithNestedKey,
    pub float64_value: f64,
}

impl Default for ComplexNestedKey {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::ComplexNestedKey::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for ComplexNestedKey {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::ComplexNestedKey;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                uint32_key: msg.uint32_key,
                nested_keys:
                    crate::vendor::test_msgs::msg::NonKeyedWithNestedKey::into_rmw_message(
                        std::borrow::Cow::Owned(msg.nested_keys),
                    )
                    .into_owned(),
                float64_value: msg.float64_value,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                uint32_key: msg.uint32_key,
                nested_keys:
                    crate::vendor::test_msgs::msg::NonKeyedWithNestedKey::into_rmw_message(
                        std::borrow::Cow::Borrowed(&msg.nested_keys),
                    )
                    .into_owned(),
                float64_value: msg.float64_value,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            uint32_key: msg.uint32_key,
            nested_keys: crate::vendor::test_msgs::msg::NonKeyedWithNestedKey::from_rmw_message(
                msg.nested_keys,
            ),
            float64_value: msg.float64_value,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Arrays {
    pub bool_values: [bool; 3],
    pub byte_values: [u8; 3],
    pub char_values: [u8; 3],
    pub float32_values: [f32; 3],
    pub float64_values: [f64; 3],
    pub int8_values: [i8; 3],
    pub uint8_values: [u8; 3],
    pub int16_values: [i16; 3],
    pub uint16_values: [u16; 3],
    pub int32_values: [i32; 3],
    pub uint32_values: [u32; 3],
    pub int64_values: [i64; 3],
    pub uint64_values: [u64; 3],
    pub string_values: [std::string::String; 3],
    pub basic_types_values: [crate::vendor::test_msgs::msg::BasicTypes; 3],
    pub constants_values: [crate::vendor::test_msgs::msg::Constants; 3],
    pub defaults_values: [crate::vendor::test_msgs::msg::Defaults; 3],
    pub bool_values_default: [bool; 3],
    pub byte_values_default: [u8; 3],
    pub char_values_default: [u8; 3],
    pub float32_values_default: [f32; 3],
    pub float64_values_default: [f64; 3],
    pub int8_values_default: [i8; 3],
    pub uint8_values_default: [u8; 3],
    pub int16_values_default: [i16; 3],
    pub uint16_values_default: [u16; 3],
    pub int32_values_default: [i32; 3],
    pub uint32_values_default: [u32; 3],
    pub int64_values_default: [i64; 3],
    pub uint64_values_default: [u64; 3],
    pub string_values_default: [std::string::String; 3],
    pub alignment_check: i32,
}

impl Default for Arrays {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::Arrays::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Arrays {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::Arrays;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values,
                byte_values: msg.byte_values,
                char_values: msg.char_values,
                float32_values: msg.float32_values,
                float64_values: msg.float64_values,
                int8_values: msg.int8_values,
                uint8_values: msg.uint8_values,
                int16_values: msg.int16_values,
                uint16_values: msg.uint16_values,
                int32_values: msg.int32_values,
                uint32_values: msg.uint32_values,
                int64_values: msg.int64_values,
                uint64_values: msg.uint64_values,
                string_values: msg.string_values.map(|elem| elem.as_str().into()),
                basic_types_values: msg.basic_types_values.map(|elem| {
                    crate::vendor::test_msgs::msg::BasicTypes::into_rmw_message(
                        std::borrow::Cow::Owned(elem),
                    )
                    .into_owned()
                }),
                constants_values: msg.constants_values.map(|elem| {
                    crate::vendor::test_msgs::msg::Constants::into_rmw_message(
                        std::borrow::Cow::Owned(elem),
                    )
                    .into_owned()
                }),
                defaults_values: msg.defaults_values.map(|elem| {
                    crate::vendor::test_msgs::msg::Defaults::into_rmw_message(
                        std::borrow::Cow::Owned(elem),
                    )
                    .into_owned()
                }),
                bool_values_default: msg.bool_values_default,
                byte_values_default: msg.byte_values_default,
                char_values_default: msg.char_values_default,
                float32_values_default: msg.float32_values_default,
                float64_values_default: msg.float64_values_default,
                int8_values_default: msg.int8_values_default,
                uint8_values_default: msg.uint8_values_default,
                int16_values_default: msg.int16_values_default,
                uint16_values_default: msg.uint16_values_default,
                int32_values_default: msg.int32_values_default,
                uint32_values_default: msg.uint32_values_default,
                int64_values_default: msg.int64_values_default,
                uint64_values_default: msg.uint64_values_default,
                string_values_default: msg.string_values_default.map(|elem| elem.as_str().into()),
                alignment_check: msg.alignment_check,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values,
                byte_values: msg.byte_values,
                char_values: msg.char_values,
                float32_values: msg.float32_values,
                float64_values: msg.float64_values,
                int8_values: msg.int8_values,
                uint8_values: msg.uint8_values,
                int16_values: msg.int16_values,
                uint16_values: msg.uint16_values,
                int32_values: msg.int32_values,
                uint32_values: msg.uint32_values,
                int64_values: msg.int64_values,
                uint64_values: msg.uint64_values,
                string_values: msg
                    .string_values
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                basic_types_values: msg
                    .basic_types_values
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::BasicTypes::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                constants_values: msg
                    .constants_values
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Constants::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                defaults_values: msg
                    .defaults_values
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Defaults::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                bool_values_default: msg.bool_values_default,
                byte_values_default: msg.byte_values_default,
                char_values_default: msg.char_values_default,
                float32_values_default: msg.float32_values_default,
                float64_values_default: msg.float64_values_default,
                int8_values_default: msg.int8_values_default,
                uint8_values_default: msg.uint8_values_default,
                int16_values_default: msg.int16_values_default,
                uint16_values_default: msg.uint16_values_default,
                int32_values_default: msg.int32_values_default,
                uint32_values_default: msg.uint32_values_default,
                int64_values_default: msg.int64_values_default,
                uint64_values_default: msg.uint64_values_default,
                string_values_default: msg
                    .string_values_default
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                alignment_check: msg.alignment_check,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            bool_values: msg.bool_values,
            byte_values: msg.byte_values,
            char_values: msg.char_values,
            float32_values: msg.float32_values,
            float64_values: msg.float64_values,
            int8_values: msg.int8_values,
            uint8_values: msg.uint8_values,
            int16_values: msg.int16_values,
            uint16_values: msg.uint16_values,
            int32_values: msg.int32_values,
            uint32_values: msg.uint32_values,
            int64_values: msg.int64_values,
            uint64_values: msg.uint64_values,
            string_values: msg.string_values.map(|elem| elem.to_string()),
            basic_types_values: msg
                .basic_types_values
                .map(crate::vendor::test_msgs::msg::BasicTypes::from_rmw_message),
            constants_values: msg
                .constants_values
                .map(crate::vendor::test_msgs::msg::Constants::from_rmw_message),
            defaults_values: msg
                .defaults_values
                .map(crate::vendor::test_msgs::msg::Defaults::from_rmw_message),
            bool_values_default: msg.bool_values_default,
            byte_values_default: msg.byte_values_default,
            char_values_default: msg.char_values_default,
            float32_values_default: msg.float32_values_default,
            float64_values_default: msg.float64_values_default,
            int8_values_default: msg.int8_values_default,
            uint8_values_default: msg.uint8_values_default,
            int16_values_default: msg.int16_values_default,
            uint16_values_default: msg.uint16_values_default,
            int32_values_default: msg.int32_values_default,
            uint32_values_default: msg.uint32_values_default,
            int64_values_default: msg.int64_values_default,
            uint64_values_default: msg.uint64_values_default,
            string_values_default: msg.string_values_default.map(|elem| elem.to_string()),
            alignment_check: msg.alignment_check,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BasicTypes {
    pub bool_value: bool,
    pub byte_value: u8,
    pub char_value: u8,
    pub float32_value: f32,
    pub float64_value: f64,
    pub int8_value: i8,
    pub uint8_value: u8,
    pub int16_value: i16,
    pub uint16_value: u16,
    pub int32_value: i32,
    pub uint32_value: u32,
    pub int64_value: i64,
    pub uint64_value: u64,
}

impl Default for BasicTypes {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::BasicTypes::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for BasicTypes {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::BasicTypes;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_value: msg.bool_value,
                byte_value: msg.byte_value,
                char_value: msg.char_value,
                float32_value: msg.float32_value,
                float64_value: msg.float64_value,
                int8_value: msg.int8_value,
                uint8_value: msg.uint8_value,
                int16_value: msg.int16_value,
                uint16_value: msg.uint16_value,
                int32_value: msg.int32_value,
                uint32_value: msg.uint32_value,
                int64_value: msg.int64_value,
                uint64_value: msg.uint64_value,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_value: msg.bool_value,
                byte_value: msg.byte_value,
                char_value: msg.char_value,
                float32_value: msg.float32_value,
                float64_value: msg.float64_value,
                int8_value: msg.int8_value,
                uint8_value: msg.uint8_value,
                int16_value: msg.int16_value,
                uint16_value: msg.uint16_value,
                int32_value: msg.int32_value,
                uint32_value: msg.uint32_value,
                int64_value: msg.int64_value,
                uint64_value: msg.uint64_value,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            bool_value: msg.bool_value,
            byte_value: msg.byte_value,
            char_value: msg.char_value,
            float32_value: msg.float32_value,
            float64_value: msg.float64_value,
            int8_value: msg.int8_value,
            uint8_value: msg.uint8_value,
            int16_value: msg.int16_value,
            uint16_value: msg.uint16_value,
            int32_value: msg.int32_value,
            uint32_value: msg.uint32_value,
            int64_value: msg.int64_value,
            uint64_value: msg.uint64_value,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundedPlainSequences {
    pub bool_values: rosidl_runtime_rs::BoundedSequence<bool, 3>,
    pub byte_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub char_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub float32_values: rosidl_runtime_rs::BoundedSequence<f32, 3>,
    pub float64_values: rosidl_runtime_rs::BoundedSequence<f64, 3>,
    pub int8_values: rosidl_runtime_rs::BoundedSequence<i8, 3>,
    pub uint8_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub int16_values: rosidl_runtime_rs::BoundedSequence<i16, 3>,
    pub uint16_values: rosidl_runtime_rs::BoundedSequence<u16, 3>,
    pub int32_values: rosidl_runtime_rs::BoundedSequence<i32, 3>,
    pub uint32_values: rosidl_runtime_rs::BoundedSequence<u32, 3>,
    pub int64_values: rosidl_runtime_rs::BoundedSequence<i64, 3>,
    pub uint64_values: rosidl_runtime_rs::BoundedSequence<u64, 3>,
    pub basic_types_values:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::BasicTypes, 3>,
    pub constants_values:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Constants, 3>,
    pub defaults_values:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Defaults, 3>,
    pub bool_values_default: rosidl_runtime_rs::BoundedSequence<bool, 3>,
    pub byte_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub char_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub float32_values_default: rosidl_runtime_rs::BoundedSequence<f32, 3>,
    pub float64_values_default: rosidl_runtime_rs::BoundedSequence<f64, 3>,
    pub int8_values_default: rosidl_runtime_rs::BoundedSequence<i8, 3>,
    pub uint8_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub int16_values_default: rosidl_runtime_rs::BoundedSequence<i16, 3>,
    pub uint16_values_default: rosidl_runtime_rs::BoundedSequence<u16, 3>,
    pub int32_values_default: rosidl_runtime_rs::BoundedSequence<i32, 3>,
    pub uint32_values_default: rosidl_runtime_rs::BoundedSequence<u32, 3>,
    pub int64_values_default: rosidl_runtime_rs::BoundedSequence<i64, 3>,
    pub uint64_values_default: rosidl_runtime_rs::BoundedSequence<u64, 3>,
    pub alignment_check: i32,
}

impl Default for BoundedPlainSequences {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::BoundedPlainSequences::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for BoundedPlainSequences {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::BoundedPlainSequences;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values,
                byte_values: msg.byte_values,
                char_values: msg.char_values,
                float32_values: msg.float32_values,
                float64_values: msg.float64_values,
                int8_values: msg.int8_values,
                uint8_values: msg.uint8_values,
                int16_values: msg.int16_values,
                uint16_values: msg.uint16_values,
                int32_values: msg.int32_values,
                uint32_values: msg.uint32_values,
                int64_values: msg.int64_values,
                uint64_values: msg.uint64_values,
                basic_types_values: msg.basic_types_values,
                constants_values: msg.constants_values,
                defaults_values: msg.defaults_values,
                bool_values_default: msg.bool_values_default,
                byte_values_default: msg.byte_values_default,
                char_values_default: msg.char_values_default,
                float32_values_default: msg.float32_values_default,
                float64_values_default: msg.float64_values_default,
                int8_values_default: msg.int8_values_default,
                uint8_values_default: msg.uint8_values_default,
                int16_values_default: msg.int16_values_default,
                uint16_values_default: msg.uint16_values_default,
                int32_values_default: msg.int32_values_default,
                uint32_values_default: msg.uint32_values_default,
                int64_values_default: msg.int64_values_default,
                uint64_values_default: msg.uint64_values_default,
                alignment_check: msg.alignment_check,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values.clone(),
                byte_values: msg.byte_values.clone(),
                char_values: msg.char_values.clone(),
                float32_values: msg.float32_values.clone(),
                float64_values: msg.float64_values.clone(),
                int8_values: msg.int8_values.clone(),
                uint8_values: msg.uint8_values.clone(),
                int16_values: msg.int16_values.clone(),
                uint16_values: msg.uint16_values.clone(),
                int32_values: msg.int32_values.clone(),
                uint32_values: msg.uint32_values.clone(),
                int64_values: msg.int64_values.clone(),
                uint64_values: msg.uint64_values.clone(),
                basic_types_values: msg.basic_types_values.clone(),
                constants_values: msg.constants_values.clone(),
                defaults_values: msg.defaults_values.clone(),
                bool_values_default: msg.bool_values_default.clone(),
                byte_values_default: msg.byte_values_default.clone(),
                char_values_default: msg.char_values_default.clone(),
                float32_values_default: msg.float32_values_default.clone(),
                float64_values_default: msg.float64_values_default.clone(),
                int8_values_default: msg.int8_values_default.clone(),
                uint8_values_default: msg.uint8_values_default.clone(),
                int16_values_default: msg.int16_values_default.clone(),
                uint16_values_default: msg.uint16_values_default.clone(),
                int32_values_default: msg.int32_values_default.clone(),
                uint32_values_default: msg.uint32_values_default.clone(),
                int64_values_default: msg.int64_values_default.clone(),
                uint64_values_default: msg.uint64_values_default.clone(),
                alignment_check: msg.alignment_check,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            bool_values: msg.bool_values,
            byte_values: msg.byte_values,
            char_values: msg.char_values,
            float32_values: msg.float32_values,
            float64_values: msg.float64_values,
            int8_values: msg.int8_values,
            uint8_values: msg.uint8_values,
            int16_values: msg.int16_values,
            uint16_values: msg.uint16_values,
            int32_values: msg.int32_values,
            uint32_values: msg.uint32_values,
            int64_values: msg.int64_values,
            uint64_values: msg.uint64_values,
            basic_types_values: msg.basic_types_values,
            constants_values: msg.constants_values,
            defaults_values: msg.defaults_values,
            bool_values_default: msg.bool_values_default,
            byte_values_default: msg.byte_values_default,
            char_values_default: msg.char_values_default,
            float32_values_default: msg.float32_values_default,
            float64_values_default: msg.float64_values_default,
            int8_values_default: msg.int8_values_default,
            uint8_values_default: msg.uint8_values_default,
            int16_values_default: msg.int16_values_default,
            uint16_values_default: msg.uint16_values_default,
            int32_values_default: msg.int32_values_default,
            uint32_values_default: msg.uint32_values_default,
            int64_values_default: msg.int64_values_default,
            uint64_values_default: msg.uint64_values_default,
            alignment_check: msg.alignment_check,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundedSequences {
    pub bool_values: rosidl_runtime_rs::BoundedSequence<bool, 3>,
    pub byte_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub char_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub float32_values: rosidl_runtime_rs::BoundedSequence<f32, 3>,
    pub float64_values: rosidl_runtime_rs::BoundedSequence<f64, 3>,
    pub int8_values: rosidl_runtime_rs::BoundedSequence<i8, 3>,
    pub uint8_values: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub int16_values: rosidl_runtime_rs::BoundedSequence<i16, 3>,
    pub uint16_values: rosidl_runtime_rs::BoundedSequence<u16, 3>,
    pub int32_values: rosidl_runtime_rs::BoundedSequence<i32, 3>,
    pub uint32_values: rosidl_runtime_rs::BoundedSequence<u32, 3>,
    pub int64_values: rosidl_runtime_rs::BoundedSequence<i64, 3>,
    pub uint64_values: rosidl_runtime_rs::BoundedSequence<u64, 3>,
    pub string_values: rosidl_runtime_rs::BoundedSequence<rosidl_runtime_rs::String, 3>,
    pub basic_types_values:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::BasicTypes, 3>,
    pub constants_values:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Constants, 3>,
    pub defaults_values:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Defaults, 3>,
    pub bool_values_default: rosidl_runtime_rs::BoundedSequence<bool, 3>,
    pub byte_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub char_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub float32_values_default: rosidl_runtime_rs::BoundedSequence<f32, 3>,
    pub float64_values_default: rosidl_runtime_rs::BoundedSequence<f64, 3>,
    pub int8_values_default: rosidl_runtime_rs::BoundedSequence<i8, 3>,
    pub uint8_values_default: rosidl_runtime_rs::BoundedSequence<u8, 3>,
    pub int16_values_default: rosidl_runtime_rs::BoundedSequence<i16, 3>,
    pub uint16_values_default: rosidl_runtime_rs::BoundedSequence<u16, 3>,
    pub int32_values_default: rosidl_runtime_rs::BoundedSequence<i32, 3>,
    pub uint32_values_default: rosidl_runtime_rs::BoundedSequence<u32, 3>,
    pub int64_values_default: rosidl_runtime_rs::BoundedSequence<i64, 3>,
    pub uint64_values_default: rosidl_runtime_rs::BoundedSequence<u64, 3>,
    pub string_values_default: rosidl_runtime_rs::BoundedSequence<rosidl_runtime_rs::String, 3>,
    pub alignment_check: i32,
}

impl Default for BoundedSequences {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::BoundedSequences::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for BoundedSequences {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::BoundedSequences;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values,
                byte_values: msg.byte_values,
                char_values: msg.char_values,
                float32_values: msg.float32_values,
                float64_values: msg.float64_values,
                int8_values: msg.int8_values,
                uint8_values: msg.uint8_values,
                int16_values: msg.int16_values,
                uint16_values: msg.uint16_values,
                int32_values: msg.int32_values,
                uint32_values: msg.uint32_values,
                int64_values: msg.int64_values,
                uint64_values: msg.uint64_values,
                string_values: msg.string_values,
                basic_types_values: msg.basic_types_values,
                constants_values: msg.constants_values,
                defaults_values: msg.defaults_values,
                bool_values_default: msg.bool_values_default,
                byte_values_default: msg.byte_values_default,
                char_values_default: msg.char_values_default,
                float32_values_default: msg.float32_values_default,
                float64_values_default: msg.float64_values_default,
                int8_values_default: msg.int8_values_default,
                uint8_values_default: msg.uint8_values_default,
                int16_values_default: msg.int16_values_default,
                uint16_values_default: msg.uint16_values_default,
                int32_values_default: msg.int32_values_default,
                uint32_values_default: msg.uint32_values_default,
                int64_values_default: msg.int64_values_default,
                uint64_values_default: msg.uint64_values_default,
                string_values_default: msg.string_values_default,
                alignment_check: msg.alignment_check,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values.clone(),
                byte_values: msg.byte_values.clone(),
                char_values: msg.char_values.clone(),
                float32_values: msg.float32_values.clone(),
                float64_values: msg.float64_values.clone(),
                int8_values: msg.int8_values.clone(),
                uint8_values: msg.uint8_values.clone(),
                int16_values: msg.int16_values.clone(),
                uint16_values: msg.uint16_values.clone(),
                int32_values: msg.int32_values.clone(),
                uint32_values: msg.uint32_values.clone(),
                int64_values: msg.int64_values.clone(),
                uint64_values: msg.uint64_values.clone(),
                string_values: msg.string_values.clone(),
                basic_types_values: msg.basic_types_values.clone(),
                constants_values: msg.constants_values.clone(),
                defaults_values: msg.defaults_values.clone(),
                bool_values_default: msg.bool_values_default.clone(),
                byte_values_default: msg.byte_values_default.clone(),
                char_values_default: msg.char_values_default.clone(),
                float32_values_default: msg.float32_values_default.clone(),
                float64_values_default: msg.float64_values_default.clone(),
                int8_values_default: msg.int8_values_default.clone(),
                uint8_values_default: msg.uint8_values_default.clone(),
                int16_values_default: msg.int16_values_default.clone(),
                uint16_values_default: msg.uint16_values_default.clone(),
                int32_values_default: msg.int32_values_default.clone(),
                uint32_values_default: msg.uint32_values_default.clone(),
                int64_values_default: msg.int64_values_default.clone(),
                uint64_values_default: msg.uint64_values_default.clone(),
                string_values_default: msg.string_values_default.clone(),
                alignment_check: msg.alignment_check,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            bool_values: msg.bool_values,
            byte_values: msg.byte_values,
            char_values: msg.char_values,
            float32_values: msg.float32_values,
            float64_values: msg.float64_values,
            int8_values: msg.int8_values,
            uint8_values: msg.uint8_values,
            int16_values: msg.int16_values,
            uint16_values: msg.uint16_values,
            int32_values: msg.int32_values,
            uint32_values: msg.uint32_values,
            int64_values: msg.int64_values,
            uint64_values: msg.uint64_values,
            string_values: msg.string_values,
            basic_types_values: msg.basic_types_values,
            constants_values: msg.constants_values,
            defaults_values: msg.defaults_values,
            bool_values_default: msg.bool_values_default,
            byte_values_default: msg.byte_values_default,
            char_values_default: msg.char_values_default,
            float32_values_default: msg.float32_values_default,
            float64_values_default: msg.float64_values_default,
            int8_values_default: msg.int8_values_default,
            uint8_values_default: msg.uint8_values_default,
            int16_values_default: msg.int16_values_default,
            uint16_values_default: msg.uint16_values_default,
            int32_values_default: msg.int32_values_default,
            uint32_values_default: msg.uint32_values_default,
            int64_values_default: msg.int64_values_default,
            uint64_values_default: msg.uint64_values_default,
            string_values_default: msg.string_values_default,
            alignment_check: msg.alignment_check,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Constants {
    pub structure_needs_at_least_one_member: u8,
}

impl Constants {
    pub const BOOL_CONST: bool = true;
    pub const BYTE_CONST: u8 = 50;
    pub const CHAR_CONST: u8 = 100;
    pub const FLOAT32_CONST: f32 = 1.125;
    pub const FLOAT64_CONST: f64 = 1.125;
    pub const INT8_CONST: i8 = -50;
    pub const UINT8_CONST: u8 = 200;
    pub const INT16_CONST: i16 = -1000;
    pub const UINT16_CONST: u16 = 2000;
    pub const INT32_CONST: i32 = -30000;
    pub const UINT32_CONST: u32 = 60000;
    pub const INT64_CONST: i64 = -40000000;
    pub const UINT64_CONST: u64 = 50000000;
}

impl Default for Constants {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::Constants::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Constants {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::Constants;

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
pub struct Defaults {
    pub bool_value: bool,
    pub byte_value: u8,
    pub char_value: u8,
    pub float32_value: f32,
    pub float64_value: f64,
    pub int8_value: i8,
    pub uint8_value: u8,
    pub int16_value: i16,
    pub uint16_value: u16,
    pub int32_value: i32,
    pub uint32_value: u32,
    pub int64_value: i64,
    pub uint64_value: u64,
}

impl Default for Defaults {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::Defaults::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Defaults {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::Defaults;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_value: msg.bool_value,
                byte_value: msg.byte_value,
                char_value: msg.char_value,
                float32_value: msg.float32_value,
                float64_value: msg.float64_value,
                int8_value: msg.int8_value,
                uint8_value: msg.uint8_value,
                int16_value: msg.int16_value,
                uint16_value: msg.uint16_value,
                int32_value: msg.int32_value,
                uint32_value: msg.uint32_value,
                int64_value: msg.int64_value,
                uint64_value: msg.uint64_value,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_value: msg.bool_value,
                byte_value: msg.byte_value,
                char_value: msg.char_value,
                float32_value: msg.float32_value,
                float64_value: msg.float64_value,
                int8_value: msg.int8_value,
                uint8_value: msg.uint8_value,
                int16_value: msg.int16_value,
                uint16_value: msg.uint16_value,
                int32_value: msg.int32_value,
                uint32_value: msg.uint32_value,
                int64_value: msg.int64_value,
                uint64_value: msg.uint64_value,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            bool_value: msg.bool_value,
            byte_value: msg.byte_value,
            char_value: msg.char_value,
            float32_value: msg.float32_value,
            float64_value: msg.float64_value,
            int8_value: msg.int8_value,
            uint8_value: msg.uint8_value,
            int16_value: msg.int16_value,
            uint16_value: msg.uint16_value,
            int32_value: msg.int32_value,
            uint32_value: msg.uint32_value,
            int64_value: msg.int64_value,
            uint64_value: msg.uint64_value,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Empty {
    pub structure_needs_at_least_one_member: u8,
}

impl Default for Empty {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::Empty::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Empty {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::Empty;

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
pub struct MultiNested {
    pub array_of_arrays: [crate::vendor::test_msgs::msg::Arrays; 3],
    pub array_of_bounded_sequences: [crate::vendor::test_msgs::msg::BoundedSequences; 3],
    pub array_of_unbounded_sequences: [crate::vendor::test_msgs::msg::UnboundedSequences; 3],
    pub bounded_sequence_of_arrays:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::Arrays, 3>,
    pub bounded_sequence_of_bounded_sequences:
        rosidl_runtime_rs::BoundedSequence<crate::vendor::test_msgs::msg::rmw::BoundedSequences, 3>,
    pub bounded_sequence_of_unbounded_sequences: rosidl_runtime_rs::BoundedSequence<
        crate::vendor::test_msgs::msg::rmw::UnboundedSequences,
        3,
    >,
    pub unbounded_sequence_of_arrays: Vec<crate::vendor::test_msgs::msg::Arrays>,
    pub unbounded_sequence_of_bounded_sequences:
        Vec<crate::vendor::test_msgs::msg::BoundedSequences>,
    pub unbounded_sequence_of_unbounded_sequences:
        Vec<crate::vendor::test_msgs::msg::UnboundedSequences>,
}

impl Default for MultiNested {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::MultiNested::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for MultiNested {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::MultiNested;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                array_of_arrays: msg.array_of_arrays.map(|elem| {
                    crate::vendor::test_msgs::msg::Arrays::into_rmw_message(
                        std::borrow::Cow::Owned(elem),
                    )
                    .into_owned()
                }),
                array_of_bounded_sequences: msg.array_of_bounded_sequences.map(|elem| {
                    crate::vendor::test_msgs::msg::BoundedSequences::into_rmw_message(
                        std::borrow::Cow::Owned(elem),
                    )
                    .into_owned()
                }),
                array_of_unbounded_sequences: msg.array_of_unbounded_sequences.map(|elem| {
                    crate::vendor::test_msgs::msg::UnboundedSequences::into_rmw_message(
                        std::borrow::Cow::Owned(elem),
                    )
                    .into_owned()
                }),
                bounded_sequence_of_arrays: msg.bounded_sequence_of_arrays,
                bounded_sequence_of_bounded_sequences: msg.bounded_sequence_of_bounded_sequences,
                bounded_sequence_of_unbounded_sequences: msg
                    .bounded_sequence_of_unbounded_sequences,
                unbounded_sequence_of_arrays: msg
                    .unbounded_sequence_of_arrays
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Arrays::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                unbounded_sequence_of_bounded_sequences: msg
                    .unbounded_sequence_of_bounded_sequences
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::BoundedSequences::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                unbounded_sequence_of_unbounded_sequences: msg
                    .unbounded_sequence_of_unbounded_sequences
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::UnboundedSequences::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                array_of_arrays: msg
                    .array_of_arrays
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Arrays::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                array_of_bounded_sequences: msg
                    .array_of_bounded_sequences
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::BoundedSequences::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                array_of_unbounded_sequences: msg
                    .array_of_unbounded_sequences
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::UnboundedSequences::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                bounded_sequence_of_arrays: msg.bounded_sequence_of_arrays.clone(),
                bounded_sequence_of_bounded_sequences: msg
                    .bounded_sequence_of_bounded_sequences
                    .clone(),
                bounded_sequence_of_unbounded_sequences: msg
                    .bounded_sequence_of_unbounded_sequences
                    .clone(),
                unbounded_sequence_of_arrays: msg
                    .unbounded_sequence_of_arrays
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Arrays::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                unbounded_sequence_of_bounded_sequences: msg
                    .unbounded_sequence_of_bounded_sequences
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::BoundedSequences::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                unbounded_sequence_of_unbounded_sequences: msg
                    .unbounded_sequence_of_unbounded_sequences
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::UnboundedSequences::into_rmw_message(
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
            array_of_arrays: msg
                .array_of_arrays
                .map(crate::vendor::test_msgs::msg::Arrays::from_rmw_message),
            array_of_bounded_sequences: msg
                .array_of_bounded_sequences
                .map(crate::vendor::test_msgs::msg::BoundedSequences::from_rmw_message),
            array_of_unbounded_sequences: msg
                .array_of_unbounded_sequences
                .map(crate::vendor::test_msgs::msg::UnboundedSequences::from_rmw_message),
            bounded_sequence_of_arrays: msg.bounded_sequence_of_arrays,
            bounded_sequence_of_bounded_sequences: msg.bounded_sequence_of_bounded_sequences,
            bounded_sequence_of_unbounded_sequences: msg.bounded_sequence_of_unbounded_sequences,
            unbounded_sequence_of_arrays: msg
                .unbounded_sequence_of_arrays
                .into_iter()
                .map(crate::vendor::test_msgs::msg::Arrays::from_rmw_message)
                .collect(),
            unbounded_sequence_of_bounded_sequences: msg
                .unbounded_sequence_of_bounded_sequences
                .into_iter()
                .map(crate::vendor::test_msgs::msg::BoundedSequences::from_rmw_message)
                .collect(),
            unbounded_sequence_of_unbounded_sequences: msg
                .unbounded_sequence_of_unbounded_sequences
                .into_iter()
                .map(crate::vendor::test_msgs::msg::UnboundedSequences::from_rmw_message)
                .collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Nested {
    pub basic_types_value: crate::vendor::test_msgs::msg::BasicTypes,
}

impl Default for Nested {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::Nested::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Nested {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::Nested;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                basic_types_value: crate::vendor::test_msgs::msg::BasicTypes::into_rmw_message(
                    std::borrow::Cow::Owned(msg.basic_types_value),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                basic_types_value: crate::vendor::test_msgs::msg::BasicTypes::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.basic_types_value),
                )
                .into_owned(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            basic_types_value: crate::vendor::test_msgs::msg::BasicTypes::from_rmw_message(
                msg.basic_types_value,
            ),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Strings {
    pub string_value: std::string::String,
    pub string_value_default1: std::string::String,
    pub string_value_default2: std::string::String,
    pub string_value_default3: std::string::String,
    pub string_value_default4: std::string::String,
    pub string_value_default5: std::string::String,
    pub bounded_string_value: rosidl_runtime_rs::BoundedString<22>,
    pub bounded_string_value_default1: rosidl_runtime_rs::BoundedString<22>,
    pub bounded_string_value_default2: rosidl_runtime_rs::BoundedString<22>,
    pub bounded_string_value_default3: rosidl_runtime_rs::BoundedString<22>,
    pub bounded_string_value_default4: rosidl_runtime_rs::BoundedString<22>,
    pub bounded_string_value_default5: rosidl_runtime_rs::BoundedString<22>,
}

impl Strings {
    pub const STRING_CONST: &'static str = "Hello world!";
}

impl Default for Strings {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::Strings::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Strings {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::Strings;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                string_value: msg.string_value.as_str().into(),
                string_value_default1: msg.string_value_default1.as_str().into(),
                string_value_default2: msg.string_value_default2.as_str().into(),
                string_value_default3: msg.string_value_default3.as_str().into(),
                string_value_default4: msg.string_value_default4.as_str().into(),
                string_value_default5: msg.string_value_default5.as_str().into(),
                bounded_string_value: msg.bounded_string_value,
                bounded_string_value_default1: msg.bounded_string_value_default1,
                bounded_string_value_default2: msg.bounded_string_value_default2,
                bounded_string_value_default3: msg.bounded_string_value_default3,
                bounded_string_value_default4: msg.bounded_string_value_default4,
                bounded_string_value_default5: msg.bounded_string_value_default5,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                string_value: msg.string_value.as_str().into(),
                string_value_default1: msg.string_value_default1.as_str().into(),
                string_value_default2: msg.string_value_default2.as_str().into(),
                string_value_default3: msg.string_value_default3.as_str().into(),
                string_value_default4: msg.string_value_default4.as_str().into(),
                string_value_default5: msg.string_value_default5.as_str().into(),
                bounded_string_value: msg.bounded_string_value.clone(),
                bounded_string_value_default1: msg.bounded_string_value_default1.clone(),
                bounded_string_value_default2: msg.bounded_string_value_default2.clone(),
                bounded_string_value_default3: msg.bounded_string_value_default3.clone(),
                bounded_string_value_default4: msg.bounded_string_value_default4.clone(),
                bounded_string_value_default5: msg.bounded_string_value_default5.clone(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            string_value: msg.string_value.to_string(),
            string_value_default1: msg.string_value_default1.to_string(),
            string_value_default2: msg.string_value_default2.to_string(),
            string_value_default3: msg.string_value_default3.to_string(),
            string_value_default4: msg.string_value_default4.to_string(),
            string_value_default5: msg.string_value_default5.to_string(),
            bounded_string_value: msg.bounded_string_value,
            bounded_string_value_default1: msg.bounded_string_value_default1,
            bounded_string_value_default2: msg.bounded_string_value_default2,
            bounded_string_value_default3: msg.bounded_string_value_default3,
            bounded_string_value_default4: msg.bounded_string_value_default4,
            bounded_string_value_default5: msg.bounded_string_value_default5,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UnboundedSequences {
    pub bool_values: Vec<bool>,
    pub byte_values: Vec<u8>,
    pub char_values: Vec<u8>,
    pub float32_values: Vec<f32>,
    pub float64_values: Vec<f64>,
    pub int8_values: Vec<i8>,
    pub uint8_values: Vec<u8>,
    pub int16_values: Vec<i16>,
    pub uint16_values: Vec<u16>,
    pub int32_values: Vec<i32>,
    pub uint32_values: Vec<u32>,
    pub int64_values: Vec<i64>,
    pub uint64_values: Vec<u64>,
    pub string_values: Vec<std::string::String>,
    pub basic_types_values: Vec<crate::vendor::test_msgs::msg::BasicTypes>,
    pub constants_values: Vec<crate::vendor::test_msgs::msg::Constants>,
    pub defaults_values: Vec<crate::vendor::test_msgs::msg::Defaults>,
    pub bool_values_default: Vec<bool>,
    pub byte_values_default: Vec<u8>,
    pub char_values_default: Vec<u8>,
    pub float32_values_default: Vec<f32>,
    pub float64_values_default: Vec<f64>,
    pub int8_values_default: Vec<i8>,
    pub uint8_values_default: Vec<u8>,
    pub int16_values_default: Vec<i16>,
    pub uint16_values_default: Vec<u16>,
    pub int32_values_default: Vec<i32>,
    pub uint32_values_default: Vec<u32>,
    pub int64_values_default: Vec<i64>,
    pub uint64_values_default: Vec<u64>,
    pub string_values_default: Vec<std::string::String>,
    pub alignment_check: i32,
}

impl Default for UnboundedSequences {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::UnboundedSequences::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for UnboundedSequences {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::UnboundedSequences;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values.into(),
                byte_values: msg.byte_values.into(),
                char_values: msg.char_values.into(),
                float32_values: msg.float32_values.into(),
                float64_values: msg.float64_values.into(),
                int8_values: msg.int8_values.into(),
                uint8_values: msg.uint8_values.into(),
                int16_values: msg.int16_values.into(),
                uint16_values: msg.uint16_values.into(),
                int32_values: msg.int32_values.into(),
                uint32_values: msg.uint32_values.into(),
                int64_values: msg.int64_values.into(),
                uint64_values: msg.uint64_values.into(),
                string_values: msg
                    .string_values
                    .into_iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
                basic_types_values: msg
                    .basic_types_values
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::BasicTypes::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                constants_values: msg
                    .constants_values
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Constants::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                defaults_values: msg
                    .defaults_values
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Defaults::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                bool_values_default: msg.bool_values_default.into(),
                byte_values_default: msg.byte_values_default.into(),
                char_values_default: msg.char_values_default.into(),
                float32_values_default: msg.float32_values_default.into(),
                float64_values_default: msg.float64_values_default.into(),
                int8_values_default: msg.int8_values_default.into(),
                uint8_values_default: msg.uint8_values_default.into(),
                int16_values_default: msg.int16_values_default.into(),
                uint16_values_default: msg.uint16_values_default.into(),
                int32_values_default: msg.int32_values_default.into(),
                uint32_values_default: msg.uint32_values_default.into(),
                int64_values_default: msg.int64_values_default.into(),
                uint64_values_default: msg.uint64_values_default.into(),
                string_values_default: msg
                    .string_values_default
                    .into_iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
                alignment_check: msg.alignment_check,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                bool_values: msg.bool_values.as_slice().into(),
                byte_values: msg.byte_values.as_slice().into(),
                char_values: msg.char_values.as_slice().into(),
                float32_values: msg.float32_values.as_slice().into(),
                float64_values: msg.float64_values.as_slice().into(),
                int8_values: msg.int8_values.as_slice().into(),
                uint8_values: msg.uint8_values.as_slice().into(),
                int16_values: msg.int16_values.as_slice().into(),
                uint16_values: msg.uint16_values.as_slice().into(),
                int32_values: msg.int32_values.as_slice().into(),
                uint32_values: msg.uint32_values.as_slice().into(),
                int64_values: msg.int64_values.as_slice().into(),
                uint64_values: msg.uint64_values.as_slice().into(),
                string_values: msg
                    .string_values
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
                basic_types_values: msg
                    .basic_types_values
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::BasicTypes::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                constants_values: msg
                    .constants_values
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Constants::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                defaults_values: msg
                    .defaults_values
                    .iter()
                    .map(|elem| {
                        crate::vendor::test_msgs::msg::Defaults::into_rmw_message(
                            std::borrow::Cow::Borrowed(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
                bool_values_default: msg.bool_values_default.as_slice().into(),
                byte_values_default: msg.byte_values_default.as_slice().into(),
                char_values_default: msg.char_values_default.as_slice().into(),
                float32_values_default: msg.float32_values_default.as_slice().into(),
                float64_values_default: msg.float64_values_default.as_slice().into(),
                int8_values_default: msg.int8_values_default.as_slice().into(),
                uint8_values_default: msg.uint8_values_default.as_slice().into(),
                int16_values_default: msg.int16_values_default.as_slice().into(),
                uint16_values_default: msg.uint16_values_default.as_slice().into(),
                int32_values_default: msg.int32_values_default.as_slice().into(),
                uint32_values_default: msg.uint32_values_default.as_slice().into(),
                int64_values_default: msg.int64_values_default.as_slice().into(),
                uint64_values_default: msg.uint64_values_default.as_slice().into(),
                string_values_default: msg
                    .string_values_default
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
                alignment_check: msg.alignment_check,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            bool_values: msg.bool_values.into_iter().collect(),
            byte_values: msg.byte_values.into_iter().collect(),
            char_values: msg.char_values.into_iter().collect(),
            float32_values: msg.float32_values.into_iter().collect(),
            float64_values: msg.float64_values.into_iter().collect(),
            int8_values: msg.int8_values.into_iter().collect(),
            uint8_values: msg.uint8_values.into_iter().collect(),
            int16_values: msg.int16_values.into_iter().collect(),
            uint16_values: msg.uint16_values.into_iter().collect(),
            int32_values: msg.int32_values.into_iter().collect(),
            uint32_values: msg.uint32_values.into_iter().collect(),
            int64_values: msg.int64_values.into_iter().collect(),
            uint64_values: msg.uint64_values.into_iter().collect(),
            string_values: msg
                .string_values
                .into_iter()
                .map(|elem| elem.to_string())
                .collect(),
            basic_types_values: msg
                .basic_types_values
                .into_iter()
                .map(crate::vendor::test_msgs::msg::BasicTypes::from_rmw_message)
                .collect(),
            constants_values: msg
                .constants_values
                .into_iter()
                .map(crate::vendor::test_msgs::msg::Constants::from_rmw_message)
                .collect(),
            defaults_values: msg
                .defaults_values
                .into_iter()
                .map(crate::vendor::test_msgs::msg::Defaults::from_rmw_message)
                .collect(),
            bool_values_default: msg.bool_values_default.into_iter().collect(),
            byte_values_default: msg.byte_values_default.into_iter().collect(),
            char_values_default: msg.char_values_default.into_iter().collect(),
            float32_values_default: msg.float32_values_default.into_iter().collect(),
            float64_values_default: msg.float64_values_default.into_iter().collect(),
            int8_values_default: msg.int8_values_default.into_iter().collect(),
            uint8_values_default: msg.uint8_values_default.into_iter().collect(),
            int16_values_default: msg.int16_values_default.into_iter().collect(),
            uint16_values_default: msg.uint16_values_default.into_iter().collect(),
            int32_values_default: msg.int32_values_default.into_iter().collect(),
            uint32_values_default: msg.uint32_values_default.into_iter().collect(),
            int64_values_default: msg.int64_values_default.into_iter().collect(),
            uint64_values_default: msg.uint64_values_default.into_iter().collect(),
            string_values_default: msg
                .string_values_default
                .into_iter()
                .map(|elem| elem.to_string())
                .collect(),
            alignment_check: msg.alignment_check,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WStrings {
    pub wstring_value: std::string::String,
    pub wstring_value_default1: std::string::String,
    pub wstring_value_default2: std::string::String,
    pub wstring_value_default3: std::string::String,
    pub array_of_wstrings: [std::string::String; 3],
    pub bounded_sequence_of_wstrings:
        rosidl_runtime_rs::BoundedSequence<rosidl_runtime_rs::WString, 3>,
    pub unbounded_sequence_of_wstrings: Vec<std::string::String>,
}

impl Default for WStrings {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::WStrings::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for WStrings {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::WStrings;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                wstring_value: msg.wstring_value.as_str().into(),
                wstring_value_default1: msg.wstring_value_default1.as_str().into(),
                wstring_value_default2: msg.wstring_value_default2.as_str().into(),
                wstring_value_default3: msg.wstring_value_default3.as_str().into(),
                array_of_wstrings: msg.array_of_wstrings.map(|elem| elem.as_str().into()),
                bounded_sequence_of_wstrings: msg.bounded_sequence_of_wstrings,
                unbounded_sequence_of_wstrings: msg
                    .unbounded_sequence_of_wstrings
                    .into_iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                wstring_value: msg.wstring_value.as_str().into(),
                wstring_value_default1: msg.wstring_value_default1.as_str().into(),
                wstring_value_default2: msg.wstring_value_default2.as_str().into(),
                wstring_value_default3: msg.wstring_value_default3.as_str().into(),
                array_of_wstrings: msg
                    .array_of_wstrings
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap(),
                bounded_sequence_of_wstrings: msg.bounded_sequence_of_wstrings.clone(),
                unbounded_sequence_of_wstrings: msg
                    .unbounded_sequence_of_wstrings
                    .iter()
                    .map(|elem| elem.as_str().into())
                    .collect(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            wstring_value: msg.wstring_value.to_string(),
            wstring_value_default1: msg.wstring_value_default1.to_string(),
            wstring_value_default2: msg.wstring_value_default2.to_string(),
            wstring_value_default3: msg.wstring_value_default3.to_string(),
            array_of_wstrings: msg.array_of_wstrings.map(|elem| elem.to_string()),
            bounded_sequence_of_wstrings: msg.bounded_sequence_of_wstrings,
            unbounded_sequence_of_wstrings: msg
                .unbounded_sequence_of_wstrings
                .into_iter()
                .map(|elem| elem.to_string())
                .collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Builtins {
    pub duration_value: crate::vendor::builtin_interfaces::msg::Duration,
    pub time_value: crate::vendor::builtin_interfaces::msg::Time,
}

impl Default for Builtins {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::msg::rmw::Builtins::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Builtins {
    type RmwMsg = crate::vendor::test_msgs::msg::rmw::Builtins;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                duration_value: crate::vendor::builtin_interfaces::msg::Duration::into_rmw_message(
                    std::borrow::Cow::Owned(msg.duration_value),
                )
                .into_owned(),
                time_value: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Owned(msg.time_value),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                duration_value: crate::vendor::builtin_interfaces::msg::Duration::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.duration_value),
                )
                .into_owned(),
                time_value: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.time_value),
                )
                .into_owned(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            duration_value: crate::vendor::builtin_interfaces::msg::Duration::from_rmw_message(
                msg.duration_value,
            ),
            time_value: crate::vendor::builtin_interfaces::msg::Time::from_rmw_message(
                msg.time_value,
            ),
        }
    }
}
