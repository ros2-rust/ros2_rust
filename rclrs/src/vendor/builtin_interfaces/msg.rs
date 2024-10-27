pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "builtin_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__builtin_interfaces__msg__Duration(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "builtin_interfaces__rosidl_generator_c")]
    extern "C" {
        fn builtin_interfaces__msg__Duration__init(msg: *mut Duration) -> bool;
        fn builtin_interfaces__msg__Duration__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Duration>,
            size: usize,
        ) -> bool;
        fn builtin_interfaces__msg__Duration__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Duration>,
        );
        fn builtin_interfaces__msg__Duration__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Duration>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Duration>,
        ) -> bool;
    }

    // Corresponds to builtin_interfaces__msg__Duration
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Duration {
        pub sec: i32,
        pub nanosec: u32,
    }

    impl Default for Duration {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !builtin_interfaces__msg__Duration__init(&mut msg as *mut _) {
                    panic!("Call to builtin_interfaces__msg__Duration__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Duration {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { builtin_interfaces__msg__Duration__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { builtin_interfaces__msg__Duration__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { builtin_interfaces__msg__Duration__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Duration {
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

    impl rosidl_runtime_rs::RmwMessage for Duration
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "builtin_interfaces/msg/Duration";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__builtin_interfaces__msg__Duration()
            }
        }
    }

    #[link(name = "builtin_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__builtin_interfaces__msg__Time(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "builtin_interfaces__rosidl_generator_c")]
    extern "C" {
        fn builtin_interfaces__msg__Time__init(msg: *mut Time) -> bool;
        fn builtin_interfaces__msg__Time__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Time>,
            size: usize,
        ) -> bool;
        fn builtin_interfaces__msg__Time__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Time>,
        );
        fn builtin_interfaces__msg__Time__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Time>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Time>,
        ) -> bool;
    }

    // Corresponds to builtin_interfaces__msg__Time
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Time {
        pub sec: i32,
        pub nanosec: u32,
    }

    impl Default for Time {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !builtin_interfaces__msg__Time__init(&mut msg as *mut _) {
                    panic!("Call to builtin_interfaces__msg__Time__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Time {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { builtin_interfaces__msg__Time__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { builtin_interfaces__msg__Time__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { builtin_interfaces__msg__Time__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Time {
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

    impl rosidl_runtime_rs::RmwMessage for Time
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "builtin_interfaces/msg/Time";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__builtin_interfaces__msg__Time(
                )
            }
        }
    }
} // mod rmw

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Duration {
    pub sec: i32,
    pub nanosec: u32,
}

impl Default for Duration {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::builtin_interfaces::msg::rmw::Duration::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Duration {
    type RmwMsg = crate::vendor::builtin_interfaces::msg::rmw::Duration;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sec: msg.sec,
                nanosec: msg.nanosec,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sec: msg.sec,
                nanosec: msg.nanosec,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            sec: msg.sec,
            nanosec: msg.nanosec,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

impl Default for Time {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::builtin_interfaces::msg::rmw::Time::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Time {
    type RmwMsg = crate::vendor::builtin_interfaces::msg::rmw::Time;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sec: msg.sec,
                nanosec: msg.nanosec,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sec: msg.sec,
                nanosec: msg.nanosec,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            sec: msg.sec,
            nanosec: msg.nanosec,
        }
    }
}
