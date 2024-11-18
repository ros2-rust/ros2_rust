pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "unique_identifier_msgs__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__unique_identifier_msgs__msg__UUID(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "unique_identifier_msgs__rosidl_generator_c")]
    extern "C" {
        fn unique_identifier_msgs__msg__UUID__init(msg: *mut UUID) -> bool;
        fn unique_identifier_msgs__msg__UUID__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<UUID>,
            size: usize,
        ) -> bool;
        fn unique_identifier_msgs__msg__UUID__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<UUID>,
        );
        fn unique_identifier_msgs__msg__UUID__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<UUID>,
            out_seq: *mut rosidl_runtime_rs::Sequence<UUID>,
        ) -> bool;
    }

    // Corresponds to unique_identifier_msgs__msg__UUID
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct UUID {
        pub uuid: [u8; 16],
    }

    impl Default for UUID {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !unique_identifier_msgs__msg__UUID__init(&mut msg as *mut _) {
                    panic!("Call to unique_identifier_msgs__msg__UUID__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for UUID {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { unique_identifier_msgs__msg__UUID__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { unique_identifier_msgs__msg__UUID__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { unique_identifier_msgs__msg__UUID__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for UUID {
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

    impl rosidl_runtime_rs::RmwMessage for UUID
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "unique_identifier_msgs/msg/UUID";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__unique_identifier_msgs__msg__UUID()
            }
        }
    }
} // mod rmw

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UUID {
    pub uuid: [u8; 16],
}

impl Default for UUID {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::unique_identifier_msgs::msg::rmw::UUID::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for UUID {
    type RmwMsg = crate::vendor::unique_identifier_msgs::msg::rmw::UUID;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { uuid: msg.uuid })
            }
            std::borrow::Cow::Borrowed(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { uuid: msg.uuid })
            }
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self { uuid: msg.uuid }
    }
}
