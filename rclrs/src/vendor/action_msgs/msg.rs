pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "action_msgs__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__action_msgs__msg__GoalInfo(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "action_msgs__rosidl_generator_c")]
    extern "C" {
        fn action_msgs__msg__GoalInfo__init(msg: *mut GoalInfo) -> bool;
        fn action_msgs__msg__GoalInfo__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<GoalInfo>,
            size: usize,
        ) -> bool;
        fn action_msgs__msg__GoalInfo__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<GoalInfo>,
        );
        fn action_msgs__msg__GoalInfo__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<GoalInfo>,
            out_seq: *mut rosidl_runtime_rs::Sequence<GoalInfo>,
        ) -> bool;
    }

    // Corresponds to action_msgs__msg__GoalInfo
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct GoalInfo {
        pub goal_id: crate::vendor::unique_identifier_msgs::msg::rmw::UUID,
        pub stamp: crate::vendor::builtin_interfaces::msg::rmw::Time,
    }

    impl Default for GoalInfo {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !action_msgs__msg__GoalInfo__init(&mut msg as *mut _) {
                    panic!("Call to action_msgs__msg__GoalInfo__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for GoalInfo {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalInfo__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalInfo__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalInfo__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for GoalInfo {
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

    impl rosidl_runtime_rs::RmwMessage for GoalInfo
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "action_msgs/msg/GoalInfo";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__action_msgs__msg__GoalInfo()
            }
        }
    }

    #[link(name = "action_msgs__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__action_msgs__msg__GoalStatus(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "action_msgs__rosidl_generator_c")]
    extern "C" {
        fn action_msgs__msg__GoalStatus__init(msg: *mut GoalStatus) -> bool;
        fn action_msgs__msg__GoalStatus__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<GoalStatus>,
            size: usize,
        ) -> bool;
        fn action_msgs__msg__GoalStatus__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<GoalStatus>,
        );
        fn action_msgs__msg__GoalStatus__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<GoalStatus>,
            out_seq: *mut rosidl_runtime_rs::Sequence<GoalStatus>,
        ) -> bool;
    }

    // Corresponds to action_msgs__msg__GoalStatus
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct GoalStatus {
        pub goal_info: crate::vendor::action_msgs::msg::rmw::GoalInfo,
        pub status: i8,
    }

    impl GoalStatus {
        /// Indicates status has not been properly set.
        pub const STATUS_UNKNOWN: i8 = 0;
        /// The goal has been accepted and is awaiting execution.
        pub const STATUS_ACCEPTED: i8 = 1;
        /// The goal is currently being executed by the action server.
        pub const STATUS_EXECUTING: i8 = 2;
        /// The client has requested that the goal be canceled and the action server has
        /// accepted the cancel request.
        pub const STATUS_CANCELING: i8 = 3;
        /// The goal was achieved successfully by the action server.
        pub const STATUS_SUCCEEDED: i8 = 4;
        /// The goal was canceled after an external request from an action client.
        pub const STATUS_CANCELED: i8 = 5;
        /// The goal was terminated by the action server without an external request.
        pub const STATUS_ABORTED: i8 = 6;
    }

    impl Default for GoalStatus {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !action_msgs__msg__GoalStatus__init(&mut msg as *mut _) {
                    panic!("Call to action_msgs__msg__GoalStatus__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for GoalStatus {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalStatus__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalStatus__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalStatus__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for GoalStatus {
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

    impl rosidl_runtime_rs::RmwMessage for GoalStatus
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "action_msgs/msg/GoalStatus";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__action_msgs__msg__GoalStatus(
                )
            }
        }
    }

    #[link(name = "action_msgs__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__action_msgs__msg__GoalStatusArray(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "action_msgs__rosidl_generator_c")]
    extern "C" {
        fn action_msgs__msg__GoalStatusArray__init(msg: *mut GoalStatusArray) -> bool;
        fn action_msgs__msg__GoalStatusArray__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<GoalStatusArray>,
            size: usize,
        ) -> bool;
        fn action_msgs__msg__GoalStatusArray__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<GoalStatusArray>,
        );
        fn action_msgs__msg__GoalStatusArray__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<GoalStatusArray>,
            out_seq: *mut rosidl_runtime_rs::Sequence<GoalStatusArray>,
        ) -> bool;
    }

    // Corresponds to action_msgs__msg__GoalStatusArray
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct GoalStatusArray {
        pub status_list:
            rosidl_runtime_rs::Sequence<crate::vendor::action_msgs::msg::rmw::GoalStatus>,
    }

    impl Default for GoalStatusArray {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !action_msgs__msg__GoalStatusArray__init(&mut msg as *mut _) {
                    panic!("Call to action_msgs__msg__GoalStatusArray__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for GoalStatusArray {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalStatusArray__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalStatusArray__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__msg__GoalStatusArray__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for GoalStatusArray {
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

    impl rosidl_runtime_rs::RmwMessage for GoalStatusArray
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "action_msgs/msg/GoalStatusArray";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__action_msgs__msg__GoalStatusArray()
            }
        }
    }
} // mod rmw

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GoalInfo {
    pub goal_id: crate::vendor::unique_identifier_msgs::msg::UUID,
    pub stamp: crate::vendor::builtin_interfaces::msg::Time,
}

impl Default for GoalInfo {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::action_msgs::msg::rmw::GoalInfo::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for GoalInfo {
    type RmwMsg = crate::vendor::action_msgs::msg::rmw::GoalInfo;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Owned(msg.goal_id),
                )
                .into_owned(),
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Owned(msg.stamp),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.goal_id),
                )
                .into_owned(),
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.stamp),
                )
                .into_owned(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::from_rmw_message(
                msg.goal_id,
            ),
            stamp: crate::vendor::builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GoalStatus {
    pub goal_info: crate::vendor::action_msgs::msg::GoalInfo,
    pub status: i8,
}

impl GoalStatus {
    /// Indicates status has not been properly set.
    pub const STATUS_UNKNOWN: i8 = 0;
    /// The goal has been accepted and is awaiting execution.
    pub const STATUS_ACCEPTED: i8 = 1;
    /// The goal is currently being executed by the action server.
    pub const STATUS_EXECUTING: i8 = 2;
    /// The client has requested that the goal be canceled and the action server has
    /// accepted the cancel request.
    pub const STATUS_CANCELING: i8 = 3;
    /// The goal was achieved successfully by the action server.
    pub const STATUS_SUCCEEDED: i8 = 4;
    /// The goal was canceled after an external request from an action client.
    pub const STATUS_CANCELED: i8 = 5;
    /// The goal was terminated by the action server without an external request.
    pub const STATUS_ABORTED: i8 = 6;
}

impl Default for GoalStatus {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::action_msgs::msg::rmw::GoalStatus::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for GoalStatus {
    type RmwMsg = crate::vendor::action_msgs::msg::rmw::GoalStatus;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_info: crate::vendor::action_msgs::msg::GoalInfo::into_rmw_message(
                    std::borrow::Cow::Owned(msg.goal_info),
                )
                .into_owned(),
                status: msg.status,
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_info: crate::vendor::action_msgs::msg::GoalInfo::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.goal_info),
                )
                .into_owned(),
                status: msg.status,
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            goal_info: crate::vendor::action_msgs::msg::GoalInfo::from_rmw_message(msg.goal_info),
            status: msg.status,
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GoalStatusArray {
    pub status_list: Vec<crate::vendor::action_msgs::msg::GoalStatus>,
}

impl Default for GoalStatusArray {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::action_msgs::msg::rmw::GoalStatusArray::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for GoalStatusArray {
    type RmwMsg = crate::vendor::action_msgs::msg::rmw::GoalStatusArray;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                status_list: msg
                    .status_list
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::action_msgs::msg::GoalStatus::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                status_list: msg
                    .status_list
                    .iter()
                    .map(|elem| {
                        crate::vendor::action_msgs::msg::GoalStatus::into_rmw_message(
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
            status_list: msg
                .status_list
                .into_iter()
                .map(crate::vendor::action_msgs::msg::GoalStatus::from_rmw_message)
                .collect(),
        }
    }
}
