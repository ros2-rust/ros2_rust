#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelGoal_Request {
    pub goal_info: crate::vendor::action_msgs::msg::GoalInfo,
}

impl Default for CancelGoal_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::action_msgs::srv::rmw::CancelGoal_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for CancelGoal_Request {
    type RmwMsg = crate::vendor::action_msgs::srv::rmw::CancelGoal_Request;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_info: crate::vendor::action_msgs::msg::GoalInfo::into_rmw_message(
                    std::borrow::Cow::Owned(msg.goal_info),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_info: crate::vendor::action_msgs::msg::GoalInfo::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.goal_info),
                )
                .into_owned(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            goal_info: crate::vendor::action_msgs::msg::GoalInfo::from_rmw_message(msg.goal_info),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelGoal_Response {
    pub return_code: i8,
    pub goals_canceling: Vec<crate::vendor::action_msgs::msg::GoalInfo>,
}

impl CancelGoal_Response {
    /// Indicates the request was accepted without any errors.
    ///
    /// One or more goals have transitioned to the CANCELING state. The
    /// goals_canceling list is not empty.
    pub const ERROR_NONE: i8 = 0;
    /// Indicates the request was rejected.
    ///
    /// No goals have transitioned to the CANCELING state. The goals_canceling list is
    /// empty.
    pub const ERROR_REJECTED: i8 = 1;
    /// Indicates the requested goal ID does not exist.
    ///
    /// No goals have transitioned to the CANCELING state. The goals_canceling list is
    /// empty.
    pub const ERROR_UNKNOWN_GOAL_ID: i8 = 2;
    /// Indicates the goal is not cancelable because it is already in a terminal state.
    ///
    /// No goals have transitioned to the CANCELING state. The goals_canceling list is
    /// empty.
    pub const ERROR_GOAL_TERMINATED: i8 = 3;
}

impl Default for CancelGoal_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::action_msgs::srv::rmw::CancelGoal_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for CancelGoal_Response {
    type RmwMsg = crate::vendor::action_msgs::srv::rmw::CancelGoal_Response;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                return_code: msg.return_code,
                goals_canceling: msg
                    .goals_canceling
                    .into_iter()
                    .map(|elem| {
                        crate::vendor::action_msgs::msg::GoalInfo::into_rmw_message(
                            std::borrow::Cow::Owned(elem),
                        )
                        .into_owned()
                    })
                    .collect(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                return_code: msg.return_code,
                goals_canceling: msg
                    .goals_canceling
                    .iter()
                    .map(|elem| {
                        crate::vendor::action_msgs::msg::GoalInfo::into_rmw_message(
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
            return_code: msg.return_code,
            goals_canceling: msg
                .goals_canceling
                .into_iter()
                .map(crate::vendor::action_msgs::msg::GoalInfo::from_rmw_message)
                .collect(),
        }
    }
}

#[link(name = "action_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__action_msgs__srv__CancelGoal(
    ) -> *const std::ffi::c_void;
}

// Corresponds to action_msgs__srv__CancelGoal
pub struct CancelGoal;

impl rosidl_runtime_rs::Service for CancelGoal {
    type Request = crate::vendor::action_msgs::srv::CancelGoal_Request;
    type Response = crate::vendor::action_msgs::srv::CancelGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__action_msgs__srv__CancelGoal()
        }
    }
}

pub mod rmw {

    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "action_msgs__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__action_msgs__srv__CancelGoal_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "action_msgs__rosidl_generator_c")]
    extern "C" {
        fn action_msgs__srv__CancelGoal_Request__init(msg: *mut CancelGoal_Request) -> bool;
        fn action_msgs__srv__CancelGoal_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<CancelGoal_Request>,
            size: usize,
        ) -> bool;
        fn action_msgs__srv__CancelGoal_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<CancelGoal_Request>,
        );
        fn action_msgs__srv__CancelGoal_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<CancelGoal_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<CancelGoal_Request>,
        ) -> bool;
    }

    // Corresponds to action_msgs__srv__CancelGoal_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct CancelGoal_Request {
        pub goal_info: crate::vendor::action_msgs::msg::rmw::GoalInfo,
    }

    impl Default for CancelGoal_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !action_msgs__srv__CancelGoal_Request__init(&mut msg as *mut _) {
                    panic!("Call to action_msgs__srv__CancelGoal_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for CancelGoal_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__srv__CancelGoal_Request__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__srv__CancelGoal_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                action_msgs__srv__CancelGoal_Request__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for CancelGoal_Request {
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

    impl rosidl_runtime_rs::RmwMessage for CancelGoal_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "action_msgs/srv/CancelGoal_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__action_msgs__srv__CancelGoal_Request()
            }
        }
    }

    #[link(name = "action_msgs__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__action_msgs__srv__CancelGoal_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "action_msgs__rosidl_generator_c")]
    extern "C" {
        fn action_msgs__srv__CancelGoal_Response__init(msg: *mut CancelGoal_Response) -> bool;
        fn action_msgs__srv__CancelGoal_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<CancelGoal_Response>,
            size: usize,
        ) -> bool;
        fn action_msgs__srv__CancelGoal_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<CancelGoal_Response>,
        );
        fn action_msgs__srv__CancelGoal_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<CancelGoal_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<CancelGoal_Response>,
        ) -> bool;
    }

    // Corresponds to action_msgs__srv__CancelGoal_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct CancelGoal_Response {
        pub return_code: i8,
        pub goals_canceling:
            rosidl_runtime_rs::Sequence<crate::vendor::action_msgs::msg::rmw::GoalInfo>,
    }

    impl CancelGoal_Response {
        /// Indicates the request was accepted without any errors.
        ///
        /// One or more goals have transitioned to the CANCELING state. The
        /// goals_canceling list is not empty.
        pub const ERROR_NONE: i8 = 0;
        /// Indicates the request was rejected.
        ///
        /// No goals have transitioned to the CANCELING state. The goals_canceling list is
        /// empty.
        pub const ERROR_REJECTED: i8 = 1;
        /// Indicates the requested goal ID does not exist.
        ///
        /// No goals have transitioned to the CANCELING state. The goals_canceling list is
        /// empty.
        pub const ERROR_UNKNOWN_GOAL_ID: i8 = 2;
        /// Indicates the goal is not cancelable because it is already in a terminal state.
        ///
        /// No goals have transitioned to the CANCELING state. The goals_canceling list is
        /// empty.
        pub const ERROR_GOAL_TERMINATED: i8 = 3;
    }

    impl Default for CancelGoal_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !action_msgs__srv__CancelGoal_Response__init(&mut msg as *mut _) {
                    panic!("Call to action_msgs__srv__CancelGoal_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for CancelGoal_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__srv__CancelGoal_Response__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { action_msgs__srv__CancelGoal_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                action_msgs__srv__CancelGoal_Response__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for CancelGoal_Response {
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

    impl rosidl_runtime_rs::RmwMessage for CancelGoal_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "action_msgs/srv/CancelGoal_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__action_msgs__srv__CancelGoal_Response()
            }
        }
    }

    #[link(name = "action_msgs__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__action_msgs__srv__CancelGoal(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to action_msgs__srv__CancelGoal
    pub struct CancelGoal;

    impl rosidl_runtime_rs::Service for CancelGoal {
        type Request = crate::vendor::action_msgs::srv::rmw::CancelGoal_Request;
        type Response = crate::vendor::action_msgs::srv::rmw::CancelGoal_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__action_msgs__srv__CancelGoal(
                )
            }
        }
    }
} // mod rmw
