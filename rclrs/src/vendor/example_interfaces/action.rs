pub mod rmw {
    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_Goal(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_Goal__init(msg: *mut Fibonacci_Goal) -> bool;
        fn example_interfaces__action__Fibonacci_Goal__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Goal>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_Goal__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Goal>,
        );
        fn example_interfaces__action__Fibonacci_Goal__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_Goal>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Goal>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_Goal
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_Goal {
        pub order: i32,
    }

    impl Default for Fibonacci_Goal {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_Goal__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__action__Fibonacci_Goal__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_Goal {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_Goal__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__action__Fibonacci_Goal__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_Goal__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_Goal {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_Goal
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_Goal";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_Goal()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_Result(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_Result__init(msg: *mut Fibonacci_Result) -> bool;
        fn example_interfaces__action__Fibonacci_Result__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Result>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_Result__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Result>,
        );
        fn example_interfaces__action__Fibonacci_Result__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_Result>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Result>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_Result
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_Result {
        pub sequence: rosidl_runtime_rs::Sequence<i32>,
    }

    impl Default for Fibonacci_Result {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_Result__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__action__Fibonacci_Result__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_Result {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_Result__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__action__Fibonacci_Result__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_Result__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_Result {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_Result
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_Result";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_Result()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_Feedback(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_Feedback__init(
            msg: *mut Fibonacci_Feedback,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_Feedback__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Feedback>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_Feedback__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Feedback>,
        );
        fn example_interfaces__action__Fibonacci_Feedback__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_Feedback>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_Feedback>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_Feedback
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_Feedback {
        pub sequence: rosidl_runtime_rs::Sequence<i32>,
    }

    impl Default for Fibonacci_Feedback {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_Feedback__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__action__Fibonacci_Feedback__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_Feedback {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_Feedback__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__action__Fibonacci_Feedback__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_Feedback__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_Feedback {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_Feedback
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_Feedback";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_Feedback()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_FeedbackMessage(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_FeedbackMessage__init(
            msg: *mut Fibonacci_FeedbackMessage,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_FeedbackMessage__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_FeedbackMessage>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_FeedbackMessage__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_FeedbackMessage>,
        );
        fn example_interfaces__action__Fibonacci_FeedbackMessage__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_FeedbackMessage>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_FeedbackMessage>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_FeedbackMessage
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_FeedbackMessage {
        pub goal_id: crate::vendor::unique_identifier_msgs::msg::rmw::UUID,
        pub feedback: crate::vendor::example_interfaces::action::rmw::Fibonacci_Feedback,
    }

    impl Default for Fibonacci_FeedbackMessage {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_FeedbackMessage__init(&mut msg as *mut _)
                {
                    panic!("Call to example_interfaces__action__Fibonacci_FeedbackMessage__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_FeedbackMessage {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_FeedbackMessage__Sequence__init(
                    seq as *mut _,
                    size,
                )
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_FeedbackMessage__Sequence__fini(seq as *mut _)
            }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_FeedbackMessage__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_FeedbackMessage {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_FeedbackMessage
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_FeedbackMessage";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_FeedbackMessage()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_SendGoal_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_SendGoal_Request__init(
            msg: *mut Fibonacci_SendGoal_Request,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_SendGoal_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Request>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_SendGoal_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Request>,
        );
        fn example_interfaces__action__Fibonacci_SendGoal_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Request>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_SendGoal_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_SendGoal_Request {
        pub goal_id: crate::vendor::unique_identifier_msgs::msg::rmw::UUID,
        pub goal: crate::vendor::example_interfaces::action::rmw::Fibonacci_Goal,
    }

    impl Default for Fibonacci_SendGoal_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_SendGoal_Request__init(&mut msg as *mut _)
                {
                    panic!("Call to example_interfaces__action__Fibonacci_SendGoal_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_SendGoal_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_SendGoal_Request__Sequence__init(
                    seq as *mut _,
                    size,
                )
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_SendGoal_Request__Sequence__fini(
                    seq as *mut _,
                )
            }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_SendGoal_Request__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_SendGoal_Request {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_SendGoal_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_SendGoal_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_SendGoal_Request()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_SendGoal_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_SendGoal_Response__init(
            msg: *mut Fibonacci_SendGoal_Response,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_SendGoal_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Response>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_SendGoal_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Response>,
        );
        fn example_interfaces__action__Fibonacci_SendGoal_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_SendGoal_Response>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_SendGoal_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_SendGoal_Response {
        pub accepted: bool,
        pub stamp: crate::vendor::builtin_interfaces::msg::rmw::Time,
    }

    impl Default for Fibonacci_SendGoal_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_SendGoal_Response__init(
                    &mut msg as *mut _,
                ) {
                    panic!("Call to example_interfaces__action__Fibonacci_SendGoal_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_SendGoal_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_SendGoal_Response__Sequence__init(
                    seq as *mut _,
                    size,
                )
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_SendGoal_Response__Sequence__fini(
                    seq as *mut _,
                )
            }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_SendGoal_Response__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_SendGoal_Response {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_SendGoal_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_SendGoal_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_SendGoal_Response()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_GetResult_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_GetResult_Request__init(
            msg: *mut Fibonacci_GetResult_Request,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_GetResult_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Request>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_GetResult_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Request>,
        );
        fn example_interfaces__action__Fibonacci_GetResult_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Request>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_GetResult_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_GetResult_Request {
        pub goal_id: crate::vendor::unique_identifier_msgs::msg::rmw::UUID,
    }

    impl Default for Fibonacci_GetResult_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_GetResult_Request__init(
                    &mut msg as *mut _,
                ) {
                    panic!("Call to example_interfaces__action__Fibonacci_GetResult_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_GetResult_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_GetResult_Request__Sequence__init(
                    seq as *mut _,
                    size,
                )
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_GetResult_Request__Sequence__fini(
                    seq as *mut _,
                )
            }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_GetResult_Request__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_GetResult_Request {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_GetResult_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_GetResult_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_GetResult_Request()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_GetResult_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__action__Fibonacci_GetResult_Response__init(
            msg: *mut Fibonacci_GetResult_Response,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_GetResult_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Response>,
            size: usize,
        ) -> bool;
        fn example_interfaces__action__Fibonacci_GetResult_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Response>,
        );
        fn example_interfaces__action__Fibonacci_GetResult_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Fibonacci_GetResult_Response>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__action__Fibonacci_GetResult_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Fibonacci_GetResult_Response {
        pub status: i8,
        pub result: crate::vendor::example_interfaces::action::rmw::Fibonacci_Result,
    }

    impl Default for Fibonacci_GetResult_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__action__Fibonacci_GetResult_Response__init(
                    &mut msg as *mut _,
                ) {
                    panic!("Call to example_interfaces__action__Fibonacci_GetResult_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Fibonacci_GetResult_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_GetResult_Response__Sequence__init(
                    seq as *mut _,
                    size,
                )
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_GetResult_Response__Sequence__fini(
                    seq as *mut _,
                )
            }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__action__Fibonacci_GetResult_Response__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for Fibonacci_GetResult_Response {
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

    impl rosidl_runtime_rs::RmwMessage for Fibonacci_GetResult_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/action/Fibonacci_GetResult_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__action__Fibonacci_GetResult_Response()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_SendGoal(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to example_interfaces__action__Fibonacci_SendGoal
    pub struct Fibonacci_SendGoal;

    impl rosidl_runtime_rs::Service for Fibonacci_SendGoal {
        type Request = crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Request;
        type Response = crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_SendGoal()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_GetResult(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to example_interfaces__action__Fibonacci_GetResult
    pub struct Fibonacci_GetResult;

    impl rosidl_runtime_rs::Service for Fibonacci_GetResult {
        type Request = crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Request;
        type Response =
            crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_GetResult()
            }
        }
    }
} // mod rmw

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_Goal {
    pub order: i32,
}

impl Default for Fibonacci_Goal {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_Goal::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_Goal {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_Goal;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { order: msg.order })
            }
            std::borrow::Cow::Borrowed(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { order: msg.order })
            }
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self { order: msg.order }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_Result {
    pub sequence: Vec<i32>,
}

impl Default for Fibonacci_Result {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_Result::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_Result {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_Result;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sequence: msg.sequence.into(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sequence: msg.sequence.as_slice().into(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            sequence: msg.sequence.into_iter().collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_Feedback {
    pub sequence: Vec<i32>,
}

impl Default for Fibonacci_Feedback {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_Feedback::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_Feedback {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_Feedback;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sequence: msg.sequence.into(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                sequence: msg.sequence.as_slice().into(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            sequence: msg.sequence.into_iter().collect(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_FeedbackMessage {
    pub goal_id: crate::vendor::unique_identifier_msgs::msg::UUID,
    pub feedback: crate::vendor::example_interfaces::action::Fibonacci_Feedback,
}

impl Default for Fibonacci_FeedbackMessage {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_FeedbackMessage::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_FeedbackMessage {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_FeedbackMessage;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Owned(msg.goal_id),
                )
                .into_owned(),
                feedback:
                    crate::vendor::example_interfaces::action::Fibonacci_Feedback::into_rmw_message(
                        std::borrow::Cow::Owned(msg.feedback),
                    )
                    .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.goal_id),
                )
                .into_owned(),
                feedback:
                    crate::vendor::example_interfaces::action::Fibonacci_Feedback::into_rmw_message(
                        std::borrow::Cow::Borrowed(&msg.feedback),
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
            feedback:
                crate::vendor::example_interfaces::action::Fibonacci_Feedback::from_rmw_message(
                    msg.feedback,
                ),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_SendGoal_Request {
    pub goal_id: crate::vendor::unique_identifier_msgs::msg::UUID,
    pub goal: crate::vendor::example_interfaces::action::Fibonacci_Goal,
}

impl Default for Fibonacci_SendGoal_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_SendGoal_Request {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Request;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Owned(msg.goal_id),
                )
                .into_owned(),
                goal: crate::vendor::example_interfaces::action::Fibonacci_Goal::into_rmw_message(
                    std::borrow::Cow::Owned(msg.goal),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.goal_id),
                )
                .into_owned(),
                goal: crate::vendor::example_interfaces::action::Fibonacci_Goal::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.goal),
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
            goal: crate::vendor::example_interfaces::action::Fibonacci_Goal::from_rmw_message(
                msg.goal,
            ),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_SendGoal_Response {
    pub accepted: bool,
    pub stamp: crate::vendor::builtin_interfaces::msg::Time,
}

impl Default for Fibonacci_SendGoal_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_SendGoal_Response {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Response;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                accepted: msg.accepted,
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Owned(msg.stamp),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                accepted: msg.accepted,
                stamp: crate::vendor::builtin_interfaces::msg::Time::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.stamp),
                )
                .into_owned(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            accepted: msg.accepted,
            stamp: crate::vendor::builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_GetResult_Request {
    pub goal_id: crate::vendor::unique_identifier_msgs::msg::UUID,
}

impl Default for Fibonacci_GetResult_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_GetResult_Request {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Request;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Owned(msg.goal_id),
                )
                .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                goal_id: crate::vendor::unique_identifier_msgs::msg::UUID::into_rmw_message(
                    std::borrow::Cow::Borrowed(&msg.goal_id),
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
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Fibonacci_GetResult_Response {
    pub status: i8,
    pub result: crate::vendor::example_interfaces::action::Fibonacci_Result,
}

impl Default for Fibonacci_GetResult_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Fibonacci_GetResult_Response {
    type RmwMsg = crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Response;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                status: msg.status,
                result:
                    crate::vendor::example_interfaces::action::Fibonacci_Result::into_rmw_message(
                        std::borrow::Cow::Owned(msg.result),
                    )
                    .into_owned(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                status: msg.status,
                result:
                    crate::vendor::example_interfaces::action::Fibonacci_Result::into_rmw_message(
                        std::borrow::Cow::Borrowed(&msg.result),
                    )
                    .into_owned(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            status: msg.status,
            result: crate::vendor::example_interfaces::action::Fibonacci_Result::from_rmw_message(
                msg.result,
            ),
        }
    }
}

#[link(name = "example_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_SendGoal(
    ) -> *const std::ffi::c_void;
}

// Corresponds to example_interfaces__action__Fibonacci_SendGoal
pub struct Fibonacci_SendGoal;

impl rosidl_runtime_rs::Service for Fibonacci_SendGoal {
    type Request = crate::vendor::example_interfaces::action::Fibonacci_SendGoal_Request;
    type Response = crate::vendor::example_interfaces::action::Fibonacci_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_SendGoal()
        }
    }
}

#[link(name = "example_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_GetResult(
    ) -> *const std::ffi::c_void;
}

// Corresponds to example_interfaces__action__Fibonacci_GetResult
pub struct Fibonacci_GetResult;

impl rosidl_runtime_rs::Service for Fibonacci_GetResult {
    type Request = crate::vendor::example_interfaces::action::Fibonacci_GetResult_Request;
    type Response = crate::vendor::example_interfaces::action::Fibonacci_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__action__Fibonacci_GetResult()
        }
    }
}

#[link(name = "example_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__example_interfaces__action__Fibonacci(
    ) -> *const std::ffi::c_void;
}

// Corresponds to example_interfaces__action__Fibonacci
pub struct Fibonacci;

impl rosidl_runtime_rs::Action for Fibonacci {
    // --- Associated types for client library users ---
    type Goal = crate::vendor::example_interfaces::action::Fibonacci_Goal;
    type Result = crate::vendor::example_interfaces::action::Fibonacci_Result;
    type Feedback = crate::vendor::example_interfaces::action::Fibonacci_Feedback;

    // --- Associated types for client library implementation ---
    type FeedbackMessage =
        crate::vendor::example_interfaces::action::rmw::Fibonacci_FeedbackMessage;
    type SendGoalService = crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal;
    type CancelGoalService = crate::vendor::action_msgs::srv::rmw::CancelGoal;
    type GetResultService = crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult;

    // --- Methods for client library implementation ---
    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_action_type_support_handle__example_interfaces__action__Fibonacci()
        }
    }

    fn create_goal_request(
        goal_id: &[u8; 16],
        goal: crate::vendor::example_interfaces::action::rmw::Fibonacci_Goal,
    ) -> crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Request {
        crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Request {
            goal_id: crate::vendor::unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
            goal,
        }
    }

    fn split_goal_request(
        request: crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Request,
    ) -> (
        [u8; 16],
        crate::vendor::example_interfaces::action::rmw::Fibonacci_Goal,
    ) {
        (request.goal_id.uuid, request.goal)
    }

    fn create_goal_response(
        accepted: bool,
        stamp: (i32, u32),
    ) -> crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Response {
        crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Response {
            accepted,
            stamp: crate::vendor::builtin_interfaces::msg::rmw::Time {
                sec: stamp.0,
                nanosec: stamp.1,
            },
        }
    }

    fn get_goal_response_accepted(
        response: &crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Response,
    ) -> bool {
        response.accepted
    }

    fn get_goal_response_stamp(
        response: &crate::vendor::example_interfaces::action::rmw::Fibonacci_SendGoal_Response,
    ) -> (i32, u32) {
        (response.stamp.sec, response.stamp.nanosec)
    }

    fn create_feedback_message(
        goal_id: &[u8; 16],
        feedback: crate::vendor::example_interfaces::action::rmw::Fibonacci_Feedback,
    ) -> crate::vendor::example_interfaces::action::rmw::Fibonacci_FeedbackMessage {
        let mut message =
            crate::vendor::example_interfaces::action::rmw::Fibonacci_FeedbackMessage::default();
        message.goal_id.uuid = *goal_id;
        message.feedback = feedback;
        message
    }

    fn split_feedback_message(
        feedback: crate::vendor::example_interfaces::action::rmw::Fibonacci_FeedbackMessage,
    ) -> (
        [u8; 16],
        crate::vendor::example_interfaces::action::rmw::Fibonacci_Feedback,
    ) {
        (feedback.goal_id.uuid, feedback.feedback)
    }

    fn create_result_request(
        goal_id: &[u8; 16],
    ) -> crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Request {
        crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Request {
            goal_id: crate::vendor::unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
        }
    }

    fn get_result_request_uuid(
        request: &crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Request,
    ) -> &[u8; 16] {
        &request.goal_id.uuid
    }

    fn create_result_response(
        status: i8,
        result: crate::vendor::example_interfaces::action::rmw::Fibonacci_Result,
    ) -> crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Response {
        crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Response {
            status,
            result,
        }
    }

    fn split_result_response(
        response: crate::vendor::example_interfaces::action::rmw::Fibonacci_GetResult_Response,
    ) -> (
        i8,
        crate::vendor::example_interfaces::action::rmw::Fibonacci_Result,
    ) {
        (response.status, response.result)
    }
}
