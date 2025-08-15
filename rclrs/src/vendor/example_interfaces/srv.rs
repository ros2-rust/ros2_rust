#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddTwoInts_Request {
    pub a: i64,
    pub b: i64,
}

impl Default for AddTwoInts_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::srv::rmw::AddTwoInts_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for AddTwoInts_Request {
    type RmwMsg = crate::vendor::example_interfaces::srv::rmw::AddTwoInts_Request;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { a: msg.a, b: msg.b })
            }
            std::borrow::Cow::Borrowed(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { a: msg.a, b: msg.b })
            }
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self { a: msg.a, b: msg.b }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddTwoInts_Response {
    pub sum: i64,
}

impl Default for AddTwoInts_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::srv::rmw::AddTwoInts_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for AddTwoInts_Response {
    type RmwMsg = crate::vendor::example_interfaces::srv::rmw::AddTwoInts_Response;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg { sum: msg.sum }),
            std::borrow::Cow::Borrowed(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { sum: msg.sum })
            }
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self { sum: msg.sum }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetBool_Request {
    pub data: bool,
}

impl Default for SetBool_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::srv::rmw::SetBool_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for SetBool_Request {
    type RmwMsg = crate::vendor::example_interfaces::srv::rmw::SetBool_Request;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { data: msg.data })
            }
            std::borrow::Cow::Borrowed(msg) => {
                std::borrow::Cow::Owned(Self::RmwMsg { data: msg.data })
            }
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self { data: msg.data }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetBool_Response {
    pub success: bool,
    pub message: std::string::String,
}

impl Default for SetBool_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::srv::rmw::SetBool_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for SetBool_Response {
    type RmwMsg = crate::vendor::example_interfaces::srv::rmw::SetBool_Response;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                success: msg.success,
                message: msg.message.as_str().into(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                success: msg.success,
                message: msg.message.as_str().into(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            success: msg.success,
            message: msg.message.to_string(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Trigger_Request {
    pub structure_needs_at_least_one_member: u8,
}

impl Default for Trigger_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::srv::rmw::Trigger_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Trigger_Request {
    type RmwMsg = crate::vendor::example_interfaces::srv::rmw::Trigger_Request;

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
pub struct Trigger_Response {
    pub success: bool,
    pub message: std::string::String,
}

impl Default for Trigger_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::example_interfaces::srv::rmw::Trigger_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Trigger_Response {
    type RmwMsg = crate::vendor::example_interfaces::srv::rmw::Trigger_Response;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        match msg_cow {
            std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                success: msg.success,
                message: msg.message.as_str().into(),
            }),
            std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
                success: msg.success,
                message: msg.message.as_str().into(),
            }),
        }
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        Self {
            success: msg.success,
            message: msg.message.to_string(),
        }
    }
}

#[link(name = "example_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__AddTwoInts(
    ) -> *const std::ffi::c_void;
}

// Corresponds to example_interfaces__srv__AddTwoInts
pub struct AddTwoInts;

impl rosidl_runtime_rs::Service for AddTwoInts {
    type Request = crate::vendor::example_interfaces::srv::AddTwoInts_Request;
    type Response = crate::vendor::example_interfaces::srv::AddTwoInts_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__AddTwoInts()
        }
    }
}

#[link(name = "example_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__SetBool(
    ) -> *const std::ffi::c_void;
}

// Corresponds to example_interfaces__srv__SetBool
pub struct SetBool;

impl rosidl_runtime_rs::Service for SetBool {
    type Request = crate::vendor::example_interfaces::srv::SetBool_Request;
    type Response = crate::vendor::example_interfaces::srv::SetBool_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__SetBool(
            )
        }
    }
}

#[link(name = "example_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__Trigger(
    ) -> *const std::ffi::c_void;
}

// Corresponds to example_interfaces__srv__Trigger
pub struct Trigger;

impl rosidl_runtime_rs::Service for Trigger {
    type Request = crate::vendor::example_interfaces::srv::Trigger_Request;
    type Response = crate::vendor::example_interfaces::srv::Trigger_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__Trigger(
            )
        }
    }
}

pub mod rmw {

    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__AddTwoInts_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__srv__AddTwoInts_Request__init(msg: *mut AddTwoInts_Request) -> bool;
        fn example_interfaces__srv__AddTwoInts_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Request>,
            size: usize,
        ) -> bool;
        fn example_interfaces__srv__AddTwoInts_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Request>,
        );
        fn example_interfaces__srv__AddTwoInts_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<AddTwoInts_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Request>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__srv__AddTwoInts_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct AddTwoInts_Request {
        pub a: i64,
        pub b: i64,
    }

    impl Default for AddTwoInts_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__srv__AddTwoInts_Request__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__srv__AddTwoInts_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for AddTwoInts_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__AddTwoInts_Request__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__AddTwoInts_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__AddTwoInts_Request__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for AddTwoInts_Request {
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

    impl rosidl_runtime_rs::RmwMessage for AddTwoInts_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/srv/AddTwoInts_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__AddTwoInts_Request()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__AddTwoInts_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__srv__AddTwoInts_Response__init(
            msg: *mut AddTwoInts_Response,
        ) -> bool;
        fn example_interfaces__srv__AddTwoInts_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Response>,
            size: usize,
        ) -> bool;
        fn example_interfaces__srv__AddTwoInts_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Response>,
        );
        fn example_interfaces__srv__AddTwoInts_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<AddTwoInts_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<AddTwoInts_Response>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__srv__AddTwoInts_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct AddTwoInts_Response {
        pub sum: i64,
    }

    impl Default for AddTwoInts_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__srv__AddTwoInts_Response__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__srv__AddTwoInts_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for AddTwoInts_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__AddTwoInts_Response__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__AddTwoInts_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__AddTwoInts_Response__Sequence__copy(
                    in_seq,
                    out_seq as *mut _,
                )
            }
        }
    }

    impl rosidl_runtime_rs::Message for AddTwoInts_Response {
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

    impl rosidl_runtime_rs::RmwMessage for AddTwoInts_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/srv/AddTwoInts_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__AddTwoInts_Response()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__SetBool_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__srv__SetBool_Request__init(msg: *mut SetBool_Request) -> bool;
        fn example_interfaces__srv__SetBool_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<SetBool_Request>,
            size: usize,
        ) -> bool;
        fn example_interfaces__srv__SetBool_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<SetBool_Request>,
        );
        fn example_interfaces__srv__SetBool_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<SetBool_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<SetBool_Request>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__srv__SetBool_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct SetBool_Request {
        pub data: bool,
    }

    impl Default for SetBool_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__srv__SetBool_Request__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__srv__SetBool_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for SetBool_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__SetBool_Request__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__SetBool_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__SetBool_Request__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for SetBool_Request {
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

    impl rosidl_runtime_rs::RmwMessage for SetBool_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/srv/SetBool_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__SetBool_Request()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__SetBool_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__srv__SetBool_Response__init(msg: *mut SetBool_Response) -> bool;
        fn example_interfaces__srv__SetBool_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<SetBool_Response>,
            size: usize,
        ) -> bool;
        fn example_interfaces__srv__SetBool_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<SetBool_Response>,
        );
        fn example_interfaces__srv__SetBool_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<SetBool_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<SetBool_Response>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__srv__SetBool_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct SetBool_Response {
        pub success: bool,
        pub message: rosidl_runtime_rs::String,
    }

    impl Default for SetBool_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__srv__SetBool_Response__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__srv__SetBool_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for SetBool_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__SetBool_Response__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__SetBool_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__SetBool_Response__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for SetBool_Response {
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

    impl rosidl_runtime_rs::RmwMessage for SetBool_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/srv/SetBool_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__SetBool_Response()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__Trigger_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__srv__Trigger_Request__init(msg: *mut Trigger_Request) -> bool;
        fn example_interfaces__srv__Trigger_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Trigger_Request>,
            size: usize,
        ) -> bool;
        fn example_interfaces__srv__Trigger_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Trigger_Request>,
        );
        fn example_interfaces__srv__Trigger_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Trigger_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Trigger_Request>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__srv__Trigger_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Trigger_Request {
        pub structure_needs_at_least_one_member: u8,
    }

    impl Default for Trigger_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__srv__Trigger_Request__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__srv__Trigger_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Trigger_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__Trigger_Request__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__Trigger_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__Trigger_Request__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for Trigger_Request {
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

    impl rosidl_runtime_rs::RmwMessage for Trigger_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/srv/Trigger_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__Trigger_Request()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__Trigger_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "example_interfaces__rosidl_generator_c")]
    extern "C" {
        fn example_interfaces__srv__Trigger_Response__init(msg: *mut Trigger_Response) -> bool;
        fn example_interfaces__srv__Trigger_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Trigger_Response>,
            size: usize,
        ) -> bool;
        fn example_interfaces__srv__Trigger_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Trigger_Response>,
        );
        fn example_interfaces__srv__Trigger_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Trigger_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Trigger_Response>,
        ) -> bool;
    }

    // Corresponds to example_interfaces__srv__Trigger_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Trigger_Response {
        pub success: bool,
        pub message: rosidl_runtime_rs::String,
    }

    impl Default for Trigger_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !example_interfaces__srv__Trigger_Response__init(&mut msg as *mut _) {
                    panic!("Call to example_interfaces__srv__Trigger_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Trigger_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__Trigger_Response__Sequence__init(seq as *mut _, size)
            }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { example_interfaces__srv__Trigger_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                example_interfaces__srv__Trigger_Response__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for Trigger_Response {
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

    impl rosidl_runtime_rs::RmwMessage for Trigger_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "example_interfaces/srv/Trigger_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__example_interfaces__srv__Trigger_Response()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__AddTwoInts(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to example_interfaces__srv__AddTwoInts
    pub struct AddTwoInts;

    impl rosidl_runtime_rs::Service for AddTwoInts {
        type Request = crate::vendor::example_interfaces::srv::rmw::AddTwoInts_Request;
        type Response = crate::vendor::example_interfaces::srv::rmw::AddTwoInts_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__AddTwoInts()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__SetBool(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to example_interfaces__srv__SetBool
    pub struct SetBool;

    impl rosidl_runtime_rs::Service for SetBool {
        type Request = crate::vendor::example_interfaces::srv::rmw::SetBool_Request;
        type Response = crate::vendor::example_interfaces::srv::rmw::SetBool_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__SetBool()
            }
        }
    }

    #[link(name = "example_interfaces__rosidl_typesupport_c")]
    extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__Trigger(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to example_interfaces__srv__Trigger
    pub struct Trigger;

    impl rosidl_runtime_rs::Service for Trigger {
        type Request = crate::vendor::example_interfaces::srv::rmw::Trigger_Request;
        type Response = crate::vendor::example_interfaces::srv::rmw::Trigger_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__example_interfaces__srv__Trigger()
            }
        }
    }
} // mod rmw
