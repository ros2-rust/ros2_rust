#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Arrays_Request {
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
}

impl Default for Arrays_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::srv::rmw::Arrays_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Arrays_Request {
    type RmwMsg = crate::vendor::test_msgs::srv::rmw::Arrays_Request;

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
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Arrays_Response {
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
}

impl Default for Arrays_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::srv::rmw::Arrays_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Arrays_Response {
    type RmwMsg = crate::vendor::test_msgs::srv::rmw::Arrays_Response;

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
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BasicTypes_Request {
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
    pub string_value: std::string::String,
}

impl Default for BasicTypes_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::srv::rmw::BasicTypes_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for BasicTypes_Request {
    type RmwMsg = crate::vendor::test_msgs::srv::rmw::BasicTypes_Request;

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
                string_value: msg.string_value.as_str().into(),
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
                string_value: msg.string_value.as_str().into(),
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
            string_value: msg.string_value.to_string(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BasicTypes_Response {
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
    pub string_value: std::string::String,
}

impl Default for BasicTypes_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::srv::rmw::BasicTypes_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for BasicTypes_Response {
    type RmwMsg = crate::vendor::test_msgs::srv::rmw::BasicTypes_Response;

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
                string_value: msg.string_value.as_str().into(),
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
                string_value: msg.string_value.as_str().into(),
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
            string_value: msg.string_value.to_string(),
        }
    }
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Empty_Request {
    pub structure_needs_at_least_one_member: u8,
}

impl Default for Empty_Request {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::srv::rmw::Empty_Request::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Empty_Request {
    type RmwMsg = crate::vendor::test_msgs::srv::rmw::Empty_Request;

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
pub struct Empty_Response {
    pub structure_needs_at_least_one_member: u8,
}

impl Default for Empty_Response {
    fn default() -> Self {
        <Self as rosidl_runtime_rs::Message>::from_rmw_message(
            crate::vendor::test_msgs::srv::rmw::Empty_Response::default(),
        )
    }
}

impl rosidl_runtime_rs::Message for Empty_Response {
    type RmwMsg = crate::vendor::test_msgs::srv::rmw::Empty_Response;

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

#[link(name = "test_msgs__rosidl_typesupport_c")]
unsafe extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Arrays(
    ) -> *const std::ffi::c_void;
}

// Corresponds to test_msgs__srv__Arrays
pub struct Arrays;

impl rosidl_runtime_rs::Service for Arrays {
    type Request = crate::vendor::test_msgs::srv::Arrays_Request;
    type Response = crate::vendor::test_msgs::srv::Arrays_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Arrays() }
    }
}

#[link(name = "test_msgs__rosidl_typesupport_c")]
unsafe extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__BasicTypes(
    ) -> *const std::ffi::c_void;
}

// Corresponds to test_msgs__srv__BasicTypes
pub struct BasicTypes;

impl rosidl_runtime_rs::Service for BasicTypes {
    type Request = crate::vendor::test_msgs::srv::BasicTypes_Request;
    type Response = crate::vendor::test_msgs::srv::BasicTypes_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__BasicTypes()
        }
    }
}

#[link(name = "test_msgs__rosidl_typesupport_c")]
unsafe extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Empty(
    ) -> *const std::ffi::c_void;
}

// Corresponds to test_msgs__srv__Empty
pub struct Empty;

impl rosidl_runtime_rs::Service for Empty {
    type Request = crate::vendor::test_msgs::srv::Empty_Request;
    type Response = crate::vendor::test_msgs::srv::Empty_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Empty() }
    }
}

pub mod rmw {

    #[cfg(feature = "serde")]
    use serde::{Deserialize, Serialize};

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Arrays_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__srv__Arrays_Request__init(msg: *mut Arrays_Request) -> bool;
        fn test_msgs__srv__Arrays_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Arrays_Request>,
            size: usize,
        ) -> bool;
        fn test_msgs__srv__Arrays_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Arrays_Request>,
        );
        fn test_msgs__srv__Arrays_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Arrays_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Arrays_Request>,
        ) -> bool;
    }

    // Corresponds to test_msgs__srv__Arrays_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Arrays_Request {
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
    }

    impl Default for Arrays_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__srv__Arrays_Request__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__srv__Arrays_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Arrays_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Arrays_Request__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Arrays_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Arrays_Request__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Arrays_Request {
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

    impl rosidl_runtime_rs::RmwMessage for Arrays_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/srv/Arrays_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Arrays_Request()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Arrays_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__srv__Arrays_Response__init(msg: *mut Arrays_Response) -> bool;
        fn test_msgs__srv__Arrays_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Arrays_Response>,
            size: usize,
        ) -> bool;
        fn test_msgs__srv__Arrays_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Arrays_Response>,
        );
        fn test_msgs__srv__Arrays_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Arrays_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Arrays_Response>,
        ) -> bool;
    }

    // Corresponds to test_msgs__srv__Arrays_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Arrays_Response {
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
    }

    impl Default for Arrays_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__srv__Arrays_Response__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__srv__Arrays_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Arrays_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Arrays_Response__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Arrays_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Arrays_Response__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Arrays_Response {
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

    impl rosidl_runtime_rs::RmwMessage for Arrays_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/srv/Arrays_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Arrays_Response()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__BasicTypes_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__srv__BasicTypes_Request__init(msg: *mut BasicTypes_Request) -> bool;
        fn test_msgs__srv__BasicTypes_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<BasicTypes_Request>,
            size: usize,
        ) -> bool;
        fn test_msgs__srv__BasicTypes_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<BasicTypes_Request>,
        );
        fn test_msgs__srv__BasicTypes_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<BasicTypes_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<BasicTypes_Request>,
        ) -> bool;
    }

    // Corresponds to test_msgs__srv__BasicTypes_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct BasicTypes_Request {
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
        pub string_value: rosidl_runtime_rs::String,
    }

    impl Default for BasicTypes_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__srv__BasicTypes_Request__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__srv__BasicTypes_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for BasicTypes_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__BasicTypes_Request__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__BasicTypes_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__BasicTypes_Request__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for BasicTypes_Request {
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

    impl rosidl_runtime_rs::RmwMessage for BasicTypes_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/srv/BasicTypes_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__BasicTypes_Request()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__BasicTypes_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__srv__BasicTypes_Response__init(msg: *mut BasicTypes_Response) -> bool;
        fn test_msgs__srv__BasicTypes_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<BasicTypes_Response>,
            size: usize,
        ) -> bool;
        fn test_msgs__srv__BasicTypes_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<BasicTypes_Response>,
        );
        fn test_msgs__srv__BasicTypes_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<BasicTypes_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<BasicTypes_Response>,
        ) -> bool;
    }

    // Corresponds to test_msgs__srv__BasicTypes_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct BasicTypes_Response {
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
        pub string_value: rosidl_runtime_rs::String,
    }

    impl Default for BasicTypes_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__srv__BasicTypes_Response__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__srv__BasicTypes_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for BasicTypes_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__BasicTypes_Response__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__BasicTypes_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe {
                test_msgs__srv__BasicTypes_Response__Sequence__copy(in_seq, out_seq as *mut _)
            }
        }
    }

    impl rosidl_runtime_rs::Message for BasicTypes_Response {
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

    impl rosidl_runtime_rs::RmwMessage for BasicTypes_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/srv/BasicTypes_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__BasicTypes_Response()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Empty_Request(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__srv__Empty_Request__init(msg: *mut Empty_Request) -> bool;
        fn test_msgs__srv__Empty_Request__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Empty_Request>,
            size: usize,
        ) -> bool;
        fn test_msgs__srv__Empty_Request__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Empty_Request>,
        );
        fn test_msgs__srv__Empty_Request__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Empty_Request>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Empty_Request>,
        ) -> bool;
    }

    // Corresponds to test_msgs__srv__Empty_Request
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Empty_Request {
        pub structure_needs_at_least_one_member: u8,
    }

    impl Default for Empty_Request {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__srv__Empty_Request__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__srv__Empty_Request__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Empty_Request {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Empty_Request__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Empty_Request__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Empty_Request__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Empty_Request {
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

    impl rosidl_runtime_rs::RmwMessage for Empty_Request
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/srv/Empty_Request";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Empty_Request(
                )
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Empty_Response(
        ) -> *const std::ffi::c_void;
    }

    #[link(name = "test_msgs__rosidl_generator_c")]
    unsafe extern "C" {
        fn test_msgs__srv__Empty_Response__init(msg: *mut Empty_Response) -> bool;
        fn test_msgs__srv__Empty_Response__Sequence__init(
            seq: *mut rosidl_runtime_rs::Sequence<Empty_Response>,
            size: usize,
        ) -> bool;
        fn test_msgs__srv__Empty_Response__Sequence__fini(
            seq: *mut rosidl_runtime_rs::Sequence<Empty_Response>,
        );
        fn test_msgs__srv__Empty_Response__Sequence__copy(
            in_seq: &rosidl_runtime_rs::Sequence<Empty_Response>,
            out_seq: *mut rosidl_runtime_rs::Sequence<Empty_Response>,
        ) -> bool;
    }

    // Corresponds to test_msgs__srv__Empty_Response
    #[repr(C)]
    #[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
    #[derive(Clone, Debug, PartialEq, PartialOrd)]
    pub struct Empty_Response {
        pub structure_needs_at_least_one_member: u8,
    }

    impl Default for Empty_Response {
        fn default() -> Self {
            unsafe {
                let mut msg = std::mem::zeroed();
                if !test_msgs__srv__Empty_Response__init(&mut msg as *mut _) {
                    panic!("Call to test_msgs__srv__Empty_Response__init() failed");
                }
                msg
            }
        }
    }

    impl rosidl_runtime_rs::SequenceAlloc for Empty_Response {
        fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Empty_Response__Sequence__init(seq as *mut _, size) }
        }
        fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Empty_Response__Sequence__fini(seq as *mut _) }
        }
        fn sequence_copy(
            in_seq: &rosidl_runtime_rs::Sequence<Self>,
            out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
        ) -> bool {
            // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
            unsafe { test_msgs__srv__Empty_Response__Sequence__copy(in_seq, out_seq as *mut _) }
        }
    }

    impl rosidl_runtime_rs::Message for Empty_Response {
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

    impl rosidl_runtime_rs::RmwMessage for Empty_Response
    where
        Self: Sized,
    {
        const TYPE_NAME: &'static str = "test_msgs/srv/Empty_Response";
        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_message_type_support_handle__test_msgs__srv__Empty_Response()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Arrays(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to test_msgs__srv__Arrays
    pub struct Arrays;

    impl rosidl_runtime_rs::Service for Arrays {
        type Request = crate::vendor::test_msgs::srv::rmw::Arrays_Request;
        type Response = crate::vendor::test_msgs::srv::rmw::Arrays_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Arrays()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__BasicTypes(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to test_msgs__srv__BasicTypes
    pub struct BasicTypes;

    impl rosidl_runtime_rs::Service for BasicTypes {
        type Request = crate::vendor::test_msgs::srv::rmw::BasicTypes_Request;
        type Response = crate::vendor::test_msgs::srv::rmw::BasicTypes_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__BasicTypes()
            }
        }
    }

    #[link(name = "test_msgs__rosidl_typesupport_c")]
    unsafe extern "C" {
        fn rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Empty(
        ) -> *const std::ffi::c_void;
    }

    // Corresponds to test_msgs__srv__Empty
    pub struct Empty;

    impl rosidl_runtime_rs::Service for Empty {
        type Request = crate::vendor::test_msgs::srv::rmw::Empty_Request;
        type Response = crate::vendor::test_msgs::srv::rmw::Empty_Response;

        fn get_type_support() -> *const std::ffi::c_void {
            // SAFETY: No preconditions for this function.
            unsafe {
                rosidl_typesupport_c__get_service_type_support_handle__test_msgs__srv__Empty()
            }
        }
    }
} // mod rmw
