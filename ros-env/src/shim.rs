//! Stubs for the ROS 2 interfaces `rclrs` needs, so that it can be built and
//! documented without a ROS 2 installation sourced.
//!
//! Enabled by the `use_ros_shim` feature, which also stops [`crate`] reading
//! `AMENT_PREFIX_PATH`, so these and a sourced workspace cannot collide.
//!
//! The structs carry the fields of the real messages because `cargo publish`
//! compiles what it packages. Everything below the field list is a stand-in and
//! panics. Only what `rclrs` refers to is covered.

/// The body of every stub that would need a ROS 2 installation to mean
/// anything.
macro_rules! no_ros {
    () => {
        unimplemented!("ros-env was built with `use_ros_shim`, so this interface is a stub")
    };
}

/// Implements [`rosidl_runtime_rs::Message`] for an idiomatic Rust type stub.
///
/// The generator emits every message twice, once with Rust types such as
/// [`Vec`] and once with the C layout the middleware reads. `Message` ties the
/// two together through its `RmwMsg` associated type, so this expects a sibling
/// `rmw` module holding a type of the same name, which is how the generated
/// code lays them out and how the stubs below are arranged.
///
/// The conversions between the two are the part that needs real messages, so
/// they panic.
macro_rules! stub_message {
    ($name:ident) => {
        impl rosidl_runtime_rs::Message for $name {
            type RmwMsg = rmw::$name;

            fn into_rmw_message(
                _: std::borrow::Cow<'_, Self>,
            ) -> std::borrow::Cow<'_, Self::RmwMsg> {
                no_ros!()
            }

            fn from_rmw_message(_: Self::RmwMsg) -> Self {
                no_ros!()
            }
        }
    };
}

/// Implements [`rosidl_runtime_rs::Message`] and
/// [`rosidl_runtime_rs::RmwMessage`] for a stub in an `rmw` module.
///
/// An RMW type is already in the middleware's layout, so it is its own
/// `RmwMsg` and the conversions are what the generated code makes them, a
/// no-op. They still panic here, because nothing should be converting stubs.
///
/// `TYPE_NAME` is the interface name as ROS 2 spells it, for instance
/// `rcl_interfaces/msg/ParameterValue`. It is passed in rather than built from
/// `$name` because the package and the kind are not recoverable from the type
/// name alone. `get_type_support` normally hands back a pointer into the
/// package's type support library.
macro_rules! stub_rmw_message {
    ($name:ident, $type_name:expr) => {
        impl rosidl_runtime_rs::Message for $name {
            type RmwMsg = Self;

            fn into_rmw_message(
                _: std::borrow::Cow<'_, Self>,
            ) -> std::borrow::Cow<'_, Self::RmwMsg> {
                no_ros!()
            }

            fn from_rmw_message(_: Self::RmwMsg) -> Self {
                no_ros!()
            }
        }

        impl rosidl_runtime_rs::RmwMessage for $name {
            const TYPE_NAME: &'static str = $type_name;

            fn get_type_support() -> *const std::ffi::c_void {
                no_ros!()
            }
        }
    };
}

/// Implements [`rosidl_runtime_rs::SequenceAlloc`] for a stub.
///
/// Required of any type appearing inside a `Sequence` or a `BoundedSequence`,
/// wherever that sequence lives. Both are bounded on it even for [`Default`],
/// so a message missing this impl also breaks every message that holds a
/// sequence of it.
macro_rules! stub_sequence_alloc {
    ($name:ident) => {
        impl rosidl_runtime_rs::SequenceAlloc for $name {
            fn sequence_init(_: &mut rosidl_runtime_rs::Sequence<Self>, _: usize) -> bool {
                no_ros!()
            }

            fn sequence_fini(_: &mut rosidl_runtime_rs::Sequence<Self>) {
                no_ros!()
            }

            fn sequence_copy(
                _: &rosidl_runtime_rs::Sequence<Self>,
                _: &mut rosidl_runtime_rs::Sequence<Self>,
            ) -> bool {
                no_ros!()
            }
        }
    };
}

/// Declares a service stub, along with the request and response messages that
/// [`rosidl_runtime_rs::Service`] requires as its associated types.
///
/// Takes the two field lists as `tt` fragments, the catch-all fragment, since
/// a list of struct fields is not one of the things a macro can match on
/// directly. They are written in braces at the call site and pasted into the
/// struct bodies unchanged.
///
/// `paste` builds the `Foo_Request` and `Foo_Response` identifiers, which plain
/// `macro_rules!` cannot do, since it has no way to concatenate one identifier
/// onto another.
macro_rules! stub_service {
    ($name:ident, $type_name:expr, { $($request:tt)* }, { $($response:tt)* }) => {
        paste::paste! {
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct $name;

            impl rosidl_runtime_rs::Service for $name {
                type Request = [<$name _Request>];
                type Response = [<$name _Response>];

                fn get_type_support() -> *const std::ffi::c_void {
                    no_ros!()
                }
            }

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct [<$name _Request>] {
                $($request)*
            }

            stub_rmw_message!([<$name _Request>], concat!($type_name, "_Request"));

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct [<$name _Response>] {
                $($response)*
            }

            stub_rmw_message!([<$name _Response>], concat!($type_name, "_Response"));
        }
    };
}

pub mod builtin_interfaces {
    pub mod msg {
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
        pub struct Time {
            pub sec: i32,
            pub nanosec: u32,
        }
        stub_message!(Time);

        pub mod rmw {
            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct Time {
                pub sec: i32,
                pub nanosec: u32,
            }
            stub_rmw_message!(Time, "builtin_interfaces/msg/Time");
            stub_sequence_alloc!(Time);
        }
    }
}

pub mod unique_identifier_msgs {
    pub mod msg {
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
        pub struct UUID {
            pub uuid: [u8; 16],
        }
        stub_message!(UUID);

        pub mod rmw {
            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct UUID {
                pub uuid: [u8; 16],
            }
            stub_rmw_message!(UUID, "unique_identifier_msgs/msg/UUID");
            stub_sequence_alloc!(UUID);
        }
    }
}

pub mod rosgraph_msgs {
    pub mod msg {
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
        pub struct Clock {
            pub clock: crate::builtin_interfaces::msg::Time,
        }
        stub_message!(Clock);

        pub mod rmw {
            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct Clock {
                pub clock: crate::builtin_interfaces::msg::rmw::Time,
            }
            stub_rmw_message!(Clock, "rosgraph_msgs/msg/Clock");
            stub_sequence_alloc!(Clock);
        }
    }
}

#[allow(non_camel_case_types)]
pub mod action_msgs {
    pub mod msg {
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
        pub struct GoalInfo {
            pub goal_id: crate::unique_identifier_msgs::msg::UUID,
            pub stamp: crate::builtin_interfaces::msg::Time,
        }
        stub_message!(GoalInfo);

        pub mod rmw {
            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct GoalInfo {
                pub goal_id: crate::unique_identifier_msgs::msg::rmw::UUID,
                pub stamp: crate::builtin_interfaces::msg::rmw::Time,
            }
            stub_rmw_message!(GoalInfo, "action_msgs/msg/GoalInfo");
            stub_sequence_alloc!(GoalInfo);
        }
    }

    pub mod srv {
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
        pub struct CancelGoal_Response {
            pub return_code: i8,
            pub goals_canceling: Vec<super::msg::GoalInfo>,
        }
        stub_message!(CancelGoal_Response);

        pub mod rmw {
            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct CancelGoal_Response {
                pub return_code: i8,
                pub goals_canceling:
                    rosidl_runtime_rs::Sequence<crate::action_msgs::msg::rmw::GoalInfo>,
            }
            stub_rmw_message!(CancelGoal_Response, "action_msgs/srv/CancelGoal_Response");
        }
    }
}

#[allow(non_camel_case_types)]
pub mod rcl_interfaces {
    pub mod msg {
        pub mod rmw {
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct ParameterType;

            impl ParameterType {
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
            stub_rmw_message!(ParameterType, "rcl_interfaces/msg/ParameterType");

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct FloatingPointRange {
                pub from_value: f64,
                pub to_value: f64,
                pub step: f64,
            }
            stub_rmw_message!(FloatingPointRange, "rcl_interfaces/msg/FloatingPointRange");
            stub_sequence_alloc!(FloatingPointRange);

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct IntegerRange {
                pub from_value: i64,
                pub to_value: i64,
                pub step: u64,
            }
            stub_rmw_message!(IntegerRange, "rcl_interfaces/msg/IntegerRange");
            stub_sequence_alloc!(IntegerRange);

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
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
            stub_rmw_message!(ParameterValue, "rcl_interfaces/msg/ParameterValue");
            stub_sequence_alloc!(ParameterValue);

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct ParameterDescriptor {
                pub name: rosidl_runtime_rs::String,
                pub type_: u8,
                pub description: rosidl_runtime_rs::String,
                pub additional_constraints: rosidl_runtime_rs::String,
                pub read_only: bool,
                pub dynamic_typing: bool,
                pub floating_point_range: rosidl_runtime_rs::BoundedSequence<FloatingPointRange, 1>,
                pub integer_range: rosidl_runtime_rs::BoundedSequence<IntegerRange, 1>,
            }
            stub_rmw_message!(
                ParameterDescriptor,
                "rcl_interfaces/msg/ParameterDescriptor"
            );
            stub_sequence_alloc!(ParameterDescriptor);

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct Parameter {
                pub name: rosidl_runtime_rs::String,
                pub value: ParameterValue,
            }
            stub_rmw_message!(Parameter, "rcl_interfaces/msg/Parameter");
            stub_sequence_alloc!(Parameter);

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct SetParametersResult {
                pub successful: bool,
                pub reason: rosidl_runtime_rs::String,
            }
            stub_rmw_message!(
                SetParametersResult,
                "rcl_interfaces/msg/SetParametersResult"
            );
            stub_sequence_alloc!(SetParametersResult);

            #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
            #[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
            pub struct ListParametersResult {
                pub names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
                pub prefixes: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
            }
            stub_rmw_message!(
                ListParametersResult,
                "rcl_interfaces/msg/ListParametersResult"
            );
            stub_sequence_alloc!(ListParametersResult);
        }
    }

    pub mod srv {
        pub mod rmw {
            use super::super::msg::rmw::*;

            stub_service!(
                DescribeParameters,
                "rcl_interfaces/srv/DescribeParameters",
                { pub names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>, },
                { pub descriptors: rosidl_runtime_rs::Sequence<ParameterDescriptor>, }
            );

            stub_service!(
                GetParameterTypes,
                "rcl_interfaces/srv/GetParameterTypes",
                { pub names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>, },
                { pub types: rosidl_runtime_rs::Sequence<u8>, }
            );

            stub_service!(
                GetParameters,
                "rcl_interfaces/srv/GetParameters",
                { pub names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>, },
                { pub values: rosidl_runtime_rs::Sequence<ParameterValue>, }
            );

            stub_service!(
                ListParameters,
                "rcl_interfaces/srv/ListParameters",
                {
                    pub prefixes: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
                    pub depth: u64,
                },
                { pub result: ListParametersResult, }
            );

            impl ListParameters_Request {
                pub const DEPTH_RECURSIVE: u64 = 0;
            }

            stub_service!(
                SetParameters,
                "rcl_interfaces/srv/SetParameters",
                { pub parameters: rosidl_runtime_rs::Sequence<Parameter>, },
                { pub results: rosidl_runtime_rs::Sequence<SetParametersResult>, }
            );

            stub_service!(
                SetParametersAtomically,
                "rcl_interfaces/srv/SetParametersAtomically",
                { pub parameters: rosidl_runtime_rs::Sequence<Parameter>, },
                { pub result: SetParametersResult, }
            );
        }
    }
}
