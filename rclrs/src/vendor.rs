//! Stubs for ROS 2 interfaces to enable documentation builds without any ROS 2 installation sourced.
//!
//! This module provides no useful implementation and should only be used for building documentation.

macro_rules! impl_message_stub {
    ($name:ident) => {
        #[derive(Clone, Default, Debug, PartialEq)]
        pub struct $name;

        impl rosidl_runtime_rs::Message for $name {
            type RmwMsg = rmw::$name;
            fn into_rmw_message(
                _: std::borrow::Cow<'_, Self>,
            ) -> std::borrow::Cow<'_, Self::RmwMsg> {
                todo!()
            }
            fn from_rmw_message(_: Self::RmwMsg) -> Self {
                todo!()
            }
        }
    };
}

macro_rules! impl_rmw_message_stub {
    ($name:ident) => {
        #[derive(Clone, Default, Debug, PartialEq)]
        pub struct $name;

        impl rosidl_runtime_rs::Message for $name {
            type RmwMsg = Self;
            fn into_rmw_message(
                _: std::borrow::Cow<'_, Self>,
            ) -> std::borrow::Cow<'_, Self::RmwMsg> {
                todo!()
            }
            fn from_rmw_message(_: Self::RmwMsg) -> Self {
                todo!()
            }
        }

        impl rosidl_runtime_rs::RmwMessage for $name
        where
            Self: Sized,
        {
            const TYPE_NAME: &'static str = "";
            fn get_type_support() -> *const std::ffi::c_void {
                todo!()
            }
        }
    };
}

macro_rules! impl_service_stub {
    ($name:ident) => {
        pub struct $name;

        impl rosidl_runtime_rs::Service for $name {
            paste::paste! {
                type Request = [<$name _Request>];
                type Response = [<$name _Response>];
            }

            fn get_type_support() -> *const std::ffi::c_void {
                todo!()
            }
        }

        paste::paste! {
            impl_rmw_message_stub!([<$name _Request>]);
            impl_rmw_message_stub!([<$name _Response>]);
        }
    };
}

macro_rules! impl_sequence_alloc_stub {
    ($name:ident) => {
        impl rosidl_runtime_rs::SequenceAlloc for $name {
            fn sequence_init(_: &mut rosidl_runtime_rs::Sequence<Self>, _: usize) -> bool {
                todo!()
            }
            fn sequence_fini(_: &mut rosidl_runtime_rs::Sequence<Self>) {
                todo!()
            }
            fn sequence_copy(
                _: &rosidl_runtime_rs::Sequence<Self>,
                _: &mut rosidl_runtime_rs::Sequence<Self>,
            ) -> bool {
                todo!()
            }
        }
    };
}

#[allow(non_camel_case_types)]
/// Stub module for the [action_msgs](https://github.com/ros2/rcl_interfaces/tree/rolling/action_msgs) ROS 2 package.
///
/// Not all messages and services are implemented, only the ones used internally by [rclrs](crate).
pub mod action_msgs {
    pub mod msg {
        impl_message_stub!(GoalInfo);

        pub mod rmw {
            impl_rmw_message_stub!(GoalInfo);
        }
    }
    pub mod srv {
        impl_message_stub!(CancelGoal_Response);
        impl_message_stub!(CancelGoal_Request);

        pub mod rmw {
            impl_rmw_message_stub!(CancelGoal_Response);
            impl_rmw_message_stub!(CancelGoal_Request);
        }
    }
}

/// Stub module for the [builtin_interfaces](https://github.com/ros2/rcl_interfaces/tree/rolling/builtin_interfaces) ROS 2 package.
///
/// Not all messages are implemented, only the ones used internally by [rclrs](crate).
pub mod builtin_interfaces {
    pub mod msg {
        impl_message_stub!(Time);

        pub mod rmw {
            impl_rmw_message_stub!(Time);
        }
    }
}

/// Stub module for the [unique_identifier_msgs](https://github.com/ros2/unique_identifier_msgs/tree/rolling) ROS 2 package.
///
/// Not all messages are implemented, only the ones used internally by [rclrs](crate).
pub mod unique_identifier_msgs {
    pub mod msg {
        impl_message_stub!(UUID);

        pub mod rmw {
            impl_rmw_message_stub!(UUID);
        }
    }
}

/// Stub module for the [rosgraph_msgs](https://github.com/ros2/rcl_interfaces/tree/rolling/rosgraph_msgs) ROS 2 package.
///
/// Not all messages are implemented, only the ones used internally by [rclrs](crate).
pub mod rosgraph_msgs {
    pub mod msg {
        impl_message_stub!(Clock);

        pub mod rmw {
            impl_rmw_message_stub!(Clock);
        }
    }
}

#[allow(non_camel_case_types)]
/// Stub module for the [rcl_interfaces](https://github.com/ros2/rcl_interfaces/tree/rolling/rcl_interfaces) ROS 2 package.
///
/// Not all messages and services are implemented, only the ones used internally by [rclrs](crate).
pub mod rcl_interfaces {
    pub mod msg {
        impl_message_stub!(ParameterType);
        impl_message_stub!(ParameterValue);

        impl_message_stub!(FloatingPointRange);
        impl_message_stub!(IntegerRange);

        pub mod rmw {
            impl_rmw_message_stub!(ParameterType);
            impl_rmw_message_stub!(ParameterValue);

            impl_rmw_message_stub!(FloatingPointRange);
            impl_sequence_alloc_stub!(FloatingPointRange);

            impl_rmw_message_stub!(IntegerRange);
            impl_sequence_alloc_stub!(IntegerRange);
        }
    }

    pub mod srv {
        pub mod rmw {
            impl_service_stub!(DescribeParameters);
            impl_service_stub!(GetParameters);
            impl_service_stub!(GetParameterTypes);
            impl_service_stub!(ListParameters);
            impl_service_stub!(SetParameters);
            impl_service_stub!(SetParametersAtomically);
        }
    }
}
