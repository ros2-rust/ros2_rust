#![no_std]

use error::RclReturnCode;

pub mod error {
    use core::{convert::TryFrom, fmt::{self, Display}};


    /// RCL specific error codes start at 100
    #[derive(Debug, PartialEq)]
    pub enum RclError {
        /// `rcl_init()` already called
        AlreadyInit = 100,
        /// `rcl_init()` not yet called
        NotInit = 101,
        /// Mismatched rmw identifier
        MismatchedRmwId = 102,
        /// Topic name does not pass validation
        TopicNameInvalid = 103,
        /// Service name (same as topic name) does not pass validation
        ServiceNameInvalid = 104,
        /// Topic name substitution is unknown
        UnknownSubstitution = 105,
        /// `rcl_shutdown()` already called
        AlreadyShutdown = 106
    }

    impl TryFrom<i32> for RclError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::AlreadyInit as i32 => Ok(Self::AlreadyInit),
                x if x == Self::NotInit as i32 => Ok(Self::NotInit),
                x if x == Self::MismatchedRmwId as i32 => Ok(Self::MismatchedRmwId),
                x if x == Self::TopicNameInvalid as i32 => Ok(Self::TopicNameInvalid),
                x if x == Self::ServiceNameInvalid as i32 => Ok(Self::ServiceNameInvalid),
                x if x == Self::UnknownSubstitution as i32 => Ok(Self::UnknownSubstitution),
                x if x == Self::AlreadyShutdown as i32 => Ok(Self::AlreadyShutdown),
                other => Err(other),
            }
        }
    }

    impl Display for RclError {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
            match self {
                Self::AlreadyInit => write!(f, "RclError: `rcl_init()` already called!"),
                Self::NotInit => write!(f, "RclError: `rcl_init() not yet called!"),
                Self::MismatchedRmwId => write!(f, "RclError: Mismatched rmw identifier!"),
                Self::TopicNameInvalid => write!(f, "RclError: Topic name does not pass validation!"),
                Self::ServiceNameInvalid => write!(f, "RclError: Service name does not pass validation!"),
                Self::UnknownSubstitution => write!(f, "RclError: Topic name substitution is unknown!"),
                Self::AlreadyShutdown => write!(f, "RclError: `rcl_shutdown()` already called!"),
            }
        }
    }

    /// Error codes indicating problems in the RCL node are in 2XX
    #[derive(Debug, PartialEq)]
    pub enum NodeError {
        /// Invalid `rcl_node_t` given
        NodeInvalid = 200,
        /// Invalid node name
        NodeInvalidName = 201,
        /// Invalid node namespace
        NodeInvalidNamespace = 202,
        /// Failed to find node name
        NodeNameNonexistent = 203
    }

    impl TryFrom<i32> for NodeError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::NodeInvalid as i32 => Ok(Self::NodeInvalid),
                x if x == Self::NodeInvalidName as i32 => Ok(Self::NodeInvalidName),
                x if x == Self::NodeInvalidNamespace as i32 => Ok(Self::NodeInvalidNamespace),
                x if x == Self::NodeNameNonexistent as i32 => Ok(Self::NodeNameNonexistent),
                other => Err(other),
            }
        }
    }

    impl Display for NodeError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::NodeInvalid => write!(f, "NodeError: Invalid `rcl_node_t` given!"),
                Self::NodeInvalidName => write!(f, "NodeError: Invalid node name!"),
                Self::NodeInvalidNamespace => write!(f, "NodeError: Invalid node namespace!"),
                Self::NodeNameNonexistent => write!(f, "NodeError: Failed to find node name!"),
            }
        }
    }

    /// Error codes indicating problems in the RCL subcriber are in 4XX
    #[derive(Debug, PartialEq)]
    pub enum SubscriberError {
        /// Invalid `rcl_subscription_t` given
        SubscriptionInvalid = 400,
        /// Failed to take a message from the subscription
        SubscriptionTakeFailed = 401
    }

    impl TryFrom<i32> for SubscriberError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::SubscriptionInvalid as i32 => Ok(Self::SubscriptionInvalid),
                x if x == Self::SubscriptionTakeFailed as i32 => Ok(Self::SubscriptionTakeFailed),
                other => Err(other),
            }
        }
    }

    impl Display for SubscriberError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::SubscriptionInvalid => write!(f, "SubscriberError: Invalid `rcl_subscription_t` given!"),
                Self::SubscriptionTakeFailed => write!(f, "SubscriberError: Failed to take a message from the subscription!"),
            }
        }
    }

    /// Error codes indicating problems in the RCL client are in 5XX
    #[derive(Debug, PartialEq)]
    pub enum ClientError {
        /// Invalid `rcl_client_t` given
        ClientInvalid = 500,
        /// Failed to take a response from the client
        ClientTakeFailed = 501
    }

    impl TryFrom<i32> for ClientError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::ClientInvalid as i32 => Ok(Self::ClientInvalid),
                x if x == Self::ClientTakeFailed as i32 => Ok(Self::ClientTakeFailed),
                other => Err(other),
            }
        }
    }

    impl Display for ClientError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::ClientInvalid => write!(f, "ClientError: Invalid `rcl_client_t` given!"),
                Self::ClientTakeFailed => write!(f, "ClientError: Failed to take a response from the client!"),
            }
        }
    }

    /// Error codes indicating problems in the RCL service are in 6XX
    #[derive(Debug, PartialEq)]
    pub enum ServiceError {
        /// Invalid `rcl_service_t` given
        ServiceInvalid = 600,
        /// Failed to take a request from the service
        ServiceTakeFailed = 601
    }

    impl TryFrom<i32> for ServiceError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::ServiceInvalid as i32 => Ok(Self::ServiceInvalid),
                x if x == Self::ServiceTakeFailed as i32 => Ok(Self::ServiceTakeFailed),
                other => Err(other),
            }
        }
    }

    impl Display for ServiceError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::ServiceInvalid => write!(f, "ServiceError: Invalid `rcl_service_t` given!"),
                Self::ServiceTakeFailed => write!(f, "ServiceError: Failed to take a request from the service!"),
            }
        }
    }

    // Error codes indicating problems in RCL guard conditions are in 7XX...
    // But as of the writing of this code, they are not implemented in `rcl/types.h`!

    /// Error codes indicating problems in the RCL timer are in 8XX
    #[derive(Debug, PartialEq)]
    pub enum TimerError {
        /// Invalid `rcl_timer_t` given
        TimerInvalid = 800,
        /// Given timer was canceled
        TimerCanceled = 801
    }

    impl TryFrom<i32> for TimerError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::TimerInvalid as i32 => Ok(Self::TimerInvalid),
                x if x == Self::TimerCanceled as i32 => Ok(Self::TimerCanceled),
                other => Err(other),
            }
        }
    }

    impl Display for TimerError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::TimerInvalid => write!(f, "TimerError: Invalid `rcl_timer_t` given!"),
                Self::TimerCanceled => write!(f, "TimerError: Given timer was canceled!"),
            }
        }
    }

    /// Error codes indicating problems with RCL wait and wait set are in 9XX
    #[derive(Debug, PartialEq)]
    pub enum WaitSetError {
        /// Invalid `rcl_wait_set_t` given
        WaitSetInvalid = 900,
        /// Given `rcl_wait_set_t` is empty
        WaitSetEmpty = 901,
        /// Given `rcl_wait_set_t` is full
        WaitSetFull = 902,
    }

    impl TryFrom<i32> for WaitSetError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::WaitSetInvalid as i32 => Ok(Self::WaitSetInvalid),
                x if x == Self::WaitSetEmpty as i32 => Ok(Self::WaitSetEmpty),
                x if x == Self::WaitSetFull as i32 => Ok(Self::WaitSetFull),
                other => Err(other),
            }
        }
    }

    impl Display for WaitSetError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::WaitSetInvalid => write!(f, "WaitSetError: Invalid `rcl_wait_set_t` given!"),
                Self::WaitSetEmpty => write!(f, "WaitSetError: Given `rcl_wait_set_t` was empty!"),
                Self::WaitSetFull => write!(f, "WaitSetError: Given `rcl_wait_set_t` was full!"),
            }
        }
    }

    /// Error codes indicating problems with RCL argument parsing are in 1XXX
    #[derive(Debug, PartialEq)]
    pub enum ParsingError {
        /// Argument is not a valid remap rule
        InvalidRemapRule = 1001,
        /// Expected one type of lexeme but got another
        WrongLexeme = 1002,
        /// Found invalid ros argument while parsing
        InvalidRosArgs = 1003,
        /// Argument is not a valid parameter rule
        InvalidParamRule = 1010,
        /// Argument is not a valid log level rule
        InvalidLogLevelRule = 1020
    }

    impl TryFrom<i32> for ParsingError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::InvalidRemapRule as i32 => Ok(Self::InvalidRemapRule),
                x if x == Self::WrongLexeme as i32 => Ok(Self::WrongLexeme),
                x if x == Self::InvalidRosArgs as i32 => Ok(Self::InvalidRosArgs),
                x if x == Self::InvalidParamRule as i32 => Ok(Self::InvalidParamRule),
                x if x == Self::InvalidLogLevelRule as i32 => Ok(Self::InvalidLogLevelRule),
                other => Err(other),
            }
        }
    }

    impl Display for ParsingError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::InvalidRemapRule => write!(f, "ParsingError: Argument is not a valid remap rule!"),
                Self::WrongLexeme => write!(f, "ParsingError: Expected one type of lexeme, but got another!"),
                Self::InvalidRosArgs => write!(f, "ParsingError: Found invalid ros argument while parsing!"),
                Self::InvalidParamRule => write!(f, "ParsingError: Argument is not a valid parameter rule!"),
                Self::InvalidLogLevelRule => write!(f, "ParsingError: Argument is not a valid log level rule!"),
            }
        }
    }

    /// Error codes indicating problems with RCL events are in 20XX
    #[derive(Debug, PartialEq)]
    pub enum EventError {
        /// Invalid `rcl_event_t` given
        EventInvalid = 2000,
        /// Failed to take an event from the event handle
        EventTakeFailed = 2001,
    }

    impl TryFrom<i32> for EventError {
        type Error = i32;

        fn try_from(value:i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::EventInvalid as i32 => Ok(Self::EventInvalid),
                x if x == Self::EventTakeFailed as i32 => Ok(Self::EventTakeFailed),
                other => Err(other),
            }
        }
    }

    impl Display for EventError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::EventInvalid => write!(f, "EventError: Invalid `rcl_event_t` given!"),
                Self::EventTakeFailed => write!(f, "EventError: Failed to take an event from the event handle!"),
            }
        }
    }

    /// Error codes indicating problems with RCL lifecycle state register are in 30XX
    #[derive(Debug, PartialEq)]
    pub enum LifecycleError {
        /// `rcl_lifecycle` state registered
        LifecycleStateRegistered = 3000,
        /// `rcl_lifecycle` state not registered
        LifecycleStateNotRegistered = 3001,
    }

    impl TryFrom<i32> for LifecycleError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                x if x == Self::LifecycleStateRegistered as i32 => Ok(Self::LifecycleStateRegistered),
                x if x == Self::LifecycleStateNotRegistered as i32 => Ok(Self::LifecycleStateNotRegistered),
                other => Err(other),
            }
        }
    }

    impl Display for LifecycleError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::LifecycleStateRegistered => write!(f, "LifecycleError: `rcl_lifecycle` state registered!"),
                Self::LifecycleStateNotRegistered => write!(f, "LifecycleError: `rcl_lifecycle` state not registered!"),
            }
        }
    }

    /// Return codes generated by an RCL command/process
    #[derive(Debug, PartialEq)]
    pub enum RclReturnCode {
        /// Success
        Ok,
        /// Unspecified error
        Error,
        /// Timeout occurred
        Timeout,
        /// Unsupported return code
        Unsupported,
        /// Failed to allocate memory
        BadAlloc,
        /// Argument to function was invalid
        InvalidArgument,
        /// `rcl`-specific error occurred
        RclError(RclError),
        /// `rcl` node-specific error occurred
        NodeError(NodeError),
        /// Invalid `rcl_publisher_t` given
        PublisherInvalid,
        /// `rcl` subscriptionerror occurred
        SubscriberError(SubscriberError),
        /// `rcl` client error occurred
        ClientError(ClientError),
        /// `rcl` service error occurred
        ServiceError(ServiceError),
        /// `rcl` timer error occurred
        TimerError(TimerError),
        /// `rcl` wait or waitset error occurred
        WaitSetError(WaitSetError),
        /// `rcl` argument parsing error occurred
        ParsingError(ParsingError),
        /// `rcl` event error occurred
        EventError(EventError),
        /// `rcl` lifecycle error occurred
        LifecycleError(LifecycleError),
        /// Unrecognized/unimplemented error code
        UnknownError(i32)
    }

    impl From<i32> for RclReturnCode {
        fn from(value: i32) -> Self {
            match value {
                0 => Self::Ok,
                1 => Self::Error,
                2 => Self::Timeout,
                3 => Self::Unsupported,
                10 => Self::BadAlloc,
                11 => Self::InvalidArgument,
                rcl_err @ 100 ..= 199 => match RclError::try_from(rcl_err) {
                    Ok(code) => Self::RclError(code),
                    Err(e) => Self::UnknownError(e),
                },
                node_err @ 200 ..= 299 => match NodeError::try_from(node_err) {
                    Ok(code) => Self::NodeError(code),
                    Err(e) => Self::UnknownError(e),
                },
                300 => Self::PublisherInvalid,
                subscriber_err @ 400 ..= 499 => match SubscriberError::try_from(subscriber_err) {
                    Ok(code) => Self::SubscriberError(code),
                    Err(e) => Self::UnknownError(e),
                },
                client_err @ 500 ..= 599 => match ClientError::try_from(client_err) {
                    Ok(code) => Self::ClientError(code),
                    Err(e) => Self::UnknownError(e),
                },
                service_err @ 600 ..= 699 => match ServiceError::try_from(service_err) {
                    Ok(code) => Self::ServiceError(code),
                    Err(e) => Self::UnknownError(e),
                },
                timer_err @ 800 ..= 899 => match TimerError::try_from(timer_err) {
                    Ok(code) => Self::TimerError(code),
                    Err(e) => Self::UnknownError(e),
                },
                waitset_err @ 900 ..= 999 => match WaitSetError::try_from(waitset_err) {
                    Ok(code) => Self::WaitSetError(code),
                    Err(e) => Self::UnknownError(e),
                },
                parse_err @ 1000 ..= 1999 => match ParsingError::try_from(parse_err) {
                    Ok(code) => Self::ParsingError(code),
                    Err(e) => Self::UnknownError(e),
                },
                event_err @ 2000 ..= 2099 => match EventError::try_from(event_err) {
                    Ok(code) => Self::EventError(code),
                    Err(e) => Self::UnknownError(e),
                },
                lifecycle_err @ 3000 ..= 3099 => match LifecycleError::try_from(lifecycle_err) {
                    Ok(code) => Self::LifecycleError(code),
                    Err(e) => Self::UnknownError(e),
                },
                other => Self::UnknownError(other),
            }
        }
    }

    impl Display for RclReturnCode {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::Ok => write!(f, "RclReturnCode: Operation successful!"),
                Self::Error => write!(f, "RclReturnCode: Unspecified error!"),
                Self::Timeout => write!(f, "RclReturnCode: Timeout occurred!"),
                Self::Unsupported => write!(f, "RclReturnCode: Unsupported return code!"),
                Self::BadAlloc => write!(f, "RclReturnCode: Failed to allocate memory!"),
                Self::InvalidArgument => write!(f, "RclReturnCode: Argument to function was invalid!"),
                Self::RclError(rcl_err) => write!(f, "RclReturnCode::{}", rcl_err),
                Self::NodeError(node_err) => write!(f, "RclReturnCode::{}", node_err),
                Self::PublisherInvalid => write!(f, "RclReturnCode: Invalid `rcl_publisher_t` given!"),
                Self::SubscriberError(subscriber_err) => write!(f, "RclReturnCode::{}", subscriber_err),
                Self::ClientError(client_err) => write!(f, "RclReturnCode::{}", client_err),
                Self::ServiceError(service_err) => write!(f, "RclReturnCode::{}", service_err),
                Self::TimerError(timer_err) => write!(f, "RclReturnCode::{}", timer_err),
                Self::WaitSetError(waitset_err) => write!(f, "RclReturnCode::{}", waitset_err),
                Self::ParsingError(parse_err) => write!(f, "RclReturnCode::{}", parse_err),
                Self::EventError(event_err) => write!(f, "RclReturnCode::{}", event_err),
                Self::LifecycleError(lifecycle_err) => write!(f, "RclReturnCode::{}", lifecycle_err),
                Self::UnknownError(unknown_err) => write!(f, "RclReturnCode: Unknown error code -> `{}`", unknown_err)
            }
        }
    }

    pub fn to_rcl_result(code: i32) -> Result<(), RclReturnCode> {
        match RclReturnCode::from(code) {
            RclReturnCode::Ok => Ok(()),
            anything_else => Err(anything_else),
        }
    }
}


pub mod traits {
    use downcast::{
        downcast, downcast_methods, downcast_methods_core, impl_downcast, Any,
    };
    use libc::uintptr_t;

    pub trait Message: Any {
        fn get_native_message(&self) -> uintptr_t;
        fn destroy_native_message(&self, message_handle: uintptr_t);
        fn read_handle(&mut self, message_handle: uintptr_t);
    }

    downcast!(dyn Message);

    pub trait MessageDefinition<T>: Message {
        fn get_type_support() -> uintptr_t;
        fn static_get_native_message(message: &T) -> uintptr_t;
        fn static_destroy_native_message(message_handle: uintptr_t);
    }
}
#[cfg(test)]
mod tests {
    use core::convert::TryFrom;

    use crate::error::{ClientError, EventError, LifecycleError, NodeError, ParsingError, RclError, RclReturnCode, ServiceError, SubscriberError, TimerError, WaitSetError};


    #[test]
    fn test_ok() {
        assert_eq!(RclReturnCode::from(0), RclReturnCode::Ok);
    }

    #[test]
    fn test_error() {
        assert_eq!(RclReturnCode::from(1), RclReturnCode::Error);
    }

    #[test]
    fn test_timeout() {
        assert_eq!(RclReturnCode::from(2), RclReturnCode::Timeout);
    }

    #[test]
    fn test_unsupported() {
        assert_eq!(RclReturnCode::from(3), RclReturnCode::Unsupported);
    }

    #[test]
    fn test_bad_alloc() {
        assert_eq!(RclReturnCode::from(10), RclReturnCode::BadAlloc);
    }

    #[test]
    fn test_invalid_arg() {
        assert_eq!(RclReturnCode::from(11), RclReturnCode::InvalidArgument);
    }

    /////////////////////
    // RclError checks //
    /////////////////////
    #[test]
    fn test_already_init() {
        assert_eq!(RclError::try_from(100).unwrap(), RclError::AlreadyInit);
        assert_eq!(RclReturnCode::from(100), RclReturnCode::RclError(RclError::AlreadyInit));
    }

    #[test]
    fn test_not_init() {
        assert_eq!(RclError::try_from(101).unwrap(), RclError::NotInit);
        assert_eq!(RclReturnCode::from(101), RclReturnCode::RclError(RclError::NotInit));
    }

    #[test]
    fn test_mismatched_rmw_id() {
        assert_eq!(RclError::try_from(102).unwrap(), RclError::MismatchedRmwId);
        assert_eq!(RclReturnCode::from(102), RclReturnCode::RclError(RclError::MismatchedRmwId));
    }

    #[test]
    fn test_topic_name_invalid() {
        assert_eq!(RclError::try_from(103).unwrap(), RclError::TopicNameInvalid);
        assert_eq!(RclReturnCode::from(103), RclReturnCode::RclError(RclError::TopicNameInvalid));
    }

    #[test]
    fn test_service_name_invaid() {
        assert_eq!(RclError::try_from(104).unwrap(), RclError::ServiceNameInvalid);
        assert_eq!(RclReturnCode::from(104), RclReturnCode::RclError(RclError::ServiceNameInvalid));
    }

    #[test]
    fn test_unknown_substitution() {
        assert_eq!(RclError::try_from(105).unwrap(), RclError::UnknownSubstitution);
        assert_eq!(RclReturnCode::from(105), RclReturnCode::RclError(RclError::UnknownSubstitution));
    }

    #[test]
    fn test_already_shutdown() {
        assert_eq!(RclError::try_from(106).unwrap(), RclError::AlreadyShutdown);
        assert_eq!(RclReturnCode::from(106), RclReturnCode::RclError(RclError::AlreadyShutdown));
    }

    //////////////////////
    // NodeError checks //
    //////////////////////
    #[test]
    fn test_node_invalid() {
        assert_eq!(NodeError::try_from(200).unwrap(), NodeError::NodeInvalid);
        assert_eq!(RclReturnCode::from(200), RclReturnCode::NodeError(NodeError::NodeInvalid));
    }

    #[test]
    fn test_node_invalid_name() {
        assert_eq!(NodeError::try_from(201).unwrap(), NodeError::NodeInvalidName);
        assert_eq!(RclReturnCode::from(201), RclReturnCode::NodeError(NodeError::NodeInvalidName));
    }

    #[test]
    fn test_node_invalid_namespace() {
        assert_eq!(NodeError::try_from(202).unwrap(), NodeError::NodeInvalidNamespace);
        assert_eq!(RclReturnCode::from(202), RclReturnCode::NodeError(NodeError::NodeInvalidNamespace));
    }

    #[test]
    fn test_node_name_nonexistent() {
        assert_eq!(NodeError::try_from(203).unwrap(), NodeError::NodeNameNonexistent);
        assert_eq!(RclReturnCode::from(203), RclReturnCode::NodeError(NodeError::NodeNameNonexistent));
    }

    /////////////////////
    // Publisher check //
    /////////////////////
    #[test]
    fn test_publisher_invalid() {
        assert_eq!(RclReturnCode::from(300), RclReturnCode::PublisherInvalid);
    }

    ////////////////////////////
    // SubscriberError checks //
    ////////////////////////////
    #[test]
    fn test_subscription_invalid() {
        assert_eq!(SubscriberError::try_from(400).unwrap(), SubscriberError::SubscriptionInvalid);
        assert_eq!(RclReturnCode::from(400), RclReturnCode::SubscriberError(SubscriberError::SubscriptionInvalid));
    }

    #[test]
    fn test_subscription_take_failed() {
        assert_eq!(SubscriberError::try_from(401).unwrap(), SubscriberError::SubscriptionTakeFailed);
        assert_eq!(RclReturnCode::from(401), RclReturnCode::SubscriberError(SubscriberError::SubscriptionTakeFailed));
    }

    ////////////////////////
    // ClientError checks //
    ////////////////////////
    #[test]
    fn test_client_invalid() {
        assert_eq!(ClientError::try_from(500).unwrap(), ClientError::ClientInvalid);
        assert_eq!(RclReturnCode::from(500), RclReturnCode::ClientError(ClientError::ClientInvalid));
    }

    #[test]
    fn test_client_take_failed() {
        assert_eq!(ClientError::try_from(501).unwrap(), ClientError::ClientTakeFailed);
        assert_eq!(RclReturnCode::from(501), RclReturnCode::ClientError(ClientError::ClientTakeFailed));
    }

    /////////////////////////
    // ServiceError checks //
    /////////////////////////
    #[test]
    fn test_service_invalid() {
        assert_eq!(ServiceError::try_from(600).unwrap(), ServiceError::ServiceInvalid);
        assert_eq!(RclReturnCode::from(600), RclReturnCode::ServiceError(ServiceError::ServiceInvalid));
    }

    #[test]
    fn test_service_take_failed() {
        assert_eq!(ServiceError::try_from(601).unwrap(), ServiceError::ServiceTakeFailed);
        assert_eq!(RclReturnCode::from(601), RclReturnCode::ServiceError(ServiceError::ServiceTakeFailed));
    }

    ///////////////////////
    // TimerError checks //
    ///////////////////////
    #[test]
    fn test_timer_invalid() {
        assert_eq!(TimerError::try_from(800).unwrap(), TimerError::TimerInvalid);
        assert_eq!(RclReturnCode::from(800), RclReturnCode::TimerError(TimerError::TimerInvalid));
    }

    #[test]
    fn test_timer_canceled() {
        assert_eq!(TimerError::try_from(801).unwrap(), TimerError::TimerCanceled);
        assert_eq!(RclReturnCode::from(801), RclReturnCode::TimerError(TimerError::TimerCanceled));
    }

    /////////////////////////
    // WaitSetError checks //
    /////////////////////////
    #[test]
    fn test_wait_set_invalid() {
        assert_eq!(WaitSetError::try_from(900).unwrap(), WaitSetError::WaitSetInvalid);
        assert_eq!(RclReturnCode::from(900), RclReturnCode::WaitSetError(WaitSetError::WaitSetInvalid));
    }

    #[test]
    fn test_wait_set_empty() {
        assert_eq!(WaitSetError::try_from(901).unwrap(), WaitSetError::WaitSetEmpty);
        assert_eq!(RclReturnCode::from(901), RclReturnCode::WaitSetError(WaitSetError::WaitSetEmpty));
    }

    #[test]
    fn test_wait_set_full() {
        assert_eq!(WaitSetError::try_from(902).unwrap(), WaitSetError::WaitSetFull);
        assert_eq!(RclReturnCode::from(902), RclReturnCode::WaitSetError(WaitSetError::WaitSetFull));
    }

    /////////////////////////
    // ParsingError checks //
    /////////////////////////
    #[test]
    fn test_invalid_remap_rule() {
        assert_eq!(ParsingError::try_from(1001).unwrap(), ParsingError::InvalidRemapRule);
        assert_eq!(RclReturnCode::from(1001), RclReturnCode::ParsingError(ParsingError::InvalidRemapRule));
    }

    #[test]
    fn test_wrong_lexeme() {
        assert_eq!(ParsingError::try_from(1002).unwrap(), ParsingError::WrongLexeme);
        assert_eq!(RclReturnCode::from(1002), RclReturnCode::ParsingError(ParsingError::WrongLexeme));
    }

    #[test]
    fn test_invalid_ros_args() {
        assert_eq!(ParsingError::try_from(1003).unwrap(), ParsingError::InvalidRosArgs);
        assert_eq!(RclReturnCode::from(1003), RclReturnCode::ParsingError(ParsingError::InvalidRosArgs));
    }

    #[test]
    fn test_invalid_param_rule() {
        assert_eq!(ParsingError::try_from(1010).unwrap(), ParsingError::InvalidParamRule);
        assert_eq!(RclReturnCode::from(1010), RclReturnCode::ParsingError(ParsingError::InvalidParamRule));
    }

    #[test]
    fn test_invalid_log_level_rule() {
        assert_eq!(ParsingError::try_from(1020).unwrap(), ParsingError::InvalidLogLevelRule);
        assert_eq!(RclReturnCode::from(1020), RclReturnCode::ParsingError(ParsingError::InvalidLogLevelRule));
    }

    ///////////////////////
    // EventError checks //
    ///////////////////////
    #[test]
    fn test_event_invalid() {
        assert_eq!(EventError::try_from(2000).unwrap(), EventError::EventInvalid);
        assert_eq!(RclReturnCode::from(2000), RclReturnCode::EventError(EventError::EventInvalid));
    }

    #[test]
    fn test_event_take_failed() {
        assert_eq!(EventError::try_from(2001).unwrap(), EventError::EventTakeFailed);
        assert_eq!(RclReturnCode::from(2001), RclReturnCode::EventError(EventError::EventTakeFailed));
    }

    ///////////////////////////
    // LifecycleError checks //
    ///////////////////////////
    #[test]
    fn test_lifecycle_state_registered() {
        assert_eq!(LifecycleError::try_from(3000).unwrap(), LifecycleError::LifecycleStateRegistered);
        assert_eq!(RclReturnCode::from(3000), RclReturnCode::LifecycleError(LifecycleError::LifecycleStateRegistered));
    }

    #[test]
    fn test_lifecycle_state_not_registered() {
        assert_eq!(LifecycleError::try_from(3001).unwrap(), LifecycleError::LifecycleStateNotRegistered);
        assert_eq!(RclReturnCode::from(3001), RclReturnCode::LifecycleError(LifecycleError::LifecycleStateNotRegistered));
    }

    ////////////////////////
    // UnknownError check //
    ////////////////////////
    #[test]
    fn test_unknown_error() {
        assert_eq!(RclReturnCode::from(-42), RclReturnCode::UnknownError(-42));
    }
}
