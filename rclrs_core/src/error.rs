// Copyright 2021 DCS Corporation, All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584

pub mod error {
    use core::convert::TryFrom;


    /// RCL specific error codes start at 100
    #[derive(Debug, PartialEq)]
    pub enum RclError {
        /// `rcl_init()` already called
        AlreadyInit,
        /// `rcl_init()` not yet called
        NotInit,
        /// Mismatched rmw identifier
        MismatchedRmwId,
        /// Topic name does not pass validation
        TopicNameInvalid,
        /// Service name (same as topic name) does not pass validation
        ServiceNameInvalid,
        /// Topic name substitution is unknown
        UnknownSubstitution,
        /// `rcl_shutdown()` already called
        AlreadyShutdown
    }

    impl TryFrom<i32> for RclError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                100 => Ok(Self::AlreadyInit),
                101 => Ok(Self::NotInit),
                102 => Ok(Self::MismatchedRmwId),
                103 => Ok(Self::TopicNameInvalid),
                104 => Ok(Self::ServiceNameInvalid),
                105 => Ok(Self::UnknownSubstitution),
                106 => Ok(Self::AlreadyShutdown),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems in the RCL node are in 2XX
    #[derive(Debug, PartialEq)]
    pub enum NodeError {
        /// Invalid `rcl_node_t` given
        NodeInvalid,
        /// Invalid node name
        NodeInvalidName,
        /// Invalid node namespace
        NodeInvalidNamespace,
        /// Failed to find node name
        NodeNameNonexistent
    }

    impl TryFrom<i32> for NodeError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                200 => Ok(Self::NodeInvalid),
                201 => Ok(Self::NodeInvalidName),
                202 => Ok(Self::NodeInvalidNamespace),
                203 => Ok(Self::NodeNameNonexistent),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems in the RCL subcriber are in 4XX
    #[derive(Debug, PartialEq)]
    pub enum SubscriberError {
        /// Invalid `rcl_subscription_t` given
        SubscriptionInvalid,
        /// Failed to take a message from the subscription
        SubscriptionTakeFailed
    }

    impl TryFrom<i32> for SubscriberError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                400 => Ok(Self::SubscriptionInvalid),
                401 => Ok(Self::SubscriptionTakeFailed),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems in the RCL client are in 5XX
    #[derive(Debug, PartialEq)]
    pub enum ClientError {
        /// Invalid `rcl_client_t` given
        ClientInvalid,
        /// Failed to take a response from the client
        ClientTakeFailed
    }

    impl TryFrom<i32> for ClientError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                500 => Ok(Self::ClientInvalid),
                501 => Ok(Self::ClientTakeFailed),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems in the RCL service are in 6XX
    #[derive(Debug, PartialEq)]
    pub enum ServiceError {
        /// Invalid `rcl_service_t` given
        ServiceInvalid,
        /// Failed to take a request from the service
        ServiceTakeFailed
    }

    impl TryFrom<i32> for ServiceError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                600 => Ok(Self::ServiceInvalid),
                601 => Ok(Self::ServiceTakeFailed),
                other => Err(other),
            }
        }
    }

    // Error codes indicating problems in RCL guard conditions are in 7XX...
    // But as of the writing of this code, they are not implemented in `rcl/types.h`!

    /// Error codes indicating problems in the RCL timer are in 8XX
    #[derive(Debug, PartialEq)]
    pub enum TimerError {
        /// Invalid `rcl_timer_t` given
        TimerInvalid,
        /// Given timer was canceled
        TimerCanceled
    }

    impl TryFrom<i32> for TimerError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                800 => Ok(Self::TimerInvalid),
                801 => Ok(Self::TimerCanceled),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems with RCL wait and wait set are in 9XX
    #[derive(Debug, PartialEq)]
    pub enum WaitSetError {
        /// Invalid `rcl_wait_set_t` given
        WaitSetInvalid,
        /// Given `rcl_wait_set_t` is empty
        WaitSetEmpty,
        /// Given `rcl_wait_set_t` is full
        WaitSetFull,
    }

    impl TryFrom<i32> for WaitSetError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                900 => Ok(Self::WaitSetInvalid),
                901 => Ok(Self::WaitSetEmpty),
                902 => Ok(Self::WaitSetFull),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems with RCL argument parsing are in 1XXX
    #[derive(Debug, PartialEq)]
    pub enum ParsingError {
        /// Argument is not a valid remap rule
        InvalidRemapRule,
        /// Expected one type of lexeme but got another
        WrongLexeme,
        /// Found invalid ros argument while parsing
        InvalidRosArgs,
        /// Argument is not a valid parameter rule
        InvalidParamRule,
        /// Argument is not a valid log level rule
        InvalidLogLevelRule
    }

    impl TryFrom<i32> for ParsingError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                1001 => Ok(Self::InvalidRemapRule),
                1002 => Ok(Self::WrongLexeme),
                1003 => Ok(Self::InvalidRosArgs),
                1010 => Ok(Self::InvalidParamRule),
                1020 => Ok(Self::InvalidLogLevelRule),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems with RCL events are in 20XX
    #[derive(Debug, PartialEq)]
    pub enum EventError {
        /// Invalid `rcl_event_t` given
        EventInvalid,
        /// Failed to take an event from the event handle
        EventTakeFailed,
    }

    impl TryFrom<i32> for EventError {
        type Error = i32;

        fn try_from(value:i32) -> Result<Self, Self::Error> {
            match value {
                2000 => Ok(Self::EventInvalid),
                2001 => Ok(Self::EventTakeFailed),
                other => Err(other),
            }
        }
    }

    /// Error codes indicating problems with RCL lifecycle state register are in 30XX
    #[derive(Debug, PartialEq)]
    pub enum LifecycleError {
        /// `rcl_lifecycle` state registered
        LifecycleStateRegistered,
        /// `rcl_lifecycle` state not registered
        LifecycleStateNotRegistered,
    }

    impl TryFrom<i32> for LifecycleError {
        type Error = i32;

        fn try_from(value: i32) -> Result<Self, Self::Error> {
            match value {
                3000 => Ok(Self::LifecycleStateRegistered),
                3001 => Ok(Self::LifecycleStateNotRegistered),
                other => Err(other),
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
}

#[cfg(test)]
mod tests {
    use core::convert::TryFrom;

    use crate::error::error::{ClientError, EventError, LifecycleError, NodeError, ParsingError, RclError, RclReturnCode, ServiceError, SubscriberError, TimerError, WaitSetError};

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