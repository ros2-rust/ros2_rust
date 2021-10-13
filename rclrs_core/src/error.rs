pub mod error {
    use core::convert::TryFrom;


    /// RCL specific error codes start at 100
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    #[derive(Debug)]
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
    use crate::error::error::RclReturnCode;

    #[test]
    fn test_ok() {
        match RclReturnCode::from(0) {
            RclReturnCode::Ok => (),
            x => panic!("Expected RclReturnCode::Ok, instead got {:?}", x)
        }
    }
}