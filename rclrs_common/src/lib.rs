pub mod error {
    use thiserror::Error;

    // #[derive(Debug)]
    // pub struct RCLError {
    //     pub code: RCLStatusCode,
    //     pub message: &'static str,
    // }

    #[derive(Debug, Error)]
    pub enum RCLStatusCode {
        #[error("Success")]
        Ok,
        #[error("Unspecified Error")]
        Error,
        #[error("Timeout occurred")]
        Timeout,
        #[error("Failed to allocate memory")]
        BadAlloc,
        #[error("Invalid argument")]
        InvalidArgument,
        #[error("Context already initialized")]
        AlreadyInit,
        #[error("Context not yet initialized")]
        NotInit,
        #[error("Mismatched RMW identifier")]
        MismatchedRmwId,
        #[error("Topic name does not pass validation")]
        TopicNameInvalid,
        #[error("Service name (same as topic name) does not pass validation")]
        ServiceNameInvalid,
        #[error("Topic name substitution is unknown")]
        UnknownSubstitution,
        #[error("Node already shutdown")]
        AlreadyShutdown,
        #[error("Invalid node given")]
        NodeInvalid,
        #[error("Invalid node name given")]
        NodeInvalidName,
        #[error("Invalid node namespace given")]
        NodeInvalidNamespace,
        #[error("Invalid publisher given")]
        PublisherInvalid,
        #[error("Invalid subscriber given")]
        SubscriptionInvalid,
        #[error("Failed to take a message from the subscription")]
        SubscriptionTakeFailed,
        #[error("Invalid client given")]
        ClientInvalid,
        #[error("Failed to take a response from the client")]
        ClientTakeFailed,
        #[error("Invalid service given")]
        ServiceInvalid,
        #[error("Failed to take a request from the service")]
        ServiceTakeFailed,
        #[error("Invalid timer given")]
        TimerInvalid,
        #[error("Given timer was canceled")]
        TimerCanceled,
        #[error("Invalid wait set given")]
        WaitSetInvalid,
        #[error("Given wait set is empty")]
        WaitSetEmpty,
        #[error("Given wait set is full")]
        WaitSetFull,
        #[error("Argument is not a valid remap rule")]
        InvalidRemapRule,
        #[error("Expected one type of lexeme, but got another")]
        WrongLexeme,
        #[error("Argument is not a valid parameter rule")]
        InvalidParamRule,
        #[error("Argument is not a valid log level")]
        InvalidLogLevelRule,
    }

    impl From<i32> for RCLStatusCode {
        fn from(error: i32) -> Self {
            match error {
                0 => RCLStatusCode::Ok,
                1 => RCLStatusCode::Error,
                2 => RCLStatusCode::Timeout,
                10 => RCLStatusCode::BadAlloc,
                11 => RCLStatusCode::InvalidArgument,
                100 => RCLStatusCode::AlreadyInit,
                101 => RCLStatusCode::NotInit,
                102 => RCLStatusCode::MismatchedRmwId,
                103 => RCLStatusCode::TopicNameInvalid,
                104 => RCLStatusCode::ServiceNameInvalid,
                105 => RCLStatusCode::UnknownSubstitution,
                106 => RCLStatusCode::AlreadyShutdown,
                200 => RCLStatusCode::NodeInvalid,
                201 => RCLStatusCode::NodeInvalidName,
                202 => RCLStatusCode::NodeInvalidNamespace,
                300 => RCLStatusCode::PublisherInvalid,
                400 => RCLStatusCode::SubscriptionInvalid,
                401 => RCLStatusCode::SubscriptionTakeFailed,
                500 => RCLStatusCode::ClientInvalid,
                501 => RCLStatusCode::ClientTakeFailed,
                600 => RCLStatusCode::ServiceInvalid,
                601 => RCLStatusCode::ServiceTakeFailed,
                800 => RCLStatusCode::TimerInvalid,
                801 => RCLStatusCode::TimerCanceled,
                900 => RCLStatusCode::WaitSetInvalid,
                901 => RCLStatusCode::WaitSetEmpty,
                902 => RCLStatusCode::WaitSetFull,
                1001 => RCLStatusCode::InvalidRemapRule,
                1002 => RCLStatusCode::WrongLexeme,
                1010 => RCLStatusCode::InvalidParamRule,
                1020 => RCLStatusCode::InvalidLogLevelRule,
                _ => unimplemented!(),
            }
        }
    }
}

pub mod traits {
    use downcast::{
        downcast, downcast_methods, downcast_methods_core, downcast_methods_std, impl_downcast, Any,
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
