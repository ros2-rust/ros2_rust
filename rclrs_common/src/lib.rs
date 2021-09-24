pub mod error {
    use thiserror::Error;

    #[derive(Debug, Error)]
    pub enum RclError {
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

    impl From<i32> for RclError {
        fn from(error: i32) -> Self {
            match error {
                0 => RclError::Ok,
                1 => RclError::Error,
                2 => RclError::Timeout,
                10 => RclError::BadAlloc,
                11 => RclError::InvalidArgument,
                100 => RclError::AlreadyInit,
                101 => RclError::NotInit,
                102 => RclError::MismatchedRmwId,
                103 => RclError::TopicNameInvalid,
                104 => RclError::ServiceNameInvalid,
                105 => RclError::UnknownSubstitution,
                106 => RclError::AlreadyShutdown,
                200 => RclError::NodeInvalid,
                201 => RclError::NodeInvalidName,
                202 => RclError::NodeInvalidNamespace,
                300 => RclError::PublisherInvalid,
                400 => RclError::SubscriptionInvalid,
                401 => RclError::SubscriptionTakeFailed,
                500 => RclError::ClientInvalid,
                501 => RclError::ClientTakeFailed,
                600 => RclError::ServiceInvalid,
                601 => RclError::ServiceTakeFailed,
                800 => RclError::TimerInvalid,
                801 => RclError::TimerCanceled,
                900 => RclError::WaitSetInvalid,
                901 => RclError::WaitSetEmpty,
                902 => RclError::WaitSetFull,
                1001 => RclError::InvalidRemapRule,
                1002 => RclError::WrongLexeme,
                1010 => RclError::InvalidParamRule,
                1020 => RclError::InvalidLogLevelRule,
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
