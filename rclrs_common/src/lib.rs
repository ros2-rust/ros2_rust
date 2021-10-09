pub mod error {
    use thiserror::Error;

    #[derive(Debug, Error)]
    pub enum RclError {
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
        #[error("Unknown RCL return code: {0}")]
        UnknownError(i32),
    }

    pub fn to_rcl_result(error_id: i32) -> Result<(), RclError> {
        match error_id {
            0 => Ok(()),
            1 => Err(RclError::Error),
            2 => Err(RclError::Timeout),
            10 => Err(RclError::BadAlloc),
            11 => Err(RclError::InvalidArgument),
            100 => Err(RclError::AlreadyInit),
            101 => Err(RclError::NotInit),
            102 => Err(RclError::MismatchedRmwId),
            103 => Err(RclError::TopicNameInvalid),
            104 => Err(RclError::ServiceNameInvalid),
            105 => Err(RclError::UnknownSubstitution),
            106 => Err(RclError::AlreadyShutdown),
            200 => Err(RclError::NodeInvalid),
            201 => Err(RclError::NodeInvalidName),
            202 => Err(RclError::NodeInvalidNamespace),
            300 => Err(RclError::PublisherInvalid),
            400 => Err(RclError::SubscriptionInvalid),
            401 => Err(RclError::SubscriptionTakeFailed),
            500 => Err(RclError::ClientInvalid),
            501 => Err(RclError::ClientTakeFailed),
            600 => Err(RclError::ServiceInvalid),
            601 => Err(RclError::ServiceTakeFailed),
            800 => Err(RclError::TimerInvalid),
            801 => Err(RclError::TimerCanceled),
            900 => Err(RclError::WaitSetInvalid),
            901 => Err(RclError::WaitSetEmpty),
            902 => Err(RclError::WaitSetFull),
            1001 => Err(RclError::InvalidRemapRule),
            1002 => Err(RclError::WrongLexeme),
            1010 => Err(RclError::InvalidParamRule),
            1020 => Err(RclError::InvalidLogLevelRule),
            unrecognized => Err(RclError::UnknownError(unrecognized)),
        }
    }

    #[derive(Debug, Error)]
    pub enum WaitSetError {
        #[error("Passed subscription was dropped")]
        DroppedSubscription,
        #[error("Rcl error occurred")]
        RclError(#[from] RclError),
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
