pub mod error {
    use thiserror::Error;

    #[derive(Debug)]
    pub struct RCLError {
        pub code: RCLStatusCode,
        pub message: &'static str,
    }

    #[derive(Debug, Error)]
    pub enum RCLStatusCode {
        #[error("Success")]
        Ok = 0,
        #[error("Unspecified Error")]
        Error = 1,
        #[error("Timeout occurred")]
        Timeout = 2,
        #[error("Failed to allocate memory")]
        BadAlloc = 10,
        #[error("Invalid argument")]
        InvalidArgument = 11,
        #[error("Context already initialized")]
        AlreadyInit = 100,
        #[error("Context not yet initialized")]
        NotInit = 101,
        #[error("Mismatched RMW identifier")]
        MismatchedRmwId = 102,
        #[error("Topic name does not pass validation")]
        TopicNameInvalid = 103,
        #[error("Service name (same as topic name) does not pass validation")]
        ServiceNameInvalid = 104,
        #[error("Topic name substitution is unknown")]
        UnknownSubstitution = 105,
        #[error("Node already shutdown")]
        AlreadyShutdown = 106,
        #[error("Invalid node given")]
        NodeInvalid = 200,
        #[error("Invalid node name given")]
        NodeInvalidName = 201,
        #[error("Invalid node namespace given")]
        NodeInvalidNamespace = 202,
        #[error("Invalid publisher given")]
        PublisherInvalid = 300,
        #[error("Invalid subscriber given")]
        SubscriptionInvalid = 400,
        #[error("Failed to take a message from the subscription")]
        SubscriptionTakeFailed = 401,
        #[error("Invalid client given")]
        ClientInvalid = 500,
        #[error("Failed to take a response from the client")]
        ClientTakeFailed = 501,
        #[error("Invalid service given")]
        ServiceInvalid = 600,
        #[error("Failed to take a request from the service")]
        ServiceTakeFailed = 601,
        #[error("Invalid timer given")]
        TimerInvalid = 800,
        #[error("Given timer was canceled")]
        TimerCanceled = 801,
        #[error("Invalid wait set given")]
        WaitSetInvalid = 900,
        #[error("Given wait set is empty")]
        WaitSetEmpty = 901,
        #[error("Given wait set is full")]
        WaitSetFull = 902,
        #[error("Argument is not a valid remap rule")]
        InvalidRemapRule = 1001,
        #[error("Expected one type of lexeme, but got another")]
        WrongLexeme = 1002,
        #[error("Argument is not a valid parameter rule")]
        InvalidParamRule = 1010,
        #[error("Argument is not a valid log level")]
        InvalidLogLevelRule = 1020,
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
