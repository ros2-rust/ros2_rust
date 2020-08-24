pub mod error {
    use failure::Fail;

    #[derive(Debug)]
    pub struct RCLError {
        pub code: RCLStatusCode,
        pub message: &'static str,
    }

    #[derive(Debug, Fail)]
    pub enum RCLStatusCode {
        #[fail(display = "success")]
        Ok = 0,
        #[fail(display = "unspecified error")]
        Error = 1,
        #[fail(display = "timeout occurred")]
        Timeout = 2,
        #[fail(display = "failed to allocate memory")]
        BadAlloc = 10,
        #[fail(display = "invalid argument")]
        InvalidArgument = 11,
        #[fail(display = "context already initialized")]
        AlreadyInit = 100,
        #[fail(display = "context not yet initialized")]
        NotInit = 101,
        #[fail(display = "mismatched rmw identifier")]
        MismatchedRmwId = 102,
        #[fail(display = "topic name does not pass validation")]
        TopicNameInvalid = 103,
        #[fail(display = "service name (same as topic name) does not pass validation")]
        ServiceNameInvalid = 104,
        #[fail(display = "topic name substitution is unknown")]
        UnknownSubstitution = 105,
        #[fail(display = "node already shutdown")]
        AlreadyShutdown = 106,
        #[fail(display = "invalid node given")]
        NodeInvalid = 200,
        #[fail(display = "invalid node name given")]
        NodeInvalidName = 201,
        #[fail(display = "invalid node namespace given")]
        NodeInvalidNamespace = 202,
        #[fail(display = "invalid publisher given")]
        PublisherInvalid = 300,
        #[fail(display = "invalid subscriber given")]
        SubscriptionInvalid = 400,
        #[fail(display = "failed to take a message from the subscription")]
        SubscriptionTakeFailed = 401,
        #[fail(display = "invalid client given")]
        ClientInvalid = 500,
        #[fail(display = "failed to take a response from the client")]
        ClientTakeFailed = 501,
        #[fail(display = "invalid service given")]
        ServiceInvalid = 600,
        #[fail(display = "failed to take a request from the service")]
        ServiceTakeFailed = 601,
        #[fail(display = "invalid timer given")]
        TimerInvalid = 800,
        #[fail(display = "given timer was canceled")]
        TimerCanceled = 801,
        #[fail(display = "invalid wait set given")]
        WaitSetInvalid = 900,
        #[fail(display = "given wait set is empty")]
        WaitSetEmpty = 901,
        #[fail(display = "given wait set is full")]
        WaitSetFull = 902,
        #[fail(display = "argument is not a valid remap rule")]
        InvalidRemapRule = 1001,
        #[fail(display = "expected one type of lexeme but got another")]
        WrongLexeme = 1002,
        #[fail(display = "argument is not a valid parameter rule")]
        InvalidParamRule = 1010,
        #[fail(display = "argument is not a valid log level")]
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
        fn destroy_native_message(&self, message_handle: uintptr_t) -> ();
        fn read_handle(&mut self, message_handle: uintptr_t) -> ();
    }

    downcast!(dyn Message);

    pub trait MessageDefinition<T>: Message {
        fn get_type_support() -> uintptr_t;
        fn static_get_native_message(message: &T) -> uintptr_t;
        fn static_destroy_native_message(message_handle: uintptr_t) -> ();
    }
}
