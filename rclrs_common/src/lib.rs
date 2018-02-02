#[macro_use]
extern crate downcast;
extern crate libc;

pub mod error {
    #[derive(Debug)]
    pub struct RCLError {
        pub code: RCLStatusCode,
        pub message: &'static str,
    }

    #[derive(Debug)]
    pub enum RCLStatusCode {
        OK = 0,
        Error = 1,
        Timeout = 2,
        BadAlloc = 10,
        InvalidArgument = 11,
        AlreadyInit = 100,
        NotInit = 101,
        MismatchedRmwId = 102,
        TopicNameInvalid = 103,
        ServiceNameInvalid = 104,
        UnknownSubstitution = 105,
        NodeInvalid = 200,
        NodeInvalidName = 201,
        NodeInvalidNamespace = 202,
        PublisherInvalid = 300,
        SubscriptionInvalid = 400,
        SubscriptionTakeFailed = 401,
        ClientInvalid = 500,
        ClientTakeFailed = 501,
        ServiceInvalid = 600,
        ServiceTakeFailed = 601,
        TimerInvalid = 800,
        TimerCanceled = 801,
        WaitSetInvalid = 900,
        WaitSetEmpty = 901,
        WaitSetFull = 902,
    }

    impl From<i32> for RCLStatusCode {
        fn from(error: i32) -> Self {
            match error {
                0 => RCLStatusCode::OK,
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
                _ => unimplemented!(),
            }
        }
    }
}

pub mod traits {
    use libc::uintptr_t;
    use downcast::Any;

    pub trait Message: Any {
        fn get_native_message(&self) -> uintptr_t;
        fn destroy_native_message(&self, message_handle: uintptr_t) -> ();
        fn read_handle(&mut self, message_handle: uintptr_t) -> ();
    }
    downcast!(Message);

    pub trait MessageDefinition<T>: Message {
        fn get_type_support() -> uintptr_t;
        fn static_get_native_message(message: &T) -> uintptr_t;
        fn static_destroy_native_message(message_handle: uintptr_t) -> ();
    }
}
