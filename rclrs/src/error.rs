use crate::rcl_bindings::*;
use std::error::Error;
use std::ffi::CStr;
use std::fmt::{self, Display};

/// The main error type.
#[derive(Debug, PartialEq)]
pub struct RclrsError {
    /// The error code.
    pub code: RclReturnCode,
    /// The error message set in the rcl layer or below.
    pub(crate) msg: Option<RclErrorMsg>,
}

impl Display for RclrsError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.code)
    }
}

/// Struct encapsulating an error message from the rcl layer or below.
///
/// This struct is intended to be returned by the `source` method in the implementation of the
/// standard [`Error`][1] trait for [`RclrsError`][2].
/// By doing this, the error message is printed as a separate item in the error chain.
/// This avoids an unreadable, inconsistent formatting of error codes and messages that would
/// likely be produced by a combined display of `RclReturnCode` and message.
///
/// [1]: std::error::Error
/// [2]: crate::RclrsError
#[derive(Debug, PartialEq)]
pub(crate) struct RclErrorMsg(String);

impl Display for RclErrorMsg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Error for RclErrorMsg {}

impl Error for RclrsError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.msg.as_ref().map(|e| e as &dyn Error)
    }
}

/// RCL specific error codes.
///
/// These are the error codes that start at 100.
#[derive(Debug, PartialEq)]
pub enum RclErrorCode {
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
    AlreadyShutdown = 106,
}

impl TryFrom<i32> for RclErrorCode {
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

impl Display for RclErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::AlreadyInit => write!(f, "RclError: `rcl_init()` already called."),
            Self::NotInit => write!(f, "RclError: `rcl_init() not yet called."),
            Self::MismatchedRmwId => write!(f, "RclError: Mismatched rmw identifier."),
            Self::TopicNameInvalid => {
                write!(f, "RclError: Topic name does not pass validation.")
            }
            Self::ServiceNameInvalid => {
                write!(f, "RclError: Service name does not pass validation.")
            }
            Self::UnknownSubstitution => {
                write!(f, "RclError: Topic name substitution is unknown.")
            }
            Self::AlreadyShutdown => write!(f, "RclError: `rcl_shutdown()` already called."),
        }
    }
}

impl Error for RclErrorCode {}

/// Error indicating problems in the RCL node (2XX).
#[derive(Debug, PartialEq)]
pub enum NodeErrorCode {
    /// Invalid `rcl_node_t` given
    NodeInvalid = 200,
    /// Invalid node name
    NodeInvalidName = 201,
    /// Invalid node namespace
    NodeInvalidNamespace = 202,
    /// Failed to find node name
    NodeNameNonexistent = 203,
}

impl TryFrom<i32> for NodeErrorCode {
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

impl Display for NodeErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NodeInvalid => write!(f, "NodeError: Invalid `rcl_node_t` given."),
            Self::NodeInvalidName => write!(f, "NodeError: Invalid node name."),
            Self::NodeInvalidNamespace => write!(f, "NodeError: Invalid node namespace."),
            Self::NodeNameNonexistent => write!(f, "NodeError: Failed to find node name."),
        }
    }
}

impl Error for NodeErrorCode {}

/// Error indicating problems in the RCL subscription (4XX).
#[derive(Debug, PartialEq)]
pub enum SubscriberErrorCode {
    /// Invalid `rcl_subscription_t` given
    SubscriptionInvalid = 400,
    /// Failed to take a message from the subscription
    SubscriptionTakeFailed = 401,
}

impl TryFrom<i32> for SubscriberErrorCode {
    type Error = i32;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            x if x == Self::SubscriptionInvalid as i32 => Ok(Self::SubscriptionInvalid),
            x if x == Self::SubscriptionTakeFailed as i32 => Ok(Self::SubscriptionTakeFailed),
            other => Err(other),
        }
    }
}

impl Display for SubscriberErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SubscriptionInvalid => {
                write!(f, "SubscriberError: Invalid `rcl_subscription_t` given.")
            }
            Self::SubscriptionTakeFailed => write!(
                f,
                "SubscriberError: Failed to take a message from the subscription."
            ),
        }
    }
}

impl Error for SubscriberErrorCode {}

/// Error indicating problems in the RCL client (5XX).
#[derive(Debug, PartialEq)]
pub enum ClientErrorCode {
    /// Invalid `rcl_client_t` given
    ClientInvalid = 500,
    /// Failed to take a response from the client
    ClientTakeFailed = 501,
}
impl TryFrom<i32> for ClientErrorCode {
    type Error = i32;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            x if x == Self::ClientInvalid as i32 => Ok(Self::ClientInvalid),
            x if x == Self::ClientTakeFailed as i32 => Ok(Self::ClientTakeFailed),
            other => Err(other),
        }
    }
}

impl Display for ClientErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ClientInvalid => write!(f, "ClientError: Invalid `rcl_client_t` given."),
            Self::ClientTakeFailed => {
                write!(f, "ClientError: Failed to take a response from the client.")
            }
        }
    }
}

impl Error for ClientErrorCode {}

/// Error indicating problems in the RCL service (6XX).
#[derive(Debug, PartialEq)]
pub enum ServiceErrorCode {
    /// Invalid `rcl_service_t` given
    ServiceInvalid = 600,
    /// Failed to take a request from the service
    ServiceTakeFailed = 601,
}

impl TryFrom<i32> for ServiceErrorCode {
    type Error = i32;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            x if x == Self::ServiceInvalid as i32 => Ok(Self::ServiceInvalid),
            x if x == Self::ServiceTakeFailed as i32 => Ok(Self::ServiceTakeFailed),
            other => Err(other),
        }
    }
}

impl Display for ServiceErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ServiceInvalid => write!(f, "ServiceError: Invalid `rcl_service_t` given."),
            Self::ServiceTakeFailed => write!(
                f,
                "ServiceError: Failed to take a request from the service."
            ),
        }
    }
}

impl Error for ServiceErrorCode {}

// Error codes indicating problems in RCL guard conditions are in 7XX...
// But as of the writing of this code, they are not implemented in `rcl/types.h`!

/// Error indicating problems in the RCL timer (8XX).
#[derive(Debug, PartialEq)]
pub enum TimerErrorCode {
    /// Invalid `rcl_timer_t` given
    TimerInvalid = 800,
    /// Given timer was canceled
    TimerCanceled = 801,
}

impl TryFrom<i32> for TimerErrorCode {
    type Error = i32;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            x if x == Self::TimerInvalid as i32 => Ok(Self::TimerInvalid),
            x if x == Self::TimerCanceled as i32 => Ok(Self::TimerCanceled),
            other => Err(other),
        }
    }
}

impl Display for TimerErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TimerInvalid => write!(f, "TimerError: Invalid `rcl_timer_t` given."),
            Self::TimerCanceled => write!(f, "TimerError: Given timer was canceled."),
        }
    }
}

impl Error for TimerErrorCode {}

/// Error indicating problems with RCL wait and wait set (9XX).
#[derive(Debug, PartialEq)]
pub enum WaitSetErrorCode {
    /// Invalid `rcl_wait_set_t` given
    WaitSetInvalid = 900,
    /// Given `rcl_wait_set_t` is empty
    WaitSetEmpty = 901,
    /// Given `rcl_wait_set_t` is full
    WaitSetFull = 902,
}

impl TryFrom<i32> for WaitSetErrorCode {
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

impl Display for WaitSetErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WaitSetInvalid => write!(f, "WaitSetError: Invalid `rcl_wait_set_t` given."),
            Self::WaitSetEmpty => write!(f, "WaitSetError: Given `rcl_wait_set_t` was empty."),
            Self::WaitSetFull => write!(f, "WaitSetError: Given `rcl_wait_set_t` was full."),
        }
    }
}

impl Error for WaitSetErrorCode {}

/// Error indicating problems with RCL argument parsing (1XXX).
#[derive(Debug, PartialEq)]
pub enum ParsingErrorCode {
    /// Argument is not a valid remap rule
    InvalidRemapRule = 1001,
    /// Expected one type of lexeme but got another
    WrongLexeme = 1002,
    /// Found invalid ros argument while parsing
    InvalidRosArgs = 1003,
    /// Argument is not a valid parameter rule
    InvalidParamRule = 1010,
    /// Argument is not a valid log level rule
    InvalidLogLevelRule = 1020,
}

impl TryFrom<i32> for ParsingErrorCode {
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

impl Display for ParsingErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRemapRule => {
                write!(f, "ParsingError: Argument is not a valid remap rule.")
            }
            Self::WrongLexeme => write!(
                f,
                "ParsingError: Expected one type of lexeme, but got another."
            ),
            Self::InvalidRosArgs => {
                write!(f, "ParsingError: Found invalid ROS argument while parsing.")
            }
            Self::InvalidParamRule => {
                write!(f, "ParsingError: Argument is not a valid parameter rule.")
            }
            Self::InvalidLogLevelRule => {
                write!(f, "ParsingError: Argument is not a valid log level rule.")
            }
        }
    }
}

impl Error for ParsingErrorCode {}

/// Error indicating problems with RCL events (20XX)
#[derive(Debug, PartialEq)]
pub enum EventErrorCode {
    /// Invalid `rcl_event_t` given
    EventInvalid = 2000,
    /// Failed to take an event from the event handle
    EventTakeFailed = 2001,
}

impl TryFrom<i32> for EventErrorCode {
    type Error = i32;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            x if x == Self::EventInvalid as i32 => Ok(Self::EventInvalid),
            x if x == Self::EventTakeFailed as i32 => Ok(Self::EventTakeFailed),
            other => Err(other),
        }
    }
}

impl Display for EventErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EventInvalid => write!(f, "EventError: Invalid `rcl_event_t` given."),
            Self::EventTakeFailed => write!(
                f,
                "EventError: Failed to take an event from the event handle."
            ),
        }
    }
}

impl Error for EventErrorCode {}

/// Error indicating problems with RCL lifecycle state registration (30XX).
#[derive(Debug, PartialEq)]
pub enum LifecycleErrorCode {
    /// `rcl_lifecycle` state registered
    LifecycleStateRegistered = 3000,
    /// `rcl_lifecycle` state not registered
    LifecycleStateNotRegistered = 3001,
}

impl TryFrom<i32> for LifecycleErrorCode {
    type Error = i32;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            x if x == Self::LifecycleStateRegistered as i32 => Ok(Self::LifecycleStateRegistered),
            x if x == Self::LifecycleStateNotRegistered as i32 => {
                Ok(Self::LifecycleStateNotRegistered)
            }
            other => Err(other),
        }
    }
}

impl Display for LifecycleErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LifecycleStateRegistered => {
                write!(f, "LifecycleError: `rcl_lifecycle` state registered.")
            }
            Self::LifecycleStateNotRegistered => {
                write!(f, "LifecycleError: `rcl_lifecycle` state not registered.")
            }
        }
    }
}

impl Error for LifecycleErrorCode {}

/// Return codes of RCL functions.
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
    RclError(RclErrorCode),
    /// `rcl` node-specific error occurred
    NodeError(NodeErrorCode),
    /// Invalid `rcl_publisher_t` given
    PublisherInvalid,
    /// `rcl` subscriptionerror occurred
    SubscriberError(SubscriberErrorCode),
    /// `rcl` client error occurred
    ClientError(ClientErrorCode),
    /// `rcl` service error occurred
    ServiceError(ServiceErrorCode),
    /// `rcl` timer error occurred
    TimerError(TimerErrorCode),
    /// `rcl` wait or waitset error occurred
    WaitSetError(WaitSetErrorCode),
    /// `rcl` argument parsing error occurred
    ParsingError(ParsingErrorCode),
    /// `rcl` event error occurred
    EventError(EventErrorCode),
    /// `rcl` lifecycle error occurred
    LifecycleError(LifecycleErrorCode),
    /// Unrecognized/unimplemented error code
    UnknownError(i32),
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
            rcl_err @ 100..=199 => match RclErrorCode::try_from(rcl_err) {
                Ok(code) => Self::RclError(code),
                Err(e) => Self::UnknownError(e),
            },
            node_err @ 200..=299 => match NodeErrorCode::try_from(node_err) {
                Ok(code) => Self::NodeError(code),
                Err(e) => Self::UnknownError(e),
            },
            300 => Self::PublisherInvalid,
            subscriber_err @ 400..=499 => match SubscriberErrorCode::try_from(subscriber_err) {
                Ok(code) => Self::SubscriberError(code),
                Err(e) => Self::UnknownError(e),
            },
            client_err @ 500..=599 => match ClientErrorCode::try_from(client_err) {
                Ok(code) => Self::ClientError(code),
                Err(e) => Self::UnknownError(e),
            },
            service_err @ 600..=699 => match ServiceErrorCode::try_from(service_err) {
                Ok(code) => Self::ServiceError(code),
                Err(e) => Self::UnknownError(e),
            },
            timer_err @ 800..=899 => match TimerErrorCode::try_from(timer_err) {
                Ok(code) => Self::TimerError(code),
                Err(e) => Self::UnknownError(e),
            },
            waitset_err @ 900..=999 => match WaitSetErrorCode::try_from(waitset_err) {
                Ok(code) => Self::WaitSetError(code),
                Err(e) => Self::UnknownError(e),
            },
            parse_err @ 1000..=1999 => match ParsingErrorCode::try_from(parse_err) {
                Ok(code) => Self::ParsingError(code),
                Err(e) => Self::UnknownError(e),
            },
            event_err @ 2000..=2099 => match EventErrorCode::try_from(event_err) {
                Ok(code) => Self::EventError(code),
                Err(e) => Self::UnknownError(e),
            },
            lifecycle_err @ 3000..=3099 => match LifecycleErrorCode::try_from(lifecycle_err) {
                Ok(code) => Self::LifecycleError(code),
                Err(e) => Self::UnknownError(e),
            },
            other => Self::UnknownError(other),
        }
    }
}

impl From<RclErrorCode> for RclReturnCode {
    fn from(err: RclErrorCode) -> Self {
        Self::RclError(err)
    }
}

impl From<NodeErrorCode> for RclReturnCode {
    fn from(err: NodeErrorCode) -> Self {
        Self::NodeError(err)
    }
}

impl From<SubscriberErrorCode> for RclReturnCode {
    fn from(err: SubscriberErrorCode) -> Self {
        Self::SubscriberError(err)
    }
}

impl From<ClientErrorCode> for RclReturnCode {
    fn from(err: ClientErrorCode) -> Self {
        Self::ClientError(err)
    }
}

impl From<ServiceErrorCode> for RclReturnCode {
    fn from(err: ServiceErrorCode) -> Self {
        Self::ServiceError(err)
    }
}

impl From<TimerErrorCode> for RclReturnCode {
    fn from(err: TimerErrorCode) -> Self {
        Self::TimerError(err)
    }
}

impl From<WaitSetErrorCode> for RclReturnCode {
    fn from(err: WaitSetErrorCode) -> Self {
        Self::WaitSetError(err)
    }
}

impl From<ParsingErrorCode> for RclReturnCode {
    fn from(err: ParsingErrorCode) -> Self {
        Self::ParsingError(err)
    }
}

impl From<EventErrorCode> for RclReturnCode {
    fn from(err: EventErrorCode) -> Self {
        Self::EventError(err)
    }
}

impl From<LifecycleErrorCode> for RclReturnCode {
    fn from(err: LifecycleErrorCode) -> Self {
        Self::LifecycleError(err)
    }
}

impl Display for RclReturnCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ok => write!(f, "RclReturnCode: Operation successful."),
            Self::Error => write!(f, "RclReturnCode: Unspecified error."),
            Self::Timeout => write!(f, "RclReturnCode: Timeout occurred."),
            Self::Unsupported => write!(f, "RclReturnCode: Unsupported return code."),
            Self::BadAlloc => write!(f, "RclReturnCode: Failed to allocate memory."),
            Self::InvalidArgument => {
                write!(f, "RclReturnCode: Argument to function was invalid.")
            }
            Self::RclError(rcl_err) => write!(f, "RclReturnCode::{}", rcl_err),
            Self::NodeError(node_err) => write!(f, "RclReturnCode::{}", node_err),
            Self::PublisherInvalid => {
                write!(f, "RclReturnCode: Invalid `rcl_publisher_t` given.")
            }
            Self::SubscriberError(subscriber_err) => {
                write!(f, "RclReturnCode::{}", subscriber_err)
            }
            Self::ClientError(client_err) => write!(f, "RclReturnCode::{}", client_err),
            Self::ServiceError(service_err) => write!(f, "RclReturnCode::{}", service_err),
            Self::TimerError(timer_err) => write!(f, "RclReturnCode::{}", timer_err),
            Self::WaitSetError(waitset_err) => write!(f, "RclReturnCode::{}", waitset_err),
            Self::ParsingError(parse_err) => write!(f, "RclReturnCode::{}", parse_err),
            Self::EventError(event_err) => write!(f, "RclReturnCode::{}", event_err),
            Self::LifecycleError(lifecycle_err) => {
                write!(f, "RclReturnCode::{}", lifecycle_err)
            }
            Self::UnknownError(unknown_err) => {
                write!(f, "RclReturnCode: Unknown error code -> `{}`", unknown_err)
            }
        }
    }
}

impl Error for RclReturnCode {}

pub(crate) fn to_rcl_result(code: i32) -> Result<(), RclrsError> {
    match RclReturnCode::from(code) {
        RclReturnCode::Ok => Ok(()),
        anything_else => {
            let mut msg = None;
            // SAFETY: No preconditions for this function.
            let error_state_ptr = unsafe { rcutils_get_error_state() };
            // The returned pointer may be null if the error state has not been set.
            if !error_state_ptr.is_null() {
                // SAFETY: Dereferencing the pointer is safe since it was checked to be non-null.
                let msg_ptr = unsafe { (*error_state_ptr).message.as_ptr() };
                // SAFETY: Pointer has been created from an array, no lifetime issues due to
                // immediate conversion to owned string.
                let s = unsafe { CStr::from_ptr(msg_ptr) }
                    .to_string_lossy()
                    .into_owned();
                msg = Some(RclErrorMsg(s));
            }
            // SAFETY: No preconditions for this function.
            unsafe { rcutils_reset_error() };
            Err(RclrsError {
                code: anything_else,
                msg,
            })
        }
    }
}

pub(crate) trait ToResult {
    fn ok(&self) -> Result<(), RclrsError>;
}

impl ToResult for rcl_ret_t {
    fn ok(&self) -> Result<(), RclrsError> {
        to_rcl_result(*self as i32)
    }
}

#[cfg(test)]
mod tests {
    use crate::error::{
        ClientErrorCode, EventErrorCode, LifecycleErrorCode, NodeErrorCode, ParsingErrorCode,
        RclErrorCode, RclReturnCode, ServiceErrorCode, SubscriberErrorCode, TimerErrorCode,
        WaitSetErrorCode,
    };

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
        assert_eq!(
            RclErrorCode::try_from(100).unwrap(),
            RclErrorCode::AlreadyInit
        );
        assert_eq!(
            RclReturnCode::from(100),
            RclReturnCode::RclError(RclErrorCode::AlreadyInit)
        );
    }

    #[test]
    fn test_not_init() {
        assert_eq!(RclErrorCode::try_from(101).unwrap(), RclErrorCode::NotInit);
        assert_eq!(
            RclReturnCode::from(101),
            RclReturnCode::RclError(RclErrorCode::NotInit)
        );
    }

    #[test]
    fn test_mismatched_rmw_id() {
        assert_eq!(
            RclErrorCode::try_from(102).unwrap(),
            RclErrorCode::MismatchedRmwId
        );
        assert_eq!(
            RclReturnCode::from(102),
            RclReturnCode::RclError(RclErrorCode::MismatchedRmwId)
        );
    }

    #[test]
    fn test_topic_name_invalid() {
        assert_eq!(
            RclErrorCode::try_from(103).unwrap(),
            RclErrorCode::TopicNameInvalid
        );
        assert_eq!(
            RclReturnCode::from(103),
            RclReturnCode::RclError(RclErrorCode::TopicNameInvalid)
        );
    }

    #[test]
    fn test_service_name_invaid() {
        assert_eq!(
            RclErrorCode::try_from(104).unwrap(),
            RclErrorCode::ServiceNameInvalid
        );
        assert_eq!(
            RclReturnCode::from(104),
            RclReturnCode::RclError(RclErrorCode::ServiceNameInvalid)
        );
    }

    #[test]
    fn test_unknown_substitution() {
        assert_eq!(
            RclErrorCode::try_from(105).unwrap(),
            RclErrorCode::UnknownSubstitution
        );
        assert_eq!(
            RclReturnCode::from(105),
            RclReturnCode::RclError(RclErrorCode::UnknownSubstitution)
        );
    }

    #[test]
    fn test_already_shutdown() {
        assert_eq!(
            RclErrorCode::try_from(106).unwrap(),
            RclErrorCode::AlreadyShutdown
        );
        assert_eq!(
            RclReturnCode::from(106),
            RclReturnCode::RclError(RclErrorCode::AlreadyShutdown)
        );
    }

    //////////////////////
    // NodeError checks //
    //////////////////////
    #[test]
    fn test_node_invalid() {
        assert_eq!(
            NodeErrorCode::try_from(200).unwrap(),
            NodeErrorCode::NodeInvalid
        );
        assert_eq!(
            RclReturnCode::from(200),
            RclReturnCode::NodeError(NodeErrorCode::NodeInvalid)
        );
    }

    #[test]
    fn test_node_invalid_name() {
        assert_eq!(
            NodeErrorCode::try_from(201).unwrap(),
            NodeErrorCode::NodeInvalidName
        );
        assert_eq!(
            RclReturnCode::from(201),
            RclReturnCode::NodeError(NodeErrorCode::NodeInvalidName)
        );
    }

    #[test]
    fn test_node_invalid_namespace() {
        assert_eq!(
            NodeErrorCode::try_from(202).unwrap(),
            NodeErrorCode::NodeInvalidNamespace
        );
        assert_eq!(
            RclReturnCode::from(202),
            RclReturnCode::NodeError(NodeErrorCode::NodeInvalidNamespace)
        );
    }

    #[test]
    fn test_node_name_nonexistent() {
        assert_eq!(
            NodeErrorCode::try_from(203).unwrap(),
            NodeErrorCode::NodeNameNonexistent
        );
        assert_eq!(
            RclReturnCode::from(203),
            RclReturnCode::NodeError(NodeErrorCode::NodeNameNonexistent)
        );
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
        assert_eq!(
            SubscriberErrorCode::try_from(400).unwrap(),
            SubscriberErrorCode::SubscriptionInvalid
        );
        assert_eq!(
            RclReturnCode::from(400),
            RclReturnCode::SubscriberError(SubscriberErrorCode::SubscriptionInvalid)
        );
    }

    #[test]
    fn test_subscription_take_failed() {
        assert_eq!(
            SubscriberErrorCode::try_from(401).unwrap(),
            SubscriberErrorCode::SubscriptionTakeFailed
        );
        assert_eq!(
            RclReturnCode::from(401),
            RclReturnCode::SubscriberError(SubscriberErrorCode::SubscriptionTakeFailed)
        );
    }

    ////////////////////////
    // ClientError checks //
    ////////////////////////
    #[test]
    fn test_client_invalid() {
        assert_eq!(
            ClientErrorCode::try_from(500).unwrap(),
            ClientErrorCode::ClientInvalid
        );
        assert_eq!(
            RclReturnCode::from(500),
            RclReturnCode::ClientError(ClientErrorCode::ClientInvalid)
        );
    }

    #[test]
    fn test_client_take_failed() {
        assert_eq!(
            ClientErrorCode::try_from(501).unwrap(),
            ClientErrorCode::ClientTakeFailed
        );
        assert_eq!(
            RclReturnCode::from(501),
            RclReturnCode::ClientError(ClientErrorCode::ClientTakeFailed)
        );
    }

    /////////////////////////
    // ServiceError checks //
    /////////////////////////
    #[test]
    fn test_service_invalid() {
        assert_eq!(
            ServiceErrorCode::try_from(600).unwrap(),
            ServiceErrorCode::ServiceInvalid
        );
        assert_eq!(
            RclReturnCode::from(600),
            RclReturnCode::ServiceError(ServiceErrorCode::ServiceInvalid)
        );
    }

    #[test]
    fn test_service_take_failed() {
        assert_eq!(
            ServiceErrorCode::try_from(601).unwrap(),
            ServiceErrorCode::ServiceTakeFailed
        );
        assert_eq!(
            RclReturnCode::from(601),
            RclReturnCode::ServiceError(ServiceErrorCode::ServiceTakeFailed)
        );
    }

    ///////////////////////
    // TimerError checks //
    ///////////////////////
    #[test]
    fn test_timer_invalid() {
        assert_eq!(
            TimerErrorCode::try_from(800).unwrap(),
            TimerErrorCode::TimerInvalid
        );
        assert_eq!(
            RclReturnCode::from(800),
            RclReturnCode::TimerError(TimerErrorCode::TimerInvalid)
        );
    }

    #[test]
    fn test_timer_canceled() {
        assert_eq!(
            TimerErrorCode::try_from(801).unwrap(),
            TimerErrorCode::TimerCanceled
        );
        assert_eq!(
            RclReturnCode::from(801),
            RclReturnCode::TimerError(TimerErrorCode::TimerCanceled)
        );
    }

    /////////////////////////
    // WaitSetError checks //
    /////////////////////////
    #[test]
    fn test_wait_set_invalid() {
        assert_eq!(
            WaitSetErrorCode::try_from(900).unwrap(),
            WaitSetErrorCode::WaitSetInvalid
        );
        assert_eq!(
            RclReturnCode::from(900),
            RclReturnCode::WaitSetError(WaitSetErrorCode::WaitSetInvalid)
        );
    }

    #[test]
    fn test_wait_set_empty() {
        assert_eq!(
            WaitSetErrorCode::try_from(901).unwrap(),
            WaitSetErrorCode::WaitSetEmpty
        );
        assert_eq!(
            RclReturnCode::from(901),
            RclReturnCode::WaitSetError(WaitSetErrorCode::WaitSetEmpty)
        );
    }

    #[test]
    fn test_wait_set_full() {
        assert_eq!(
            WaitSetErrorCode::try_from(902).unwrap(),
            WaitSetErrorCode::WaitSetFull
        );
        assert_eq!(
            RclReturnCode::from(902),
            RclReturnCode::WaitSetError(WaitSetErrorCode::WaitSetFull)
        );
    }

    /////////////////////////
    // ParsingError checks //
    /////////////////////////
    #[test]
    fn test_invalid_remap_rule() {
        assert_eq!(
            ParsingErrorCode::try_from(1001).unwrap(),
            ParsingErrorCode::InvalidRemapRule
        );
        assert_eq!(
            RclReturnCode::from(1001),
            RclReturnCode::ParsingError(ParsingErrorCode::InvalidRemapRule)
        );
    }

    #[test]
    fn test_wrong_lexeme() {
        assert_eq!(
            ParsingErrorCode::try_from(1002).unwrap(),
            ParsingErrorCode::WrongLexeme
        );
        assert_eq!(
            RclReturnCode::from(1002),
            RclReturnCode::ParsingError(ParsingErrorCode::WrongLexeme)
        );
    }

    #[test]
    fn test_invalid_ros_args() {
        assert_eq!(
            ParsingErrorCode::try_from(1003).unwrap(),
            ParsingErrorCode::InvalidRosArgs
        );
        assert_eq!(
            RclReturnCode::from(1003),
            RclReturnCode::ParsingError(ParsingErrorCode::InvalidRosArgs)
        );
    }

    #[test]
    fn test_invalid_param_rule() {
        assert_eq!(
            ParsingErrorCode::try_from(1010).unwrap(),
            ParsingErrorCode::InvalidParamRule
        );
        assert_eq!(
            RclReturnCode::from(1010),
            RclReturnCode::ParsingError(ParsingErrorCode::InvalidParamRule)
        );
    }

    #[test]
    fn test_invalid_log_level_rule() {
        assert_eq!(
            ParsingErrorCode::try_from(1020).unwrap(),
            ParsingErrorCode::InvalidLogLevelRule
        );
        assert_eq!(
            RclReturnCode::from(1020),
            RclReturnCode::ParsingError(ParsingErrorCode::InvalidLogLevelRule)
        );
    }

    ///////////////////////
    // EventError checks //
    ///////////////////////
    #[test]
    fn test_event_invalid() {
        assert_eq!(
            EventErrorCode::try_from(2000).unwrap(),
            EventErrorCode::EventInvalid
        );
        assert_eq!(
            RclReturnCode::from(2000),
            RclReturnCode::EventError(EventErrorCode::EventInvalid)
        );
    }

    #[test]
    fn test_event_take_failed() {
        assert_eq!(
            EventErrorCode::try_from(2001).unwrap(),
            EventErrorCode::EventTakeFailed
        );
        assert_eq!(
            RclReturnCode::from(2001),
            RclReturnCode::EventError(EventErrorCode::EventTakeFailed)
        );
    }

    ///////////////////////////
    // LifecycleError checks //
    ///////////////////////////
    #[test]
    fn test_lifecycle_state_registered() {
        assert_eq!(
            LifecycleErrorCode::try_from(3000).unwrap(),
            LifecycleErrorCode::LifecycleStateRegistered
        );
        assert_eq!(
            RclReturnCode::from(3000),
            RclReturnCode::LifecycleError(LifecycleErrorCode::LifecycleStateRegistered)
        );
    }

    #[test]
    fn test_lifecycle_state_not_registered() {
        assert_eq!(
            LifecycleErrorCode::try_from(3001).unwrap(),
            LifecycleErrorCode::LifecycleStateNotRegistered
        );
        assert_eq!(
            RclReturnCode::from(3001),
            RclReturnCode::LifecycleError(LifecycleErrorCode::LifecycleStateNotRegistered)
        );
    }

    ////////////////////////
    // UnknownError check //
    ////////////////////////
    #[test]
    fn test_unknown_error() {
        assert_eq!(RclReturnCode::from(-42), RclReturnCode::UnknownError(-42));
    }
}
