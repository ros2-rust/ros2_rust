use std::{
    error::Error,
    ffi::{CStr, NulError},
    fmt::{self, Display},
    sync::PoisonError,
};

use crate::{rcl_bindings::*, DeclarationError, ReadyKind};

/// The main error type.
#[derive(Debug, PartialEq, Eq)]
pub enum RclrsError {
    /// An error originating in the `rcl` layer.
    RclError {
        /// The error code.
        code: RclReturnCode,
        /// The error message set in the `rcl` layer or below.
        msg: Option<RclErrorMsg>,
    },
    /// An unknown error originating in the `rcl` layer.
    UnknownRclError {
        /// The error code.
        code: i32,
        /// The error message set in the `rcl` layer or below.
        msg: Option<RclErrorMsg>,
    },
    /// A string provided to `rclrs` could not be converted into a `CString`.
    StringContainsNul {
        /// The string that contains a nul byte.
        s: String,
        /// The error indicating the position of the nul byte.
        err: NulError,
    },
    /// It was attempted to add a waitable to a wait set twice.
    AlreadyAddedToWaitSet,
    /// A negative duration was obtained from rcl which should have been positive.
    ///
    /// The value represents nanoseconds.
    NegativeDuration(i64),
    /// The guard condition that you tried to trigger is not owned by the
    /// [`GuardCondition`][crate::GuardCondition] instance.
    UnownedGuardCondition,
    /// The payload given to a primitive that belongs to a worker was the wrong
    /// type.
    InvalidPayload {
        /// The payload type expected by the primitive
        expected: std::any::TypeId,
        /// The payload type given by the worker
        received: std::any::TypeId,
    },
    /// An error happened while declaring a parameter.
    ParameterDeclarationError(crate::DeclarationError),
    /// A mutex used internally has been [poisoned][std::sync::PoisonError].
    PoisonedMutex,
    /// An [`crate::RclPrimitive`] received ready information that is not
    /// compatible with its type.
    InvalidReadyInformation {
        /// The expected format of the ready information (default-initialized)
        expected: ReadyKind,
        /// The ready information that was received.
        received: ReadyKind,
    },
    /// From rcl documentation for rcl_action_accept_new_goal:
    ///
    /// If a failure occurs, `NULL` is returned and an error message is set.
    /// Possible reasons for failure:
    ///   - action server is invalid
    ///   - goal info is invalid
    ///   - goal ID is already being tracked by the action server
    ///   - memory allocation failure
    ///
    /// We have no way of diagnosing which of these errors caused the failure, so
    /// all we can do is indicate that an error occurred with accepting the goal.
    /// However, the implementation of rclrs automatically protects from all of
    /// these errors except memory allocation failure.
    GoalAcceptanceError,
}

impl RclrsError {
    /// Returns true if the error was due to a timeout, otherwise returns false.
    pub fn is_timeout(&self) -> bool {
        matches!(
            self,
            RclrsError::RclError {
                code: RclReturnCode::Timeout,
                ..
            }
        )
    }

    /// Returns true if the error was because a subscription, service, or client
    /// take failed, otherwise returns false.
    pub fn is_take_failed(&self) -> bool {
        matches!(
            self,
            RclrsError::RclError {
                code: RclReturnCode::SubscriptionTakeFailed
                    | RclReturnCode::ServiceTakeFailed
                    | RclReturnCode::ClientTakeFailed
                    | RclReturnCode::ActionServerTakeFailed
                    | RclReturnCode::ActionClientTakeFailed
                    | RclReturnCode::EventTakeFailed,
                ..
            }
        )
    }
}

impl Display for RclrsError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            RclrsError::RclError { code, .. } => write!(f, "{}", code),
            RclrsError::UnknownRclError { code, .. } => write!(f, "{}", code),
            RclrsError::StringContainsNul { s, .. } => {
                write!(f, "Could not convert string '{}' to CString", s)
            }
            RclrsError::AlreadyAddedToWaitSet => {
                write!(
                    f,
                    "Could not add entity to wait set because it was already added to a wait set"
                )
            }
            RclrsError::NegativeDuration(duration) => {
                write!(
                    f,
                    "A duration was negative when it should not have been: {duration:?}"
                )
            }
            RclrsError::UnownedGuardCondition => {
                write!(
                    f,
                    "Could not trigger guard condition because it is not owned by rclrs"
                )
            }
            RclrsError::InvalidPayload { expected, received } => {
                write!(
                    f,
                    "Received invalid payload: expected {expected:?}, received {received:?}",
                )
            }
            RclrsError::ParameterDeclarationError(err) => {
                write!(f, "An error occurred while declaring a parameter: {err}",)
            }
            RclrsError::PoisonedMutex => {
                write!(f, "A mutex used internally has been poisoned")
            }
            RclrsError::InvalidReadyInformation { expected, received } => {
                write!(
                    f,
                    "Invalid ready information was provided. This suggests an error \
                    in how the wait set is being used.\
                    \n - Expected information: {expected:?}\
                    \n - Actual: {received:?}",
                )
            }
            RclrsError::GoalAcceptanceError => {
                write!(
                    f,
                    "An error occurred while trying to accept an action server goal",
                )
            }
        }
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
#[derive(Debug, PartialEq, Eq)]
pub struct RclErrorMsg(pub(crate) String);

impl Display for RclErrorMsg {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Error for RclErrorMsg {}

impl Error for RclrsError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            RclrsError::RclError { msg, .. } => msg.as_ref().map(|e| e as &dyn Error),
            RclrsError::UnknownRclError { msg, .. } => msg.as_ref().map(|e| e as &dyn Error),
            RclrsError::StringContainsNul { err, .. } => Some(err).map(|e| e as &dyn Error),
            // TODO(@mxgrey): We should provide source information for these other types.
            // It should be easy to do this using the thiserror crate.
            RclrsError::AlreadyAddedToWaitSet => None,
            RclrsError::NegativeDuration(_) => None,
            RclrsError::UnownedGuardCondition => None,
            RclrsError::InvalidPayload { .. } => None,
            RclrsError::ParameterDeclarationError(_) => None,
            RclrsError::PoisonedMutex => None,
            RclrsError::InvalidReadyInformation { .. } => None,
            RclrsError::GoalAcceptanceError => None,
        }
    }
}

/// Return codes of `rcl` functions.
///
/// This type corresponds to `rcl_ret_t`.
/// Most of these return codes should never occur in an `rclrs` application,
/// since they are returned when `rcl` functions are used wrongly..
#[repr(i32)]
#[derive(Debug, PartialEq, Eq)]
pub enum RclReturnCode {
    /// Success
    Ok = 0,
    /// Unspecified error
    Error = 1,
    /// Timeout occurred
    Timeout = 2,
    /// Unsupported return code
    Unsupported = 3,
    /// Failed to allocate memory
    BadAlloc = 10,
    /// Argument to function was invalid
    InvalidArgument = 11,
    // ====== 1xx: `rcl`-specific errors ======
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
    // ====== 2xx: node-specific errors ======
    /// Invalid `rcl_node_t` given
    NodeInvalid = 200,
    /// Invalid node name
    NodeInvalidName = 201,
    /// Invalid node namespace
    NodeInvalidNamespace = 202,
    /// Failed to find node name
    NodeNameNonexistent = 203,
    // ====== 3XX: publisher-specific errors ======
    /// Invalid `rcl_publisher_t` given
    PublisherInvalid = 300,
    // ====== 4XX: subscription-specific errors ======
    /// Invalid `rcl_subscription_t` given
    SubscriptionInvalid = 400,
    /// Failed to take a message from the subscription
    SubscriptionTakeFailed = 401,
    // ====== 5XX: client-specific errors ======
    /// Invalid `rcl_client_t` given
    ClientInvalid = 500,
    /// Failed to take a response from the client
    ClientTakeFailed = 501,
    // ====== 6XX: service-specific errors ======
    /// Invalid `rcl_service_t` given
    ServiceInvalid = 600,
    /// Failed to take a request from the service
    ServiceTakeFailed = 601,
    // ====== 8XX: timer-specific errors ======
    /// Invalid `rcl_timer_t` given
    TimerInvalid = 800,
    /// Given timer was canceled
    TimerCanceled = 801,
    // ====== 9XX: wait set-specific errors ======
    /// Invalid `rcl_wait_set_t` given
    WaitSetInvalid = 900,
    /// Given `rcl_wait_set_t` is empty
    WaitSetEmpty = 901,
    /// Given `rcl_wait_set_t` is full
    WaitSetFull = 902,
    // ====== 10XX: argument parsing errors ======
    /// Argument is not a valid remap rule
    InvalidRemapRule = 1001,
    /// Expected one type of lexeme but got another
    WrongLexeme = 1002,
    /// Found invalid ROS argument while parsing
    InvalidRosArgs = 1003,
    /// Argument is not a valid parameter rule
    InvalidParamRule = 1010,
    /// Argument is not a valid log level rule
    InvalidLogLevelRule = 1020,
    // ====== 20XX: event-specific errors ======
    /// Invalid `rcl_event_t` given
    EventInvalid = 2000,
    /// Failed to take an event from the event handle
    EventTakeFailed = 2001,
    // ====== 2XXX: action-specific errors ======
    /// Action name does not pass validation
    // TODO(nwn): Consult with upstream about this reused error code.
    // ActionNameInvalid = 2000,
    /// Action goal accepted
    ActionGoalAccepted = 2100,
    /// Action goal rejected
    ActionGoalRejected = 2101,
    /// Action client is invalid
    ActionClientInvalid = 2102,
    /// Action client failed to take response
    ActionClientTakeFailed = 2103,
    /// Action server is invalid
    ActionServerInvalid = 2200,
    /// Action server failed to take request
    ActionServerTakeFailed = 2201,
    /// Action goal handle invalid
    ActionGoalHandleInvalid = 2300,
    /// Action invalid event
    ActionGoalEventInvalid = 2301,
    // ====== 30XX: lifecycle-specific errors ======
    /// `rcl_lifecycle` state registered
    LifecycleStateRegistered = 3000,
    /// `rcl_lifecycle` state not registered
    LifecycleStateNotRegistered = 3001,
}

impl From<DeclarationError> for RclrsError {
    fn from(value: DeclarationError) -> Self {
        RclrsError::ParameterDeclarationError(value)
    }
}

impl<T> From<PoisonError<T>> for RclrsError {
    fn from(_: PoisonError<T>) -> Self {
        RclrsError::PoisonedMutex
    }
}

impl TryFrom<i32> for RclReturnCode {
    type Error = i32;

    fn try_from(value: i32) -> Result<Self, i32> {
        let code = match value {
            x if x == Self::Ok as i32 => Self::Ok,
            x if x == Self::Error as i32 => Self::Error,
            x if x == Self::Timeout as i32 => Self::Timeout,
            x if x == Self::Unsupported as i32 => Self::Unsupported,
            x if x == Self::BadAlloc as i32 => Self::BadAlloc,
            x if x == Self::InvalidArgument as i32 => Self::InvalidArgument,
            x if x == Self::AlreadyInit as i32 => Self::AlreadyInit,
            x if x == Self::NotInit as i32 => Self::NotInit,
            x if x == Self::MismatchedRmwId as i32 => Self::MismatchedRmwId,
            x if x == Self::TopicNameInvalid as i32 => Self::TopicNameInvalid,
            x if x == Self::ServiceNameInvalid as i32 => Self::ServiceNameInvalid,
            x if x == Self::UnknownSubstitution as i32 => Self::UnknownSubstitution,
            x if x == Self::AlreadyShutdown as i32 => Self::AlreadyShutdown,
            x if x == Self::NodeInvalid as i32 => Self::NodeInvalid,
            x if x == Self::NodeInvalidName as i32 => Self::NodeInvalidName,
            x if x == Self::NodeInvalidNamespace as i32 => Self::NodeInvalidNamespace,
            x if x == Self::NodeNameNonexistent as i32 => Self::NodeNameNonexistent,
            x if x == Self::PublisherInvalid as i32 => Self::PublisherInvalid,
            x if x == Self::SubscriptionInvalid as i32 => Self::SubscriptionInvalid,
            x if x == Self::SubscriptionTakeFailed as i32 => Self::SubscriptionTakeFailed,
            x if x == Self::ClientInvalid as i32 => Self::ClientInvalid,
            x if x == Self::ClientTakeFailed as i32 => Self::ClientTakeFailed,
            x if x == Self::ServiceInvalid as i32 => Self::ServiceInvalid,
            x if x == Self::ServiceTakeFailed as i32 => Self::ServiceTakeFailed,
            x if x == Self::TimerInvalid as i32 => Self::TimerInvalid,
            x if x == Self::TimerCanceled as i32 => Self::TimerCanceled,
            x if x == Self::WaitSetInvalid as i32 => Self::WaitSetInvalid,
            x if x == Self::WaitSetEmpty as i32 => Self::WaitSetEmpty,
            x if x == Self::WaitSetFull as i32 => Self::WaitSetFull,
            x if x == Self::InvalidRemapRule as i32 => Self::InvalidRemapRule,
            x if x == Self::WrongLexeme as i32 => Self::WrongLexeme,
            x if x == Self::InvalidRosArgs as i32 => Self::InvalidRosArgs,
            x if x == Self::InvalidParamRule as i32 => Self::InvalidParamRule,
            x if x == Self::InvalidLogLevelRule as i32 => Self::InvalidLogLevelRule,
            x if x == Self::EventInvalid as i32 => Self::EventInvalid,
            x if x == Self::EventTakeFailed as i32 => Self::EventTakeFailed,
            // x if x == Self::ActionNameInvalid as i32 => Self::ActionNameInvalid,
            x if x == Self::ActionGoalAccepted as i32 => Self::ActionGoalAccepted,
            x if x == Self::ActionGoalRejected as i32 => Self::ActionGoalRejected,
            x if x == Self::ActionClientInvalid as i32 => Self::ActionClientInvalid,
            x if x == Self::ActionClientTakeFailed as i32 => Self::ActionClientTakeFailed,
            x if x == Self::ActionServerInvalid as i32 => Self::ActionServerInvalid,
            x if x == Self::ActionServerTakeFailed as i32 => Self::ActionServerTakeFailed,
            x if x == Self::ActionGoalHandleInvalid as i32 => Self::ActionGoalHandleInvalid,
            x if x == Self::ActionGoalEventInvalid as i32 => Self::ActionGoalEventInvalid,
            x if x == Self::LifecycleStateRegistered as i32 => Self::LifecycleStateRegistered,
            x if x == Self::LifecycleStateNotRegistered as i32 => Self::LifecycleStateNotRegistered,
            other => {
                return Err(other);
            }
        };
        Ok(code)
    }
}

impl Display for RclReturnCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Self::Ok => "Operation successful (RCL_RET_OK).",
            Self::Error => "Unspecified error (RCL_RET_ERROR).",
            Self::Timeout => "Timeout occurred (RCL_RET_TIMEOUT).",
            Self::Unsupported => "Unsupported return code (RCL_RET_UNSUPPORTED).",
            Self::BadAlloc => "Failed to allocate memory (RCL_RET_BAD_ALLOC).",
            Self::InvalidArgument => "Argument to function was invalid (RCL_RET_INVALID_ARGUMENT).",
            Self::AlreadyInit => "`rcl_init()` already called (RCL_RET_ALREADY_INIT).",
            Self::NotInit => "`rcl_init() not yet called (RCL_RET_NOT_INIT).",
            Self::MismatchedRmwId => "Mismatched rmw identifier (RCL_RET_MISMATCHED_RMW_ID).",
            Self::TopicNameInvalid => {
                "Topic name does not pass validation (RCL_RET_TOPIC_NAME_INVALID)."
            }
            Self::ServiceNameInvalid => {
                "Service name does not pass validation (RCL_RET_SERVICE_NAME_INVALID)."
            }
            Self::UnknownSubstitution => {
                "Topic name substitution is unknown (RCL_RET_UNKNOWN_SUBSTITUTION)."
            }
            Self::AlreadyShutdown => "`rcl_shutdown()` already called (RCL_RET_ALREADY_SHUTDOWN).",
            Self::NodeInvalid => "Invalid `rcl_node_t` given (RCL_RET_NODE_INVALID).",
            Self::NodeInvalidName => "Invalid node name (RCL_RET_NODE_INVALID_NAME).",
            Self::NodeInvalidNamespace => {
                "Invalid node namespace (RCL_RET_NODE_INVALID_NAMESPACE)."
            }
            Self::NodeNameNonexistent => {
                "Failed to find node name (RCL_RET_NODE_NAME_NON_EXISTENT)."
            }
            Self::PublisherInvalid => {
                "Invalid `rcl_publisher_t` given (RCL_RET_PUBLISHER_INVALID)."
            }
            Self::SubscriptionInvalid => {
                "Invalid `rcl_subscription_t` given (RCL_RET_SUBSCRIPTION_INVALID)."
            }
            Self::SubscriptionTakeFailed => {
                "Failed to take a message from the subscription (RCL_RET_SUBSCRIPTION_TAKE_FAILED)."
            }
            Self::ClientInvalid => "Invalid `rcl_client_t` given (RCL_RET_CLIENT_INVALID).",
            Self::ClientTakeFailed => {
                "Failed to take a response from the client (RCL_RET_CLIENT_TAKE_FAILED)."
            }
            Self::ServiceInvalid => "Invalid `rcl_service_t` given (RCL_RET_SERVICE_INVALID).",
            Self::ServiceTakeFailed => {
                "Failed to take a request from the service (RCL_RET_SERVICE_TAKE_FAILED)."
            }
            Self::TimerInvalid => "Invalid `rcl_timer_t` given (RCL_RET_TIMER_INVALID).",
            Self::TimerCanceled => "Given timer was canceled (RCL_RET_TIMER_CANCELED).",
            Self::WaitSetInvalid => "Invalid `rcl_wait_set_t` given (RCL_RET_WAIT_SET_INVALID).",
            Self::WaitSetEmpty => "Given `rcl_wait_set_t` was empty (RCL_RET_WAIT_SET_EMPTY).",
            Self::WaitSetFull => "Given `rcl_wait_set_t` was full (RCL_RET_WAIT_SET_FULL).",
            Self::InvalidRemapRule => {
                "Argument is not a valid remap rule (RCL_RET_INVALID_REMAP_RULE)."
            }
            Self::WrongLexeme => {
                "Expected one type of lexeme, but got another (RCL_RET_WRONG_LEXEME)"
            }
            Self::InvalidRosArgs => {
                "Found invalid ROS argument while parsing (RCL_RET_INVALID_ROS_ARGS)."
            }
            Self::InvalidParamRule => {
                "Argument is not a valid parameter rule (RCL_RET_INVALID_PARAM_RULE)."
            }
            Self::InvalidLogLevelRule => {
                "Argument is not a valid log level rule (RCL_RET_INVALID_LOG_LEVEL_RULE)."
            }
            Self::EventInvalid => "Invalid `rcl_event_t` given (RCL_RET_EVENT_INVALID).",
            Self::EventTakeFailed => {
                "Failed to take an event from the event handle (RCL_RET_EVENT_TAKE_FAILED)."
            }
            // Self::ActionNameInvalid => "Action name does not pass validation (RCL_RET_ACTION_NAME_INVALID).",
            Self::ActionGoalAccepted => "Action goal accepted (RCL_RET_ACTION_GOAL_ACCEPTED).",
            Self::ActionGoalRejected => "Action goal rejected (RCL_RET_ACTION_GOAL_REJECTED).",
            Self::ActionClientInvalid => {
                "Action client is invalid (RCL_RET_ACTION_CLIENT_INVALID)."
            }
            Self::ActionClientTakeFailed => {
                "Action client failed to take response (RCL_RET_ACTION_CLIENT_TAKE_FAILED)."
            }
            Self::ActionServerInvalid => {
                "Action server is invalid (RCL_RET_ACTION_SERVER_INVALID)."
            }
            Self::ActionServerTakeFailed => {
                "Action server failed to take request (RCL_RET_ACTION_SERVER_TAKE_FAILED)."
            }
            Self::ActionGoalHandleInvalid => {
                "Action goal handle invalid (RCL_RET_ACTION_GOAL_HANDLE_INVALID)."
            }
            Self::ActionGoalEventInvalid => {
                "Action invalid event (RCL_RET_ACTION_GOAL_EVENT_INVALID)."
            }
            Self::LifecycleStateRegistered => {
                "`rcl_lifecycle` state registered (RCL_RET_LIFECYCLE_STATE_REGISTERED)."
            }
            Self::LifecycleStateNotRegistered => {
                "`rcl_lifecycle` state not registered (RCL_RET_LIFECYCLE_STATE_NOT_REGISTERED)."
            }
        };
        write!(f, "{}", s)
    }
}

impl Error for RclReturnCode {}

pub(crate) fn to_rclrs_result(ret: i32) -> Result<(), RclrsError> {
    if ret == 0 {
        return Ok(());
    }
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
    // Finally, try to parse it into a return code.
    Err(match RclReturnCode::try_from(ret) {
        Ok(code) => RclrsError::RclError { code, msg },
        Err(code) => RclrsError::UnknownRclError { code, msg },
    })
}

pub(crate) trait ToResult {
    fn ok(&self) -> Result<(), RclrsError>;
}

impl ToResult for rcl_ret_t {
    fn ok(&self) -> Result<(), RclrsError> {
        to_rclrs_result(*self)
    }
}

/// A helper trait to disregard timeouts as not an error.
pub trait RclrsErrorFilter {
    /// Get the first error available, or Ok(()) if there are no errors.
    fn first_error(self) -> Result<(), RclrsError>;

    /// If the result was a timeout error, change it to `Ok(())`.
    fn timeout_ok(self) -> Self;

    /// If a subscription, service, or client take failed, change the result
    /// to be `Ok(())`.
    fn take_failed_ok(self) -> Self;

    /// Some error types just indicate an early termination but do not indicate
    /// that anything in the system has misbehaved. This filters out anything
    /// that is part of the normal operation of rcl.
    fn ignore_non_errors(self) -> Self
    where
        Self: Sized,
    {
        self.timeout_ok().take_failed_ok()
    }
}

impl RclrsErrorFilter for Result<(), RclrsError> {
    fn first_error(self) -> Result<(), RclrsError> {
        self
    }

    fn timeout_ok(self) -> Result<(), RclrsError> {
        match self {
            Ok(()) => Ok(()),
            Err(err) => {
                if err.is_timeout() {
                    Ok(())
                } else {
                    Err(err)
                }
            }
        }
    }

    fn take_failed_ok(self) -> Result<(), RclrsError> {
        match self {
            Err(err) => {
                if err.is_take_failed() {
                    // Spurious wakeup - this may happen even when a waitset indicated that
                    // work was ready, so we won't report it as an error
                    Ok(())
                } else {
                    Err(err)
                }
            }
            other => other,
        }
    }
}

impl RclrsErrorFilter for Vec<RclrsError> {
    fn first_error(mut self) -> Result<(), RclrsError> {
        if self.is_empty() {
            return Ok(());
        }

        Err(self.remove(0))
    }

    fn timeout_ok(mut self) -> Self {
        self.retain(|err| !err.is_timeout());
        self
    }

    fn take_failed_ok(mut self) -> Self {
        self.retain(|err| !err.is_take_failed());
        self
    }
}

/// A helper trait to handle common error handling flows
pub trait TakeFailedAsNone {
    /// The type you would receive when there is no error
    type T;

    /// If the result has an error indicating that a take failed, convert the
    /// output into an `Ok(None)`. Any other error returns `Err(error)`. If there
    /// is no error, return `Ok(Some(value))`.
    fn take_failed_as_none(self) -> Result<Option<Self::T>, RclrsError>;
}

impl<T> TakeFailedAsNone for Result<T, RclrsError> {
    type T = T;
    fn take_failed_as_none(self) -> Result<Option<T>, RclrsError> {
        match self {
            Ok(value) => Ok(Some(value)),
            Err(err) => {
                if err.is_take_failed() {
                    return Ok(None);
                }

                return Err(err);
            }
        }
    }
}
