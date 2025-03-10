use crate::{rcl_bindings::RCUTILS_LOG_SEVERITY, Clock};
use std::{borrow::Borrow, ffi::CString, time::Duration};

/// These parameters determine the behavior of an instance of logging.
#[derive(Debug, Clone, Copy)]
pub struct LogParams<'a> {
    /// The name of the logger
    logger_name: LoggerName<'a>,
    /// The severity of the logging instance.
    severity: LogSeverity,
    /// Specify when a log message should be published (See[`LoggingOccurrence`] above)
    occurs: LogOccurrence,
    /// Specify a publication throttling interval for the message.  A value of ZERO (0) indicates that the
    /// message should not be throttled. Otherwise, the message will only be published once the specified
    /// interval has elapsed. This field is typically used to limit the output from high-frequency messages,
    /// e.g. if `log!(logger.throttle(Duration::from_secs(1)), "message");` is called every 10ms, it will
    /// nevertheless only be published once per second.
    throttle: Duration,
    /// Specify a clock to use for throttling. By default this will be [`ThrottleClock::SteadyTime`].
    throttle_clock: ThrottleClock<'a>,
    /// The log message will only published if the specified expression evaluates to true
    only_if: bool,
}

impl<'a> LogParams<'a> {
    /// Create a set of default log parameters, given the name of a logger
    pub fn new(logger_name: LoggerName<'a>) -> Self {
        Self {
            logger_name,
            severity: Default::default(),
            occurs: Default::default(),
            throttle: Duration::new(0, 0),
            throttle_clock: Default::default(),
            only_if: true,
        }
    }

    /// Get the logger name
    pub fn get_logger_name(&self) -> &LoggerName {
        &self.logger_name
    }

    /// Get the severity of the log
    pub fn get_severity(&self) -> LogSeverity {
        self.severity
    }

    /// Get the occurrence
    pub fn get_occurence(&self) -> LogOccurrence {
        self.occurs
    }

    /// Get the throttle interval duration
    pub fn get_throttle(&self) -> Duration {
        self.throttle
    }

    /// Get the throttle clock
    pub fn get_throttle_clock(&self) -> ThrottleClock<'a> {
        self.throttle_clock
    }

    /// Get the arbitrary filter set by the user
    pub fn get_user_filter(&self) -> bool {
        self.only_if
    }
}

/// Methods for defining the behavior of a logging instance.
///
/// This trait is implemented by Logger, Node, and anything that a `&str` can be
/// [borrowed][1] from, such as string literals (`"my_str"`), [`String`], or
/// [`Cow<str>`][2].
///
/// [1]: Borrow
/// [2]: std::borrow::Cow
pub trait ToLogParams<'a>: Sized {
    /// Convert the object into LogParams with default settings
    fn to_log_params(self) -> LogParams<'a>;

    /// The logging should only happen once
    fn once(self) -> LogParams<'a> {
        self.occurs(LogOccurrence::Once)
    }

    /// The first time the logging happens, we should skip it
    fn skip_first(self) -> LogParams<'a> {
        self.occurs(LogOccurrence::SkipFirst)
    }

    /// Set the occurrence behavior of the log instance
    fn occurs(self, occurs: LogOccurrence) -> LogParams<'a> {
        let mut params = self.to_log_params();
        params.occurs = occurs;
        params
    }

    /// Set a throttling interval during which this log will not publish. A value
    /// of zero will never block the message from being published, and this is the
    /// default behavior.
    ///
    /// A negative duration is not valid, but will be treated as a zero duration.
    fn throttle(self, throttle: Duration) -> LogParams<'a> {
        let mut params = self.to_log_params();
        params.throttle = throttle;
        params
    }

    /// Set the clock that will be used to control [throttling][Self::throttle].
    fn throttle_clock(self, clock: ThrottleClock<'a>) -> LogParams<'a> {
        let mut params = self.to_log_params();
        params.throttle_clock = clock;
        params
    }

    /// The log will not be published if a `false` expression is passed into
    /// this function.
    ///
    /// Other factors may prevent the log from being published if a `true` is
    /// passed in, such as `ToLogParams::throttle` or `ToLogParams::once`
    /// filtering the log.
    fn only_if(self, only_if: bool) -> LogParams<'a> {
        let mut params = self.to_log_params();
        params.only_if = only_if;
        params
    }

    /// Log as a debug message.
    fn debug(self) -> LogParams<'a> {
        self.severity(LogSeverity::Debug)
    }

    /// Log as an informative message. This is the default, so you don't
    /// generally need to use this.
    fn info(self) -> LogParams<'a> {
        self.severity(LogSeverity::Info)
    }

    /// Log as a warning message.
    fn warn(self) -> LogParams<'a> {
        self.severity(LogSeverity::Warn)
    }

    /// Log as an error message.
    fn error(self) -> LogParams<'a> {
        self.severity(LogSeverity::Error)
    }

    /// Log as a fatal message.
    fn fatal(self) -> LogParams<'a> {
        self.severity(LogSeverity::Fatal)
    }

    /// Set the severity for this instance of logging. The default value will be
    /// [`LogSeverity::Info`].
    fn severity(self, severity: LogSeverity) -> LogParams<'a> {
        let mut params = self.to_log_params();
        params.severity = severity;
        params
    }
}

/// This is used to borrow a logger name which might be validated (e.g. came
/// from a [`Logger`][1] struct) or not (e.g. a user-defined `&str`). If an
/// unvalidated logger name is used with one of the logging macros, we will log
/// an error about it, and the original log message will be logged with the
/// default logger.
///
/// [1]: crate::Logger
#[derive(Debug, Clone, Copy)]
pub enum LoggerName<'a> {
    /// The logger name is already available as a valid CString
    Validated(&'a CString),
    /// The logger name has not been validated yet
    Unvalidated(&'a str),
}

/// Logging severity.
//
// TODO(@mxgrey): Consider whether this is redundant with RCUTILS_LOG_SEVERITY.
// Perhaps we can customize the output of bindgen to automatically change the name
// of RCUTILS_LOG_SEVERITY to just LogSeverity so it's more idiomatic and then
// export it from the rclrs module.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LogSeverity {
    /// Use the severity level of the parent logger (or the root logger if the
    /// current logger has no parent)
    Unset,
    /// For messages that are not needed outside of debugging.
    Debug,
    /// For messages that provide useful information about the state of the
    /// application.
    Info,
    /// For messages that indicate something unusual or unintended might have happened.
    Warn,
    /// For messages that indicate an error has occurred which may cause the application
    /// to misbehave.
    Error,
    /// For messages that indicate an error has occurred which is so severe that the
    /// application should terminate because it cannot recover.
    ///
    /// Using this severity level will not automatically cause the application to
    /// terminate, the application developer must decide how to do that on a
    /// case-by-case basis.
    Fatal,
}

impl LogSeverity {
    pub(super) fn as_native(&self) -> RCUTILS_LOG_SEVERITY {
        use crate::rcl_bindings::rcl_log_severity_t::*;
        match self {
            LogSeverity::Unset => RCUTILS_LOG_SEVERITY_UNSET,
            LogSeverity::Debug => RCUTILS_LOG_SEVERITY_DEBUG,
            LogSeverity::Info => RCUTILS_LOG_SEVERITY_INFO,
            LogSeverity::Warn => RCUTILS_LOG_SEVERITY_WARN,
            LogSeverity::Error => RCUTILS_LOG_SEVERITY_ERROR,
            LogSeverity::Fatal => RCUTILS_LOG_SEVERITY_FATAL,
        }
    }

    /// This is only used by the log output handler during testing, so it will
    /// not be compiled when testing is not configured
    #[cfg(test)]
    pub(crate) fn from_native(native: i32) -> Self {
        use crate::rcl_bindings::rcl_log_severity_t::*;
        match native {
            _ if native == RCUTILS_LOG_SEVERITY_UNSET as i32 => LogSeverity::Unset,
            _ if native == RCUTILS_LOG_SEVERITY_DEBUG as i32 => LogSeverity::Debug,
            _ if native == RCUTILS_LOG_SEVERITY_INFO as i32 => LogSeverity::Info,
            _ if native == RCUTILS_LOG_SEVERITY_WARN as i32 => LogSeverity::Warn,
            _ if native == RCUTILS_LOG_SEVERITY_ERROR as i32 => LogSeverity::Error,
            _ if native == RCUTILS_LOG_SEVERITY_FATAL as i32 => LogSeverity::Fatal,
            _ => panic!("Invalid native severity received: {}", native),
        }
    }
}

impl Default for LogSeverity {
    fn default() -> Self {
        Self::Info
    }
}

/// Specify when a log message should be published
#[derive(Debug, Clone, Copy)]
pub enum LogOccurrence {
    /// Every message will be published if all other conditions are met
    All,
    /// The message will only be published on the first occurrence (Note: no other conditions apply)
    Once,
    /// The log message will not be published on the first occurrence, but will be published on
    /// each subsequent occurrence (assuming all other conditions are met)
    SkipFirst,
}

/// This parameter can specify a type of clock for a logger to use when throttling.
#[derive(Debug, Default, Clone, Copy)]
pub enum ThrottleClock<'a> {
    /// Use [`std::time::Instant`] as a clock.
    #[default]
    SteadyTime,
    /// Use [`std::time::SystemTime`] as a clock.
    SystemTime,
    /// Use some [`Clock`] as a clock.
    Clock(&'a Clock),
}

impl Default for LogOccurrence {
    fn default() -> Self {
        Self::All
    }
}

// Anything that we can borrow a string from can be used as if it's a logger and
// turned into LogParams
impl<'a, T: Borrow<str>> ToLogParams<'a> for &'a T {
    fn to_log_params(self) -> LogParams<'a> {
        LogParams::new(LoggerName::Unvalidated(self.borrow()))
    }
}

impl<'a> ToLogParams<'a> for &'a str {
    fn to_log_params(self) -> LogParams<'a> {
        LogParams::new(LoggerName::Unvalidated(self))
    }
}

impl<'a> ToLogParams<'a> for LogParams<'a> {
    fn to_log_params(self) -> LogParams<'a> {
        self
    }
}
