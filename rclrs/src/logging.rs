// Copyright (c) 2019 Sequence Planner
// SPDX-License-Identifier: Apache-2.0 AND MIT
// Adapted from https://github.com/sequenceplanner/r2r/blob/89cec03d07a1496a225751159cbc7bfb529d9dd1/r2r/src/utils.rs
// Further adapted from https://github.com/mvukov/rules_ros2/pull/371

use std::{ffi::CString, time::Duration};

use crate::{rcl_bindings::*, ENTITY_LIFECYCLE_MUTEX};

mod logging_configuration;
pub(crate) use logging_configuration::*;

/// Calls the underlying rclutils logging function
/// Don't call this directly, use the logging macros instead.
///
/// # Panics
///
/// This function might panic in the following scenarios:
/// - A logger_name is provided that is not a valid c-string, e.g. contains extraneous null characters
/// - The user-supplied "msg" is not a valid c-string, e.g. contains extraneous null characters
/// - When called if the lock is already held by the current thread.
/// - If the construction of CString objects used to create the log output fail,
///   although, this highly unlikely to occur in most cases
#[doc(hidden)]
pub fn log(msg: &str, logger_name: &str, file: &str, line: u32, severity: LogSeverity) {
    // currently not possible to get function name in rust.
    // see https://github.com/rust-lang/rfcs/pull/2818
    let function = CString::new("").unwrap();
    let file = CString::new(file).unwrap();
    let location = rcutils_log_location_t {
        function_name: function.as_ptr(),
        file_name: file.as_ptr(),
        line_number: line as usize,
    };
    let format = CString::new("%s").unwrap();
    let logger_name = CString::new(logger_name)
        .expect("Logger name is a valid c style string, e.g. check for extraneous null characters");
    let message = CString::new(msg)
        .expect("Valid c style string required, e.g. check for extraneous null characters");
    let severity = severity.to_native();

    let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
    // SAFETY: Global variables are protected via ENTITY_LIFECYCLE_MUTEX, no other preconditions are required
    unsafe {
        rcutils_log(
            &location,
            severity as i32,
            logger_name.as_ptr(),
            format.as_ptr(),
            message.as_ptr(),
        );
    }
}

/// Logging severity
#[doc(hidden)]
pub enum LogSeverity {
    Unset,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,
}

impl LogSeverity {
    fn to_native(&self) -> RCUTILS_LOG_SEVERITY {
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
}

#[derive(Debug)]
/// Specify when a log message should be published
pub enum LoggingOccurrence {
    /// The log message will always be published (assuming all other conditions are met)
    Always,
    /// The message will only be published on the first occurrence (Note: no other conditions apply)
    Once,
    /// The log message will not be published on the first occurrence, but will be published on
    /// each subsequent occurrence (assuming all other conditions are met)
    SkipFirst,
}

/// Specify conditions that must be met for a log message to be published
///
/// The struct provides the following convenience functions to construct conditions that match
/// behaviour available in the `rclcpp` and `rclpy` libraries.
///
/// When will my log message be output?
///
/// - `Once`: A message with the [`LoggingOccurrence::Once`] value will be published once only
///           regardless of any other conditions
/// - `SkipFirst`: A message with the [`LoggingOccurrence::SkipFirst`] value will never be published
///                on the first encounter regardless of any other conditions.  After the first
///                encounter, the behaviour is identical to the [`LoggingOccurrence::Always`] setting.
/// - `Always`: The log message will be output if all additional conditions are true:
///     - The current time + the `publish_interval` > the last time the message was published.
///         - The default value for `publish_interval` is 0, i.e. the interval check will always pass
///     - The `log_if_true` expression evaluates to TRUE.
///         - The default value for the `log_if_true` field is TRUE.
pub struct LogConditions {
    /// Specify when a log message should be published (See[`LoggingOccurrence`] above)
    pub occurs: LoggingOccurrence,
    /// Specify the publication interval of the message.  A value of ZERO (0) indicates that the
    /// message should be published every time, otherwise, the message will only be published once
    /// the specified interval has elapsed.
    /// This field is typically used to limit the output from high-frequency messages, e.g. instead
    /// of publishing a log message every 10 milliseconds, the `publish_interval` can be configured
    /// such that the message is published every 10 seconds.
    pub publish_interval: Duration,
    /// The log message will only published if the specified expression evaluates to true
    pub log_if_true: bool,
}

impl LogConditions {
    /// Default construct an instance
    pub fn new() -> Self {
        Self {
            occurs: LoggingOccurrence::Always,
            publish_interval: Duration::ZERO,
            log_if_true: true,
        }
    }

    /// Only publish this message the first time it is encountered
    pub fn once() -> Self {
        Self {
            occurs: LoggingOccurrence::Once,
            publish_interval: Duration::ZERO,
            log_if_true: true,
        }
    }

    /// Do not publish the message the first time it is encountered
    pub fn skip_first() -> Self {
        Self {
            occurs: LoggingOccurrence::SkipFirst,
            publish_interval: Duration::ZERO,
            log_if_true: true,
        }
    }

    /// Do not publish the first time this message is encountered and publish
    /// at the specified `publish_interval` thereafter
    pub fn skip_first_throttle(publish_interval: Duration) -> Self {
        Self {
            occurs: LoggingOccurrence::SkipFirst,
            publish_interval,
            log_if_true: true,
        }
    }

    /// Throttle the message to the supplied publish_interval
    /// e.g. set `publish_interval` to 1000ms to limit publishing to once a second
    pub fn throttle(publish_interval: Duration) -> Self {
        Self {
            occurs: LoggingOccurrence::Always,
            publish_interval,
            log_if_true: true,
        }
    }

    /// Permitting logging if the supplied expression evaluates to true
    /// Uses default LoggingOccurrence (Always) and publish_interval (no throttling)
    pub fn log_if_true(log_if_true: bool) -> Self {
        Self {
            occurs: LoggingOccurrence::Always,
            publish_interval: Duration::ZERO,
            log_if_true,
        }
    }
}

/// log_with_conditions
/// Helper macro to log a message using the ROS2 RCUTILS library
///
/// The macro supports two general calling formats:
/// 1. Plain message string e.g. as per println! macro
/// 2. With calling conditions that provide some restrictions on the output of the message
/// (see [`LogConditions`] above)
///
/// It is expected that, typically, the macro will be called using one of the wrapper macros,
/// e.g. log_debug!, etc, however, it is relatively straight forward to call the macro directly
/// if you really want to.
///
/// # Examples
///
/// ```
/// use rclrs::{log_debug, log_error, log_fatal, log_info, log_warn, LogConditions, LoggingOccurrence};
/// use std::sync::Mutex;
/// use std::time::Duration;
/// use std::env;
///
/// let context = rclrs::Context::new(env::args()).unwrap();
/// let node = rclrs::Node::new(&context, "log_example_node").unwrap();
///
/// log_debug!(&node.name(), "Simple message");
/// let some_variable = 43;
/// log_debug!(&node.name(), "Simple message {some_variable}");
/// log_fatal!(&node.name(), "Simple message from {}", node.name());
/// log_warn!(&node.name(), LogConditions::once(), "Only log this the first time");
/// log_error!(&node.name(),
///            LogConditions::skip_first_throttle(Duration::from_millis(1000)),
///            "Noisy error that we expect the first time");
///
/// let count = 0;
/// log_info!(&node.name(), LogConditions { occurs: LoggingOccurrence::Always,
///                                         publish_interval: Duration::from_millis(1000),
///                                         log_if_true: count % 10 == 0, },
///           "Manually constructed LogConditions");
/// ```
///
/// # Panics
///
/// It is theoretically possible for the call to panic if the Mutex used for the throttling is
/// poisoned although this should not be possible.
#[macro_export]
macro_rules! log_with_conditions {
    // The variable args captured by the $(, $($args:tt)*)?)) code allows us to omit (or include)
    // parameters in the simple message case, e.g. to write
    // ```
    // log_error!(<logger>, "Log with no params"); // OR
    // log_error!(<logger>, "Log with useful info {}", error_reason);
    ($severity: expr, $logger_name: expr, $msg_start: literal $(, $($args:tt)*)?) => {
        $crate::log(&std::fmt::format(format_args!($msg_start, $($($args)*)?)), $logger_name, file!(), line!(), $severity);
    };
    ($severity: expr, $logger_name: expr, $conditions: expr, $($args:tt)*) => {
        // Adding these use statements here due an issue like this one:
        // https://github.com/intellij-rust/intellij-rust/issues/9853
        // Note: that issue appears to be specific to jetbrains intellisense however,
        // observed same/similar behaviour with rust-analyzer/rustc
        use std::sync::Once;
        use std::time::SystemTime;

        let log_conditions: $crate::LogConditions = $conditions;
        let mut allow_logging = true;
        match log_conditions.occurs {
            // Create the static variables here so we get a per-instance static
            $crate::LoggingOccurrence::Once => {
                static LOG_ONCE: std::sync::Once = std::sync::Once::new();
                LOG_ONCE.call_once(|| {
                    $crate::log(&std::fmt::format(format_args!($($args)*)), $logger_name, file!(), line!(), $severity);
                });
                allow_logging = false;
            }
            $crate::LoggingOccurrence::SkipFirst => {
                // Noop, just make sure we exit the first time...
                static SKIP_FIRST: std::sync::Once = std::sync::Once::new();
                SKIP_FIRST.call_once(|| {
                    // Only disable logging the first time
                    allow_logging = false;
                });

            }
            // Drop out
            $crate::LoggingOccurrence::Always => (),
        }

        // If we have a throttle period AND logging has not already been disabled due to SkipFirst settings
        if log_conditions.publish_interval > std::time::Duration::ZERO && allow_logging {
            let mut ignore_first_timeout = false;
            // Need to initialise to a constant
            static LAST_LOG_TIME: Mutex<std::time::SystemTime> = Mutex::new(std::time::SystemTime::UNIX_EPOCH);

            static INIT_LAST_LOG_TIME: std::sync::Once = std::sync::Once::new();
            // Set the last log time to "now", but let us log the message the first time we hit this
            // code, i.e. initial behaviour is expired.
            // Note: If this is part of a SkipFirst macro call, we will only hit this code on the second iteration.
            INIT_LAST_LOG_TIME.call_once(|| {
                let mut last_log_time = LAST_LOG_TIME.lock().unwrap();
                *last_log_time = std::time::SystemTime::now();
                ignore_first_timeout = true;
            });

            let mut last_log_time = LAST_LOG_TIME.lock().unwrap();
            if std::time::SystemTime::now() >= *last_log_time + log_conditions.publish_interval {
                // Update our time stamp
                *last_log_time = std::time::SystemTime::now();
            }
            else if !ignore_first_timeout {
                // Timer has not expired (and this is not the first time through here)
                allow_logging = false;
            }
        }

        // At this point we've validated the skip/throttle operations, and we now check that any
        // expression supplied also evaluates to true, e.g. if timer has expired but expression is
        // false, we won't print
        if (allow_logging && log_conditions.log_if_true)
        {
            $crate::log(&std::fmt::format(format_args!($($args)*)), $logger_name, file!(), line!(), $severity);
        }
    };
}

/// Debug log message.
#[macro_export]
macro_rules! log_debug {
    ($logger_name: expr, $($args:tt)*) => {{
        $crate::log_with_conditions!($crate::LogSeverity::Debug, $logger_name, $($args)*);
    }}
}

/// Info log message.
#[macro_export]
macro_rules! log_info {
    ($logger_name: expr, $($args:tt)*) => {{
        $crate::log_with_conditions!($crate::LogSeverity::Info, $logger_name, $($args)*);
    }}
}

/// Warning log message.
#[macro_export]
macro_rules! log_warn {
    ($logger_name: expr, $($args:tt)*) => {{
        $crate::log_with_conditions!($crate::LogSeverity::Warn, $logger_name, $($args)*);
    }}
}

/// Error log message.
#[macro_export]
macro_rules! log_error {
    ($logger_name: expr, $($args:tt)*) => {{
        $crate::log_with_conditions!($crate::LogSeverity::Error, $logger_name, $($args)*);
    }}
}

/// Fatal log message.
#[macro_export]
macro_rules! log_fatal {
    ($logger_name: expr, $($args:tt)*) => {{
        $crate::log_with_conditions!($crate::LogSeverity::Fatal, $logger_name, $($args)*);
    }}
}
