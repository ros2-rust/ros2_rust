// Copyright (c) 2019 Sequence Planner
// SPDX-License-Identifier: Apache-2.0 AND MIT
// Adapted from https://github.com/sequenceplanner/r2r/blob/89cec03d07a1496a225751159cbc7bfb529d9dd1/r2r/src/utils.rs
// Further adapted from https://github.com/mvukov/rules_ros2/pull/371

use std::{
    collections::HashMap,
    ffi::CString,
    sync::{Mutex, OnceLock},
};

use crate::{rcl_bindings::*, ENTITY_LIFECYCLE_MUTEX};

mod logging_configuration;
pub(crate) use logging_configuration::*;

mod log_params;
pub use log_params::*;

mod logger;
pub use logger::*;

/// log a message to rosout
///
/// # Examples
///
/// ```
/// use rclrs::{log, ToLogParams};
/// use std::sync::Mutex;
/// use std::time::Duration;
/// use std::env;
///
/// let context = rclrs::Context::new(env::args()).unwrap();
/// let node = rclrs::Node::new(&context, "test_node").unwrap();
///
/// log!(node.debug(), "Simple debug message");
/// let some_variable = 43;
/// log!(node.debug(), "Formatted debug message: {some_variable}");
/// log!(node.fatal(), "Fatal message from {}", node.name());
/// log!(node.warn().once(), "Only log this the first time");
/// log!(
///     node
///     .error()
///     .skip_first()
///     .interval(Duration::from_millis(1000)),
///     "Noisy error that we expect the first time"
/// );
///
/// let count = 0;
/// log!(
///     node
///     .info()
///     .interval(Duration::from_millis(1000))
///     .only_if(count % 10 == 0),
///     "Manually constructed LogConditions",
/// );
/// ```
///
/// All of the above examples will also work with the severity-specific log macros,
/// but any severity that gets passed in will be overridden:
/// - [`log_debug`][crate::log_debug]
/// - [`log_info`][crate::log_info]
/// - [`log_warn`][crate::log_warn]
/// - [`log_error`][crate::log_error]
/// - [`log_fatal`][crate::log_fatal]
///
/// # Panics
///
/// It is theoretically possible for the call to panic if the Mutex used for the throttling is
/// poisoned although this should never happen.
#[macro_export]
macro_rules! log {
    // The variable args captured by the $(, $($args:tt)*)?)) code allows us to omit (or include)
    // formatting parameters in the simple message case, e.g. to write
    // ```
    // log_error!(<logger>, "Log with no params"); // OR
    // log_error!(<logger>, "Log with useful info {}", error_reason);
    ($to_log_params: expr, $($args:tt)*) => {{
        // Adding these use statements here due an issue like this one:
        // https://github.com/intellij-rust/intellij-rust/issues/9853
        // Note: that issue appears to be specific to jetbrains intellisense however,
        // observed same/similar behaviour with rust-analyzer/rustc
        use std::sync::{Once, OnceLock, Mutex};
        use std::time::SystemTime;

        // We wrap the functional body of the macro inside of a closure which
        // we immediately trigger. This allows us to use `return` to exit the
        // macro early without causing the calling function to also try to
        // return early.
        (|| {
            let params = $crate::ToLogParams::to_log_params($to_log_params);

            if !params.get_user_filter() {
                // The user filter is telling us not to log this time, so exit
                // before doing anything else.
                return;
            }

            let mut first_time = false;
            static REMEMBER_FIRST_TIME: Once = Once::new();
            REMEMBER_FIRST_TIME.call_once(|| first_time = true);

            let logger_name = params.get_logger_name();
            let severity = params.get_severity();

            match params.get_occurence() {
                // Create the static variables here so we get a per-instance static
                $crate::LogOccurrence::Once => {
                    if first_time {
                        $crate::log_unconditional!(severity, logger_name, $($args)*);
                    }
                    // Since we've already logged once, we should never log again,
                    // so just exit right now.
                    return;
                }
                $crate::LogOccurrence::SkipFirst => {
                    if first_time {
                        // This is the first time that we're seeing this log, and we
                        // were told to skip the first one, so just exit right away.
                        return;
                    }
                }
                // Do nothing
                $crate::LogOccurrence::All => (),
            }

            // If we have a throttle interval then check if we're inside or outside
            // of that interval.
            let interval = params.get_interval();
            if interval > std::time::Duration::ZERO {
                static LAST_LOG_TIME: OnceLock<Mutex<SystemTime>> = OnceLock::new();
                let last_log_time = LAST_LOG_TIME.get_or_init(|| {
                    Mutex::new(std::time::SystemTime::now())
                });

                if !first_time {
                    let now = std::time::SystemTime::now();
                    let mut previous = last_log_time.lock().unwrap();
                    if now >= *previous + interval {
                        *previous = now;
                    } else {
                        // We are still inside the throttle interval, so just exit
                        // here.
                        return;
                    }
                }
            }

            // All filters have been checked, so go ahead and publish the message
            $crate::log_unconditional!(severity, logger_name, $($args)*);
        })();
    }};
}

/// Debug log message. See [`log`] for usage.
#[macro_export]
macro_rules! log_debug {
    ($to_log_params: expr, $($args:tt)*) => {{
        let log_params = $crate::ToLogParams::to_log_params($to_log_params);
        $crate::log!(log_params.debug(), $($args)*);
    }}
}

/// Info log message. See [`log`] for usage.
#[macro_export]
macro_rules! log_info {
    ($to_log_params: expr, $($args:tt)*) => {{
        let log_params = $crate::ToLogParams::to_log_params($to_log_params);
        $crate::log!(log_params.info(), $($args)*);
    }}
}

/// Warning log message. See [`log`] for usage.
#[macro_export]
macro_rules! log_warn {
    ($to_log_params: expr, $($args:tt)*) => {{
        let log_params = $crate::ToLogParams::to_log_params($to_log_params);
        $crate::log!(log_params.warn(), $($args)*);
    }}
}

/// Error log message. See [`log`] for usage.
#[macro_export]
macro_rules! log_error {
    ($to_log_params: expr, $($args:tt)*) => {{
        let log_params = $crate::ToLogParams::to_log_params($to_log_params);
        $crate::log!(log_params.error(), $($args)*);
    }}
}

/// Fatal log message. See [`log`] for usage.
#[macro_export]
macro_rules! log_fatal {
    ($to_log_params: expr, $($args:tt)*) => {{
        let log_params = $crate::ToLogParams::to_log_params($to_log_params);
        $crate::log!(log_params.fatal(), $($args)*);
    }}
}

/// A logging mechanism that does not have any conditions: It will definitely
/// publish the log. This is only meant for internal use, but needs to be exported
/// in order for [`log`] to work.
#[doc(hidden)]
#[macro_export]
macro_rules! log_unconditional {
    ($severity: expr, $logger_name: expr, $($args:tt)*) => {{
        use std::{ffi::CString, sync::OnceLock};

        // Only allocate a CString for the function name once per call to this macro.
        static FUNCTION_NAME: OnceLock<CString> = OnceLock::new();
        let function_name = FUNCTION_NAME.get_or_init(|| {
            CString::new($crate::function!()).unwrap_or(
                CString::new("<invalid name>").unwrap()
            )
        });

        // Only allocate a CString for the file name once per call to this macro.
        static FILE_NAME: OnceLock<CString> = OnceLock::new();
        let file_name = FILE_NAME.get_or_init(|| {
            CString::new(file!()).unwrap_or(
                CString::new("<invalid name>").unwrap()
            )
        });

        // We have to allocate a CString for the message every time because the
        // formatted data may have changed. We could consider having an alternative
        // macro for string literals that only allocates once, but it not obvious
        // how to guarantee that the user only passes in an unchanging string literal.
        match CString::new(std::fmt::format(format_args!($($args)*))) {
            Ok(message) => {
                // SAFETY: impl_log is actually completely safe to call, we just
                // mark it as unsafe to discourage downstream users from calling it
                unsafe { $crate::impl_log($severity, $logger_name, &message, &function_name, &file_name, line!()) };
            }
            Err(err) => {
                let message = CString::new(format!(
                    "Unable to format log message into a valid c-string. Error: {}", err
                )).unwrap();

                // SAFETY: impl_log is actually completely safe to call, we just
                // mark it as unsafe to discourage downstream users from calling it
                unsafe {
                    $crate::impl_log(
                        $crate::LogSeverity::Error,
                        &$crate::LoggerName::Unvalidated("logger"),
                        &message,
                        &function_name,
                        &file_name,
                        line!(),
                    );
                }
            }
        }
    }}
}

/// Calls the underlying rclutils logging function
/// Don't call this directly, use the logging macros instead, i.e. [`log`].
///
/// SAFETY: This function is not actually unsafe, but it is not meant to be part of the public
/// API, so we mark it as unsafe to discourage users from trying to use it. They should use
/// one of the of log! macros instead. We can't make it private because it needs to be used
/// by exported macros.
#[doc(hidden)]
pub unsafe fn impl_log(
    severity: LogSeverity,
    logger_name: &LoggerName,
    message: &CString,
    function: &CString,
    file: &CString,
    line: u32,
) {
    // We use a closure here because there are several different points in this
    // function where we may need to run this same logic.
    let send_log = |severity: LogSeverity, logger_name: &CString, message: &CString| {
        let location = rcutils_log_location_t {
            function_name: function.as_ptr(),
            file_name: file.as_ptr(),
            line_number: line as usize,
        };
        static FORMAT_CSTR: OnceLock<CString> = OnceLock::new();
        let format_cstr = FORMAT_CSTR.get_or_init(|| CString::new("%s").unwrap());

        let severity = severity.as_native();

        let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: Global variables are protected via ENTITY_LIFECYCLE_MUTEX, no other preconditions are required
        unsafe {
            rcutils_log(
                &location,
                severity as i32,
                logger_name.as_ptr(),
                format_cstr.as_ptr(),
                message.as_ptr(),
            );
        }
    };

    match logger_name {
        LoggerName::Validated(c_name) => {
            // The logger name is pre-validated, so just go ahead and use it.
            send_log(severity, c_name, message);
        }
        LoggerName::Unvalidated(str_name) => {
            // The name was not validated before being passed in.
            //
            // We maintain a hashmap of previously validated loggers so
            // we don't need to reallocate the CString on every log instance.
            // This is done inside of the function impl_log instead of in a macro
            // so that this map is global for the entire application.
            static NAME_MAP: OnceLock<Mutex<HashMap<String, CString>>> = OnceLock::new();
            let name_map = NAME_MAP.get_or_init(Default::default);

            {
                // We need to keep name_map locked while we call send_log, but
                // we also need to make sure it gets unlocked right afterward
                // if the condition fails, otherwise this function would
                // deadlock on itself when handling the error case of the logger
                // name being invalid. So we keep name_map_guard in this extra
                // scope to isolate its lifespan.
                let name_map_guard = name_map.lock().unwrap();
                if let Some(c_name) = name_map_guard.get(*str_name) {
                    // The map name has been used before, so we just use the
                    // pre-existing CString
                    send_log(severity, c_name, message);

                    // We return right away because the remainder of this
                    // function just allocates and validates a new CString for
                    // the logger name.
                    return;
                }
            }

            // The unvalidated logger name has not been used before, so we need
            // to convert it and add it to the name_map now.
            let c_name = match CString::new(str_name.to_string()) {
                Ok(c_name) => c_name,
                Err(_) => {
                    static INVALID_MSG: OnceLock<CString> = OnceLock::new();
                    let invalid_msg = INVALID_MSG.get_or_init(|| {
                        CString::new(
                            "Failed to convert logger name into a c-string. \
                            Check for null terminators inside the string.",
                        )
                        .unwrap()
                    });
                    static INTERNAL_LOGGER_NAME: OnceLock<CString> = OnceLock::new();
                    let internal_logger_name =
                        INTERNAL_LOGGER_NAME.get_or_init(|| CString::new("logger").unwrap());
                    send_log(severity, internal_logger_name, invalid_msg);
                    return;
                }
            };

            send_log(severity, &c_name, message);
            name_map
                .lock()
                .unwrap()
                .insert(str_name.to_string(), c_name);
        }
    }
}

/// Used internally by logging macros to get the name of the function that called the
/// logging macro. This is not meant for public use, but we need to export it so the
/// other exported macros can use it. We should remove it if an official function! macro
/// is ever offered.
#[macro_export]
#[doc(hidden)]
macro_rules! function {
    () => {{
        fn f() {}
        fn type_name_of<T>(_: T) -> &'static str {
            std::any::type_name::<T>()
        }
        let name = type_name_of(f);
        name.strip_suffix("::f").unwrap()
    }};
}

#[cfg(test)]
mod tests {
    use crate::{test_helpers::*, *};

    #[test]
    fn test_logging_macros() -> Result<(), RclrsError> {
        let graph = construct_test_graph("test_logging_macros")?;
        let node = graph.node1;

        log!(&*node, "Logging with node dereference");

        for _ in 0..10 {
            log!(node.once(), "Logging once");
        }

        log!(node.logger(), "Logging with node logger");
        log!(node.debug(), "Debug from node");
        log!(node.info(), "Info from node");
        log!(node.warn(), "Warn from node");
        log!(node.error(), "Error from node");
        log!(node.fatal(), "Fatal from node");

        log_debug!(node.logger(), "log_debug macro");
        log_info!(node.logger(), "log_info macro");
        log_warn!(node.logger(), "log_warn macro");
        log_error!(node.logger(), "log_error macro");
        log_fatal!(node.logger(), "log_fatal macro");

        log!(node.only_if(false), "This should not be logged",);
        log!(node.only_if(true), "This should be logged",);

        for i in 0..3 {
            log!(node.warn().skip_first(), "Formatted warning #{}", i);
        }

        node.logger().set_level(LogSeverity::Debug).unwrap();
        log_debug!(node.logger(), "This debug message appears");
        node.logger().set_level(LogSeverity::Info).unwrap();
        log_debug!(node.logger(), "This debug message does not");

        log!("custom logger name", "message for custom logger");
        for _ in 0..3 {
            log!(
                "custom logger name".once(),
                "one-time message for custom logger"
            );
        }

        for _ in 0..3 {
            log!(
                "custom logger name".error().skip_first(),
                "error for custom logger",
            );
        }

        Ok(())
    }

    #[test]
    fn test_function_macro() {
        assert_eq!(function!(), "rclrs::logging::tests::test_function_macro");
    }
}
