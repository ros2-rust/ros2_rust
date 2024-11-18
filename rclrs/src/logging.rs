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
///     .throttle(Duration::from_millis(1000)),
///     "Noisy error that we expect the first time"
/// );
///
/// let count = 0;
/// log!(
///     node
///     .info()
///     .throttle(Duration::from_millis(1000))
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

            // If we have a throttle duration then check if we're inside or outside
            // of that interval.
            let throttle = params.get_throttle();
            if throttle > std::time::Duration::ZERO {
                static LAST_LOG_TIME: OnceLock<Mutex<SystemTime>> = OnceLock::new();
                let last_log_time = LAST_LOG_TIME.get_or_init(|| {
                    Mutex::new(std::time::SystemTime::now())
                });

                if !first_time {
                    let now = std::time::SystemTime::now();
                    let mut previous = last_log_time.lock().unwrap();
                    if now >= *previous + throttle {
                        *previous = now;
                    } else {
                        // We are still inside the throttle interval, so just exit here.
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
            // This call to function! is nested within two layers of closures,
            // so we need to strip away those suffixes or else users will be
            // misled. If we ever restructure these macros or if Rust changes
            // the way it names closures, this implementation detail may need to
            // change.
            let function_name = $crate::function!()
                .strip_suffix("::{{closure}}::{{closure}}")
                .unwrap();

            CString::new(function_name).unwrap_or(
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

        static FORMAT_STRING: OnceLock<CString> = OnceLock::new();
        let format_string = FORMAT_STRING.get_or_init(|| CString::new("%s").unwrap());

        let severity = severity.as_native();

        let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();

        #[cfg(test)]
        {
            // If we are compiling for testing purposes, when the default log
            // output handler is being used we need to use the format_string,
            // but when our custom log output handler is being used we need to
            // pass the raw message string so that it can be viewed by the
            // custom log output handler, allowing us to use it for test assertions.
            if log_handler::is_using_custom_handler() {
                // We are using the custom log handler that is only used during
                // logging tests, so pass the raw message as the format string.
                unsafe {
                    // SAFETY: The global mutex is locked as _lifecycle
                    rcutils_log(
                        &location,
                        severity as i32,
                        logger_name.as_ptr(),
                        message.as_ptr(),
                    );
                }
            } else {
                // We are using the normal log handler so call rcutils_log the normal way.
                unsafe {
                    // SAFETY: The global mutex is locked as _lifecycle
                    rcutils_log(
                        &location,
                        severity as i32,
                        logger_name.as_ptr(),
                        format_string.as_ptr(),
                        message.as_ptr(),
                    );
                }
            }
        }

        #[cfg(not(test))]
        {
            unsafe {
                // SAFETY: The global mutex is locked as _lifecycle
                rcutils_log(
                    &location,
                    severity as i32,
                    logger_name.as_ptr(),
                    format_string.as_ptr(),
                    message.as_ptr(),
                );
            }
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
    use crate::{log_handler::*, test_helpers::*, *};
    use std::sync::Mutex;

    #[test]
    fn test_logging_macros() -> Result<(), RclrsError> {
        // This test ensures that strings which are being sent to the logger are
        // being sanitized correctly. Rust generally and our logging macro in
        // particular do not use C-style formatting strings, but rcutils expects
        // to receive C-style formatting strings alongside variadic arguments
        // that describe how to fill in the formatting.
        //
        // If we pass the final string into rcutils as the format with no
        // variadic arguments, then it may trigger a crash or undefined behavior
        // if the message happens to contain any % symbols. In particular %n
        // will trigger a crash when no variadic arguments are given because it
        // attempts to write to a buffer. If no buffer is given, a seg fault
        // happens.
        log!("please do not crash", "%n");

        let graph = construct_test_graph("test_logging_macros")?;

        let log_collection: Arc<Mutex<Vec<LogEntry<'static>>>> = Arc::new(Mutex::new(Vec::new()));
        let inner_log_collection = log_collection.clone();

        log_handler::set_logging_output_handler(move |log_entry: log_handler::LogEntry| {
            inner_log_collection
                .lock()
                .unwrap()
                .push(log_entry.into_owned());
        })
        .unwrap();

        let last_logger_name = || {
            log_collection
                .lock()
                .unwrap()
                .last()
                .unwrap()
                .logger_name
                .clone()
        };

        let last_message = || {
            log_collection
                .lock()
                .unwrap()
                .last()
                .unwrap()
                .message
                .clone()
        };

        let last_location = || {
            log_collection
                .lock()
                .unwrap()
                .last()
                .unwrap()
                .location
                .clone()
        };

        let last_severity = || log_collection.lock().unwrap().last().unwrap().severity;

        let count_message = |message: &str| {
            let mut count = 0;
            for log in log_collection.lock().unwrap().iter() {
                if log.message == message {
                    count += 1;
                }
            }
            count
        };

        let node = graph.node1;

        log!(&*node, "Logging with node dereference");
        assert_eq!(last_logger_name(), node.logger().name());
        assert_eq!(last_message(), "Logging with node dereference");
        assert_eq!(last_severity(), LogSeverity::Info);
        assert_eq!(
            last_location().function_name,
            "rclrs::logging::tests::test_logging_macros",
        );

        for _ in 0..10 {
            log!(node.once(), "Logging once");
        }
        assert_eq!(count_message("Logging once"), 1);
        assert_eq!(last_severity(), LogSeverity::Info);

        log!(node.logger(), "Logging with node logger");
        assert_eq!(last_message(), "Logging with node logger");
        assert_eq!(last_severity(), LogSeverity::Info);

        log!(node.debug(), "Debug from node");
        // The default severity level is Info so we should not see the last message
        assert_ne!(last_message(), "Debug from node");
        assert_ne!(last_severity(), LogSeverity::Debug);

        log!(node.info(), "Info from node");
        assert_eq!(last_message(), "Info from node");
        assert_eq!(last_severity(), LogSeverity::Info);

        log!(node.warn(), "Warn from node");
        assert_eq!(last_message(), "Warn from node");
        assert_eq!(last_severity(), LogSeverity::Warn);

        log!(node.error(), "Error from node");
        assert_eq!(last_message(), "Error from node");
        assert_eq!(last_severity(), LogSeverity::Error);

        log!(node.fatal(), "Fatal from node");
        assert_eq!(last_message(), "Fatal from node");
        assert_eq!(last_severity(), LogSeverity::Fatal);

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
        assert_eq!(count_message("Formatted warning #0"), 0);
        assert_eq!(count_message("Formatted warning #1"), 1);
        assert_eq!(count_message("Formatted warning #2"), 1);

        node.logger().set_level(LogSeverity::Debug).unwrap();
        log_debug!(node.logger(), "This debug message appears");
        assert_eq!(last_message(), "This debug message appears");
        assert_eq!(last_severity(), LogSeverity::Debug);

        node.logger().set_level(LogSeverity::Info).unwrap();
        log_debug!(node.logger(), "This debug message does not appear");
        assert_ne!(last_message(), "This debug message does not appear");

        log!("custom logger name", "message for custom logger");
        assert_eq!(last_logger_name(), "custom logger name");
        assert_eq!(last_message(), "message for custom logger");

        for _ in 0..3 {
            log!(
                "custom logger name once".once(),
                "one-time message for custom logger",
            );
        }
        assert_eq!(last_logger_name(), "custom logger name once");
        assert_eq!(last_severity(), LogSeverity::Info);
        assert_eq!(count_message("one-time message for custom logger"), 1);

        for _ in 0..3 {
            log!(
                "custom logger name skip".error().skip_first(),
                "error for custom logger",
            );
        }
        assert_eq!(last_logger_name(), "custom logger name skip");
        assert_eq!(last_severity(), LogSeverity::Error);
        assert_eq!(count_message("error for custom logger"), 2);

        reset_logging_output_handler();

        Ok(())
    }

    #[test]
    fn test_function_macro() {
        assert_eq!(function!(), "rclrs::logging::tests::test_function_macro");
    }
}
