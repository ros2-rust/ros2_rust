use std::sync::{Arc, Mutex, OnceLock, Weak};

use crate::{rcl_bindings::*, RclrsError, ToResult, ENTITY_LIFECYCLE_MUTEX};

struct LoggingConfiguration {
    lifecycle: Mutex<Weak<LoggingLifecycle>>,
}

pub(crate) struct LoggingLifecycle;

impl LoggingLifecycle {
    fn new(args: &rcl_arguments_t) -> Result<Self, RclrsError> {
        // SAFETY:
        // * Lock the mutex as we cannot guarantee that rcl_* functions are protecting their global variables
        // * This is only called by Self::configure, which requires that a valid context was passed to it
        // * No other preconditions for calling this function
        unsafe {
            let allocator = rcutils_get_default_allocator();
            let _lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            rcl_logging_configure(args, &allocator).ok()?;
        }
        Ok(Self)
    }

    /// SAFETY: Ensure rcl_context_t is valid before passing it in.
    pub(crate) unsafe fn configure(
        context: &rcl_context_t,
    ) -> Result<Arc<LoggingLifecycle>, RclrsError> {
        static CONFIGURATION: OnceLock<LoggingConfiguration> = OnceLock::new();
        let configuration = CONFIGURATION.get_or_init(|| LoggingConfiguration {
            lifecycle: Mutex::new(Weak::new()),
        });

        let mut lifecycle = configuration.lifecycle.lock().unwrap();
        if let Some(arc_lifecycle) = lifecycle.upgrade() {
            return Ok(arc_lifecycle);
        }
        let arc_lifecycle = Arc::new(LoggingLifecycle::new(&context.global_arguments)?);
        *lifecycle = Arc::downgrade(&arc_lifecycle);
        Ok(arc_lifecycle)
    }
}

impl Drop for LoggingLifecycle {
    fn drop(&mut self) {
        let _lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        unsafe {
            rcl_logging_fini();
        }
    }
}

#[cfg(test)]
pub(crate) mod log_handler {
    //! This module provides a way to customize how log output is handled. For
    //! now we are not making this a private API and are only using it for tests
    //! in rclrs. We can consider making it public in the future, but we should
    //! put more consideration into the API before doing so, and more crucially
    //! we need to figure out a way to process C-style formatting strings with
    //! a [`va_list`] from inside of Rust, which seems to be very messy.

    use std::{
        borrow::Cow,
        ffi::CStr,
        sync::{
            atomic::{AtomicBool, Ordering},
            OnceLock,
        },
    };

    use crate::{rcl_bindings::*, LogSeverity, ENTITY_LIFECYCLE_MUTEX};

    /// Global variable that allows a custom log handler to be set. This log
    /// handler will be applied throughout the entire application and cannot be
    /// replaced with a different custom log handler. If you want to be able to
    /// change the log handler over the lifetime of your application, you should
    /// design your own custom handler with an Arc<Mutex<T>> inside that allows
    /// its own behavior to be modified.
    static LOGGING_OUTPUT_HANDLER: OnceLock<RawLogHandler> = OnceLock::new();

    /// Alias for an arbitrary log handler that is compatible with raw rcl types
    pub(crate) type RawLogHandler = Box<
        dyn Fn(
                *const rcutils_log_location_t, // location
                std::os::raw::c_int,           // severity
                *const std::os::raw::c_char,   // logger name
                rcutils_time_point_value_t,    // timestamp
                *const std::os::raw::c_char,   // format
                *mut va_list,                  // formatting arguments
            )
            + 'static
            + Send
            + Sync,
    >;

    /// This is an idiomatic representation of all the information for a log entry
    #[derive(Clone)]
    pub(crate) struct LogEntry<'a> {
        pub(crate) location: LogLocation<'a>,
        pub(crate) severity: LogSeverity,
        pub(crate) logger_name: Cow<'a, str>,
        pub(crate) timestamp: i64,
        pub(crate) message: Cow<'a, str>,
    }

    impl<'a> LogEntry<'a> {
        /// Change the entry from something borrowed into something owned
        pub(crate) fn into_owned(self) -> LogEntry<'static> {
            LogEntry {
                location: self.location.into_owned(),
                severity: self.severity,
                logger_name: Cow::Owned(self.logger_name.into_owned()),
                timestamp: self.timestamp,
                message: Cow::Owned(self.message.into_owned()),
            }
        }
    }

    /// Rust-idiomatic representation of the location of a log
    #[derive(Debug, Clone)]
    pub(crate) struct LogLocation<'a> {
        pub function_name: Cow<'a, str>,
        pub file_name: Cow<'a, str>,
        pub line_number: usize,
    }

    impl<'a> LogLocation<'a> {
        pub(crate) fn into_owned(self) -> LogLocation<'static> {
            LogLocation {
                function_name: Cow::Owned(self.function_name.into_owned()),
                file_name: Cow::Owned(self.file_name.into_owned()),
                line_number: self.line_number,
            }
        }
    }

    #[derive(Debug)]
    pub(crate) struct OutputHandlerAlreadySet;

    static USING_CUSTOM_HANDLER: OnceLock<AtomicBool> = OnceLock::new();

    /// Set an idiomatic log hander
    pub(crate) fn set_logging_output_handler(
        handler: impl Fn(LogEntry) + 'static + Send + Sync,
    ) -> Result<(), OutputHandlerAlreadySet> {
        let raw_handler = Box::new(
            move |raw_location: *const rcutils_log_location_t,
                  raw_severity: std::os::raw::c_int,
                  raw_logger_name: *const std::os::raw::c_char,
                  raw_timestamp: rcutils_time_point_value_t,
                  raw_format: *const std::os::raw::c_char,
                  // NOTE: In the rclrs logging test we are choosing to format
                  // the full message in advance when using the custom handler,
                  // so the format field always contains the finished formatted
                  // message. Therefore we can just ignore the raw formatting
                  // arguments.
                  _raw_formatting_arguments: *mut va_list| {
                unsafe {
                    // NOTE: We use .unwrap() extensively inside this function because
                    // it only gets used during tests. We should reconsider this if
                    // we ever make this public.
                    let location = LogLocation {
                        function_name: Cow::Borrowed(
                            CStr::from_ptr((*raw_location).function_name)
                                .to_str()
                                .unwrap(),
                        ),
                        file_name: Cow::Borrowed(
                            CStr::from_ptr((*raw_location).file_name).to_str().unwrap(),
                        ),
                        line_number: (*raw_location).line_number,
                    };
                    let severity = LogSeverity::from_native(raw_severity);
                    let logger_name =
                        Cow::Borrowed(CStr::from_ptr(raw_logger_name).to_str().unwrap());
                    let timestamp: i64 = raw_timestamp;
                    let message = Cow::Borrowed(CStr::from_ptr(raw_format).to_str().unwrap());
                    handler(LogEntry {
                        location,
                        severity,
                        logger_name,
                        timestamp,
                        message,
                    });
                }
            },
        );

        set_raw_logging_output_handler(raw_handler)
    }

    /// Set the logging output handler directly
    pub(crate) fn set_raw_logging_output_handler(
        handler: RawLogHandler,
    ) -> Result<(), OutputHandlerAlreadySet> {
        LOGGING_OUTPUT_HANDLER
            .set(handler)
            .map_err(|_| OutputHandlerAlreadySet)?;
        let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        unsafe {
            // SAFETY:
            // - We have locked the global mutex
            rcutils_logging_set_output_handler(Some(rclrs_logging_output_handler));
        }

        USING_CUSTOM_HANDLER
            .get_or_init(|| AtomicBool::new(false))
            .store(true, Ordering::Release);
        Ok(())
    }

    pub(crate) fn is_using_custom_handler() -> bool {
        USING_CUSTOM_HANDLER
            .get_or_init(|| AtomicBool::new(false))
            .load(Ordering::Acquire)
    }

    /// This function exists so that we can give a raw function pointer to
    /// rcutils_logging_set_output_handler, which is needed by its API.
    extern "C" fn rclrs_logging_output_handler(
        location: *const rcutils_log_location_t,
        severity: std::os::raw::c_int,
        logger_name: *const std::os::raw::c_char,
        timestamp: rcutils_time_point_value_t,
        message: *const std::os::raw::c_char,
        logging_output: *mut va_list,
    ) {
        let handler = LOGGING_OUTPUT_HANDLER.get().unwrap();
        (*handler)(
            location,
            severity,
            logger_name,
            timestamp,
            message,
            logging_output,
        );
    }

    /// Reset the logging output handler to the default one
    pub(crate) fn reset_logging_output_handler() {
        let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        unsafe {
            // SAFETY: The global mutex is locked. No other precondition is
            // required.
            rcutils_logging_set_output_handler(Some(rcl_logging_multiple_output_handler));
        }
        USING_CUSTOM_HANDLER
            .get_or_init(|| AtomicBool::new(false))
            .store(false, Ordering::Release);
    }
}
