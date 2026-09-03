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

    use std::{borrow::Cow, cell::RefCell, ffi::CStr, sync::Arc};

    use crate::{rcl_bindings::*, LogSeverity};

    thread_local! {
        /// Thread-local variable that allows tests to observe log entries.
        static LOG_OBSERVER: RefCell<Option<LogObserver>> = RefCell::new(None);
    }

    /// Alias for a test observer of log entries emitted through rclrs logging macros.
    pub(crate) type LogObserver = Arc<dyn Fn(LogEntry) + 'static + Send + Sync>;

    /// This is an idiomatic representation of all the information for a log entry
    #[derive(Clone)]
    pub(crate) struct LogEntry<'a> {
        pub(crate) location: LogLocation<'a>,
        pub(crate) severity: LogSeverity,
        pub(crate) logger_name: Cow<'a, str>,
        pub(crate) timestamp: i64,
        pub(crate) message: Cow<'a, str>,
    }

    impl LogEntry<'_> {
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

    impl LogLocation<'_> {
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

    /// Set an idiomatic log hander
    pub(crate) fn set_logging_output_handler(
        handler: impl Fn(LogEntry) + 'static + Send + Sync,
    ) -> Result<(), OutputHandlerAlreadySet> {
        let observer: LogObserver = Arc::new(handler);
        LOG_OBSERVER.with(|handler_slot| {
            let mut handler_slot = handler_slot.borrow_mut();
            if handler_slot.is_some() {
                return Err(OutputHandlerAlreadySet);
            }
            *handler_slot = Some(observer);
            Ok(())
        })
    }

    pub(crate) fn is_using_custom_handler() -> bool {
        LOG_OBSERVER.with(|handler_slot| handler_slot.borrow().is_some())
    }

    /// Dispatch a log entry to the active test handler, if one is set.
    ///
    /// SAFETY: The raw pointers must be valid for the duration of the call.
    pub(crate) unsafe fn dispatch_logging_output_handler(
        location: *const rcutils_log_location_t,
        severity: std::os::raw::c_int,
        logger_name: *const std::os::raw::c_char,
        timestamp: rcutils_time_point_value_t,
        message: *const std::os::raw::c_char,
    ) {
        let handler = LOG_OBSERVER.with(|handler_slot| handler_slot.borrow().as_ref().cloned());
        let Some(handler) = handler else {
            return;
        };

        let location = if location.is_null() {
            LogLocation {
                function_name: Cow::Borrowed(""),
                file_name: Cow::Borrowed(""),
                line_number: 0,
            }
        } else {
            LogLocation {
                function_name: Cow::Borrowed(
                    CStr::from_ptr((*location).function_name).to_str().unwrap(),
                ),
                file_name: Cow::Borrowed(CStr::from_ptr((*location).file_name).to_str().unwrap()),
                line_number: (*location).line_number,
            }
        };
        let severity = LogSeverity::from_native(severity);
        let logger_name = if logger_name.is_null() {
            Cow::Borrowed("")
        } else {
            Cow::Borrowed(CStr::from_ptr(logger_name).to_str().unwrap())
        };
        let message = if message.is_null() {
            Cow::Borrowed("")
        } else {
            Cow::Borrowed(CStr::from_ptr(message).to_str().unwrap())
        };

        handler(LogEntry {
            location,
            severity,
            logger_name,
            timestamp,
            message,
        });
    }

    /// Reset the logging output handler to the default one
    pub(crate) fn reset_logging_output_handler() {
        LOG_OBSERVER.with(|handler_slot| *handler_slot.borrow_mut() = None);
    }

    pub(crate) fn null_timestamp() -> rcutils_time_point_value_t {
        // This value is only used by the Rust-side test hook. The real rcutils
        // output handler still receives the timestamp produced by rcutils_log.
        0
    }
}
