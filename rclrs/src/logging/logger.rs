use std::{borrow::Borrow, ffi::CString, sync::Arc};

use crate::{
    rcl_bindings::{rcutils_logging_set_default_logger_level, rcutils_logging_set_logger_level},
    LogParams, LogSeverity, LoggerName, RclrsError, ToLogParams, ToResult, ENTITY_LIFECYCLE_MUTEX,
};

/// Logger can be passed in as the first argument into one of the [logging][1]
/// macros provided by rclrs. When passing it into one of the logging macros,
/// you can optionally apply any of the methods from [`ToLogParams`] to tweak
/// the logging behavior.
///
/// You can obtain a Logger in the following ways:
/// - Calling [`Node::logger`][2] to get the logger of a Node
/// - Using [`Logger::create_child`] to create a child logger for an existing logger
/// - Using [`Logger::new`] to create a new logger with any name that you'd like
/// - Using [`Logger::default()`] to access the global default logger
///
/// Note that if you are passing the Logger of a Node into one of the logging macros,
/// then you can choose to simply pass in `&node` instead of `node.logger()`.
///
/// [1]: crate::log
/// [2]: crate::Node::logger
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Logger {
    name: Arc<str>,
    c_name: Arc<CString>,
}

impl Logger {
    /// Create a new logger with the given name.
    pub fn new(name: impl Borrow<str>) -> Result<Logger, RclrsError> {
        let name: Arc<str> = name.borrow().into();
        let c_name = match CString::new((*name).to_owned()) {
            Ok(c_name) => c_name,
            Err(err) => {
                return Err(RclrsError::StringContainsNul {
                    s: (*name).to_owned(),
                    err,
                });
            }
        };

        Ok(Self {
            name,
            c_name: Arc::new(c_name),
        })
    }

    /// Create a new logger which will be a child of this logger.
    ///
    /// If the name of this logger is `parent_name`, then the name for the new
    /// child will be '"parent_name.child_name"`.
    ///
    /// If this is the default logger (whose name is an empty string), then the
    /// name for the new child will simply be the value in `child_name`.
    pub fn create_child(&self, child_name: impl Borrow<str>) -> Result<Logger, RclrsError> {
        if self.name.is_empty() {
            Self::new(child_name)
        } else {
            Self::new(format!("{}.{}", &self.name, child_name.borrow()))
        }
    }

    /// Get the name of this logger
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Set the severity level of this logger
    pub fn set_level(&self, severity: LogSeverity) -> Result<(), RclrsError> {
        // SAFETY: The precondition are:
        // - we are passing in a valid CString, which is already taken care of during construction of the Logger
        // - the severity level is a valid value, which is guaranteed by the rigid enum definition
        // - not thread-safe, so we lock the global mutex before calling this
        let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        unsafe {
            rcutils_logging_set_logger_level(self.c_name.as_ptr(), severity.as_native() as i32).ok()
        }
    }

    /// Set the severity level of the default logger which acts as the root ancestor
    /// of all other loggers.
    pub fn set_default_level(severity: LogSeverity) {
        // SAFETY: The preconditions are:
        // - the severity level is a valid value, which is guaranteed by the rigid enum definition
        // - not thread-safe, so we lock the global mutex before calling this
        let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        unsafe {
            rcutils_logging_set_default_logger_level(severity.as_native() as i32);
        }
    }
}

impl<'a> ToLogParams<'a> for &'a Logger {
    fn to_log_params(self) -> LogParams<'a> {
        LogParams::new(LoggerName::Validated(&self.c_name))
    }
}

impl Default for Logger {
    fn default() -> Self {
        Self::new("").unwrap()
    }
}

impl std::hash::Hash for Logger {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.name.hash(state);
    }
}
