// Copyright (c) 2019 Sequence Planner
// SPDX-License-Identifier: Apache-2.0 AND MIT
// Adapted from https://github.com/sequenceplanner/r2r/blob/89cec03d07a1496a225751159cbc7bfb529d9dd1/r2r/src/utils.rs

use std::{
    ffi::CString,
    sync::{Mutex, Once},
};

use crate::error;
use crate::rcl_bindings::*;

static INIT: Once = Once::new();
static LOG_GUARD: Mutex<()> = Mutex::new(());

/// Don't call this directly, use the logging macros instead.
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
    let logger_name = CString::new(logger_name).unwrap();
    let message = CString::new(msg).unwrap();
    let severity = severity.to_native();

    INIT.call_once(|| {
        let ret = unsafe { rcutils_logging_initialize() };
        if let Err(code) = error::to_rclrs_result(ret) {
            panic!("Failed to initialize logging: {:?}", code);
        }
    });
    let _guard = LOG_GUARD.lock().unwrap();
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

/// A helper macro to log the message.
#[macro_export]
macro_rules! __impl_log {
    ($logger_name:expr, $msg:expr, $file:expr, $line:expr, $severity:expr) => {{
        $crate::log(
            &std::fmt::format($msg),
            $logger_name,
            $file,
            $line,
            $severity,
        );
    }};
}

/// Debug log message.
#[macro_export]
macro_rules! log_debug {
  ($logger_name:expr, $($args:tt)*) => {{
      $crate::__impl_log!($logger_name, format_args!($($args)*),
                          file!(), line!(), $crate::LogSeverity::Debug)
  }}
}

/// Info log message.
#[macro_export]
macro_rules! log_info {
  ($logger_name:expr, $($args:tt)*) => {{
      $crate::__impl_log!($logger_name, format_args!($($args)*),
                          file!(), line!(), $crate::LogSeverity::Info)
  }}
}

/// Warning log message.
#[macro_export]
macro_rules! log_warn {
  ($logger_name:expr, $($args:tt)*) => {{
      $crate::__impl_log!($logger_name, format_args!($($args)*),
                          file!(), line!(), $crate::LogSeverity::Warn)
  }}
}

/// Error log message.
#[macro_export]
macro_rules! log_error {
  ($logger_name:expr, $($args:tt)*) => {{
      $crate::__impl_log!($logger_name, format_args!($($args)*),
                          file!(), line!(), $crate::LogSeverity::Error)
  }}
}

/// Fatal log message.
#[macro_export]
macro_rules! log_fatal {
  ($logger_name:expr, $($args:tt)*) => {{
      $crate::__impl_log!($logger_name, format_args!($($args)*),
                          file!(), line!(), $crate::LogSeverity::Fatal)
  }}
}
