use std::sync::{Arc, LazyLock, Mutex, Weak};

use crate::{
    rcl_bindings::{
        rcl_arguments_t, rcl_context_t, rcl_logging_configure, rcl_logging_fini,
        rcutils_get_default_allocator,
    },
    RclrsError, ToResult, ENTITY_LIFECYCLE_MUTEX,
};

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
        static CONFIGURATION: LazyLock<LoggingConfiguration> =
            LazyLock::new(|| LoggingConfiguration {
                lifecycle: Mutex::new(Weak::new()),
            });

        let mut lifecycle = CONFIGURATION.lifecycle.lock().unwrap();
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
