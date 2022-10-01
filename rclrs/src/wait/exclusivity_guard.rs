use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crate::RclrsError;

/// A helper struct for tracking whether the waitable is currently in a wait set.
///
/// When this struct is constructed, which happens when adding an entity to the wait set,
/// it checks that the atomic boolean is false and sets it to true.
/// When it is dropped, which happens when it is removed from the wait set,
/// or the wait set itself is dropped, it sets the atomic bool to false.
pub(super) struct ExclusivityGuard<T> {
    in_use_by_wait_set: Arc<AtomicBool>,
    pub(super) waitable: T,
}

impl<T> Drop for ExclusivityGuard<T> {
    fn drop(&mut self) {
        self.in_use_by_wait_set.store(false, Ordering::Relaxed)
    }
}

impl<T> ExclusivityGuard<T> {
    pub fn new(waitable: T, in_use_by_wait_set: Arc<AtomicBool>) -> Result<Self, RclrsError> {
        if in_use_by_wait_set
            .compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed)
            .is_err()
        {
            return Err(RclrsError::AlreadyAddedToWaitSet);
        }
        Ok(Self {
            in_use_by_wait_set,
            waitable,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    use super::*;

    #[test]
    fn test_exclusivity_guard() {
        let atomic = Arc::new(AtomicBool::new(false));
        let eg = ExclusivityGuard::new((), Arc::clone(&atomic)).unwrap();
        assert!(ExclusivityGuard::new((), Arc::clone(&atomic)).is_err());
        drop(eg);
        assert!(!atomic.load(Ordering::Relaxed));
        assert!(ExclusivityGuard::new((), Arc::clone(&atomic)).is_ok());
    }
}
