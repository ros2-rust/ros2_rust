// #[cfg(test)]

pub(crate) mod graph_helpers;
pub(crate) use self::graph_helpers::*;

pub(crate) fn assert_send<T: Send>() {}
pub(crate) fn assert_sync<T: Sync>() {}

use crate::{Executor, SpinOptions};
use std::time::{Duration, Instant};

/// Spins the executor until a condition returns true or timeout is reached.
/// Returns true if condition was met, false if timed out.
pub(crate) fn spin_until_condition<F>(
    executor: &mut Executor,
    mut condition: F,
    timeout: Duration,
) -> bool
where
    F: FnMut() -> bool,
{
    let start = Instant::now();
    while !condition() {
        if start.elapsed() >= timeout {
            return false;
        }
        executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(10)));
    }
    true
}
