// #[cfg(test)]

pub(crate) mod graph_helpers;
pub(crate) use self::graph_helpers::*;

pub(crate) fn assert_send<T: Send>() {}
pub(crate) fn assert_sync<T: Sync>() {}
