use std::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

/// A wrapper providing additional drop-logic for the contained value.
///
/// When this wrapper is dropped, the contained value will be passed into the given function before
/// being destructed.
pub(crate) struct DropGuard<T> {
    value: ManuallyDrop<T>,
    drop_fn: fn(T),
}

impl<T> DropGuard<T> {
    /// Create a new `DropGuard` with the given value and drop function.
    pub fn new(value: T, drop_fn: fn(T)) -> Self {
        Self {
            value: ManuallyDrop::new(value),
            drop_fn,
        }
    }
}

impl<T> Deref for DropGuard<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.value
    }
}

impl<T> DerefMut for DropGuard<T> {
    fn deref_mut(&mut self) -> &mut T {
        &mut self.value
    }
}

impl<T> Drop for DropGuard<T> {
    fn drop(&mut self) {
        // SAFETY: ManuallyDrop::take() leaves `self.value` in an uninitialized state, meaning that
        // it must not be accessed further. This is guaranteed since `self` is being dropped and
        // cannot be accessed after this function completes. Moreover, the strict ownership of
        // `self.value` means that it cannot be accessed by `self.drop_fn`'s drop function either.
        let value = unsafe { ManuallyDrop::take(&mut self.value) };
        (self.drop_fn)(value);
    }
}
