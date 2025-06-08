use std::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut, Drop, Fn},
};

/// A wrapper providing additional drop-logic for the contained value.
///
/// When this wrapper is dropped, the contained value will be passed into the given function before
/// being destructed.
pub(crate) struct DropGuard<T, F: Fn(T)> {
    value: ManuallyDrop<T>,
    drop_fn: F,
}

impl<T, F: Fn(T)> DropGuard<T, F> {
    /// Create a new `DropGuard` with the given value and drop function.
    pub fn new(value: T, drop_fn: F) -> Self {
        Self {
            value: ManuallyDrop::new(value),
            drop_fn,
        }
    }
}

impl<T, F: Fn(T)> Deref for DropGuard<T, F> {
    type Target = T;

    fn deref(&self) -> &T {
        &*self.value
    }
}

impl<T, F: Fn(T)> DerefMut for DropGuard<T, F> {
    fn deref_mut(&mut self) -> &mut T {
        &mut *self.value
    }
}

impl<T, F: Fn(T)> Drop for DropGuard<T, F> {
    fn drop(&mut self) {
        // SAFETY: ManuallyDrop::take() leaves `self.value` in an uninitialized state, meaning that
        // it must not be accessed further. This is guaranteed since `self` is being dropped and
        // cannot be accessed after this function completes. Moreover, the strict ownership of
        // `self.value` means that it cannot be accessed by `self.drop_fn`'s drop function either.
        let value = unsafe { ManuallyDrop::take(&mut self.value) };
        (self.drop_fn)(value);
    }
}
