use std::{
    fmt::{self, Debug},
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use rosidl_runtime_rs::{
    PrimitiveSequence, PrimitiveSequenceAlloc, Sequence, SequenceAlloc, SequenceExceedsBoundsError,
};

use super::check;

// Primitive sequences use the ROS primitive layout. Message and string
// sequences use Sequence<T>; proxy elements retain runtime type metadata.
// Bounded wrappers check the upper bound before replacing sequence storage.

/// An immutable proxy object.
///
/// This trait is unsafe because memory errors will occur if size_in_memory() is
/// implemented incorrectly.
#[doc(hidden)]
pub unsafe trait Proxy<'msg> {
    // In the case of strings, this is the string length upper bound,
    // and in the case of messages, it is its structure.
    type Metadata: 'msg + Copy;
    // How many bytes does each element take up in the underlying sequence?
    fn size_in_memory(metadata: Self::Metadata) -> usize;
    // This function is unsafe because the bytes must correspond to the proxied object.
    unsafe fn new(bytes: &'msg [u8], metadata: Self::Metadata) -> Self;
}

/// An mutable proxy object.
///
/// This trait is unsafe because memory errors will occur if size_in_memory() is
/// implemented incorrectly.
#[doc(hidden)]
pub unsafe trait ProxyMut<'msg> {
    // In the case of strings, this is the string length upper bound,
    // and in the case of messages, it is its structure.
    type Metadata: 'msg + Copy;
    // How many bytes does each element take up in the underlying sequence?
    fn size_in_memory(metadata: Self::Metadata) -> usize;
    // This function is unsafe because the bytes must correspond to the proxied object.
    unsafe fn new(bytes: &'msg mut [u8], metadata: Self::Metadata) -> Self;
}

// ========================= Abstracting over proxy vs direct sequences =========================

// This trait abstracts over &mut Sequence<T> vs ProxySequence<T>.
#[doc(hidden)]
pub trait InnerSequence<T: PartialEq>: PartialEq {
    fn as_slice(&self) -> &[T];
    fn as_mut_slice(&mut self) -> &mut [T];
    // "Unchecked" means that it doesn't know about the upper bound of the sequence.
    fn resize_unchecked(&mut self, resize_function: ResizeFunction, len: usize);
}

#[doc(hidden)]
pub struct ProxySequence<'msg, T: ProxyMut<'msg>> {
    // The underlying storage
    sequence: &'msg mut TypeErasedSequence,
    // The user-facing objects
    proxies: Vec<T>,
    // To recreate the proxies
    metadata: T::Metadata,
}

impl<'msg, T> InnerSequence<T> for ProxySequence<'msg, T>
where
    T: PartialEq + ProxyMut<'msg>,
{
    fn as_slice(&self) -> &[T] {
        self.proxies.as_slice()
    }

    fn as_mut_slice(&mut self) -> &mut [T] {
        self.proxies.as_mut_slice()
    }

    /// This will fini all messages in the sequence and re-initialize it from scratch.
    fn resize_unchecked(&mut self, resize_function: ResizeFunction, len: usize) {
        let is_ok =
            unsafe { resize_function(self.sequence as *mut _ as *mut std::os::raw::c_void, len) };
        assert!(is_ok);

        // Recalculate the message proxies
        self.proxies = unsafe { self.sequence.proxy_elems_mut(self.metadata) };
    }
}

impl<'msg, T> InnerSequence<T> for &'msg mut Sequence<T>
where
    T: PartialEq + SequenceAlloc,
{
    fn as_slice(&self) -> &[T] {
        // self.as_slice() would call this trait method itself
        Sequence::as_slice(self)
    }

    fn as_mut_slice(&mut self) -> &mut [T] {
        // self.as_mut_slice() would call this trait method itself
        Sequence::as_mut_slice(self)
    }

    /// This will fini all messages in the sequence and re-initialize it from scratch.
    fn resize_unchecked(&mut self, resize_function: ResizeFunction, len: usize) {
        let is_ok = unsafe { resize_function(*self as *mut _ as *mut std::os::raw::c_void, len) };
        assert!(is_ok);
    }
}

impl<'msg, T> InnerSequence<T> for &'msg mut PrimitiveSequence<T>
where
    T: PartialEq + PrimitiveSequenceAlloc,
{
    fn as_slice(&self) -> &[T] {
        PrimitiveSequence::as_slice(self)
    }

    fn as_mut_slice(&mut self) -> &mut [T] {
        PrimitiveSequence::as_mut_slice(self)
    }

    fn resize_unchecked(&mut self, _resize_function: ResizeFunction, len: usize) {
        **self = PrimitiveSequence::new(len);
    }
}

impl<'msg, T> PartialEq for ProxySequence<'msg, T>
where
    T: PartialEq + ProxyMut<'msg>,
{
    fn eq(&self, other: &Self) -> bool {
        self.proxies.eq(&other.proxies)
    }
}

// This links the element type T to the inner sequence type: &mut Sequence<T> or ProxySequence<T>.
#[doc(hidden)]
pub trait DynamicSequenceElementMut<'msg>: Debug + PartialEq + Sized {
    type InnerSequence: InnerSequence<Self>;
}

// If the element type is an rosidl_runtime_rs type, the sequence type is &mut Sequence<T>
impl<'msg, T> DynamicSequenceElementMut<'msg> for T
where
    T: Debug + PartialEq + SequenceAlloc + 'static,
{
    type InnerSequence = &'msg mut Sequence<T>;
}

// ========================= The TypeErasedSequence helper =========================

/// A Sequence whose type is not statically known.
///
/// This is an internal helper struct whose layout, like rosidl_runtime_rs::Sequence,
/// matches that of the type generated by rosidl_generator_c.
#[repr(C)]
pub(crate) struct TypeErasedSequence {
    pub(super) data: *mut std::os::raw::c_void,
    pub(super) size: usize,
    pub(super) capacity: usize,
}

impl TypeErasedSequence {
    pub(super) unsafe fn proxy_elems<'msg, T>(&self, metadata: T::Metadata) -> Vec<T>
    where
        T: Proxy<'msg>,
    {
        let element_size = T::size_in_memory(metadata);
        if self.data.is_null() {
            return Vec::new();
        };
        let sequence_data =
            std::slice::from_raw_parts(self.data as *const u8, self.size * element_size);
        check::<T>(sequence_data);
        sequence_data
            .chunks(element_size)
            .map(|bytes| T::new(bytes, metadata))
            .collect()
    }

    pub(super) unsafe fn proxy_elems_mut<'msg, T>(&self, metadata: T::Metadata) -> Vec<T>
    where
        T: ProxyMut<'msg>,
    {
        let element_size = T::size_in_memory(metadata);
        if self.data.is_null() {
            return Vec::new();
        };
        let sequence_data =
            std::slice::from_raw_parts_mut(self.data as *mut u8, self.size * element_size);
        check::<T>(sequence_data);
        sequence_data
            .chunks_mut(element_size)
            .map(|bytes| T::new(bytes, metadata))
            .collect()
    }
}

// ==========================================================================
// ======================== Immutable sequence types ========================
// ==========================================================================

/// An unbounded sequence.
///
/// This type dereferences to `&[T]`.
#[derive(PartialEq, Eq)]
pub struct DynamicSequence<'msg, T>
where
    T: Proxy<'msg>,
{
    elements: Box<[T]>,
    // Not sure if this is strictly needed, but it's nice to be consistent
    phantom: PhantomData<&'msg u8>,
}

// BorrowedOrOwnedSlice – a specialized version of Cow.
// Cow cannot be used because it requires T to be Clone.
#[derive(PartialEq, Eq)]
enum BooSlice<'msg, T> {
    Borrowed(&'msg [T]),
    Owned(Box<[T]>),
}

/// A bounded sequence whose upper bound is only known at runtime.
#[derive(PartialEq, Eq)]
pub struct DynamicBoundedSequence<'msg, T> {
    boo: BooSlice<'msg, T>,
    upper_bound: usize,
}

// ------------------------- impl for DynamicSequence -------------------------

impl<'msg, T> Debug for DynamicSequence<'msg, T>
where
    T: Debug + Proxy<'msg>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.elements.iter().fmt(f)
    }
}

impl<'msg, T> Deref for DynamicSequence<'msg, T>
where
    T: Proxy<'msg>,
{
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        &*self.elements
    }
}

impl<'msg, T> DynamicSequence<'msg, T>
where
    T: Proxy<'msg>,
{
    pub(super) unsafe fn new_proxy(bytes: &'msg [u8], metadata: T::Metadata) -> Self {
        let sequence = &*(bytes.as_ptr() as *const TypeErasedSequence);
        let elements = sequence.proxy_elems(metadata).into_boxed_slice();
        Self {
            elements,
            phantom: PhantomData,
        }
    }

    /// See [`Sequence::as_slice()`][1].
    ///
    /// [1]: rosidl_runtime_rs::Sequence::as_slice
    pub fn as_slice(&self) -> &[T] {
        &*self.elements
    }
}

// ------------------------- impl for DynamicBoundedSequence -------------------------

impl<'msg, T> BooSlice<'msg, T> {
    fn as_slice(&self) -> &[T] {
        match self {
            BooSlice::Borrowed(slice) => slice,
            BooSlice::Owned(boxed_slice) => &**boxed_slice,
        }
    }
}

impl<'msg, T> Debug for DynamicBoundedSequence<'msg, T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.boo.as_slice().fmt(f)
    }
}

impl<'msg, T> Deref for DynamicBoundedSequence<'msg, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        self.boo.as_slice()
    }
}

impl<'msg, T> DynamicBoundedSequence<'msg, T>
where
    T: PrimitiveSequenceAlloc,
{
    pub(super) unsafe fn new_primitive(bytes: &'msg [u8], upper_bound: usize) -> Self {
        let sequence = &*(bytes.as_ptr() as *const PrimitiveSequence<T>);
        let slice = sequence.as_slice();
        Self {
            boo: BooSlice::Borrowed(slice),
            upper_bound,
        }
    }
}

impl<'msg, T> DynamicBoundedSequence<'msg, T>
where
    T: SequenceAlloc,
{
    pub(super) unsafe fn new_native(bytes: &'msg [u8], upper_bound: usize) -> Self {
        let sequence = &*(bytes.as_ptr() as *const Sequence<T>);
        let slice = sequence.as_slice();
        Self {
            boo: BooSlice::Borrowed(slice),
            upper_bound,
        }
    }
}

impl<'msg, T> DynamicBoundedSequence<'msg, T>
where
    T: Proxy<'msg>,
{
    pub(super) unsafe fn new_proxy(
        bytes: &'msg [u8],
        upper_bound: usize,
        metadata: T::Metadata,
    ) -> Self {
        let sequence = &*(bytes.as_ptr() as *const TypeErasedSequence);
        Self {
            boo: BooSlice::Owned(sequence.proxy_elems(metadata).into_boxed_slice()),
            upper_bound,
        }
    }
}

impl<'msg, T> DynamicBoundedSequence<'msg, T> {
    /// See [`Sequence::as_slice()`][1].
    ///
    /// [1]: rosidl_runtime_rs::Sequence::as_slice
    pub fn as_slice(&self) -> &[T] {
        self.boo.as_slice()
    }

    /// Returns the maximum length of this sequence.
    pub fn upper_bound(&self) -> usize {
        self.upper_bound
    }
}

// ==========================================================================
// ========================= Mutable sequence types =========================
// ==========================================================================

// The resize function from the type support library does not preserve the elements.
// It just calls fini + init.
pub(super) type ResizeFunction =
    unsafe extern "C" fn(arg1: *mut std::os::raw::c_void, size: usize) -> bool;

/// An unbounded sequence.
///
/// This type dereferences to `&[T]` and `&mut [T]`.
pub struct DynamicSequenceMut<'msg, T: DynamicSequenceElementMut<'msg>> {
    // This is either &mut Sequence<T> or ProxySequence<T>
    sequence: T::InnerSequence,
    resize_function: ResizeFunction,
}

impl<'msg, T: DynamicSequenceElementMut<'msg>> PartialEq for DynamicSequenceMut<'msg, T> {
    fn eq(&self, other: &Self) -> bool {
        self.sequence == other.sequence
    }
}

/// A bounded sequence whose upper bound is only known at runtime.
///
/// This is conceptually the same as a [`BoundedSequence<T>`][1].
///
/// This type dereferences to `&[T]` and `&mut [T]`.
///
/// [1]: rosidl_runtime_rs::BoundedSequence
#[derive(PartialEq)]
pub struct DynamicBoundedSequenceMut<'msg, T: DynamicSequenceElementMut<'msg>> {
    inner: DynamicSequenceMut<'msg, T>,
    upper_bound: usize,
}

/// A mutable bounded primitive sequence whose bound is known at runtime.
#[derive(PartialEq)]
pub struct DynamicBoundedPrimitiveSequenceMut<'msg, T: PrimitiveSequenceAlloc> {
    sequence: &'msg mut PrimitiveSequence<T>,
    upper_bound: usize,
}

impl<T> Debug for DynamicBoundedPrimitiveSequenceMut<'_, T>
where
    T: Debug + PrimitiveSequenceAlloc,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.sequence.fmt(f)
    }
}

impl<T: PrimitiveSequenceAlloc> Deref for DynamicBoundedPrimitiveSequenceMut<'_, T> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.sequence.as_slice()
    }
}

impl<T: PrimitiveSequenceAlloc> DerefMut for DynamicBoundedPrimitiveSequenceMut<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.sequence.as_mut_slice()
    }
}

impl<'msg, T: PrimitiveSequenceAlloc> DynamicBoundedPrimitiveSequenceMut<'msg, T> {
    pub(super) unsafe fn new_primitive(bytes: &'msg mut [u8], upper_bound: usize) -> Self {
        Self {
            sequence: &mut *(bytes.as_mut_ptr() as *mut PrimitiveSequence<T>),
            upper_bound,
        }
    }

    /// Returns the maximum length of this sequence.
    pub fn upper_bound(&self) -> usize {
        self.upper_bound
    }

    /// Returns the sequence elements as a slice.
    pub fn as_slice(&self) -> &[T] {
        self.sequence.as_slice()
    }

    /// Returns the sequence elements as a mutable slice.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.sequence.as_mut_slice()
    }

    /// Tries to reset this sequence to `len` zero-initialized elements.
    pub fn try_reset(&mut self, len: usize) -> Result<(), SequenceExceedsBoundsError> {
        if len > self.upper_bound {
            return Err(SequenceExceedsBoundsError {
                len,
                upper_bound: self.upper_bound,
            });
        }
        *self.sequence = PrimitiveSequence::new(len);
        Ok(())
    }

    /// Resets this sequence to empty.
    pub fn clear(&mut self) {
        self.try_reset(0).unwrap();
    }
}

// ------------------------- impl for DynamicSequenceMut -------------------------

impl<'msg, T> Debug for DynamicSequenceMut<'msg, T>
where
    T: DynamicSequenceElementMut<'msg>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.sequence.as_slice().fmt(f)
    }
}

impl<'msg, T> Deref for DynamicSequenceMut<'msg, T>
where
    T: DynamicSequenceElementMut<'msg>,
{
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        self.sequence.as_slice()
    }
}

impl<'msg, T> DerefMut for DynamicSequenceMut<'msg, T>
where
    T: DynamicSequenceElementMut<'msg>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.sequence.as_mut_slice()
    }
}

impl<'msg, T> DynamicSequenceMut<'msg, T>
where
    T: SequenceAlloc
        + DynamicSequenceElementMut<'msg, InnerSequence = &'msg mut Sequence<T>>
        + 'static,
{
    pub(super) unsafe fn new_native(
        bytes: &'msg mut [u8],
        resize_function: ResizeFunction,
    ) -> Self {
        let sequence = &mut *(bytes.as_mut_ptr() as *mut Sequence<T>);
        Self {
            sequence,
            resize_function,
        }
    }
}

impl<'msg, T> DynamicSequenceMut<'msg, T>
where
    T: ProxyMut<'msg> + DynamicSequenceElementMut<'msg, InnerSequence = ProxySequence<'msg, T>>,
{
    pub(super) unsafe fn new_proxy(
        bytes: &'msg mut [u8],
        metadata: T::Metadata,
        resize_function: ResizeFunction,
    ) -> Self {
        // SAFETY: TypeErasedSequence has the same layout as any
        // rosidl-generated C sequence type, and the lifetime is correct too.
        let sequence = &mut *(bytes.as_mut_ptr() as *mut TypeErasedSequence);
        let proxies = sequence.proxy_elems_mut(metadata);
        let sequence = ProxySequence {
            sequence,
            proxies,
            metadata,
        };
        Self {
            sequence,
            resize_function,
        }
    }
}

impl<'msg, T> DynamicSequenceMut<'msg, T>
where
    T: DynamicSequenceElementMut<'msg>,
{
    /// See [`Sequence::as_slice()`][1].
    ///
    /// [1]: rosidl_runtime_rs::Sequence::as_slice
    pub fn as_slice(&self) -> &[T] {
        self.sequence.as_slice()
    }

    /// See [`Sequence::as_mut_slice()`][1].
    ///
    /// [1]: rosidl_runtime_rs::Sequence::as_mut_slice
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.sequence.as_mut_slice()
    }

    /// Resets this sequence to an empty sequence.
    pub fn clear(&mut self) {
        self.reset(0);
    }

    /// Resets this sequence to a new sequence of `len` elements with default values.
    pub fn reset(&mut self, len: usize) {
        self.sequence.resize_unchecked(self.resize_function, len)
    }
}

// ------------------------- impl for DynamicBoundedSequenceMut -------------------------

impl<'msg, T> Debug for DynamicBoundedSequenceMut<'msg, T>
where
    T: DynamicSequenceElementMut<'msg>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.inner.fmt(f)
    }
}

impl<'msg, T: DynamicSequenceElementMut<'msg>> Deref for DynamicBoundedSequenceMut<'msg, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        self.inner.deref()
    }
}

impl<'msg, T: DynamicSequenceElementMut<'msg>> DerefMut for DynamicBoundedSequenceMut<'msg, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.deref_mut()
    }
}

impl<'msg, T> DynamicBoundedSequenceMut<'msg, T>
where
    T: SequenceAlloc
        + DynamicSequenceElementMut<'msg, InnerSequence = &'msg mut Sequence<T>>
        + 'static,
{
    pub(super) unsafe fn new_native(
        bytes: &'msg mut [u8],
        upper_bound: usize,
        resize_function: ResizeFunction,
    ) -> Self {
        let inner = DynamicSequenceMut::new_native(bytes, resize_function);
        Self { inner, upper_bound }
    }
}

impl<'msg, T> DynamicBoundedSequenceMut<'msg, T>
where
    T: ProxyMut<'msg> + DynamicSequenceElementMut<'msg, InnerSequence = ProxySequence<'msg, T>>,
{
    pub(super) unsafe fn new_proxy(
        bytes: &'msg mut [u8],
        metadata: T::Metadata,
        upper_bound: usize,
        resize_function: ResizeFunction,
    ) -> Self {
        let inner = DynamicSequenceMut::new_proxy(bytes, metadata, resize_function);
        Self { inner, upper_bound }
    }
}

impl<'msg, T: DynamicSequenceElementMut<'msg>> DynamicBoundedSequenceMut<'msg, T> {
    /// See [`Sequence::as_slice()`][1].
    ///
    /// [1]: rosidl_runtime_rs::Sequence::as_slice
    pub fn as_slice(&self) -> &[T] {
        self.inner.as_slice()
    }

    /// See [`Sequence::as_mut_slice()`][1].
    ///
    /// [1]: rosidl_runtime_rs::Sequence::as_mut_slice
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.inner.as_mut_slice()
    }

    /// Returns the maximum length of this sequence.
    pub fn upper_bound(&self) -> usize {
        self.upper_bound
    }

    /// Resets this sequence to an empty sequence.
    pub fn clear(&mut self) {
        self.inner.clear();
    }

    /// Tries to reset this sequence to a new sequence of `len` elements with default values.
    ///
    /// This is only successful if `len` is less than or equal to the [upper bound][1], otherwise
    /// the sequence is unmodified.
    ///
    /// [1]: Self::upper_bound
    pub fn try_reset(&mut self, len: usize) -> Result<(), SequenceExceedsBoundsError> {
        if len > self.upper_bound {
            Err(SequenceExceedsBoundsError {
                len,
                upper_bound: self.upper_bound,
            })
        } else {
            self.inner.reset(len);
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    unsafe extern "C" fn reject_resize(_: *mut std::ffi::c_void, _: usize) -> bool {
        false
    }
    unsafe extern "C" fn accept_resize(_: *mut std::ffi::c_void, _: usize) -> bool {
        true
    }

    #[test]
    fn primitive_reset_initializes_values_and_checks_bounds() {
        let mut sequence = PrimitiveSequence::from(&[true, true][..]);
        (&mut sequence).resize_unchecked(reject_resize, 3);
        assert_eq!(sequence.as_slice(), &[false; 3]);
        let mut bounded = DynamicBoundedPrimitiveSequenceMut {
            sequence: &mut sequence,
            upper_bound: 4,
        };
        assert!(bounded.try_reset(5).is_err());
        assert_eq!(bounded.as_slice(), &[false; 3]);
        bounded.as_mut_slice()[0] = true;
        bounded.try_reset(4).unwrap();
        assert_eq!(bounded.as_slice(), &[false; 4]);
        assert_eq!(bounded.upper_bound(), 4);
        bounded.clear();
        assert!(bounded.is_empty());
    }

    #[test]
    fn dynamic_sequence_equality_compares_values() {
        let mut first = Sequence::from(vec![rosidl_runtime_rs::String::from("value")]);
        let mut second = first.clone();
        let first = DynamicSequenceMut::<rosidl_runtime_rs::String> {
            sequence: &mut first,
            resize_function: reject_resize,
        };
        let second = DynamicSequenceMut::<rosidl_runtime_rs::String> {
            sequence: &mut second,
            resize_function: accept_resize,
        };
        assert_eq!(first, second);
    }
}
