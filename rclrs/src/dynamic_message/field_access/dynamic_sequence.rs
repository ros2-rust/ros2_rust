use std::fmt::{self, Debug};
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};

use rosidl_runtime_rs::{Sequence, SequenceAlloc, SequenceExceedsBoundsError};

use super::check;

// We cannot always use &Sequence<T> and &mut Sequence<T> as types for accessing sequence fields.
// This is for two reasons:
// 1.  The type T might be a sub-message, or a bounded string/wstring. These types have some
//     associated metadata, namely the message structure and the string length bound, which needs
//     to be available when accessing/modifying the type T. Therefore, fields of this type are
//     accessed via their "proxy" wrappers, such as `DynamicMessageView`, which include the
//     metadata. The memory layout of the proxy type doesn't match that of T, so we cannot cast
//     Sequence<T> to Sequence<Proxy>.
// 2a. The sequence might be bounded. The API of Sequence<T> allows unchecked length changes.
// 2b. For a bounded sequence, it would be nice if the sequence type had a getter for the upper
//     bound, but Sequence<T> doesn't know the upper bound.
//
// So, following these criteria, these are the possible sequence types for each combination of
// bounded/unbounded, mutable/immutable, and native/proxy element type:
//
// seq kind  | mutability | element type | possible sequence types
// ----------+------------+--------------+--------------
// unbounded | immutable  | native       | &Sequence<T> or &[T]*
// unbounded | immutable  | proxy        | Box<T>*
// unbounded | mutable    | native       | &mut Sequence<T>*
// unbounded | mutable    | proxy        | custom type that can store proxy objects
// bounded   | immutable  | native       | (usize, Box<T>) or (usize, &Sequence<T>)*
// bounded   | immutable  | proxy        | (usize, Box<T>)*
// bounded   | mutable    | native       | custom type that enforces upper bound
// bounded   | mutable    | proxy        | custom type that enforces upper bound and can store proxy objects
//
// * or an equivalent custom type
//
// This module chooses to expose the following types for this purpose:
//
// seq kind  | mutability | element type | sequence type
// ----------+------------+--------------+--------------
// unbounded | immutable  | native       | &Sequence<T>
// unbounded | immutable  | proxy        | DynamicSequence<T> (newtype of Box<T>)
// unbounded | mutable    | native       | &mut Sequence<T>
// unbounded | mutable    | proxy        | DynamicSequenceMut<T>
// bounded   | immutable  | native       | DynamicBoundedSequence<T> (similar to Cow<[T]>)
// bounded   | immutable  | proxy        | DynamicBoundedSequence<T> (similar to Cow<[T]>)
// bounded   | mutable    | native       | DynamicBoundedSequenceMut<T> (based on DynamicSequenceMut<T>)
// bounded   | mutable    | proxy        | DynamicBoundedSequenceMut<T> (based on DynamicSequenceMut<T>)
//
// That means that DynamicBoundedSequence and DynamicBoundedSequenceMut must be able to hold
// both native and proxy objects.

// ========================= Abstracting over different proxy types =========================

// These traits abstract over types that are "proxy objects" for the actual data stored
// in a message. See also the explanation above for context.
//
// There are three types implementing these traits (dynamic versions of String, WString and messages).
// Without these traits, you'd have to e.g. write three versions of the ProxySequence struct, and
// of its implementation of the InnerSequence trait, and much more.

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
        let is_ok = unsafe { resize_function(self as *mut _ as *mut std::os::raw::c_void, len) };
        assert!(is_ok);
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
    T: SequenceAlloc,
{
    pub(super) unsafe fn new_primitive(bytes: &'msg [u8], upper_bound: usize) -> Self {
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

impl<'msg, T: SequenceAlloc> DynamicBoundedSequence<'msg, T> {
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
#[derive(PartialEq)]
pub struct DynamicSequenceMut<'msg, T: DynamicSequenceElementMut<'msg>> {
    // This is either &mut Sequence<T> or ProxySequence<T>
    sequence: T::InnerSequence,
    resize_function: ResizeFunction,
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
    pub(super) unsafe fn new_primitive(
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
    pub(super) unsafe fn new_primitive(
        bytes: &'msg mut [u8],
        upper_bound: usize,
        resize_function: ResizeFunction,
    ) -> Self {
        let inner = DynamicSequenceMut::new_primitive(bytes, resize_function);
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
