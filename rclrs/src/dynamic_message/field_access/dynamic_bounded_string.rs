use std::convert::AsMut;
use std::fmt::{self, Display};
use std::num::NonZeroUsize;
use std::ops::{Deref, DerefMut};

use rosidl_runtime_rs::StringExceedsBoundsError;

use super::{DynamicSequenceElementMut, Proxy, ProxyMut, ProxySequence};

/// A bounded String whose upper bound is only known at runtime.
///
/// It derefs to a [`rosidl_runtime_rs::String`], which allows conversion
/// into a regular string, and more.
#[derive(Debug, Hash, PartialOrd, PartialEq, Eq, Ord)]
pub struct DynamicBoundedString<'msg> {
    pub(super) inner: &'msg rosidl_runtime_rs::String,
    pub(super) upper_bound: NonZeroUsize,
}

/// A bounded WString whose upper bound is only known at runtime.
///
/// It derefs to a [`rosidl_runtime_rs::WString`], which allows conversion
/// into a regular string, and more.
#[derive(Debug, Hash, PartialOrd, PartialEq, Eq, Ord)]
pub struct DynamicBoundedWString<'msg> {
    pub(super) inner: &'msg rosidl_runtime_rs::WString,
    pub(super) upper_bound: NonZeroUsize,
}

/// A bounded String whose upper bound is only known at runtime.
///
/// Like its immutable counterpart [`DynamicBoundedString`], it derefs to a
/// [`rosidl_runtime_rs::String`], but not mutably. This is to make sure that
/// methods which change the string's length can not be accessed, so that the
/// length never exceeds the upper bound.
/// Instead, this type provides its own mutation methods, which check the length,
/// an an [`AsMut`][1] instance.
///
/// [1]: std::convert::AsMut
#[derive(Debug, Hash, PartialOrd, PartialEq, Eq, Ord)]
pub struct DynamicBoundedStringMut<'msg> {
    pub(super) inner: &'msg mut rosidl_runtime_rs::String,
    pub(super) upper_bound: NonZeroUsize,
}

/// A bounded WString whose upper bound is only known at runtime.
///
/// Like its immutable counterpart [`DynamicBoundedWString`], it derefs to a
/// [`rosidl_runtime_rs::WString`], but not mutably. This is to make sure that
/// methods which change the string's length can not be accessed, so that the
/// length never exceeds the upper bound.
/// Instead, this type provides its own mutation methods, which check the length,
/// an an [`AsMut`][1] instance.
///
/// [1]: std::convert::AsMut
#[derive(Debug, Hash, PartialOrd, PartialEq, Eq, Ord)]
pub struct DynamicBoundedWStringMut<'msg> {
    pub(super) inner: &'msg mut rosidl_runtime_rs::WString,
    pub(super) upper_bound: NonZeroUsize,
}

// ========================= impl for DynamicBounded(W)String =========================

impl<'msg> Deref for DynamicBoundedString<'msg> {
    type Target = rosidl_runtime_rs::String;
    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

impl<'msg> Display for DynamicBoundedString<'msg> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.inner.fmt(f)
    }
}

unsafe impl<'msg> Proxy<'msg> for DynamicBoundedString<'msg> {
    type Metadata = NonZeroUsize; // String upper bound

    fn size_in_memory(_: NonZeroUsize) -> usize {
        std::mem::size_of::<rosidl_runtime_rs::String>()
    }

    unsafe fn new(bytes: &'msg [u8], string_upper_bound: NonZeroUsize) -> Self {
        let inner = &*(bytes.as_ptr() as *const rosidl_runtime_rs::String);
        Self {
            inner,
            upper_bound: string_upper_bound,
        }
    }
}

impl<'msg> DynamicBoundedString<'msg> {
    /// Returns the maximum length of this string.
    pub fn upper_bound(&self) -> NonZeroUsize {
        self.upper_bound
    }
}

impl<'msg> DynamicBoundedWString<'msg> {
    /// Returns the maximum length of this string.
    pub fn upper_bound(&self) -> NonZeroUsize {
        self.upper_bound
    }
}

impl<'msg> Deref for DynamicBoundedWString<'msg> {
    type Target = rosidl_runtime_rs::WString;
    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

impl<'msg> Display for DynamicBoundedWString<'msg> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.inner.fmt(f)
    }
}

unsafe impl<'msg> Proxy<'msg> for DynamicBoundedWString<'msg> {
    type Metadata = NonZeroUsize; // String upper bound

    fn size_in_memory(_: NonZeroUsize) -> usize {
        std::mem::size_of::<rosidl_runtime_rs::WString>()
    }

    unsafe fn new(bytes: &'msg [u8], string_upper_bound: NonZeroUsize) -> Self {
        let inner = &*(bytes.as_ptr() as *const rosidl_runtime_rs::WString);
        Self {
            inner,
            upper_bound: string_upper_bound,
        }
    }
}

// ========================= impl for DynamicBounded(W)StringMut =========================

impl<'msg> AsMut<[std::os::raw::c_char]> for DynamicBoundedStringMut<'msg> {
    fn as_mut(&mut self) -> &mut [std::os::raw::c_char] {
        self.inner.deref_mut()
    }
}

impl<'msg> Deref for DynamicBoundedStringMut<'msg> {
    type Target = rosidl_runtime_rs::String;
    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

impl<'msg> Display for DynamicBoundedStringMut<'msg> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.inner.fmt(f)
    }
}

impl<'msg> DynamicSequenceElementMut<'msg> for DynamicBoundedStringMut<'msg> {
    type InnerSequence = ProxySequence<'msg, Self>;
}

unsafe impl<'msg> ProxyMut<'msg> for DynamicBoundedStringMut<'msg> {
    type Metadata = NonZeroUsize; // String upper bound

    fn size_in_memory(_: NonZeroUsize) -> usize {
        std::mem::size_of::<rosidl_runtime_rs::String>()
    }

    unsafe fn new(bytes: &'msg mut [u8], string_upper_bound: NonZeroUsize) -> Self {
        let inner = &mut *(bytes.as_mut_ptr() as *mut rosidl_runtime_rs::String);
        Self {
            inner,
            upper_bound: string_upper_bound,
        }
    }
}

impl<'msg> DynamicBoundedStringMut<'msg> {
    /// Returns the maximum length of this string.
    pub fn upper_bound(&self) -> NonZeroUsize {
        self.upper_bound
    }

    /// If the given string is not too long, assign it to self.
    pub fn try_assign(&mut self, s: &str) -> Result<(), StringExceedsBoundsError> {
        let length = s.chars().count();
        if length <= self.upper_bound.into() {
            *self.inner = rosidl_runtime_rs::String::from(s);
            Ok(())
        } else {
            Err(StringExceedsBoundsError {
                len: length,
                upper_bound: self.upper_bound.into(),
            })
        }
    }
}

impl<'msg> AsMut<[std::os::raw::c_ushort]> for DynamicBoundedWStringMut<'msg> {
    fn as_mut(&mut self) -> &mut [std::os::raw::c_ushort] {
        self.inner.deref_mut()
    }
}

impl<'msg> Deref for DynamicBoundedWStringMut<'msg> {
    type Target = rosidl_runtime_rs::WString;
    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

impl<'msg> Display for DynamicBoundedWStringMut<'msg> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.inner.fmt(f)
    }
}

impl<'msg> DynamicSequenceElementMut<'msg> for DynamicBoundedWStringMut<'msg> {
    type InnerSequence = ProxySequence<'msg, Self>;
}

unsafe impl<'msg> ProxyMut<'msg> for DynamicBoundedWStringMut<'msg> {
    type Metadata = NonZeroUsize; // String upper bound

    fn size_in_memory(_: NonZeroUsize) -> usize {
        std::mem::size_of::<rosidl_runtime_rs::WString>()
    }

    unsafe fn new(bytes: &'msg mut [u8], string_upper_bound: NonZeroUsize) -> Self {
        let inner = &mut *(bytes.as_mut_ptr() as *mut rosidl_runtime_rs::WString);
        Self {
            inner,
            upper_bound: string_upper_bound,
        }
    }
}

impl<'msg> DynamicBoundedWStringMut<'msg> {
    /// Returns the maximum length of this string.
    pub fn upper_bound(&self) -> NonZeroUsize {
        self.upper_bound
    }

    /// If the given string is not too long, assign it to self.
    pub fn try_assign(&mut self, s: &str) -> Result<(), StringExceedsBoundsError> {
        let length = s.chars().count();
        if length <= self.upper_bound.into() {
            *self.inner = rosidl_runtime_rs::WString::from(s);
            Ok(())
        } else {
            Err(StringExceedsBoundsError {
                len: length,
                upper_bound: self.upper_bound.into(),
            })
        }
    }
}
