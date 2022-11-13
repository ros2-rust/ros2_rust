use std::cmp::Ordering;
use std::ffi::CStr;
use std::fmt::{self, Debug, Display};
use std::hash::{Hash, Hasher};
use std::ops::{Deref, DerefMut};

#[cfg(feature = "serde")]
mod serde;

use crate::sequence::Sequence;
use crate::traits::SequenceAlloc;

/// A zero-terminated string of 8-bit characters.
///
/// The layout of this type is the same as `rosidl_runtime_c__String`. See the
/// [`Message`](crate::Message) trait for background information on this topic.
///
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::String;
/// let mut s = String::from("Grüß Gott!");
/// // Conversion back to a std::string::String is done with the ToString trait from the standard
/// // library.
/// assert_eq!(&s.to_string(), "Grüß Gott!");
/// ```
#[repr(C)]
pub struct String {
    /// Dynamic memory in this type is allocated and deallocated by C, but this is a detail that is managed by
    /// the relevant functions and trait impls.
    data: *mut std::os::raw::c_char,
    size: usize,
    capacity: usize,
}

/// A zero-terminated string of 16-bit characters.
///
/// The layout of this type is the same as `rosidl_runtime_c__U16String`. See the
/// [`Message`](crate::Message) trait for background information on this topic.
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::WString;
/// let mut s = WString::from("Grüß Gott!");
/// // Conversion back to a std::string::String is done with the ToString trait from the standard
/// // library.
/// assert_eq!(&s.to_string(), "Grüß Gott!");
/// ```
#[repr(C)]
pub struct WString {
    data: *mut std::os::raw::c_ushort,
    size: usize,
    capacity: usize,
}

/// A zero-terminated string of 8-bit characters with a length limit.
///
/// The same as [`String`], but it cannot be constructed from a string that is too large.
/// The length is measured as the number of Unicode scalar values, not bytes.
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::BoundedString;
/// let mut maybe_str = BoundedString::<3>::try_from("noo!");
/// assert!(maybe_str.is_err());
/// maybe_str = BoundedString::<3>::try_from("ok!");
/// assert!(maybe_str.is_ok());
/// let bounded_str = maybe_str.unwrap();
/// assert_eq!(&bounded_str.to_string(), "ok!");
/// ```
#[derive(Clone, Default, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct BoundedString<const N: usize> {
    inner: String,
}

/// A zero-terminated string of 16-bit characters with a length limit.
///
/// The same as [`WString`], but it cannot be constructed from a string that is too large.
/// The length is measured as the number of Unicode scalar values, not bytes.
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::BoundedWString;
/// let mut maybe_wstr = BoundedWString::<3>::try_from("noo!");
/// assert!(maybe_wstr.is_err());
/// maybe_wstr = BoundedWString::<3>::try_from("ok!");
/// assert!(maybe_wstr.is_ok());
/// let bounded_wstr = maybe_wstr.unwrap();
/// assert_eq!(&bounded_wstr.to_string(), "ok!");
/// ```
#[derive(Clone, Default, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct BoundedWString<const N: usize> {
    inner: WString,
}

/// Error type for [`BoundedString::try_from()`] and [`BoundedWString::try_from()`].
#[derive(Debug)]
pub struct StringExceedsBoundsError {
    /// The actual length the string would have after the operation.
    pub len: usize,
    /// The upper bound on the string length.
    pub upper_bound: usize,
}

// ========================= impls for String and WString =========================

// There is a lot of redundancy between String and WString, which this macro aims to reduce.
macro_rules! string_impl {
    ($string:ty, $char_type:ty, $unsigned_char_type:ty, $string_conversion_func:ident, $init:ident, $fini:ident, $assignn:ident, $sequence_init:ident, $sequence_fini:ident, $sequence_copy:ident) => {
        #[link(name = "rosidl_runtime_c")]
        extern "C" {
            fn $init(s: *mut $string) -> bool;
            fn $fini(s: *mut $string);
            fn $assignn(s: *mut $string, value: *const $char_type, n: usize) -> bool;
            fn $sequence_init(seq: *mut Sequence<$string>, size: usize) -> bool;
            fn $sequence_fini(seq: *mut Sequence<$string>);
            fn $sequence_copy(
                in_seq: *const Sequence<$string>,
                out_seq: *mut Sequence<$string>,
            ) -> bool;
        }

        impl Clone for $string {
            fn clone(&self) -> Self {
                let mut msg = Self::default();
                // SAFETY: This is doing the same thing as rosidl_runtime_c__String__copy.
                if !unsafe { $assignn(&mut msg as *mut _, self.data as *const _, self.size) } {
                    panic!("$assignn failed");
                }
                msg
            }
        }

        impl Debug for $string {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
                Debug::fmt(&self.to_string(), f)
            }
        }

        impl Default for $string {
            fn default() -> Self {
                let mut msg = Self {
                    data: std::ptr::null_mut(),
                    size: 0,
                    capacity: 0,
                };
                // SAFETY: Passing in a zeroed string is safe.
                if !unsafe { $init(&mut msg as *mut _) } {
                    panic!("$init failed");
                }
                msg
            }
        }

        // It's not guaranteed that there are no interior null bytes, hence no Deref to CStr.
        // This does not include the null byte at the end!
        impl Deref for $string {
            type Target = [$char_type];
            fn deref(&self) -> &Self::Target {
                // SAFETY: self.data points to self.size consecutive, initialized elements and
                // isn't modified externally.
                unsafe { std::slice::from_raw_parts(self.data, self.size) }
            }
        }

        impl DerefMut for $string {
            fn deref_mut(&mut self) -> &mut Self::Target {
                // SAFETY: self.data points to self.size consecutive, initialized elements and
                // isn't modified externally.
                unsafe { std::slice::from_raw_parts_mut(self.data, self.size) }
            }
        }

        impl Display for $string {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
                // SAFETY: Same as deref, but with an additional cast to the unsigned type.
                // See also https://users.rust-lang.org/t/how-to-convert-i8-to-u8/16308/11
                let u8_slice = unsafe {
                    std::slice::from_raw_parts(self.data as *mut $unsigned_char_type, self.size)
                };
                let converted = std::string::String::$string_conversion_func(u8_slice);
                Display::fmt(&converted, f)
            }
        }

        impl Drop for $string {
            fn drop(&mut self) {
                // SAFETY: There are no special preconditions to the fini function.
                unsafe {
                    $fini(self as *mut _);
                }
            }
        }

        impl Eq for $string {}

        impl Extend<char> for $string {
            fn extend<I: IntoIterator<Item = char>>(&mut self, iter: I) {
                let mut s = self.to_string();
                s.extend(iter);
                *self = Self::from(s.as_str());
            }
        }

        impl<'a> Extend<&'a char> for $string {
            fn extend<I: IntoIterator<Item = &'a char>>(&mut self, iter: I) {
                self.extend(iter.into_iter().cloned());
            }
        }

        impl FromIterator<char> for $string {
            fn from_iter<I: IntoIterator<Item = char>>(iter: I) -> Self {
                let mut buf = <$string>::default();
                buf.extend(iter);
                buf
            }
        }

        impl<'a> FromIterator<&'a char> for $string {
            fn from_iter<I: IntoIterator<Item = &'a char>>(iter: I) -> Self {
                let mut buf = <$string>::default();
                buf.extend(iter);
                buf
            }
        }

        impl Hash for $string {
            fn hash<H: Hasher>(&self, state: &mut H) {
                self.deref().hash(state)
            }
        }

        impl Ord for $string {
            fn cmp(&self, other: &Self) -> Ordering {
                self.deref().cmp(other.deref())
            }
        }

        impl PartialEq for $string {
            fn eq(&self, other: &Self) -> bool {
                self.deref().eq(other.deref())
            }
        }

        impl PartialOrd for $string {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                self.deref().partial_cmp(other.deref())
            }
        }

        // SAFETY: A string is a simple data structure, and therefore not thread-specific.
        unsafe impl Send for $string {}
        // SAFETY: A string does not have interior mutability, so it can be shared.
        unsafe impl Sync for $string {}

        impl SequenceAlloc for $string {
            fn sequence_init(seq: &mut Sequence<Self>, size: usize) -> bool {
                // SAFETY: There are no special preconditions to the sequence_init function.
                unsafe { $sequence_init(seq as *mut _, size) }
            }
            fn sequence_fini(seq: &mut Sequence<Self>) {
                // SAFETY: There are no special preconditions to the sequence_fini function.
                unsafe { $sequence_fini(seq as *mut _) }
            }
            fn sequence_copy(in_seq: &Sequence<Self>, out_seq: &mut Sequence<Self>) -> bool {
                // SAFETY: There are no special preconditions to the sequence_copy function.
                unsafe { $sequence_copy(in_seq as *const _, out_seq as *mut _) }
            }
        }
    };
}

string_impl!(
    String,
    std::os::raw::c_char,
    u8,
    from_utf8_lossy,
    rosidl_runtime_c__String__init,
    rosidl_runtime_c__String__fini,
    rosidl_runtime_c__String__assignn,
    rosidl_runtime_c__String__Sequence__init,
    rosidl_runtime_c__String__Sequence__fini,
    rosidl_runtime_c__String__Sequence__copy
);
string_impl!(
    WString,
    std::os::raw::c_ushort,
    u16,
    from_utf16_lossy,
    rosidl_runtime_c__U16String__init,
    rosidl_runtime_c__U16String__fini,
    rosidl_runtime_c__U16String__assignn,
    rosidl_runtime_c__U16String__Sequence__init,
    rosidl_runtime_c__U16String__Sequence__fini,
    rosidl_runtime_c__U16String__Sequence__copy
);

impl From<&str> for String {
    fn from(s: &str) -> Self {
        let mut msg = Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        // SAFETY: It's okay to pass a non-zero-terminated string here since assignn uses the
        // specified length and will append the 0 byte to the dest string itself.
        if !unsafe {
            rosidl_runtime_c__String__assignn(&mut msg as *mut _, s.as_ptr() as *const _, s.len())
        } {
            panic!("rosidl_runtime_c__String__assignn failed");
        }
        msg
    }
}

impl String {
    /// Creates a CStr from this String.
    ///
    /// This scales with the length of the string but does not create copy of the string.
    /// See also [`CStr::from_ptr()`].
    pub fn to_cstr(&self) -> &CStr {
        // SAFETY: self.data is a valid pointer and won't change.
        // Also, the lifetime of the CStr is the same as self, which is correct.
        unsafe { CStr::from_ptr(self.data as *const _) }
    }
}

impl From<&str> for WString {
    fn from(s: &str) -> Self {
        let mut msg = Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        let buf: Vec<u16> = s.encode_utf16().collect();
        // SAFETY: It's okay to pass a non-zero-terminated string here since assignn uses the
        // specified length and will append the 0 to the dest string itself.
        if !unsafe {
            rosidl_runtime_c__U16String__assignn(
                &mut msg as *mut _,
                buf.as_ptr() as *const _,
                buf.len(),
            )
        } {
            panic!("rosidl_runtime_c__U16String__assignn failed");
        }
        msg
    }
}

// ========================= impl for BoundedString =========================

impl<const N: usize> Debug for BoundedString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Debug::fmt(&self.inner, f)
    }
}

impl<const N: usize> Deref for BoundedString<N> {
    type Target = [std::os::raw::c_char];
    fn deref(&self) -> &Self::Target {
        self.inner.deref()
    }
}

impl<const N: usize> DerefMut for BoundedString<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.deref_mut()
    }
}

impl<const N: usize> Display for BoundedString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Display::fmt(&self.inner, f)
    }
}

impl<const N: usize> SequenceAlloc for BoundedString<N> {
    fn sequence_init(seq: &mut Sequence<Self>, size: usize) -> bool {
        // SAFETY: There are no special preconditions to the rosidl_runtime_c__String__Sequence__init function.
        unsafe {
            rosidl_runtime_c__String__Sequence__init(seq as *mut Sequence<Self> as *mut _, size)
        }
    }
    fn sequence_fini(seq: &mut Sequence<Self>) {
        // SAFETY: There are no special preconditions to the rosidl_runtime_c__String__Sequence__fini function.
        unsafe { rosidl_runtime_c__String__Sequence__fini(seq as *mut Sequence<Self> as *mut _) }
    }
    fn sequence_copy(in_seq: &Sequence<Self>, out_seq: &mut Sequence<Self>) -> bool {
        // SAFETY: Transmute of a transparent type to the inner type is fine
        unsafe {
            <String as SequenceAlloc>::sequence_copy(
                std::mem::transmute::<&Sequence<Self>, &Sequence<String>>(in_seq),
                std::mem::transmute::<&mut Sequence<Self>, &mut Sequence<String>>(out_seq),
            )
        }
    }
}

impl<const N: usize> TryFrom<&str> for BoundedString<N> {
    type Error = StringExceedsBoundsError;
    fn try_from(s: &str) -> Result<Self, Self::Error> {
        let length = s.chars().count();
        if length <= N {
            Ok(Self {
                inner: String::from(s),
            })
        } else {
            Err(StringExceedsBoundsError {
                len: length,
                upper_bound: N,
            })
        }
    }
}

// ========================= impl for BoundedWString =========================

impl<const N: usize> Debug for BoundedWString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Debug::fmt(&self.inner, f)
    }
}

impl<const N: usize> Deref for BoundedWString<N> {
    type Target = [u16];
    fn deref(&self) -> &Self::Target {
        self.inner.deref()
    }
}

impl<const N: usize> DerefMut for BoundedWString<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.deref_mut()
    }
}

impl<const N: usize> Display for BoundedWString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Display::fmt(&self.inner, f)
    }
}

impl<const N: usize> SequenceAlloc for BoundedWString<N> {
    fn sequence_init(seq: &mut Sequence<Self>, size: usize) -> bool {
        // SAFETY: There are no special preconditions to the rosidl_runtime_c__U16String__Sequence__init function.
        unsafe {
            rosidl_runtime_c__U16String__Sequence__init(seq as *mut Sequence<Self> as *mut _, size)
        }
    }
    fn sequence_fini(seq: &mut Sequence<Self>) {
        // SAFETY: There are no special preconditions to the rosidl_runtime_c__U16String__Sequence__fini function.
        unsafe { rosidl_runtime_c__U16String__Sequence__fini(seq as *mut Sequence<Self> as *mut _) }
    }
    fn sequence_copy(in_seq: &Sequence<Self>, out_seq: &mut Sequence<Self>) -> bool {
        // SAFETY: Transmute of a transparent type to the inner type is fine
        unsafe {
            <WString as SequenceAlloc>::sequence_copy(
                std::mem::transmute::<&Sequence<Self>, &Sequence<WString>>(in_seq),
                std::mem::transmute::<&mut Sequence<Self>, &mut Sequence<WString>>(out_seq),
            )
        }
    }
}

impl<const N: usize> TryFrom<&str> for BoundedWString<N> {
    type Error = StringExceedsBoundsError;
    fn try_from(s: &str) -> Result<Self, Self::Error> {
        let length = s.chars().count();
        if length <= N {
            Ok(Self {
                inner: WString::from(s),
            })
        } else {
            Err(StringExceedsBoundsError {
                len: length,
                upper_bound: N,
            })
        }
    }
}

// ========================= impl for StringExceedsBoundsError =========================

impl Display for StringExceedsBoundsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        write!(
            f,
            "BoundedString with upper bound {} initialized with len {}",
            self.upper_bound, self.len
        )
    }
}

impl std::error::Error for StringExceedsBoundsError {}

#[cfg(test)]
mod tests {
    use quickcheck::{Arbitrary, Gen};

    use super::*;

    impl Arbitrary for String {
        fn arbitrary(g: &mut Gen) -> Self {
            std::string::String::arbitrary(g).as_str().into()
        }
    }

    impl Arbitrary for WString {
        fn arbitrary(g: &mut Gen) -> Self {
            std::string::String::arbitrary(g).as_str().into()
        }
    }

    impl Arbitrary for BoundedString<256> {
        fn arbitrary(g: &mut Gen) -> Self {
            let len = u8::arbitrary(g);
            let s: std::string::String = (0..len).map(|_| char::arbitrary(g)).collect();
            s.as_str().try_into().unwrap()
        }
    }

    impl Arbitrary for BoundedWString<256> {
        fn arbitrary(g: &mut Gen) -> Self {
            let len = u8::arbitrary(g);
            let s: std::string::String = (0..len).map(|_| char::arbitrary(g)).collect();
            s.as_str().try_into().unwrap()
        }
    }

    #[test]
    fn string_from_char_iterator() {
        // Base char case
        let expected = String::from("abc");
        let actual = "abc".chars().collect::<String>();

        assert_eq!(expected, actual);

        // Empty case
        let expected = String::from("");
        let actual = "".chars().collect::<String>();

        assert_eq!(expected, actual);

        // Non-ascii char case
        let expected = String::from("Grüß Gott! 𝕊");
        let actual = "Grüß Gott! 𝕊".chars().collect::<String>();

        assert_eq!(expected, actual);
    }

    #[test]
    fn extend_string_with_char_iterator() {
        let expected = WString::from("abcdef");
        let mut actual = WString::from("abc");
        actual.extend("def".chars());

        assert_eq!(expected, actual);
    }

    #[test]
    fn wstring_from_char_iterator() {
        // Base char case
        let expected = WString::from("abc");
        let actual = "abc".chars().collect::<WString>();

        assert_eq!(expected, actual);

        // Empty case
        let expected = WString::from("");
        let actual = "".chars().collect::<WString>();

        assert_eq!(expected, actual);

        // Non-ascii char case
        let expected = WString::from("Grüß Gott! 𝕊");
        let actual = "Grüß Gott! 𝕊".chars().collect::<WString>();

        assert_eq!(expected, actual);
    }

    #[test]
    fn extend_wstring_with_char_iterator() {
        let expected = WString::from("abcdef");
        let mut actual = WString::from("abc");
        actual.extend("def".chars());

        assert_eq!(expected, actual);
    }
}
