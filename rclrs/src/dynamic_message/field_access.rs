use rosidl_runtime_rs::Sequence;

use super::{BaseType, MessageFieldInfo, ValueKind};

mod dynamic_bounded_string;
mod dynamic_message_view;
mod dynamic_sequence;
pub use dynamic_bounded_string::*;
pub use dynamic_message_view::*;
pub use dynamic_sequence::*;

// Note:
// This module defines a bunch of structs in two flavors: immutable and mutable.
// It's annoying, but imo still better than doing something hard-to-read like
// https://lab.whitequark.org/notes/2016-12-13/abstracting-over-mutability-in-rust/

/// Helper function for use with reinterpret()/reinterpret_array()
fn check<T>(bytes: &[u8]) {
    assert!(bytes.len() >= std::mem::size_of::<T>());
    let align = std::mem::align_of::<T>();
    assert_eq!(bytes.as_ptr().align_offset(align), 0);
}

// The purpose of this macro is to reduce duplication between the value types for the
// mutable and immutable cases.
macro_rules! define_value_types {
    ($select:ident) => {

        // Creates either an immutable or a mutable reference to the type.
        macro_rules! make_ref { ($lt:lifetime, $type:ty) => {
            $select!(
                immutable => &$lt $type,
                mutable => &$lt mut $type
            )
        }}

        /// A single value.
        // The field variants are for the most part self-explaining.
        #[allow(missing_docs)]
        #[derive(Debug, PartialEq)]
        pub enum SimpleValue<'msg> {
            Float(make_ref!('msg, f32)),
            Double(make_ref!('msg, f64)),
            /// It's platform-dependent what the size of long double is.
            /// Here's a pointer, you figure it out.
            LongDouble($select!(
                immutable => *const u8,
                mutable => *mut u8
            )),
            Char(make_ref!('msg, u8)),
            WChar(make_ref!('msg, u16)),
            Boolean(make_ref!('msg, bool)),
            Octet(make_ref!('msg, u8)),
            Uint8(make_ref!('msg, u8)),
            Int8(make_ref!('msg, i8)),
            Uint16(make_ref!('msg, u16)),
            Int16(make_ref!('msg, i16)),
            Uint32(make_ref!('msg, u32)),
            Int32(make_ref!('msg, i32)),
            Uint64(make_ref!('msg, u64)),
            Int64(make_ref!('msg, i64)),
            String(make_ref!('msg, rosidl_runtime_rs::String)),
            BoundedString($select!(
                immutable => DynamicBoundedString<'msg>,
                mutable => DynamicBoundedStringMut<'msg>
            )),
            WString(make_ref!('msg, rosidl_runtime_rs::WString)),
            BoundedWString($select!(
                immutable => DynamicBoundedWString<'msg>,
                mutable => DynamicBoundedWStringMut<'msg>
            )),
            Message($select!(
                immutable => DynamicMessageView<'msg>,
                mutable => DynamicMessageViewMut<'msg>
            )),
        }

        /// An array of values.
        // The field variants are for the most part self-explaining.
        #[allow(missing_docs)]
        #[derive(Debug, PartialEq)]
        pub enum ArrayValue<'msg> {
            FloatArray(make_ref!('msg, [f32])),
            DoubleArray(make_ref!('msg, [f64])),
            /// It's platform-dependent what the size of long double is.
            /// Here's a pointer and an array size, you figure it out.
            LongDoubleArray($select!(
                immutable => *const u8,
                mutable => *mut u8
            ), usize),
            CharArray(make_ref!('msg, [u8])),
            WCharArray(make_ref!('msg, [u16])),
            BooleanArray(make_ref!('msg, [bool])),
            OctetArray(make_ref!('msg, [u8])),
            Uint8Array(make_ref!('msg, [u8])),
            Int8Array(make_ref!('msg, [i8])),
            Uint16Array(make_ref!('msg, [u16])),
            Int16Array(make_ref!('msg, [i16])),
            Uint32Array(make_ref!('msg, [u32])),
            Int32Array(make_ref!('msg, [i32])),
            Uint64Array(make_ref!('msg, [u64])),
            Int64Array(make_ref!('msg, [i64])),
            StringArray(make_ref!('msg, [rosidl_runtime_rs::String])),
            BoundedStringArray($select!(
                immutable => Box<[DynamicBoundedString<'msg>]>,
                mutable => Box<[DynamicBoundedStringMut<'msg>]>
            )),
            WStringArray(make_ref!('msg, [rosidl_runtime_rs::WString])),
            BoundedWStringArray($select!(
                immutable => Box<[DynamicBoundedWString<'msg>]>,
                mutable => Box<[DynamicBoundedWStringMut<'msg>]>
            )),
            MessageArray($select!(
                immutable => Box<[DynamicMessageView<'msg>]>,
                mutable => Box<[DynamicMessageViewMut<'msg>]>
            )),
        }

        /// A sequence of unbounded length.
        // The field variants are for the most part self-explaining.
        // Developers: Please also see the explanation in dynamic_sequence.rs.
        #[allow(missing_docs)]
        #[derive(Debug, PartialEq)]
        pub enum SequenceValue<'msg> {
            FloatSequence(make_ref!('msg, Sequence<f32>)),
            DoubleSequence(make_ref!('msg, Sequence<f64>)),
            /// It's platform-dependent what the size of long double is.
            /// Here's a pointer to the [`Sequence`][1] struct.
            ///
            /// [1]: rosidl_runtime_rs::Sequence
            LongDoubleSequence($select!(
                immutable => *const u8,
                mutable => *mut u8
            )),
            CharSequence(make_ref!('msg, Sequence<u8>)),
            WCharSequence(make_ref!('msg, Sequence<u16>)),
            BooleanSequence(make_ref!('msg, Sequence<bool>)),
            OctetSequence(make_ref!('msg, Sequence<u8>)),
            Uint8Sequence(make_ref!('msg, Sequence<u8>)),
            Int8Sequence(make_ref!('msg, Sequence<i8>)),
            Uint16Sequence(make_ref!('msg, Sequence<u16>)),
            Int16Sequence(make_ref!('msg, Sequence<i16>)),
            Uint32Sequence(make_ref!('msg, Sequence<u32>)),
            Int32Sequence(make_ref!('msg, Sequence<i32>)),
            Uint64Sequence(make_ref!('msg, Sequence<u64>)),
            Int64Sequence(make_ref!('msg, Sequence<i64>)),
            StringSequence(make_ref!('msg, Sequence<rosidl_runtime_rs::String>)),
            /// This variant is not a [`Sequence`][1], since there is no suitable element type
            /// that both matches the underlying struct layout and includes information about
            /// the string length bound.
            ///
            /// [1]: rosidl_runtime_rs::Sequence
            BoundedStringSequence($select!(
                immutable => DynamicSequence<'msg, DynamicBoundedString<'msg>>,
                mutable => DynamicSequenceMut<'msg, DynamicBoundedStringMut<'msg>>
            )),
            WStringSequence(make_ref!('msg, Sequence<rosidl_runtime_rs::WString>)),
            /// This variant is not a [`Sequence`][1], since there is no suitable element type
            /// that both matches the underlying struct layout and includes information about
            /// the string length bound.
            ///
            /// [1]: rosidl_runtime_rs::Sequence
            BoundedWStringSequence($select!(
                immutable => DynamicSequence<'msg, DynamicBoundedWString<'msg>>,
                mutable => DynamicSequenceMut<'msg, DynamicBoundedWStringMut<'msg>>
            )),
            /// This variant is not a [`Sequence<DynamicMessageView>`][1], since the actual
            /// element type has a different size and layout from [`DynamicMessageView`][2].
            ///
            /// [1]: rosidl_runtime_rs::Sequence
            /// [2]: DynamicMessageView
            MessageSequence($select!(
                immutable => DynamicSequence<'msg, DynamicMessageView<'msg>>,
                mutable => DynamicSequenceMut<'msg, DynamicMessageViewMut<'msg>>)
            ),
        }

        // Internal type alias to avoid repeating this a hundred times
        type BoundedSequence<'msg, T> = $select!(
            immutable => DynamicBoundedSequence<'msg, T>,
            mutable => DynamicBoundedSequenceMut<'msg, T>
        );

        /// A sequence of bounded length.
        // The field variants are for the most part self-explaining.
        // Developers: Please also see the explanation in dynamic_sequence.rs.
        #[allow(missing_docs)]
        #[derive(Debug, PartialEq)]
        pub enum BoundedSequenceValue<'msg> {
            FloatBoundedSequence(BoundedSequence<'msg, f32>),
            DoubleBoundedSequence(BoundedSequence<'msg, f64>),
            /// It's platform-dependent what the size of long double is.
            /// Here's a pointer to the [`BoundedSequence`][1] struct and the upper bound.
            ///
            /// [1]: rosidl_runtime_rs::BoundedSequence
            LongDoubleBoundedSequence($select!(
                immutable => *const u8,
                mutable => *mut u8
            ), usize),
            CharBoundedSequence(BoundedSequence<'msg, u8>),
            WCharBoundedSequence(BoundedSequence<'msg, u16>),
            BooleanBoundedSequence(BoundedSequence<'msg, bool>),
            OctetBoundedSequence(BoundedSequence<'msg, u8>),
            Uint8BoundedSequence(BoundedSequence<'msg, u8>),
            Int8BoundedSequence(BoundedSequence<'msg, i8>),
            Uint16BoundedSequence(BoundedSequence<'msg, u16>),
            Int16BoundedSequence(BoundedSequence<'msg, i16>),
            Uint32BoundedSequence(BoundedSequence<'msg, u32>),
            Int32BoundedSequence(BoundedSequence<'msg, i32>),
            Uint64BoundedSequence(BoundedSequence<'msg, u64>),
            Int64BoundedSequence(BoundedSequence<'msg, i64>),
            StringBoundedSequence(BoundedSequence<'msg, rosidl_runtime_rs::String>),
            BoundedStringBoundedSequence($select!(
                immutable => DynamicBoundedSequence<'msg, DynamicBoundedString<'msg>>,
                mutable => DynamicBoundedSequenceMut<'msg, DynamicBoundedStringMut<'msg>>)
            ),
            WStringBoundedSequence(BoundedSequence<'msg, rosidl_runtime_rs::WString>),
            BoundedWStringBoundedSequence($select!(
                immutable => DynamicBoundedSequence<'msg, DynamicBoundedWString<'msg>>,
                mutable => DynamicBoundedSequenceMut<'msg, DynamicBoundedWStringMut<'msg>>)
            ),
            MessageBoundedSequence($select!(
                immutable => DynamicBoundedSequence<'msg, DynamicMessageView<'msg>>,
                mutable => DynamicBoundedSequenceMut<'msg, DynamicMessageViewMut<'msg>>)
            ),
        }

        // SAFETY: This is in effect a transmutation.
        //
        // Here is why this is safe when used as intended, i.e. to reinterpret bytes inside
        // a dynamic message to their true types, assuming the introspection library is not lying:
        // * There is no (safe) way for users of rcrls to cause an invalid bit pattern for T to
        //   be written to the storage of the dynamic message. All accesses are through references to T.
        // * Correct alignment is ensured during the allocation of the storage for the dynamic message
        // * The lifetime of the input slice is preserved in the produced reference, so
        //   this works exactly like borrowing a regular field from a message – the message
        //   can never be dropped while a borrow exists etc.
        // * This function does not transmute & to &mut
        // * This is only used for primitive values and rosidl_runtime_rs types marked as repr(C),
        //   so there is no risk of reinterpreting as a type with undefined layout.
        unsafe fn reinterpret<'a, T>(bytes: make_ref!('a, [u8])) -> make_ref!('a, T) {
            check::<T>(bytes);
            $select!(
                immutable => { &*(bytes.as_ptr() as *const T) },
                mutable => { &mut *(bytes.as_mut_ptr() as *mut T) }
            )
        }

        // SAFETY: This is in effect a transmutation. The caller of this function must ensure
        // that the bytes correspond to a valid [T].
        //
        // See also the reinterpret() function.
        //
        // std::slice::from_raw_parts is the correct way to transmute a slice.
        // We can't rely on the internal representation of slices (or other stdlib types).
        unsafe fn reinterpret_array<'a, T>(bytes: make_ref!('a, [u8]), array_size: usize) -> make_ref!('a, [T]) {
            check::<T>(bytes);
            $select!(
                immutable => { std::slice::from_raw_parts(bytes.as_ptr() as *const T, array_size) },
                mutable => { std::slice::from_raw_parts_mut(bytes.as_mut_ptr() as *mut T, array_size) }
            )
        }

        impl<'msg> SimpleValue<'msg> {
            pub(super) unsafe fn new(
                bytes: make_ref!('msg, [u8]),
                field_info: &'msg MessageFieldInfo,
            ) -> Self {
                match &field_info.base_type {
                    BaseType::Float => SimpleValue::Float(reinterpret::<f32>(bytes)),
                    BaseType::Double => SimpleValue::Double(reinterpret::<f64>(bytes)),
                    BaseType::LongDouble => SimpleValue::LongDouble($select!(
                        immutable => bytes.as_ptr(),
                        mutable => bytes.as_mut_ptr()
                    )),
                    BaseType::Char => SimpleValue::Char(reinterpret::<u8>(bytes)),
                    BaseType::WChar => SimpleValue::WChar(reinterpret::<u16>(bytes)),
                    BaseType::Boolean => {
                        assert!(bytes[0] <= 1);
                        SimpleValue::Boolean(reinterpret::<bool>(bytes))
                    }
                    BaseType::Octet => SimpleValue::Octet(reinterpret::<u8>(bytes)),
                    BaseType::Uint8 => SimpleValue::Uint8(reinterpret::<u8>(bytes)),
                    BaseType::Int8 => SimpleValue::Int8(reinterpret::<i8>(bytes)),
                    BaseType::Uint16 => SimpleValue::Uint16(reinterpret::<u16>(bytes)),
                    BaseType::Int16 => SimpleValue::Int16(reinterpret::<i16>(bytes)),
                    BaseType::Uint32 => SimpleValue::Uint32(reinterpret::<u32>(bytes)),
                    BaseType::Int32 => SimpleValue::Int32(reinterpret::<i32>(bytes)),
                    BaseType::Uint64 => SimpleValue::Uint64(reinterpret::<u64>(bytes)),
                    BaseType::Int64 => SimpleValue::Int64(reinterpret::<i64>(bytes)),
                    BaseType::String => SimpleValue::String(reinterpret::<rosidl_runtime_rs::String>(bytes)),
                    BaseType::BoundedString { upper_bound } => {
                        SimpleValue::BoundedString($select!(
                            immutable => DynamicBoundedString {
                                inner: reinterpret::<rosidl_runtime_rs::String>(bytes),
                                upper_bound: *upper_bound,
                            },
                            mutable => DynamicBoundedStringMut {
                                inner: reinterpret::<rosidl_runtime_rs::String>(bytes),
                                upper_bound: *upper_bound,
                            }
                        ))
                    }
                    BaseType::WString => SimpleValue::WString(reinterpret::<rosidl_runtime_rs::WString>(bytes)),
                    BaseType::BoundedWString { upper_bound } => {
                        SimpleValue::BoundedWString($select!(
                            immutable => DynamicBoundedWString {
                                inner: reinterpret::<rosidl_runtime_rs::WString>(bytes),
                                upper_bound: *upper_bound,
                            },
                            mutable => DynamicBoundedWStringMut {
                                inner: reinterpret::<rosidl_runtime_rs::WString>(bytes),
                                upper_bound: *upper_bound,
                            }
                        ))
                    }
                    BaseType::Message(structure) => SimpleValue::Message($select!(
                        immutable => DynamicMessageView {
                            storage: &bytes[..structure.size],
                            structure: &*structure,
                        },
                        mutable => DynamicMessageViewMut {
                            storage: &mut bytes[..structure.size],
                            structure: &*structure,
                        }
                    )),
                }
            }
        }

        impl<'msg> ArrayValue<'msg> {
            pub(super) unsafe fn new(
                bytes: make_ref!('msg, [u8]),
                field_info: &'msg MessageFieldInfo,
                array_length: usize,
            ) -> Self {
                match &field_info.base_type {
                    BaseType::Float => {
                        ArrayValue::FloatArray(reinterpret_array::<f32>(bytes, array_length))
                    }
                    BaseType::Double => {
                        ArrayValue::DoubleArray(reinterpret_array::<f64>(bytes, array_length))
                    }
                    BaseType::LongDouble => {
                        ArrayValue::LongDoubleArray($select!(
                            immutable => bytes.as_ptr(),
                            mutable => bytes.as_mut_ptr()
                        ), array_length)
                    }
                    BaseType::Char => {
                        ArrayValue::CharArray(reinterpret_array::<u8>(bytes, array_length))
                    }
                    BaseType::WChar => {
                        ArrayValue::WCharArray(reinterpret_array::<u16>(bytes, array_length))
                    }
                    BaseType::Boolean => {
                        assert!(bytes[0] <= 1);
                        ArrayValue::BooleanArray(reinterpret_array::<bool>(
                            bytes,
                            array_length,
                        ))
                    }
                    BaseType::Octet => {
                        ArrayValue::OctetArray(reinterpret_array::<u8>(bytes, array_length))
                    }
                    BaseType::Uint8 => {
                        ArrayValue::Uint8Array(reinterpret_array::<u8>(bytes, array_length))
                    }
                    BaseType::Int8 => {
                        ArrayValue::Int8Array(reinterpret_array::<i8>(bytes, array_length))
                    }
                    BaseType::Uint16 => {
                        ArrayValue::Uint16Array(reinterpret_array::<u16>(bytes, array_length))
                    }
                    BaseType::Int16 => {
                        ArrayValue::Int16Array(reinterpret_array::<i16>(bytes, array_length))
                    }
                    BaseType::Uint32 => {
                        ArrayValue::Uint32Array(reinterpret_array::<u32>(bytes, array_length))
                    }
                    BaseType::Int32 => {
                        ArrayValue::Int32Array(reinterpret_array::<i32>(bytes, array_length))
                    }
                    BaseType::Uint64 => {
                        ArrayValue::Uint64Array(reinterpret_array::<u64>(bytes, array_length))
                    }
                    BaseType::Int64 => {
                        ArrayValue::Int64Array(reinterpret_array::<i64>(bytes, array_length))
                    }
                    BaseType::String => {
                        ArrayValue::StringArray(reinterpret_array::<rosidl_runtime_rs::String>(
                            bytes,
                            array_length,
                        ))
                    }
                    BaseType::BoundedString { upper_bound } => {
                        let slice = reinterpret_array::<rosidl_runtime_rs::String>(
                            bytes,
                            array_length,
                        );
                        let dynamic_bounded_strings: Vec<_> = slice
                            .into_iter()
                            .map(|inner| $select!(
                                immutable => DynamicBoundedString {
                                    inner,
                                    upper_bound: *upper_bound,
                                },
                                mutable => DynamicBoundedStringMut {
                                    inner,
                                    upper_bound: *upper_bound,
                                }
                            ))
                            .collect();
                        ArrayValue::BoundedStringArray(
                            dynamic_bounded_strings.into_boxed_slice(),
                        )
                    }
                    BaseType::WString => {
                        ArrayValue::WStringArray(reinterpret_array::<rosidl_runtime_rs::WString>(
                            bytes,
                            array_length,
                        ))
                    }
                    BaseType::BoundedWString { upper_bound } => {
                        let slice = reinterpret_array::<rosidl_runtime_rs::WString>(
                            bytes,
                            array_length,
                        );
                        let dynamic_bounded_wstrings: Vec<_> = slice
                            .into_iter()
                            .map(|inner| $select!(
                                immutable => DynamicBoundedWString {
                                    inner,
                                    upper_bound: *upper_bound,
                                },
                                mutable => DynamicBoundedWStringMut {
                                    inner,
                                    upper_bound: *upper_bound,
                                }
                            ))
                            .collect();
                        ArrayValue::BoundedWStringArray(
                            dynamic_bounded_wstrings.into_boxed_slice(),
                        )
                    }
                    BaseType::Message(structure) => {
                        let messages: Vec<_> = $select!(
                            immutable => bytes.chunks(structure.size)
                                .take(array_length)
                                .map(|chunk| DynamicMessageView  {
                                        storage: chunk,
                                        structure: &*structure,
                                })
                                .collect(),
                            mutable => bytes.chunks_mut(structure.size)
                                .take(array_length)
                                .map(|chunk| DynamicMessageViewMut  {
                                        storage: chunk,
                                        structure: &*structure,
                                })
                                .collect()
                        );
                        ArrayValue::MessageArray(messages.into_boxed_slice())
                    }
                }
            }
        }

        impl<'msg> SequenceValue<'msg> {
            pub(super) unsafe fn new(
                bytes: make_ref!('msg, [u8]),
                field_info: &'msg MessageFieldInfo,
            ) -> Self {
                match &field_info.base_type {
                    BaseType::Float => {
                        SequenceValue::FloatSequence(reinterpret::<Sequence<f32>>(bytes))
                    }
                    BaseType::Double => {
                        SequenceValue::DoubleSequence(reinterpret::<Sequence<f64>>(bytes))
                    }
                    BaseType::LongDouble => SequenceValue::LongDoubleSequence($select!(
                        immutable => bytes.as_ptr(),
                        mutable => bytes.as_mut_ptr()
                    )),
                    BaseType::Char => {
                        SequenceValue::CharSequence(reinterpret::<Sequence<u8>>(bytes))
                    }
                    BaseType::WChar => {
                        SequenceValue::WCharSequence(reinterpret::<Sequence<u16>>(bytes))
                    }
                    BaseType::Boolean => {
                        SequenceValue::BooleanSequence(reinterpret::<Sequence<bool>>(bytes))
                    }
                    BaseType::Octet => {
                        SequenceValue::OctetSequence(reinterpret::<Sequence<u8>>(bytes))
                    }
                    BaseType::Uint8 => {
                        SequenceValue::Uint8Sequence(reinterpret::<Sequence<u8>>(bytes))
                    }
                    BaseType::Int8 => {
                        SequenceValue::Int8Sequence(reinterpret::<Sequence<i8>>(bytes))
                    }
                    BaseType::Uint16 => {
                        SequenceValue::Uint16Sequence(reinterpret::<Sequence<u16>>(bytes))
                    }
                    BaseType::Int16 => {
                        SequenceValue::Int16Sequence(reinterpret::<Sequence<i16>>(bytes))
                    }
                    BaseType::Uint32 => {
                        SequenceValue::Uint32Sequence(reinterpret::<Sequence<u32>>(bytes))
                    }
                    BaseType::Int32 => {
                        SequenceValue::Int32Sequence(reinterpret::<Sequence<i32>>(bytes))
                    }
                    BaseType::Uint64 => {
                        SequenceValue::Uint64Sequence(reinterpret::<Sequence<u64>>(bytes))
                    }
                    BaseType::Int64 => {
                        SequenceValue::Int64Sequence(reinterpret::<Sequence<i64>>(bytes))
                    }
                    BaseType::String => {
                        SequenceValue::StringSequence(reinterpret::<
                            Sequence<rosidl_runtime_rs::String>,
                        >(bytes))
                    }
                    BaseType::BoundedString { upper_bound } => {
                        SequenceValue::BoundedStringSequence(
                            $select!(
                                immutable => {
                                    DynamicSequence::new_proxy(
                                        bytes,
                                        *upper_bound
                                    )
                                },
                                mutable => DynamicSequenceMut::new_proxy(
                                    bytes,
                                    *upper_bound,
                                    field_info.resize_function.unwrap(),
                                )
                            )
                        )
                    }
                    BaseType::WString => {
                        SequenceValue::WStringSequence(reinterpret::<
                            Sequence<rosidl_runtime_rs::WString>,
                        >(bytes))
                    }
                    BaseType::BoundedWString { upper_bound } => {
                        SequenceValue::BoundedWStringSequence(
                            $select!(
                                immutable => {
                                    DynamicSequence::new_proxy(
                                        bytes,
                                        *upper_bound
                                    )
                                },
                                mutable => DynamicSequenceMut::new_proxy(
                                    bytes,
                                    *upper_bound,
                                    field_info.resize_function.unwrap(),
                                )
                            )
                        )
                    }
                    BaseType::Message(structure) => {
                        SequenceValue::MessageSequence($select!(
                            immutable => {
                                        DynamicSequence::new_proxy(
                                            bytes,
                                            &**structure,
                                        )
                                    },
                            mutable => DynamicSequenceMut::new_proxy(
                                bytes,
                                &**structure,
                                field_info.resize_function.unwrap()
                            )
                        ))
                    }
                }
            }
        }

        impl<'msg> BoundedSequenceValue<'msg> {
            pub(super) unsafe fn new(
                bytes: make_ref!('msg, [u8]),
                field_info: &'msg MessageFieldInfo,
                sequence_upper_bound: usize,
            ) -> Self {
                match &field_info.base_type {
                    BaseType::Float => {
                        BoundedSequenceValue::FloatBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Double => {
                        BoundedSequenceValue::DoubleBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::LongDouble => BoundedSequenceValue::LongDoubleBoundedSequence(
                        $select!(
                            immutable => bytes.as_ptr(),
                            mutable => bytes.as_mut_ptr()
                        ),
                        sequence_upper_bound,
                    ),
                    BaseType::Char => {
                        BoundedSequenceValue::CharBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::WChar => {
                        BoundedSequenceValue::WCharBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Boolean => {
                        BoundedSequenceValue::BooleanBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Octet => {
                        BoundedSequenceValue::OctetBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Uint8 => {
                        BoundedSequenceValue::Uint8BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Int8 => {
                        BoundedSequenceValue::Int8BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Uint16 => {
                        BoundedSequenceValue::Uint16BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Int16 => {
                        BoundedSequenceValue::Int16BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Uint32 => {
                        BoundedSequenceValue::Uint32BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Int32 => {
                        BoundedSequenceValue::Int32BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Uint64 => {
                        BoundedSequenceValue::Uint64BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::Int64 => {
                        BoundedSequenceValue::Int64BoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap(),
                            )
                        ))
                    }
                    BaseType::String => {
                        BoundedSequenceValue::StringBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap()
                            )
                        ))
                    }
                    BaseType::BoundedString { upper_bound } => {
                        BoundedSequenceValue::BoundedStringBoundedSequence($select!(
                            immutable => { DynamicBoundedSequence::new_proxy(bytes, sequence_upper_bound, *upper_bound) },
                            mutable => DynamicBoundedSequenceMut::new_proxy(
                                bytes,
                                *upper_bound,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap()
                            )
                        ))
                    }
                    BaseType::WString => {
                        BoundedSequenceValue::WStringBoundedSequence($select!(
                            immutable => {
                                DynamicBoundedSequence::new_primitive(
                                    bytes,
                                    sequence_upper_bound
                                )
                            },
                            mutable => DynamicBoundedSequenceMut::new_primitive(
                                bytes,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap()
                            )
                        ))
                    }
                    BaseType::BoundedWString { upper_bound } => {
                        BoundedSequenceValue::BoundedWStringBoundedSequence($select!(
                            immutable => { DynamicBoundedSequence::new_proxy(bytes, sequence_upper_bound, *upper_bound) },
                            mutable => DynamicBoundedSequenceMut::new_proxy(
                                bytes,
                                *upper_bound,
                                sequence_upper_bound,
                                field_info.resize_function.unwrap()
                            )
                        ))
                    }
                    BaseType::Message(structure) => BoundedSequenceValue::MessageBoundedSequence($select!(
                        immutable => {
                            DynamicBoundedSequence::new_proxy(bytes, sequence_upper_bound, &**structure)
                        },
                        mutable => DynamicBoundedSequenceMut::new_proxy(
                            bytes,
                            &**structure,
                            sequence_upper_bound,
                            field_info.resize_function.unwrap(),
                        )
                    )),
                }
            }
        }
    };
}

mod immutable {
    use super::*;

    macro_rules! select_immutable {
        (immutable => $a:ty, mutable => $b:ty) => {
            $a
        };
        (immutable => $a:expr, mutable => $b:expr) => {
            $a
        };
    }

    define_value_types!(select_immutable);
}

pub use immutable::ArrayValue;
pub use immutable::BoundedSequenceValue;
pub use immutable::SequenceValue;
pub use immutable::SimpleValue;

/// The value of a field in a [`DynamicMessage`][1].
///
/// This type, and all the types inside it, are reference types – they contain
/// only a reference to the underlying data.
///
/// [1]: crate::dynamic_message::DynamicMessage
#[derive(Debug, PartialEq)]
pub enum Value<'msg> {
    /// A single value.
    Simple(SimpleValue<'msg>),
    /// An array of values.
    Array(ArrayValue<'msg>),
    /// A sequence of unbounded length.
    Sequence(SequenceValue<'msg>),
    /// A sequence of bounded length.
    BoundedSequence(BoundedSequenceValue<'msg>),
}
impl<'msg> Value<'msg> {
    pub(crate) unsafe fn new(
        value_bytes: &'msg [u8],
        field_info: &'msg MessageFieldInfo,
    ) -> Option<Value<'msg>> {
        Some(match field_info.value_kind {
            ValueKind::Simple => Value::Simple(SimpleValue::new(value_bytes, field_info)),
            ValueKind::Array { length } => {
                Value::Array(ArrayValue::new(value_bytes, field_info, length))
            }
            ValueKind::Sequence => Value::Sequence(SequenceValue::new(value_bytes, field_info)),
            ValueKind::BoundedSequence { upper_bound } => Value::BoundedSequence(
                BoundedSequenceValue::new(value_bytes, field_info, upper_bound),
            ),
        })
    }
}

mod mutable {
    use super::*;

    macro_rules! select_mutable {
        (immutable => $a:ty, mutable => $b:ty) => {
            $b
        };
        (immutable => $a:expr, mutable => $b:expr) => {
            $b
        };
    }

    define_value_types!(select_mutable);
}

pub use mutable::ArrayValue as ArrayValueMut;
pub use mutable::BoundedSequenceValue as BoundedSequenceValueMut;
pub use mutable::SequenceValue as SequenceValueMut;
pub use mutable::SimpleValue as SimpleValueMut;

/// The value of a field in a [`DynamicMessage`][1].
///
/// This type, and all the types inside it, are reference types – they contain
/// only a reference to the underlying data.
///
/// [1]: crate::dynamic_message::DynamicMessage
#[derive(Debug, PartialEq)]
pub enum ValueMut<'msg> {
    /// A single value.
    Simple(SimpleValueMut<'msg>),
    /// An array of values.
    Array(ArrayValueMut<'msg>),
    /// A sequence of unbounded length.
    Sequence(SequenceValueMut<'msg>),
    /// A sequence of bounded length.
    BoundedSequence(BoundedSequenceValueMut<'msg>),
}

impl<'msg> ValueMut<'msg> {
    pub(crate) unsafe fn new(
        value_bytes: &'msg mut [u8],
        field_info: &'msg MessageFieldInfo,
    ) -> ValueMut<'msg> {
        match field_info.value_kind {
            ValueKind::Simple => ValueMut::Simple(SimpleValueMut::new(value_bytes, field_info)),
            ValueKind::Array { length } => {
                ValueMut::Array(ArrayValueMut::new(value_bytes, field_info, length))
            }
            ValueKind::Sequence => {
                ValueMut::Sequence(SequenceValueMut::new(value_bytes, field_info))
            }
            ValueKind::BoundedSequence { upper_bound } => ValueMut::BoundedSequence(
                BoundedSequenceValueMut::new(value_bytes, field_info, upper_bound),
            ),
        }
    }
}
