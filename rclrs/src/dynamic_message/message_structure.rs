use std::ffi::CStr;
use std::num::NonZeroUsize;

use super::TypeErasedSequence;
use crate::rcl_bindings::rosidl_typesupport_introspection_c__MessageMember as rosidl_message_member_t;
use crate::rcl_bindings::rosidl_typesupport_introspection_c__MessageMembers as rosidl_message_members_t;
use crate::rcl_bindings::*;

/// Possible base types for fields in a message.
// The field variants are self-explaining, no need to add redundant documentation.
#[allow(missing_docs)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BaseType {
    /// AKA `float32` in ROS .msg files.
    Float,
    /// AKA `float64` in ROS .msg files.
    Double,
    LongDouble,
    Char,
    WChar,
    Boolean,
    /// AKA `byte` in ROS .msg files.
    Octet,
    /// AKA `char` in ROS .msg files
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Uint64,
    Int64,
    String,
    BoundedString {
        upper_bound: NonZeroUsize,
    },
    WString,
    BoundedWString {
        upper_bound: NonZeroUsize,
    },
    Message(Box<MessageStructure>),
}

/// A description of a single field in a [`DynamicMessage`][1].
///
/// The concrete type of a field is the combination of its [`BaseType`] with its [`ValueKind`].
/// That is, the base types exist as single values, arrays, bounded sequences and unbounded sequences.
///
/// [1]: crate::dynamic_message::DynamicMessage
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MessageFieldInfo {
    /// The field name.
    pub name: String,
    /// The base type – number, string, etc.
    pub base_type: BaseType,
    /// Whether the field is a simple value, an array, or a (bounded) sequence.
    pub value_kind: ValueKind,
    pub(crate) string_upper_bound: usize,
    pub(crate) resize_function:
        Option<unsafe extern "C" fn(arg1: *mut std::os::raw::c_void, size: usize) -> bool>,
    pub(crate) offset: usize,
}

/// A description of the structure of a message.
///
/// Namely, the list of fields and their types.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MessageStructure {
    /// The set of fields in the message, ordered by their offset in the message.
    ///
    /// A `Vec` is easier to handle and faster than a `HashMap` for typical numbers of message fields.
    /// If you need a `HashMap`, simply create your own from this `Vec`.
    pub fields: Vec<MessageFieldInfo>,
    /// The size of this structure in bytes.
    pub size: usize,
    /// The namespace of this type. This is something like `geometry_msgs__msg`.
    pub namespace: String,
    /// The name of this type. This does not contain the package name.
    pub type_name: String,
}

/// Information on whether a field is a single value or a list of some kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValueKind {
    /// This field is a single value, which includes string values.
    Simple,
    /// This field is an array of values.
    Array {
        /// The array length.
        length: usize,
    },
    /// This field is a [`Sequence`][1] of values.
    ///
    /// [1]: rosidl_runtime_rs::Sequence
    Sequence,
    /// This field is a [`BoundedSequence`][1] of values.
    ///
    /// [1]: rosidl_runtime_rs::BoundedSequence
    BoundedSequence {
        /// The maximum sequence length.
        upper_bound: usize,
    },
}

// ========================= impl for BaseType =========================

impl BaseType {
    // The inner message type support will be nullptr except for the case of a nested message.
    // That function must be unsafe, since it is possible to safely create a garbage non-null
    // pointer.
    unsafe fn new(
        type_id: u8,
        string_upper_bound: Option<NonZeroUsize>,
        inner: *const rosidl_message_type_support_t,
    ) -> Self {
        match u32::from(type_id) {
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT as u32 => Self::Float,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE as u32 => Self::Double,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE as u32 => {
                Self::LongDouble
            }
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_CHAR as u32 => Self::Char,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR as u32 => Self::WChar,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN as u32 => Self::Boolean,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_OCTET as u32 => Self::Octet,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 as u32 => Self::Uint8,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_INT8 as u32 => Self::Int8,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 as u32 => Self::Uint16,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_INT16 as u32 => Self::Int16,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 as u32 => Self::Uint32,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_INT32 as u32 => Self::Int32,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 as u32 => Self::Uint64,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_INT64 as u32 => Self::Int64,
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_STRING as u32 => {
                match string_upper_bound {
                    None => Self::String,
                    Some(upper_bound) => Self::BoundedString { upper_bound },
                }
            }
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING as u32 => {
                match string_upper_bound {
                    None => Self::WString,
                    Some(upper_bound) => Self::BoundedWString { upper_bound },
                }
            }
            x if x == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE as u32 => {
                assert!(!inner.is_null());
                let type_support: &rosidl_message_type_support_t = &*inner;
                let message_members: &rosidl_message_members_t =
                    // SAFETY: The data pointer is supposed to be always valid.
                    &*(type_support.data as *const rosidl_message_members_t);
                let structure = MessageStructure::from_rosidl_message_members(message_members);
                Self::Message(Box::new(structure))
            }
            _ => panic!("Invalid field type"),
        }
    }

    /// Returns the size of a single element of this base type.
    ///
    /// None is returned for LongDouble, which is of platform-dependent size.
    pub(crate) fn size(&self) -> Option<usize> {
        match self {
            BaseType::Float => Some(4),
            BaseType::Double => Some(8),
            BaseType::LongDouble => None,
            BaseType::Char => Some(1),
            BaseType::WChar => Some(2),
            BaseType::Boolean => Some(1),
            BaseType::Octet => Some(1),
            BaseType::Uint8 => Some(1),
            BaseType::Int8 => Some(1),
            BaseType::Uint16 => Some(2),
            BaseType::Int16 => Some(2),
            BaseType::Uint32 => Some(4),
            BaseType::Int32 => Some(4),
            BaseType::Uint64 => Some(8),
            BaseType::Int64 => Some(8),
            BaseType::String => Some(std::mem::size_of::<rosidl_runtime_rs::String>()),
            BaseType::BoundedString { .. } => {
                Some(std::mem::size_of::<rosidl_runtime_rs::String>())
            }
            BaseType::WString => Some(std::mem::size_of::<rosidl_runtime_rs::WString>()),
            BaseType::BoundedWString { .. } => {
                Some(std::mem::size_of::<rosidl_runtime_rs::WString>())
            }
            BaseType::Message(structure) => Some(structure.size),
        }
    }
}

// ========================= impl for MessageFieldInfo =========================

impl MessageFieldInfo {
    // That function must be unsafe, since it is possible to safely create a garbage non-null
    // pointer and store it in a rosidl_message_member_t.
    unsafe fn from(rosidl_message_member: &rosidl_message_member_t) -> Self {
        debug_assert!(!rosidl_message_member.name_.is_null());
        let name = /*unsafe*/ { CStr::from_ptr(rosidl_message_member.name_) }
                    .to_string_lossy()
                    .into_owned();
        let value_kind = match (
            rosidl_message_member.is_array_,
            rosidl_message_member.resize_function.is_some(),
            rosidl_message_member.is_upper_bound_,
        ) {
            (false, _, _) => ValueKind::Simple,
            (true, false, _) => ValueKind::Array {
                length: rosidl_message_member.array_size_,
            },
            (true, true, false) => {
                assert_eq!(rosidl_message_member.array_size_, 0);
                ValueKind::Sequence
            }
            (true, true, true) => ValueKind::BoundedSequence {
                upper_bound: rosidl_message_member.array_size_,
            },
        };
        Self {
            name,
            base_type: BaseType::new(
                rosidl_message_member.type_id_,
                NonZeroUsize::new(rosidl_message_member.string_upper_bound_),
                rosidl_message_member.members_,
            ),
            value_kind,
            string_upper_bound: rosidl_message_member.string_upper_bound_,
            resize_function: rosidl_message_member.resize_function,
            offset: usize::try_from(rosidl_message_member.offset_).unwrap(),
        }
    }
}

impl MessageFieldInfo {
    /// Returns the size of the field in the message.
    ///
    /// For sequences, it's the size of the sequence struct (ptr + size + capacity),
    /// not the size that the elements take up in memory.
    pub(crate) fn size(&self) -> Option<usize> {
        match self.value_kind {
            ValueKind::Simple => self.base_type.size(),
            ValueKind::Array { length } => self.base_type.size().map(|size| length * size),
            ValueKind::Sequence | ValueKind::BoundedSequence { .. } => {
                Some(std::mem::size_of::<TypeErasedSequence>())
            }
        }
    }
}

// ========================= impl for MessageStructure =========================

impl MessageStructure {
    /// Parses the C struct containing a list of fields.
    // That function must be unsafe, since it is possible to safely create a garbage non-null
    // pointer and store it in a rosidl_message_members_t.
    pub(crate) unsafe fn from_rosidl_message_members(
        message_members: &rosidl_message_members_t,
    ) -> Self {
        debug_assert!(!message_members.members_.is_null());
        let num_fields: usize = usize::try_from(message_members.member_count_).unwrap();
        let mut fields: Vec<_> = (0..num_fields)
            .map(|i| {
                // SAFETY: This is an array as per the documentation
                let rosidl_message_member: &rosidl_message_member_t =
                    /*unsafe*/ { &*message_members.members_.add(i) };
                // SAFETY: This is a valid string pointer
                MessageFieldInfo::from(rosidl_message_member)
            })
            .collect();
        fields.sort_by_key(|field_info| field_info.offset);
        // SAFETY: Immediate conversion into owned string.
        let namespace = /*unsafe*/ {
            CStr::from_ptr(message_members.message_namespace_)
                .to_string_lossy()
                .into_owned()
        };
        // SAFETY: Immediate conversion into owned string.
        let type_name = /*unsafe*/ {
            CStr::from_ptr(message_members.message_name_)
                .to_string_lossy()
                .into_owned()
        };
        Self {
            fields,
            size: message_members.size_of_,
            namespace,
            type_name,
        }
    }

    /// Gets the field info corresponding to the specified field name, if any.
    pub fn get_field_info(&self, field_name: &str) -> Option<&MessageFieldInfo> {
        self.fields.iter().find(|field| field.name == field_name)
    }
}
