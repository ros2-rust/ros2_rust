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
        use rosidl_typesupport_introspection_c_field_types::*;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamic_message::*;
    use test_msgs::msg;

    use std::num::NonZeroUsize;

    #[test]
    fn max_alignment_is_8() {
        // The DynamicMessage type makes sure that its storage is aligned to 8
        let alignments = [
            std::mem::align_of::<msg::Builtins>(),
            std::mem::align_of::<msg::Arrays>(),
            std::mem::align_of::<msg::Empty>(),
            std::mem::align_of::<msg::Strings>(),
            std::mem::align_of::<msg::BoundedSequences>(),
            std::mem::align_of::<msg::Nested>(),
            std::mem::align_of::<msg::MultiNested>(),
            std::mem::align_of::<msg::UnboundedSequences>(),
            std::mem::align_of::<msg::WStrings>(),
            std::mem::align_of::<msg::Constants>(),
            std::mem::align_of::<msg::BasicTypes>(),
            std::mem::align_of::<msg::Defaults>(),
        ];
        assert_eq!(alignments.into_iter().max().unwrap(), 8);
    }

    #[test]
    fn message_structure_is_accurate() {
        let arrays_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/Arrays".try_into().unwrap()).unwrap();
        let arrays_structure = Box::new(arrays_metadata.structure().clone());
        let builtins_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/Builtins".try_into().unwrap()).unwrap();
        let builtins_structure = Box::new(builtins_metadata.structure().clone());
        let duration_metadata =
            DynamicMessageMetadata::new("builtin_interfaces/msg/Duration".try_into().unwrap())
                .unwrap();
        let duration_structure = Box::new(duration_metadata.structure().clone());
        let empty_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/Empty".try_into().unwrap()).unwrap();
        let empty_structure = Box::new(empty_metadata.structure().clone());
        let time_metadata =
            DynamicMessageMetadata::new("builtin_interfaces/msg/Time".try_into().unwrap()).unwrap();
        let time_structure = Box::new(time_metadata.structure().clone());
        let basic_types_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/BasicTypes".try_into().unwrap()).unwrap();
        let basic_types_structure = Box::new(basic_types_metadata.structure().clone());
        let bounded_sequences_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/BoundedSequences".try_into().unwrap())
                .unwrap();
        let bounded_sequences_structure = Box::new(bounded_sequences_metadata.structure().clone());
        let constants_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/Constants".try_into().unwrap()).unwrap();
        let constants_structure = Box::new(constants_metadata.structure().clone());
        let multi_nested_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/MultiNested".try_into().unwrap()).unwrap();
        let multi_nested_structure = Box::new(multi_nested_metadata.structure().clone());
        let nested_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/Nested".try_into().unwrap()).unwrap();
        let nested_structure = Box::new(nested_metadata.structure().clone());
        let defaults_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/Defaults".try_into().unwrap()).unwrap();
        let defaults_structure = Box::new(defaults_metadata.structure().clone());
        let strings_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/Strings".try_into().unwrap()).unwrap();
        let strings_structure = Box::new(strings_metadata.structure().clone());
        let wstrings_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/WStrings".try_into().unwrap()).unwrap();
        let wstrings_structure = Box::new(wstrings_metadata.structure().clone());
        let unbounded_sequences_metadata =
            DynamicMessageMetadata::new("test_msgs/msg/UnboundedSequences".try_into().unwrap())
                .unwrap();
        let unbounded_sequences_structure =
            Box::new(unbounded_sequences_metadata.structure().clone());

        let mut message_structures_and_fields = vec![];

        // --------------------- Arrays ---------------------

        let arrays_fields = vec![
            (
                "bool_values",
                BaseType::Boolean,
                ValueKind::Array { length: 3 },
            ),
            (
                "byte_values",
                BaseType::Octet,
                ValueKind::Array { length: 3 },
            ),
            (
                "char_values",
                BaseType::Uint8, // the msg to idl conversion converts char to uint8
                ValueKind::Array { length: 3 },
            ),
            (
                "float32_values",
                BaseType::Float,
                ValueKind::Array { length: 3 },
            ),
            (
                "float64_values",
                BaseType::Double,
                ValueKind::Array { length: 3 },
            ),
            (
                "int8_values",
                BaseType::Int8,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint8_values",
                BaseType::Uint8,
                ValueKind::Array { length: 3 },
            ),
            (
                "int16_values",
                BaseType::Int16,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint16_values",
                BaseType::Uint16,
                ValueKind::Array { length: 3 },
            ),
            (
                "int32_values",
                BaseType::Int32,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint32_values",
                BaseType::Uint32,
                ValueKind::Array { length: 3 },
            ),
            (
                "int64_values",
                BaseType::Int64,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint64_values",
                BaseType::Uint64,
                ValueKind::Array { length: 3 },
            ),
            (
                "string_values",
                BaseType::String,
                ValueKind::Array { length: 3 },
            ),
            (
                "basic_types_values",
                BaseType::Message(basic_types_structure.clone()),
                ValueKind::Array { length: 3 },
            ),
            (
                "constants_values",
                BaseType::Message(constants_structure.clone()),
                ValueKind::Array { length: 3 },
            ),
            (
                "defaults_values",
                BaseType::Message(defaults_structure.clone()),
                ValueKind::Array { length: 3 },
            ),
            (
                "bool_values_default",
                BaseType::Boolean,
                ValueKind::Array { length: 3 },
            ),
            (
                "byte_values_default",
                BaseType::Octet,
                ValueKind::Array { length: 3 },
            ),
            (
                "char_values_default",
                BaseType::Uint8, // the msg to idl conversion converts char to uint8
                ValueKind::Array { length: 3 },
            ),
            (
                "float32_values_default",
                BaseType::Float,
                ValueKind::Array { length: 3 },
            ),
            (
                "float64_values_default",
                BaseType::Double,
                ValueKind::Array { length: 3 },
            ),
            (
                "int8_values_default",
                BaseType::Int8,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint8_values_default",
                BaseType::Uint8,
                ValueKind::Array { length: 3 },
            ),
            (
                "int16_values_default",
                BaseType::Int16,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint16_values_default",
                BaseType::Uint16,
                ValueKind::Array { length: 3 },
            ),
            (
                "int32_values_default",
                BaseType::Int32,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint32_values_default",
                BaseType::Uint32,
                ValueKind::Array { length: 3 },
            ),
            (
                "int64_values_default",
                BaseType::Int64,
                ValueKind::Array { length: 3 },
            ),
            (
                "uint64_values_default",
                BaseType::Uint64,
                ValueKind::Array { length: 3 },
            ),
            (
                "string_values_default",
                BaseType::String,
                ValueKind::Array { length: 3 },
            ),
            ("alignment_check", BaseType::Int32, ValueKind::Simple),
        ];

        message_structures_and_fields.push(("Arrays", arrays_structure.clone(), arrays_fields));

        // --------------------- BasicTypes ---------------------

        let basic_types_fields = vec![
            ("bool_value", BaseType::Boolean, ValueKind::Simple),
            ("byte_value", BaseType::Octet, ValueKind::Simple),
            ("char_value", BaseType::Uint8, ValueKind::Simple),
            ("float32_value", BaseType::Float, ValueKind::Simple),
            ("float64_value", BaseType::Double, ValueKind::Simple),
            ("int8_value", BaseType::Int8, ValueKind::Simple),
            ("uint8_value", BaseType::Uint8, ValueKind::Simple),
            ("int16_value", BaseType::Int16, ValueKind::Simple),
            ("uint16_value", BaseType::Uint16, ValueKind::Simple),
            ("int32_value", BaseType::Int32, ValueKind::Simple),
            ("uint32_value", BaseType::Uint32, ValueKind::Simple),
            ("int64_value", BaseType::Int64, ValueKind::Simple),
            ("uint64_value", BaseType::Uint64, ValueKind::Simple),
        ];

        message_structures_and_fields.push((
            "BasicTypes",
            basic_types_structure.clone(),
            basic_types_fields,
        ));

        // --------------------- BoundedSequences ---------------------

        let bounded_sequences_fields = vec![
            (
                "bool_values",
                BaseType::Boolean,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "byte_values",
                BaseType::Octet,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "char_values",
                BaseType::Uint8, // the msg to idl conversion converts char to uint8
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "float32_values",
                BaseType::Float,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "float64_values",
                BaseType::Double,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int8_values",
                BaseType::Int8,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint8_values",
                BaseType::Uint8,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int16_values",
                BaseType::Int16,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint16_values",
                BaseType::Uint16,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int32_values",
                BaseType::Int32,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint32_values",
                BaseType::Uint32,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int64_values",
                BaseType::Int64,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint64_values",
                BaseType::Uint64,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "string_values",
                BaseType::String,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "basic_types_values",
                BaseType::Message(basic_types_structure.clone()),
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "constants_values",
                BaseType::Message(constants_structure.clone()),
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "defaults_values",
                BaseType::Message(defaults_structure.clone()),
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "bool_values_default",
                BaseType::Boolean,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "byte_values_default",
                BaseType::Octet,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "char_values_default",
                BaseType::Uint8, // the msg to idl conversion converts char to uint8
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "float32_values_default",
                BaseType::Float,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "float64_values_default",
                BaseType::Double,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int8_values_default",
                BaseType::Int8,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint8_values_default",
                BaseType::Uint8,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int16_values_default",
                BaseType::Int16,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint16_values_default",
                BaseType::Uint16,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int32_values_default",
                BaseType::Int32,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint32_values_default",
                BaseType::Uint32,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "int64_values_default",
                BaseType::Int64,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "uint64_values_default",
                BaseType::Uint64,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "string_values_default",
                BaseType::String,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            ("alignment_check", BaseType::Int32, ValueKind::Simple),
        ];

        message_structures_and_fields.push((
            "BoundedSequences",
            bounded_sequences_structure.clone(),
            bounded_sequences_fields,
        ));

        // --------------------- Builtins ---------------------

        let builtins_fields = vec![
            (
                "duration_value",
                BaseType::Message(duration_structure.clone()),
                ValueKind::Simple,
            ),
            (
                "time_value",
                BaseType::Message(time_structure.clone()),
                ValueKind::Simple,
            ),
        ];

        message_structures_and_fields.push((
            "Builtins",
            builtins_structure.clone(),
            builtins_fields,
        ));

        // --------------------- Constants ---------------------

        let constants_fields: Vec<(&'static str, BaseType, ValueKind)> = vec![(
            "structure_needs_at_least_one_member",
            BaseType::Uint8,
            ValueKind::Simple,
        )];

        message_structures_and_fields.push((
            "Constants",
            constants_structure.clone(),
            constants_fields,
        ));

        // --------------------- Defaults ---------------------

        let defaults_fields = vec![
            ("bool_value", BaseType::Boolean, ValueKind::Simple),
            ("byte_value", BaseType::Octet, ValueKind::Simple),
            // the msg to idl conversion converts char to uint8
            ("char_value", BaseType::Uint8, ValueKind::Simple),
            ("float32_value", BaseType::Float, ValueKind::Simple),
            ("float64_value", BaseType::Double, ValueKind::Simple),
            ("int8_value", BaseType::Int8, ValueKind::Simple),
            ("uint8_value", BaseType::Uint8, ValueKind::Simple),
            ("int16_value", BaseType::Int16, ValueKind::Simple),
            ("uint16_value", BaseType::Uint16, ValueKind::Simple),
            ("int32_value", BaseType::Int32, ValueKind::Simple),
            ("uint32_value", BaseType::Uint32, ValueKind::Simple),
            ("int64_value", BaseType::Int64, ValueKind::Simple),
            ("uint64_value", BaseType::Uint64, ValueKind::Simple),
        ];

        message_structures_and_fields.push((
            "Defaults",
            defaults_structure.clone(),
            defaults_fields,
        ));

        // --------------------- Empty ---------------------

        let empty_fields: Vec<(&'static str, BaseType, ValueKind)> = vec![(
            "structure_needs_at_least_one_member",
            BaseType::Uint8,
            ValueKind::Simple,
        )];

        message_structures_and_fields.push(("Empty", empty_structure.clone(), empty_fields));

        // --------------------- MultiNested ---------------------

        let multi_nested_fields = vec![
            (
                "array_of_arrays",
                BaseType::Message(arrays_structure.clone()),
                ValueKind::Array { length: 3 },
            ),
            (
                "array_of_bounded_sequences",
                BaseType::Message(bounded_sequences_structure.clone()),
                ValueKind::Array { length: 3 },
            ),
            (
                "array_of_unbounded_sequences",
                BaseType::Message(unbounded_sequences_structure.clone()),
                ValueKind::Array { length: 3 },
            ),
            (
                "bounded_sequence_of_arrays",
                BaseType::Message(arrays_structure.clone()),
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "bounded_sequence_of_bounded_sequences",
                BaseType::Message(bounded_sequences_structure.clone()),
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "bounded_sequence_of_unbounded_sequences",
                BaseType::Message(unbounded_sequences_structure.clone()),
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "unbounded_sequence_of_arrays",
                BaseType::Message(arrays_structure.clone()),
                ValueKind::Sequence,
            ),
            (
                "unbounded_sequence_of_bounded_sequences",
                BaseType::Message(bounded_sequences_structure.clone()),
                ValueKind::Sequence,
            ),
            (
                "unbounded_sequence_of_unbounded_sequences",
                BaseType::Message(unbounded_sequences_structure.clone()),
                ValueKind::Sequence,
            ),
        ];

        message_structures_and_fields.push((
            "MultiNested",
            multi_nested_structure.clone(),
            multi_nested_fields,
        ));

        // --------------------- Nested ---------------------

        let nested_fields = vec![(
            "basic_types_value",
            BaseType::Message(basic_types_structure.clone()),
            ValueKind::Simple,
        )];

        message_structures_and_fields.push(("Nested", nested_structure.clone(), nested_fields));

        // --------------------- Strings ---------------------

        let strings_fields = vec![
            ("string_value", BaseType::String, ValueKind::Simple),
            ("string_value_default1", BaseType::String, ValueKind::Simple),
            ("string_value_default2", BaseType::String, ValueKind::Simple),
            ("string_value_default3", BaseType::String, ValueKind::Simple),
            ("string_value_default4", BaseType::String, ValueKind::Simple),
            ("string_value_default5", BaseType::String, ValueKind::Simple),
            (
                "bounded_string_value",
                BaseType::BoundedString {
                    upper_bound: NonZeroUsize::new(22).unwrap(),
                },
                ValueKind::Simple,
            ),
            (
                "bounded_string_value_default1",
                BaseType::BoundedString {
                    upper_bound: NonZeroUsize::new(22).unwrap(),
                },
                ValueKind::Simple,
            ),
            (
                "bounded_string_value_default2",
                BaseType::BoundedString {
                    upper_bound: NonZeroUsize::new(22).unwrap(),
                },
                ValueKind::Simple,
            ),
            (
                "bounded_string_value_default3",
                BaseType::BoundedString {
                    upper_bound: NonZeroUsize::new(22).unwrap(),
                },
                ValueKind::Simple,
            ),
            (
                "bounded_string_value_default4",
                BaseType::BoundedString {
                    upper_bound: NonZeroUsize::new(22).unwrap(),
                },
                ValueKind::Simple,
            ),
            (
                "bounded_string_value_default5",
                BaseType::BoundedString {
                    upper_bound: NonZeroUsize::new(22).unwrap(),
                },
                ValueKind::Simple,
            ),
        ];

        message_structures_and_fields.push(("Strings", strings_structure.clone(), strings_fields));

        // --------------------- UnboundedSequences ---------------------

        let unbounded_sequences_fields = vec![
            ("bool_values", BaseType::Boolean, ValueKind::Sequence),
            ("byte_values", BaseType::Octet, ValueKind::Sequence),
            // the msg to idl conversion converts char to uint8
            ("char_values", BaseType::Uint8, ValueKind::Sequence),
            ("float32_values", BaseType::Float, ValueKind::Sequence),
            ("float64_values", BaseType::Double, ValueKind::Sequence),
            ("int8_values", BaseType::Int8, ValueKind::Sequence),
            ("uint8_values", BaseType::Uint8, ValueKind::Sequence),
            ("int16_values", BaseType::Int16, ValueKind::Sequence),
            ("uint16_values", BaseType::Uint16, ValueKind::Sequence),
            ("int32_values", BaseType::Int32, ValueKind::Sequence),
            ("uint32_values", BaseType::Uint32, ValueKind::Sequence),
            ("int64_values", BaseType::Int64, ValueKind::Sequence),
            ("uint64_values", BaseType::Uint64, ValueKind::Sequence),
            ("string_values", BaseType::String, ValueKind::Sequence),
            (
                "basic_types_values",
                BaseType::Message(basic_types_structure.clone()),
                ValueKind::Sequence,
            ),
            (
                "constants_values",
                BaseType::Message(constants_structure.clone()),
                ValueKind::Sequence,
            ),
            (
                "defaults_values",
                BaseType::Message(defaults_structure.clone()),
                ValueKind::Sequence,
            ),
            (
                "bool_values_default",
                BaseType::Boolean,
                ValueKind::Sequence,
            ),
            ("byte_values_default", BaseType::Octet, ValueKind::Sequence),
            // the msg to idl conversion converts char to uint8
            ("char_values_default", BaseType::Uint8, ValueKind::Sequence),
            (
                "float32_values_default",
                BaseType::Float,
                ValueKind::Sequence,
            ),
            (
                "float64_values_default",
                BaseType::Double,
                ValueKind::Sequence,
            ),
            ("int8_values_default", BaseType::Int8, ValueKind::Sequence),
            ("uint8_values_default", BaseType::Uint8, ValueKind::Sequence),
            ("int16_values_default", BaseType::Int16, ValueKind::Sequence),
            (
                "uint16_values_default",
                BaseType::Uint16,
                ValueKind::Sequence,
            ),
            ("int32_values_default", BaseType::Int32, ValueKind::Sequence),
            (
                "uint32_values_default",
                BaseType::Uint32,
                ValueKind::Sequence,
            ),
            ("int64_values_default", BaseType::Int64, ValueKind::Sequence),
            (
                "uint64_values_default",
                BaseType::Uint64,
                ValueKind::Sequence,
            ),
            (
                "string_values_default",
                BaseType::String,
                ValueKind::Sequence,
            ),
            ("alignment_check", BaseType::Int32, ValueKind::Simple),
        ];

        message_structures_and_fields.push((
            "UnboundedSequences",
            unbounded_sequences_structure.clone(),
            unbounded_sequences_fields,
        ));

        // --------------------- WStrings ---------------------

        let wstrings_fields = vec![
            ("wstring_value", BaseType::WString, ValueKind::Simple),
            (
                "wstring_value_default1",
                BaseType::WString,
                ValueKind::Simple,
            ),
            (
                "wstring_value_default2",
                BaseType::WString,
                ValueKind::Simple,
            ),
            (
                "wstring_value_default3",
                BaseType::WString,
                ValueKind::Simple,
            ),
            (
                "array_of_wstrings",
                BaseType::WString,
                ValueKind::Array { length: 3 },
            ),
            (
                "bounded_sequence_of_wstrings",
                BaseType::WString,
                ValueKind::BoundedSequence { upper_bound: 3 },
            ),
            (
                "unbounded_sequence_of_wstrings",
                BaseType::WString,
                ValueKind::Sequence,
            ),
        ];

        message_structures_and_fields.push((
            "WStrings",
            wstrings_structure.clone(),
            wstrings_fields,
        ));

        // --------------------- Running the tests ---------------------

        for (message_name, structure, fields) in message_structures_and_fields {
            assert_eq!(
                structure
                    .fields
                    .iter()
                    .map(|field_info| field_info.name.to_owned())
                    .collect::<Vec<_>>(),
                fields
                    .iter()
                    .map(|pair| pair.0.to_owned())
                    .collect::<Vec<_>>(),
                "in message '{}'",
                message_name
            );

            for (field_name, expected_base_type, expected_value_kind) in fields {
                let field_type = structure.get_field_info(field_name).unwrap();
                assert_eq!(
                    field_type.base_type, expected_base_type,
                    "for field '{}' in message '{}'",
                    field_name, message_name
                );
                assert_eq!(
                    field_type.value_kind, expected_value_kind,
                    "for field '{}' in message '{}'",
                    field_name, message_name,
                );
            }
        }
    }

    #[test]
    fn dynamic_message_has_defaults() {
        let dyn_msg = DynamicMessage::new("test_msgs/msg/Defaults".try_into().unwrap()).unwrap();
        assert_eq!(
            dyn_msg.get("bool_value"),
            Some(Value::Simple(SimpleValue::Boolean(&true)))
        );
        assert_eq!(
            dyn_msg.get("byte_value"),
            Some(Value::Simple(SimpleValue::Octet(&50u8)))
        );
        assert_eq!(
            dyn_msg.get("char_value"),
            Some(Value::Simple(SimpleValue::Uint8(&100u8)))
        );
        assert_eq!(
            dyn_msg.get("float32_value"),
            Some(Value::Simple(SimpleValue::Float(&1.125f32)))
        );
        assert_eq!(
            dyn_msg.get("float64_value"),
            Some(Value::Simple(SimpleValue::Double(&1.125f64)))
        );
        assert_eq!(
            dyn_msg.get("int8_value"),
            Some(Value::Simple(SimpleValue::Int8(&-50i8)))
        );
        assert_eq!(
            dyn_msg.get("uint8_value"),
            Some(Value::Simple(SimpleValue::Uint8(&200u8)))
        );
        assert_eq!(
            dyn_msg.get("int16_value"),
            Some(Value::Simple(SimpleValue::Int16(&-1000i16)))
        );
        assert_eq!(
            dyn_msg.get("uint16_value"),
            Some(Value::Simple(SimpleValue::Uint16(&2000u16)))
        );
        assert_eq!(
            dyn_msg.get("int32_value"),
            Some(Value::Simple(SimpleValue::Int32(&-30000i32)))
        );
        assert_eq!(
            dyn_msg.get("uint32_value"),
            Some(Value::Simple(SimpleValue::Uint32(&60000u32)))
        );
        assert_eq!(
            dyn_msg.get("int64_value"),
            Some(Value::Simple(SimpleValue::Int64(&-40000000i64)))
        );
        assert_eq!(
            dyn_msg.get("uint64_value"),
            Some(Value::Simple(SimpleValue::Uint64(&50000000u64)))
        );

        let _dyn_msg = DynamicMessage::new("test_msgs/msg/Arrays".try_into().unwrap()).unwrap();
        let _dyn_msg =
            DynamicMessage::new("test_msgs/msg/UnboundedSequences".try_into().unwrap()).unwrap();
        let _dyn_msg =
            DynamicMessage::new("test_msgs/msg/BoundedSequences".try_into().unwrap()).unwrap();
    }
}
