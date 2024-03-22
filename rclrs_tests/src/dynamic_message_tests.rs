use std::num::NonZeroUsize;

use rclrs::dynamic_message::*;

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
    let arrays_metadata = DynamicMessageMetadata::new("test_msgs/msg/Arrays").unwrap();
    let arrays_structure = Box::new(arrays_metadata.structure().clone());
    let builtins_metadata = DynamicMessageMetadata::new("test_msgs/msg/Builtins").unwrap();
    let builtins_structure = Box::new(builtins_metadata.structure().clone());
    let duration_metadata = DynamicMessageMetadata::new("builtin_interfaces/msg/Duration").unwrap();
    let duration_structure = Box::new(duration_metadata.structure().clone());
    let empty_metadata = DynamicMessageMetadata::new("test_msgs/msg/Empty").unwrap();
    let empty_structure = Box::new(empty_metadata.structure().clone());
    let time_metadata = DynamicMessageMetadata::new("builtin_interfaces/msg/Time").unwrap();
    let time_structure = Box::new(time_metadata.structure().clone());
    let basic_types_metadata = DynamicMessageMetadata::new("test_msgs/msg/BasicTypes").unwrap();
    let basic_types_structure = Box::new(basic_types_metadata.structure().clone());
    let bounded_sequences_metadata =
        DynamicMessageMetadata::new("test_msgs/msg/BoundedSequences").unwrap();
    let bounded_sequences_structure = Box::new(bounded_sequences_metadata.structure().clone());
    let constants_metadata = DynamicMessageMetadata::new("test_msgs/msg/Constants").unwrap();
    let constants_structure = Box::new(constants_metadata.structure().clone());
    let multi_nested_metadata = DynamicMessageMetadata::new("test_msgs/msg/MultiNested").unwrap();
    let multi_nested_structure = Box::new(multi_nested_metadata.structure().clone());
    let nested_metadata = DynamicMessageMetadata::new("test_msgs/msg/Nested").unwrap();
    let nested_structure = Box::new(nested_metadata.structure().clone());
    let defaults_metadata = DynamicMessageMetadata::new("test_msgs/msg/Defaults").unwrap();
    let defaults_structure = Box::new(defaults_metadata.structure().clone());
    let strings_metadata = DynamicMessageMetadata::new("test_msgs/msg/Strings").unwrap();
    let strings_structure = Box::new(strings_metadata.structure().clone());
    let wstrings_metadata = DynamicMessageMetadata::new("test_msgs/msg/WStrings").unwrap();
    let wstrings_structure = Box::new(wstrings_metadata.structure().clone());
    let unbounded_sequences_metadata =
        DynamicMessageMetadata::new("test_msgs/msg/UnboundedSequences").unwrap();
    let unbounded_sequences_structure = Box::new(unbounded_sequences_metadata.structure().clone());

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

    message_structures_and_fields.push(("Builtins", builtins_structure.clone(), builtins_fields));

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

    message_structures_and_fields.push(("Defaults", defaults_structure.clone(), defaults_fields));

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

    message_structures_and_fields.push(("WStrings", wstrings_structure.clone(), wstrings_fields));

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

    // Explicitly drop to avoid clippy warnings
    drop(arrays_structure);
    drop(builtins_structure);
    drop(duration_structure);
    drop(empty_structure);
    drop(time_structure);
    drop(basic_types_structure);
    drop(bounded_sequences_structure);
    drop(constants_structure);
    drop(multi_nested_structure);
    drop(nested_structure);
    drop(defaults_structure);
    drop(strings_structure);
    drop(wstrings_structure);
    drop(unbounded_sequences_structure);
}
