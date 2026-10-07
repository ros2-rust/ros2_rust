// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

use rclrs::{
    BoundedSequenceValue, BoundedSequenceValueMut, DynamicBoundedSequence,
    DynamicBoundedSequenceMut, DynamicMessage, SequenceValue, SequenceValueMut, Value, ValueMut,
};
use rosidl_runtime_rs::Sequence;

#[test]
fn unbounded_primitive_fields_support_named_access() {
    let mut message =
        DynamicMessage::new("test_msgs/msg/UnboundedSequences".try_into().unwrap()).unwrap();
    let Some(ValueMut::Sequence(SequenceValueMut::Uint8Sequence(values))) =
        message.get_mut("uint8_values")
    else {
        panic!("expected a byte sequence");
    };
    let values: &mut Sequence<u8> = values;
    *values = vec![10, 20, 30].into();
    let Some(Value::Sequence(SequenceValue::Uint8Sequence(values))) = message.get("uint8_values")
    else {
        panic!("expected a byte sequence");
    };
    let values: &Sequence<u8> = values;
    assert_eq!(values.as_slice(), &[10, 20, 30]);
    assert!(message.get("string_values").is_some());
    assert!(message.get("basic_types_values").is_some());
}

#[test]
fn bounded_primitive_fields_preserve_contents_when_reset_exceeds_bound() {
    let mut message =
        DynamicMessage::new("test_msgs/msg/BoundedSequences".try_into().unwrap()).unwrap();
    let Some(ValueMut::BoundedSequence(BoundedSequenceValueMut::Uint8BoundedSequence(values))) =
        message.get_mut("uint8_values")
    else {
        panic!("expected a bounded byte sequence");
    };
    let mut values: DynamicBoundedSequenceMut<'_, u8> = values;
    values.try_reset(3).unwrap();
    values.as_mut_slice().copy_from_slice(&[10, 20, 30]);
    assert!(values.try_reset(4).is_err());
    let Some(Value::BoundedSequence(BoundedSequenceValue::Uint8BoundedSequence(values))) =
        message.get("uint8_values")
    else {
        panic!("expected a bounded byte sequence");
    };
    let values: DynamicBoundedSequence<'_, u8> = values;
    assert_eq!(values.as_slice(), &[10, 20, 30]);
    assert_eq!(
        std::panic::catch_unwind(|| values.as_slice().len()).unwrap(),
        3
    );
    assert_eq!(
        std::panic::catch_unwind(move || values.as_slice().len()).unwrap(),
        3
    );
    assert!(message.get("string_values").is_some());
    assert!(message.get("basic_types_values").is_some());
}

#[test]
fn sequence_messages_support_field_iteration_and_debug() {
    for name in ["UnboundedSequences", "BoundedSequences"] {
        let mut message =
            DynamicMessage::new(format!("test_msgs/msg/{name}").as_str().try_into().unwrap())
                .unwrap();
        let fields: Vec<_> = message
            .structure()
            .fields
            .iter()
            .map(|field| field.name.clone())
            .collect();
        for field in fields {
            assert!(message.get(&field).is_some(), "{name}.{field}");
            assert!(message.get_mut(&field).is_some(), "{name}.{field}");
        }
        assert_eq!(message.iter().count(), message.structure().fields.len());
        assert_eq!(message.iter_mut().count(), message.structure().fields.len());
        assert!(!format!("{:?}", message.view()).is_empty());
    }
}

#[cfg(feature = "rosidl-buffer")]
#[test]
fn bounded_opaque_fields_support_lookup_iteration_and_debug() {
    use ros_env::test_msgs;
    use rosidl_runtime_rs::{native, BoundedBuffer};

    let buffer = native::into_buffer(native::ffi::create_cpu(&[7, 8, 9]).unwrap()).unwrap();
    let buffer: BoundedBuffer<u8, 3> = buffer.try_into().unwrap();
    // Exercise the native view directly; generated bounded fields are CPU-only.
    let native = test_msgs::msg::rmw::BoundedSequences {
        uint8_values: buffer.into_sequence().into(),
        ..Default::default()
    };
    let mut message = DynamicMessage::convert_from_rmw_message(native).unwrap();
    let Some(Value::BoundedSequence(BoundedSequenceValue::Uint8BoundedSequence(values))) =
        message.get("uint8_values")
    else {
        panic!("expected a bounded byte sequence");
    };
    let values: DynamicBoundedSequence<'_, u8> = values;
    assert!(values.is_rosidl_buffer());
    assert_eq!(values.len(), 3);
    assert_eq!(values.upper_bound(), 3);
    assert_eq!(values.try_to_vec().unwrap(), [7, 8, 9]);
    assert!(std::panic::catch_unwind(|| values.as_slice()).is_err());
    assert_eq!(message.iter().count(), message.structure().fields.len());
    assert!(!format!("{:?}", message.view()).is_empty());
    let Some(ValueMut::BoundedSequence(BoundedSequenceValueMut::Uint8BoundedSequence(values))) =
        message.get_mut("uint8_values")
    else {
        panic!("expected a bounded byte sequence");
    };
    let mut values: DynamicBoundedSequenceMut<'_, u8> = values;
    assert_eq!(values.len(), 3);
    assert!(!format!("{values:?}").is_empty());
    assert!(values.try_reset(4).is_err());
    values.try_reset(2).unwrap();
    values.as_mut_slice().copy_from_slice(&[4, 6]);
    assert_eq!(values.as_slice(), [4, 6]);
}
