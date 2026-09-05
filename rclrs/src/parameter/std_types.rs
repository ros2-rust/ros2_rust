//! [`ParameterVariant`] implementations for common Rust types.
//!
//! ROS 2 has nine parameter types, and [`value`](super::value) implements [`ParameterVariant`]
//! for the Rust type that represents each of them most directly, such as `i64`, `f64`,
//! `Arc<str>` and `Arc<[i64]>`. Those are the right types for the parameter machinery, but they
//! are not always the types an application would otherwise use.
//!
//! This module widens the set of types a parameter can be declared with, so that a `Vec<String>`
//! or a `u16` can be used where the ROS 2 representation happens to be a string array or an
//! integer. Conversions in this direction can be partial: not every `i64` fits a `u16`. Values
//! that do not fit are rejected by the type's [`TryFrom`] before they are ever stored, so a
//! parameter declared as `u16` can never hold something that is not one.
//!
//! Every scalar type here has a `Vec` form, which uses whichever ROS 2 array type holds the
//! scalar's representation. A `Vec<u16>` is therefore an integer array whose elements are each
//! checked against the range of a `u16`, and the rejection message says which element was at
//! fault. The one exception is `Vec<u8>`, which is a ROS 2 *byte* array rather than an integer
//! array, since that is what ROS 2 has a byte array for.
//!
//! # Unsupported integer types
//!
//! `u64`, `usize`, `i128` and `u128` are deliberately absent. A parameter's value has to be
//! representable in ROS 2, whose integer type is `i64`, and every way of handling a value above
//! `i64::MAX` silently produces a number the application did not ask for, whether by saturating,
//! by wrapping, or by panicking inside an infallible `From`. Use `i64`, or `u32` when the value
//! must be unsigned and small.

use std::{
    ops::{Range, RangeTo},
    path::PathBuf,
    sync::Arc,
};

use crate::{
    parameter::{ParameterRange, ParameterRanges},
    ParameterKind, ParameterValue, ParameterValueError, ParameterVariant,
};

/// Implements the three traits needed to declare a parameter as `$t`, where `$t` maps onto the
/// ROS 2 integer type but holds a narrower range of values.
macro_rules! impl_narrow_integer {
    ($($t:ty),* $(,)?) => { $(
        impl From<$t> for ParameterValue {
            fn from(value: $t) -> Self {
                ParameterValue::Integer(value.into())
            }
        }

        impl TryFrom<ParameterValue> for $t {
            type Error = ParameterValueError;

            fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
                match value {
                    ParameterValue::Integer(v) => <$t>::try_from(v).map_err(|_| {
                        ParameterValueError::Invalid(format!(
                            "{v} is out of range for {}, which accepts {}..={}",
                            stringify!($t),
                            <$t>::MIN,
                            <$t>::MAX,
                        ))
                    }),
                    _ => Err(ParameterValueError::TypeMismatch),
                }
            }
        }

        // Ranges are written in the field's own type, so `range = 1024..=65535` on a `u16`
        // means what it appears to. `ParameterRanges` stores the ROS 2 representation.
        //
        // A bound the declaration leaves open is still bounded by the type, so it is filled in
        // from the type's own limits. That puts the narrowing into the descriptor's
        // `IntegerRange`, where `ros2 param describe` and rqt_reconfigure can read it, instead
        // of leaving them to discover it by having a value rejected.
        impl From<ParameterRange<$t>> for ParameterRanges {
            fn from(range: ParameterRange<$t>) -> Self {
                ParameterRange::<i64> {
                    lower: Some(range.lower.map_or(i64::from(<$t>::MIN), i64::from)),
                    upper: Some(range.upper.map_or(i64::from(<$t>::MAX), i64::from)),
                    step: range.step.map(i64::from),
                }
                .into()
            }
        }

        // An exclusive end names the value one past the last the range admits, which for an
        // integer is a value the type can hold. The generic conversions cover the inclusive and
        // open forms for every parameter type, so only these two are per type.
        impl From<Range<$t>> for ParameterRange<$t> {
            fn from(range: Range<$t>) -> Self {
                Self {
                    lower: Some(range.start),
                    upper: Some(range.end.saturating_sub(1)),
                    step: None,
                }
            }
        }

        impl From<RangeTo<$t>> for ParameterRange<$t> {
            fn from(range: RangeTo<$t>) -> Self {
                Self {
                    lower: None,
                    upper: Some(range.end.saturating_sub(1)),
                    step: None,
                }
            }
        }

        impl ParameterVariant for $t {
            type Range = ParameterRange<$t>;

            fn kind() -> ParameterKind {
                ParameterKind::Integer
            }

            fn type_constraints() -> Option<Arc<str>> {
                Some(format!("{}..={}", <$t>::MIN, <$t>::MAX).into())
            }
        }
    )* };
}

impl_narrow_integer!(i8, i16, i32, u8, u16, u32);

impl From<f32> for ParameterValue {
    fn from(value: f32) -> Self {
        ParameterValue::Double(value.into())
    }
}

impl TryFrom<ParameterValue> for f32 {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::Double(v) => {
                let narrowed = v as f32;
                // Narrowing is lossy in the mantissa, which is inherent to asking for an `f32`,
                // but a finite value must not silently become an infinity.
                if narrowed.is_finite() || !v.is_finite() {
                    Ok(narrowed)
                } else {
                    Err(ParameterValueError::Invalid(format!(
                        "{v} is out of range for f32"
                    )))
                }
            }
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl From<ParameterRange<f32>> for ParameterRanges {
    fn from(range: ParameterRange<f32>) -> Self {
        ParameterRange::<f64> {
            lower: range.lower.map(f64::from),
            upper: range.upper.map(f64::from),
            step: range.step.map(f64::from),
        }
        .into()
    }
}

/// An exclusive end names the largest `f32` below it, as it does for the other numeric types.
impl From<Range<f32>> for ParameterRange<f32> {
    fn from(range: Range<f32>) -> Self {
        Self {
            lower: Some(range.start),
            upper: Some(next_down_f32(range.end)),
            step: None,
        }
    }
}

impl From<RangeTo<f32>> for ParameterRange<f32> {
    fn from(range: RangeTo<f32>) -> Self {
        Self {
            lower: None,
            upper: Some(next_down_f32(range.end)),
            step: None,
        }
    }
}

/// The largest `f32` strictly below `value`, for the same reason the `f64` version exists.
fn next_down_f32(value: f32) -> f32 {
    if value.is_nan() || value == f32::NEG_INFINITY {
        return value;
    }
    if value == 0.0 {
        return -f32::from_bits(1);
    }
    if value > 0.0 {
        f32::from_bits(value.to_bits() - 1)
    } else {
        f32::from_bits(value.to_bits() + 1)
    }
}

impl ParameterVariant for f32 {
    type Range = ParameterRange<f32>;

    fn kind() -> ParameterKind {
        ParameterKind::Double
    }
}

impl From<String> for ParameterValue {
    fn from(value: String) -> Self {
        ParameterValue::String(value.into())
    }
}

impl TryFrom<ParameterValue> for String {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::String(v) => Ok(v.to_string()),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for String {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::String
    }
}

/// Note that a path which is not valid UTF-8 cannot be represented as a ROS 2 string and is
/// converted lossily, since ROS 2 parameter strings are UTF-8.
impl From<PathBuf> for ParameterValue {
    fn from(value: PathBuf) -> Self {
        ParameterValue::String(value.to_string_lossy().as_ref().into())
    }
}

impl TryFrom<ParameterValue> for PathBuf {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::String(v) => Ok(PathBuf::from(v.as_ref())),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for PathBuf {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::String
    }
}

/// Implements the traits needed to declare a parameter as `Vec<$item>`, for the array kinds
/// whose ROS 2 representation is `Arc<[$item]>`.
macro_rules! impl_vec_of {
    ($(($item:ty, $variant:ident, $kind:ident)),* $(,)?) => { $(
        impl From<Vec<$item>> for ParameterValue {
            fn from(value: Vec<$item>) -> Self {
                ParameterValue::$variant(value.into())
            }
        }

        impl TryFrom<ParameterValue> for Vec<$item> {
            type Error = ParameterValueError;

            fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
                match value {
                    ParameterValue::$variant(v) => Ok(v.to_vec()),
                    _ => Err(ParameterValueError::TypeMismatch),
                }
            }
        }

        impl ParameterVariant for Vec<$item> {
            type Range = ();

            fn kind() -> ParameterKind {
                ParameterKind::$kind
            }
        }
    )* };
}

impl_vec_of!(
    (u8, ByteArray, ByteArray),
    (bool, BoolArray, BoolArray),
    (i64, IntegerArray, IntegerArray),
    (f64, DoubleArray, DoubleArray),
);

/// Reports which element of an array failed to convert, since "out of range for u16" is not much
/// help on its own when the array has forty entries.
fn element_error(index: usize, error: ParameterValueError) -> ParameterValueError {
    let reason = match error {
        ParameterValueError::Invalid(reason) => reason,
        other => other.to_string(),
    };
    ParameterValueError::Invalid(format!("element {index}: {reason}"))
}

/// Implements the traits needed to declare a parameter as `Vec<$item>`, for an item type whose
/// ROS 2 representation is `$scalar` and which therefore lives in the `$array` array type.
///
/// Reading an element back goes through the item type's own [`TryFrom`], so whatever an element of
/// that type may be is decided in exactly one place, and the error says which element was at fault.
macro_rules! impl_vec_of_convertible {
    ($(($item:ty, $array:ident, $scalar:ident, $kind:ident, $to_element:expr)),* $(,)?) => { $(
        impl From<Vec<$item>> for ParameterValue {
            fn from(value: Vec<$item>) -> Self {
                ParameterValue::$array(value.into_iter().map($to_element).collect())
            }
        }

        impl TryFrom<ParameterValue> for Vec<$item> {
            type Error = ParameterValueError;

            fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
                match value {
                    ParameterValue::$array(items) => items
                        .iter()
                        .enumerate()
                        .map(|(index, item)| {
                            <$item>::try_from(ParameterValue::$scalar(item.clone()))
                                .map_err(|error| element_error(index, error))
                        })
                        .collect(),
                    _ => Err(ParameterValueError::TypeMismatch),
                }
            }
        }

        impl ParameterVariant for Vec<$item> {
            type Range = ();

            fn kind() -> ParameterKind {
                ParameterKind::$kind
            }

            fn type_constraints() -> Option<Arc<str>> {
                <$item as ParameterVariant>::type_constraints()
                    .map(|constraints| format!("every element {constraints}").into())
            }
        }
    )* };
}

// `Vec<u8>` is deliberately absent: ROS 2 has a byte array type, so a sequence of bytes is that
// rather than an integer array. It is implemented above, alongside the other array types that need
// no element conversion.
impl_vec_of_convertible!(
    (i8, IntegerArray, Integer, IntegerArray, i64::from),
    (i16, IntegerArray, Integer, IntegerArray, i64::from),
    (i32, IntegerArray, Integer, IntegerArray, i64::from),
    (u16, IntegerArray, Integer, IntegerArray, i64::from),
    (u32, IntegerArray, Integer, IntegerArray, i64::from),
    (f32, DoubleArray, Double, DoubleArray, f64::from),
    (
        PathBuf,
        StringArray,
        String,
        StringArray,
        |path: PathBuf| { Arc::from(path.to_string_lossy().as_ref()) }
    ),
);

impl From<Vec<String>> for ParameterValue {
    fn from(value: Vec<String>) -> Self {
        ParameterValue::StringArray(value.into_iter().map(Arc::from).collect())
    }
}

impl TryFrom<ParameterValue> for Vec<String> {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::StringArray(v) => Ok(v.iter().map(|s| s.to_string()).collect()),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Vec<String> {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::StringArray
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::*;

    /// Asserts that a value survives a round trip through `ParameterValue`.
    fn round_trip<T>(value: T)
    where
        T: ParameterVariant + PartialEq + std::fmt::Debug,
    {
        let stored: ParameterValue = value.clone().into();
        let Ok(recovered) = T::try_from(stored) else {
            panic!("{value:?} did not convert back from its ParameterValue");
        };
        assert_eq!(recovered, value);
    }

    #[test]
    fn test_round_trips() {
        round_trip(String::from("hello"));
        round_trip(PathBuf::from("/dev/ttyUSB0"));

        round_trip(vec![String::from("a"), String::from("b")]);
        round_trip(vec![PathBuf::from("/dev/a"), PathBuf::from("/dev/b")]);
        round_trip(vec![1i64, 2, 3]);
        round_trip(vec![1.5f64, 2.5]);
        round_trip(vec![true, false]);
        round_trip(vec![1u8, 2, 3]);
        round_trip(vec![-5i8, 5]);
        round_trip(vec![-5000i16, 5000]);
        round_trip(vec![-70000i32, 70000]);
        round_trip(vec![0u16, 65535]);
        round_trip(vec![0u32, 4_000_000_000]);
        round_trip(vec![1.5f32, -2.5]);

        round_trip(1.5f32);
        round_trip(-5i8);
        round_trip(-5000i16);
        round_trip(-70000i32);
        round_trip(200u8);
        round_trip(65535u16);
        round_trip(4_000_000_000u32);
    }

    #[test]
    fn test_narrow_integers_reject_out_of_range_values() {
        // The kind is right and the value is a perfectly good i64, but it is not a u16.
        let err = u16::try_from(ParameterValue::Integer(70000)).unwrap_err();
        assert!(
            matches!(&err, ParameterValueError::Invalid(reason)
                if reason.contains("70000") && reason.contains("0..=65535")),
            "unhelpful reason: {err}"
        );

        assert!(u16::try_from(ParameterValue::Integer(-1)).is_err());
        assert!(i8::try_from(ParameterValue::Integer(128)).is_err());
        assert!(u8::try_from(ParameterValue::Integer(256)).is_err());
        assert!(i32::try_from(ParameterValue::Integer(i64::MAX)).is_err());

        // Boundaries are accepted.
        assert_eq!(
            u16::try_from(ParameterValue::Integer(65535)).unwrap(),
            65535
        );
        assert_eq!(i8::try_from(ParameterValue::Integer(-128)).unwrap(), -128);
    }

    #[test]
    fn test_f32_rejects_values_it_cannot_represent() {
        assert!(f32::try_from(ParameterValue::Double(1e300)).is_err());
        assert!(f32::try_from(ParameterValue::Double(-1e300)).is_err());

        // Non-finite values pass through unchanged rather than being rejected as out of range.
        assert!(f32::try_from(ParameterValue::Double(f64::INFINITY))
            .unwrap()
            .is_infinite());
        assert!(f32::try_from(ParameterValue::Double(f64::NAN))
            .unwrap()
            .is_nan());
    }

    /// An array of a narrower type holds the same values as the scalar type does, and says which
    /// element is at fault when one of them does not fit.
    #[test]
    fn test_arrays_check_every_element() {
        let too_big = ParameterValue::IntegerArray(vec![1, 2, 70000].into());
        let err = Vec::<u16>::try_from(too_big).unwrap_err();
        assert!(
            matches!(&err, ParameterValueError::Invalid(reason)
                if reason.contains("element 2") && reason.contains("0..=65535")),
            "the reason should name the element: {err}"
        );

        assert!(Vec::<u16>::try_from(ParameterValue::IntegerArray(vec![-1].into())).is_err());
        assert!(Vec::<f32>::try_from(ParameterValue::DoubleArray(vec![1e300].into())).is_err());

        // An array of the wrong ROS 2 type is a different problem from a bad element.
        assert!(matches!(
            Vec::<u16>::try_from(ParameterValue::DoubleArray(vec![1.0].into())),
            Err(ParameterValueError::TypeMismatch)
        ));
    }

    /// A sequence of bytes is a ROS 2 byte array, not an integer array, which is why `Vec<u8>` is
    /// the one array type that does not follow its scalar.
    #[test]
    fn test_byte_arrays_are_not_integer_arrays() {
        assert_eq!(Vec::<u8>::kind(), ParameterKind::ByteArray);
        assert_eq!(Vec::<u16>::kind(), ParameterKind::IntegerArray);
        assert_eq!(u8::kind(), ParameterKind::Integer);
    }

    #[test]
    fn test_arrays_report_the_constraints_of_their_elements() {
        assert_eq!(
            Vec::<u16>::type_constraints().as_deref(),
            Some("every element 0..=65535")
        );
        // An element type with nothing to say about itself leaves the array with nothing either.
        assert_eq!(Vec::<f64>::type_constraints(), None);
    }

    #[test]
    fn test_declaring_parameters_with_ergonomic_types() {
        let node = Context::default()
            .create_basic_executor()
            .create_node("std_types")
            .unwrap();

        let name: MandatoryParameter<String> = node
            .declare_parameter("name")
            .default("robot".to_string())
            .mandatory()
            .unwrap();
        assert_eq!(name.get(), "robot");

        let wheels: MandatoryParameter<Vec<String>> = node
            .declare_parameter("wheels")
            .default(vec!["left".to_string(), "right".to_string()])
            .mandatory()
            .unwrap();
        assert_eq!(wheels.get(), vec!["left", "right"]);

        let device: MandatoryParameter<PathBuf> = node
            .declare_parameter("device")
            .default(PathBuf::from("/dev/ttyUSB0"))
            .mandatory()
            .unwrap();
        assert_eq!(device.get(), PathBuf::from("/dev/ttyUSB0"));

        let ports: MandatoryParameter<Vec<u16>> = node
            .declare_parameter("ports")
            .default(vec![8080, 9090])
            .mandatory()
            .unwrap();
        assert_eq!(ports.get(), vec![8080, 9090]);
    }

    /// A `Vec<u16>` parameter cannot be made to hold an integer array that is not one, even by a
    /// caller going through the untyped interface with a well-formed array of `i64`.
    #[test]
    fn test_narrow_arrays_cannot_be_corrupted() {
        let node = Context::default()
            .create_basic_executor()
            .create_node("std_types_arrays")
            .unwrap();

        let ports: MandatoryParameter<Vec<u16>> = node
            .declare_parameter("ports")
            .default(vec![8080u16])
            .mandatory()
            .unwrap();

        let err = node
            .use_undeclared_parameters()
            .set::<Vec<i64>>("ports", vec![1, 70000])
            .unwrap_err();
        assert!(matches!(err, ParameterValueError::Invalid(_)), "{err}");
        assert_eq!(ports.get(), vec![8080]);
    }

    /// Ranges on a narrow integer parameter are expressed in that integer's own type, and are
    /// enforced against the ROS 2 representation.
    #[test]
    fn test_narrow_integer_ranges() {
        let node = Context::default()
            .create_basic_executor()
            .create_node("std_types_ranges")
            .unwrap();

        let port: MandatoryParameter<u16> = node
            .declare_parameter("port")
            .default(8080)
            .range(ParameterRange {
                lower: Some(1024),
                upper: Some(49151),
                step: None,
            })
            .mandatory()
            .unwrap();

        assert_eq!(port.get(), 8080);
        assert!(port.set(80u16).is_err(), "below the range");
        assert!(port.set(50000u16).is_err(), "above the range");
        port.set(9000u16).unwrap();
        assert_eq!(port.get(), 9000);
    }

    /// The narrowing is reported as the descriptor's `IntegerRange`, which is what rqt and
    /// `ros2 param describe` read, and not only as free-text constraints.
    /// An exclusive range is written in the parameter's own type, so the bound is checked against
    /// what that type can hold and the end names the value one past the last one admitted.
    #[test]
    fn test_a_narrow_integer_takes_an_exclusive_range() {
        let node = Context::default()
            .create_basic_executor()
            .create_node("narrow_exclusive")
            .unwrap();

        let port: MandatoryParameter<u16> = node
            .declare_parameter("port")
            .default(8080)
            .range(1024..49152)
            .mandatory()
            .unwrap();

        assert!(port.set(49151u16).is_ok());
        assert!(port.set(49152u16).is_err());
        assert!(port.set(1023u16).is_err());

        let descriptor = crate::parameter::test_support::parameter_descriptor(&node, "port");
        let range = descriptor.integer_range.first().unwrap();
        assert_eq!((range.from_value, range.to_value), (1024, 49151));
    }

    #[test]
    fn test_narrow_integers_report_their_range_in_the_descriptor() {
        use crate::parameter::test_support::parameter_descriptor;

        let node = Context::default()
            .create_basic_executor()
            .create_node("std_types_descriptor")
            .unwrap();

        // No range on the declaration: the type is the only thing that bounds the value.
        let _implied: MandatoryParameter<u16> = node
            .declare_parameter("implied")
            .default(8080)
            .mandatory()
            .unwrap();
        // A half-open range keeps its declared bound and takes the type's for the other end.
        let _half_open: MandatoryParameter<u16> = node
            .declare_parameter("half_open")
            .default(8080)
            .range(ParameterRange {
                lower: Some(1024),
                upper: None,
                step: None,
            })
            .mandatory()
            .unwrap();
        let _signed: MandatoryParameter<i8> = node
            .declare_parameter("signed")
            .default(0)
            .mandatory()
            .unwrap();

        let bounds = |name| {
            let descriptor = parameter_descriptor(&node, name);
            let range = descriptor
                .integer_range
                .first()
                .expect("a narrow integer is always bounded, so it always has a range");
            (range.from_value, range.to_value)
        };
        assert_eq!(bounds("implied"), (0, 65535));
        assert_eq!(bounds("half_open"), (1024, 65535));
        assert_eq!(bounds("signed"), (-128, 127));

        // An `i64` is bounded by nothing narrower than the ROS 2 type, so it still reports no
        // range unless the declaration asked for one.
        let _plain: MandatoryParameter<i64> = node
            .declare_parameter("plain")
            .default(0)
            .mandatory()
            .unwrap();
        assert!(parameter_descriptor(&node, "plain")
            .integer_range
            .is_empty());
    }
}
