use std::{ffi::CStr, sync::Arc};

use crate::{
    parameter::{ParameterRange, ParameterRanges},
    rcl_bindings::*,
    vendor::rcl_interfaces::msg::rmw::{ParameterType, ParameterValue as RmwParameterValue},
    ParameterValueError,
};

/// A parameter value.
///
/// Such a value can be specified in YAML format on the command line, or in a parameter file.
/// For instance `--param foo:='[1, 2, 3]'` specfies an `IntegerArray` value for the `foo` parameter.
#[derive(Clone, Debug, PartialEq)]
pub enum ParameterValue {
    /// A boolean value.
    ///
    /// YAML examples: `true`, `True`, `yes`, `y`.
    Bool(bool),
    /// An i64 value.
    ///
    /// YAML examples: `1`, `-30`, `0x7C`.
    Integer(i64),
    /// An f64 value.
    ///
    /// YAML examples: `2.0`, `8e-3`.
    Double(f64),
    /// A string.
    ///
    /// YAML examples: `""`, `"björk"`, `"42"`.
    ///
    /// Unquoted strings are also possible, but not recommended,
    /// because they may be interpreted as another data type.
    String(Arc<str>),
    /// An array of u8.
    ///
    /// YAML example: Not possible to specify as YAML.
    ByteArray(Arc<[u8]>),
    /// An array of booleans.
    ///
    /// YAML example: `[true, false, false]`.
    BoolArray(Arc<[bool]>),
    /// An array of i64.
    ///
    /// YAML example: `[3, 4]`.
    IntegerArray(Arc<[i64]>),
    /// An array of f64.
    ///
    /// YAML example: `[5.0, 6e2]`.
    DoubleArray(Arc<[f64]>),
    /// An array of strings.
    ///
    /// YAML example: `["abc", ""]`.
    StringArray(Arc<[Arc<str>]>),
}

/// Describes the parameter's type. Similar to `ParameterValue` but also includes a `Dynamic`
/// variant for dynamic parameters.
#[derive(Clone, Debug, PartialEq)]
pub enum ParameterKind {
    /// A boolean parameter.
    Bool,
    /// An i64 value.
    Integer,
    /// An f64 value.
    Double,
    /// A string.
    String,
    /// An array of u8.
    ByteArray,
    /// An array of booleans.
    BoolArray,
    /// An array of i64.
    IntegerArray,
    /// An array of f64.
    DoubleArray,
    /// An array of strings.
    StringArray,
    /// A dynamic parameter that can change its type at runtime.
    Dynamic,
}

impl From<bool> for ParameterValue {
    fn from(value: bool) -> ParameterValue {
        ParameterValue::Bool(value)
    }
}

impl From<i64> for ParameterValue {
    fn from(value: i64) -> ParameterValue {
        ParameterValue::Integer(value)
    }
}

impl From<f64> for ParameterValue {
    fn from(value: f64) -> ParameterValue {
        ParameterValue::Double(value)
    }
}

impl From<Arc<str>> for ParameterValue {
    fn from(value: Arc<str>) -> ParameterValue {
        ParameterValue::String(value)
    }
}

impl From<Arc<[u8]>> for ParameterValue {
    fn from(value: Arc<[u8]>) -> ParameterValue {
        ParameterValue::ByteArray(value)
    }
}

impl From<Arc<[bool]>> for ParameterValue {
    fn from(value: Arc<[bool]>) -> ParameterValue {
        ParameterValue::BoolArray(value)
    }
}

impl From<Arc<[i64]>> for ParameterValue {
    fn from(value: Arc<[i64]>) -> ParameterValue {
        ParameterValue::IntegerArray(value)
    }
}

impl From<Arc<[f64]>> for ParameterValue {
    fn from(value: Arc<[f64]>) -> ParameterValue {
        ParameterValue::DoubleArray(value)
    }
}

impl From<Arc<[Arc<str>]>> for ParameterValue {
    fn from(value: Arc<[Arc<str>]>) -> ParameterValue {
        ParameterValue::StringArray(value)
    }
}

/// A trait that describes a value that can be converted into a parameter.
pub trait ParameterVariant: Into<ParameterValue> + Clone + TryFrom<ParameterValue> {
    /// The type used to describe the range of this parameter.
    type Range: Into<ParameterRanges> + Default + Clone;

    /// Returns the `ParameterKind` of the implemented type.
    fn kind() -> ParameterKind;
}

impl TryFrom<ParameterValue> for bool {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::Bool(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for bool {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::Bool
    }
}

impl TryFrom<ParameterValue> for i64 {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::Integer(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for i64 {
    type Range = ParameterRange<i64>;

    fn kind() -> ParameterKind {
        ParameterKind::Integer
    }
}

impl TryFrom<ParameterValue> for f64 {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::Double(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for f64 {
    type Range = ParameterRange<f64>;

    fn kind() -> ParameterKind {
        ParameterKind::Double
    }
}

impl TryFrom<ParameterValue> for Arc<str> {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::String(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Arc<str> {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::String
    }
}

impl TryFrom<ParameterValue> for Arc<[u8]> {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::ByteArray(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Arc<[u8]> {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::ByteArray
    }
}

impl TryFrom<ParameterValue> for Arc<[bool]> {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::BoolArray(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Arc<[bool]> {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::BoolArray
    }
}

impl TryFrom<ParameterValue> for Arc<[i64]> {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::IntegerArray(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Arc<[i64]> {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::IntegerArray
    }
}

impl TryFrom<ParameterValue> for Arc<[f64]> {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::DoubleArray(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Arc<[f64]> {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::DoubleArray
    }
}

impl TryFrom<ParameterValue> for Arc<[Arc<str>]> {
    type Error = ParameterValueError;

    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::StringArray(v) => Ok(v),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Arc<[Arc<str>]> {
    type Range = ();

    fn kind() -> ParameterKind {
        ParameterKind::StringArray
    }
}

impl ParameterVariant for ParameterValue {
    type Range = ParameterRanges;

    fn kind() -> ParameterKind {
        ParameterKind::Dynamic
    }
}

impl From<ParameterValue> for RmwParameterValue {
    fn from(value: ParameterValue) -> Self {
        match value {
            ParameterValue::Bool(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_BOOL,
                bool_value: v,
                ..Default::default()
            },
            ParameterValue::Integer(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_INTEGER,
                integer_value: v,
                ..Default::default()
            },
            ParameterValue::Double(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_DOUBLE,
                double_value: v,
                ..Default::default()
            },
            ParameterValue::String(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_STRING,
                string_value: v.into(),
                ..Default::default()
            },
            ParameterValue::ByteArray(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_BYTE_ARRAY,
                byte_array_value: (*v).into(),
                ..Default::default()
            },
            ParameterValue::BoolArray(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_BOOL_ARRAY,
                bool_array_value: (*v).into(),
                ..Default::default()
            },
            ParameterValue::IntegerArray(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_INTEGER_ARRAY,
                integer_array_value: (*v).into(),
                ..Default::default()
            },
            ParameterValue::DoubleArray(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_DOUBLE_ARRAY,
                double_array_value: (*v).into(),
                ..Default::default()
            },
            ParameterValue::StringArray(v) => RmwParameterValue {
                type_: ParameterType::PARAMETER_STRING_ARRAY,
                string_array_value: v.iter().map(|v| v.clone().into()).collect(),
                ..Default::default()
            },
        }
    }
}

/// An error that occured when trying to convert a parameter from an
/// `rcl_interfaces::msg::ParameterValue`
pub enum RmwParameterConversionError {
    /// The parameter type was not valid.
    InvalidParameterType,
}

impl TryFrom<RmwParameterValue> for ParameterValue {
    type Error = RmwParameterConversionError;

    fn try_from(param: RmwParameterValue) -> Result<Self, Self::Error> {
        // TODO(luca) how to deal with PARAMETER_NOT_SET? Should we allow service calls to unset
        // parameters?
        match param.type_ {
            ParameterType::PARAMETER_BOOL => Ok(ParameterValue::Bool(param.bool_value)),
            ParameterType::PARAMETER_INTEGER => Ok(ParameterValue::Integer(param.integer_value)),
            ParameterType::PARAMETER_DOUBLE => Ok(ParameterValue::Double(param.double_value)),
            ParameterType::PARAMETER_STRING => Ok(ParameterValue::String(
                param.string_value.to_string().into(),
            )),
            ParameterType::PARAMETER_BYTE_ARRAY => {
                Ok(ParameterValue::ByteArray((*param.byte_array_value).into()))
            }
            ParameterType::PARAMETER_BOOL_ARRAY => {
                Ok(ParameterValue::BoolArray((*param.bool_array_value).into()))
            }
            ParameterType::PARAMETER_INTEGER_ARRAY => Ok(ParameterValue::IntegerArray(
                (*param.integer_array_value).into(),
            )),
            ParameterType::PARAMETER_DOUBLE_ARRAY => Ok(ParameterValue::DoubleArray(
                (*param.double_array_value).into(),
            )),
            ParameterType::PARAMETER_STRING_ARRAY => Ok(ParameterValue::StringArray(
                param
                    .string_array_value
                    .iter()
                    .map(|s| s.to_string().into())
                    .collect::<Vec<_>>()
                    .into(),
            )),
            _ => Err(RmwParameterConversionError::InvalidParameterType),
        }
    }
}

impl ParameterValue {
    // Panics if the rcl_variant_t does not have exactly one field set.
    //
    // This function is unsafe because it is possible to pass in an rcl_variant_t
    // containing dangling pointers, or incorrect array sizes.
    pub(crate) unsafe fn from_rcl_variant(var: &rcl_variant_t) -> Self {
        let num_active: u8 = [
            !var.bool_value.is_null(),
            !var.integer_value.is_null(),
            !var.double_value.is_null(),
            !var.string_value.is_null(),
            !var.byte_array_value.is_null(),
            !var.bool_array_value.is_null(),
            !var.integer_array_value.is_null(),
            !var.double_array_value.is_null(),
            !var.string_array_value.is_null(),
        ]
        .into_iter()
        .map(u8::from)
        .sum();
        assert_eq!(num_active, 1);
        // Note: This code has no unsafe blocks because it is inside an unsafe function.
        // In general, the following operations are as safe as they can be, because
        // only non-null pointers are dereferenced, and strings and arrays are copied immediately,
        // so there are no concerns about choosing the correct lifetime.
        //
        // Of course, a pointer being not null is not a guarantee that it points to a valid value.
        // However, it cannot be checked that it points to a valid value. Similarly for array sizes.
        // This is why this function must be unsafe itself.
        if !var.bool_value.is_null() {
            ParameterValue::Bool(*var.bool_value)
        } else if !var.integer_value.is_null() {
            ParameterValue::Integer(*var.integer_value)
        } else if !var.double_value.is_null() {
            ParameterValue::Double(*var.double_value)
        } else if !var.string_value.is_null() {
            let cstr = CStr::from_ptr(var.string_value);
            let s = cstr.to_string_lossy().into_owned();
            ParameterValue::String(s.into())
        } else if !var.byte_array_value.is_null() {
            let rcl_byte_array = &*var.byte_array_value;
            let slice = std::slice::from_raw_parts(rcl_byte_array.values, rcl_byte_array.size);
            ParameterValue::ByteArray(slice.into())
        } else if !var.bool_array_value.is_null() {
            let rcl_bool_array = &*var.bool_array_value;
            let slice = std::slice::from_raw_parts(rcl_bool_array.values, rcl_bool_array.size);
            ParameterValue::BoolArray(slice.into())
        } else if !var.integer_array_value.is_null() {
            let rcl_integer_array = &*var.integer_array_value;
            let slice =
                std::slice::from_raw_parts(rcl_integer_array.values, rcl_integer_array.size);
            ParameterValue::IntegerArray(slice.into())
        } else if !var.double_array_value.is_null() {
            let rcl_double_array = &*var.double_array_value;
            let slice = std::slice::from_raw_parts(rcl_double_array.values, rcl_double_array.size);
            ParameterValue::DoubleArray(slice.into())
        } else if !var.string_array_value.is_null() {
            let rcutils_string_array = &*var.string_array_value;
            let slice =
                std::slice::from_raw_parts(rcutils_string_array.data, rcutils_string_array.size);
            let strings = slice
                .iter()
                .map(|&ptr| {
                    debug_assert!(!ptr.is_null());
                    let cstr = CStr::from_ptr(ptr);
                    Arc::from(cstr.to_string_lossy())
                })
                .collect::<Vec<_>>();
            ParameterValue::StringArray(strings.into())
        } else {
            unreachable!()
        }
    }

    pub(crate) fn rcl_parameter_type(&self) -> u8 {
        match self {
            ParameterValue::Bool(_) => ParameterType::PARAMETER_BOOL,
            ParameterValue::Integer(_) => ParameterType::PARAMETER_INTEGER,
            ParameterValue::Double(_) => ParameterType::PARAMETER_DOUBLE,
            ParameterValue::String(_) => ParameterType::PARAMETER_STRING,
            ParameterValue::ByteArray(_) => ParameterType::PARAMETER_BYTE_ARRAY,
            ParameterValue::BoolArray(_) => ParameterType::PARAMETER_BOOL_ARRAY,
            ParameterValue::IntegerArray(_) => ParameterType::PARAMETER_INTEGER_ARRAY,
            ParameterValue::DoubleArray(_) => ParameterType::PARAMETER_DOUBLE_ARRAY,
            ParameterValue::StringArray(_) => ParameterType::PARAMETER_STRING_ARRAY,
        }
    }

    /// Returns the `ParameterKind` for the parameter.
    pub(crate) fn kind(&self) -> ParameterKind {
        match self {
            ParameterValue::Bool(_) => ParameterKind::Bool,
            ParameterValue::Integer(_) => ParameterKind::Integer,
            ParameterValue::Double(_) => ParameterKind::Double,
            ParameterValue::String(_) => ParameterKind::String,
            ParameterValue::ByteArray(_) => ParameterKind::ByteArray,
            ParameterValue::BoolArray(_) => ParameterKind::BoolArray,
            ParameterValue::IntegerArray(_) => ParameterKind::IntegerArray,
            ParameterValue::DoubleArray(_) => ParameterKind::DoubleArray,
            ParameterValue::StringArray(_) => ParameterKind::StringArray,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Context, RclrsError, ToResult};

    // TODO(luca) tests for all from / to ParameterVariant functions

    #[test]
    fn test_parameter_value() -> Result<(), RclrsError> {
        // This test is not a test of the YAML parser or argument parser, only a test that the
        // correct ParameterValue variant is obtained from rcl_variants found in the wild.
        let input_output_pairs = [
            ("true", ParameterValue::Bool(true)),
            ("1", ParameterValue::Integer(1)),
            ("1.0", ParameterValue::Double(1.0)),
            ("'1.0'", ParameterValue::String(Arc::from("1.0"))),
            (
                "[yes, no]",
                ParameterValue::BoolArray(Arc::from([true, false])),
            ),
            ("[-3, 2]", ParameterValue::IntegerArray(Arc::from([-3, 2]))),
            (
                "[-3.0, 2.0]",
                ParameterValue::DoubleArray(Arc::from([-3.0, 2.0])),
            ),
            (
                "['yes']",
                ParameterValue::StringArray(Arc::from([Arc::from("yes")])),
            ),
        ];
        for pair in input_output_pairs {
            let ctx = Context::new([
                String::from("--ros-args"),
                String::from("-p"),
                format!("foo:={}", pair.0),
            ])?;
            let mut rcl_params = std::ptr::null_mut();
            unsafe {
                rcl_arguments_get_param_overrides(
                    &ctx.handle.rcl_context.lock().unwrap().global_arguments,
                    &mut rcl_params,
                )
                .ok()?;
            }
            assert!(!rcl_params.is_null());
            assert_eq!(unsafe { (*rcl_params).num_nodes }, 1);
            let rcl_node_params = unsafe { &(*(*rcl_params).params) };
            assert_eq!(rcl_node_params.num_params, 1);
            let rcl_variant = unsafe { &(*rcl_node_params.parameter_values) };
            let param_value = unsafe { ParameterValue::from_rcl_variant(rcl_variant) };
            assert_eq!(param_value, pair.1);
            unsafe { rcl_yaml_node_struct_fini(rcl_params) };
        }
        Ok(())
    }
}
