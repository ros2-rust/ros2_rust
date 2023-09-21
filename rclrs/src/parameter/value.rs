use std::ffi::CStr;
use std::sync::Arc;

use crate::rcl_bindings::*;

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

#[derive(Clone, Debug, PartialEq)]
pub enum ParameterKind {
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
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

pub trait ParameterVariant: Into<ParameterValue> {
    // TODO(luca) should we use try_from?
    fn maybe_from(value: ParameterValue) -> Option<Self>;

    fn kind() -> ParameterKind;
}

impl ParameterVariant for bool {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::Bool(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::Bool
    }
}

impl ParameterVariant for i64 {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::Integer(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::Integer
    }
}

impl ParameterVariant for f64 {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::Double(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::Double
    }
}

impl ParameterVariant for Arc<str> {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::String(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::String
    }
}

impl ParameterVariant for Arc<[u8]> {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::ByteArray(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::ByteArray
    }
}

impl ParameterVariant for Arc<[bool]> {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::BoolArray(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::BoolArray
    }
}

impl ParameterVariant for Arc<[i64]> {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::IntegerArray(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::IntegerArray
    }
}

impl ParameterVariant for Arc<[f64]> {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::DoubleArray(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::DoubleArray
    }
}

impl ParameterVariant for Arc<[Arc<str>]> {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        match value {
            ParameterValue::StringArray(v) => Some(v),
            _ => None,
        }
    }

    fn kind() -> ParameterKind {
        ParameterKind::StringArray
    }
}

impl ParameterVariant for ParameterValue {
    fn maybe_from(value: ParameterValue) -> Option<Self> {
        Some(value)
    }

    fn kind() -> ParameterKind {
        ParameterKind::Dynamic
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

    /// Returns the ParameterKind of the current variant, as a static kind.
    pub(crate) fn static_kind(&self) -> ParameterKind {
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
                    &ctx.rcl_context_mtx.lock().unwrap().global_arguments,
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
