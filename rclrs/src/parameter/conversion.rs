//! How a Rust type is represented as a ROS 2 parameter value.

use std::{fmt::Display, sync::Arc};

use super::{ParameterKind, ParameterValue, ParameterVariant};

/// How a Rust type is represented as a ROS 2 parameter value.
///
/// A parameter's conversion is fixed when it is declared and carried with it from then on. It is
/// a value rather than a trait implementation, and the orphan rules do not apply to values, so a
/// crate can declare a parameter of a type it does not own. `std::time::Duration` and the types
/// of any other dependency are all reachable this way.
///
/// Every [`ParameterVariant`] describes one, assembled by [`Self::of_variant`], so this is needed
/// only to use a type that has no implementation, or to choose a representation other than the one
/// a type declares for itself.
///
/// The conversion back from a [`ParameterValue`] is fallible, because a parameter value arrives
/// from a parameter file or from a remote `SetParameters` call and cannot be trusted to be
/// representable. A conversion that genuinely cannot fail can return `Ok` unconditionally.
///
/// # Example
///
/// A duration stored as a number of seconds. The functions `std` already provides have the shapes
/// this needs, so there is nothing to write by hand.
///
/// ```
/// # use rclrs::*;
/// # use std::time::Duration;
/// let seconds = ParameterConversion::double(Duration::as_secs_f64, Duration::try_from_secs_f64);
///
/// let executor = Context::default().create_basic_executor();
/// let node = executor.create_node("drive_controller")?;
/// let timeout = node
///     .declare_parameter_with("timeout", seconds)
///     .default(Duration::from_millis(500))
///     .mandatory()?;
///
/// assert_eq!(timeout.get(), Duration::from_millis(500));
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub struct ParameterConversion<T> {
    kind: ParameterKind,
    /// What a value of this conversion may be, in words, for the descriptor's
    /// `additional_constraints`. Nothing enforces it, and a declaration that states its own wins.
    constraints: Option<Arc<str>>,
    /// The bounds the representation itself imposes, such as the range a `u16` can hold. Used by
    /// a declaration that sets no range of its own.
    ranges: super::ParameterRanges,
    to_value: Arc<dyn Fn(&T) -> ParameterValue + Send + Sync>,
    from_value: Arc<dyn Fn(ParameterValue) -> Result<T, String> + Send + Sync>,
}

// Written out rather than derived, because deriving would ask for `T: Clone` and a conversion is
// clonable whatever it converts.
impl<T> Clone for ParameterConversion<T> {
    fn clone(&self) -> Self {
        Self {
            kind: self.kind,
            constraints: self.constraints.clone(),
            ranges: self.ranges.clone(),
            to_value: Arc::clone(&self.to_value),
            from_value: Arc::clone(&self.from_value),
        }
    }
}

impl<T> std::fmt::Debug for ParameterConversion<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParameterConversion")
            .field("kind", &self.kind)
            .finish_non_exhaustive()
    }
}

/// Builds the body of a constructor for one ROS 2 parameter type.
///
/// Each one is the same shape: wrap the value on the way out, and on the way in check that the
/// value is of the expected kind before handing it to the caller's function. The mismatch case is
/// what stops a conversion from ever seeing a value it was not written for.
macro_rules! conversion_constructor {
    ($name:ident, $wire:ty, $variant:ident, $described:literal) => {
        #[doc = concat!("A `T` represented as ", $described, ".")]
        ///
        /// `to` is how a value of the type becomes the stored value, and `from` how it is read
        /// back. `from` may fail, and its error is reported as the reason the value was refused.
        /// The error type has to outlive the conversion, which holds `from` for as long as the
        /// parameter is declared.
        pub fn $name<E: Display + 'static>(
            to: fn(&T) -> $wire,
            from: fn($wire) -> Result<T, E>,
        ) -> ParameterConversion<T> {
            ParameterConversion {
                kind: ParameterKind::$variant,
                constraints: ::core::option::Option::None,
                ranges: ::core::default::Default::default(),
                to_value: Arc::new(move |value| ParameterValue::$variant(to(value))),
                from_value: Arc::new(move |value| match value {
                    ParameterValue::$variant(wire) => from(wire).map_err(|err| err.to_string()),
                    other => Err(format!("expected {}, got {:?}", $described, other.kind())),
                }),
            }
        }
    };
}

impl<T: 'static> ParameterConversion<T> {
    conversion_constructor!(boolean, bool, Bool, "a boolean");
    conversion_constructor!(integer, i64, Integer, "an integer");
    conversion_constructor!(double, f64, Double, "a double");
    conversion_constructor!(string, Arc<str>, String, "a string");
    conversion_constructor!(byte_array, Arc<[u8]>, ByteArray, "a byte array");
    conversion_constructor!(
        boolean_array,
        Arc<[bool]>,
        BoolArray,
        "an array of booleans"
    );
    conversion_constructor!(
        integer_array,
        Arc<[i64]>,
        IntegerArray,
        "an array of integers"
    );
    conversion_constructor!(double_array, Arc<[f64]>, DoubleArray, "an array of doubles");
    conversion_constructor!(
        string_array,
        Arc<[Arc<str>]>,
        StringArray,
        "an array of strings"
    );

    /// The ROS 2 parameter type a value of this conversion is stored as.
    pub fn kind(&self) -> ParameterKind {
        self.kind
    }

    /// Describes in words what a value of this conversion may be.
    ///
    /// Reported as the descriptor's `additional_constraints` by any declaration that does not
    /// state constraints of its own, so that `ros2 param describe` can say what a value has to
    /// satisfy when the rule is not one a range can express.
    pub fn with_constraints(mut self, constraints: impl Into<Arc<str>>) -> Self {
        self.constraints = Some(constraints.into());
        self
    }

    /// The constraints this conversion describes, if any.
    pub fn constraints(&self) -> Option<Arc<str>> {
        self.constraints.clone()
    }

    /// The bounds the representation imposes on its own, before any the declaration adds.
    pub fn ranges(&self) -> super::ParameterRanges {
        self.ranges.clone()
    }

    /// Converts a value into the parameter value that represents it.
    pub fn to_value(&self, value: &T) -> ParameterValue {
        (self.to_value)(value)
    }

    /// Converts a stored parameter value back, reporting why if it cannot.
    pub fn from_value(&self, value: ParameterValue) -> Result<T, String> {
        (self.from_value)(value)
    }
}

impl<T: ParameterVariant> ParameterConversion<T> {
    /// The conversion a [`ParameterVariant`] describes for itself.
    ///
    /// Assembled from the type's kind, its constraints and its own `Into` and `TryFrom`, so the
    /// trait states those once and there is no second way to say the same thing. Every parameter
    /// declared by naming its type alone uses this.
    pub fn of_variant() -> Self {
        Self {
            kind: T::kind(),
            constraints: T::type_constraints(),
            // A narrow type is bounded by what it can hold, whether or not a declaration says so.
            ranges: <T::Range as Default>::default().into(),
            to_value: Arc::new(|value: &T| value.clone().into()),
            from_value: Arc::new(|value| T::try_from(value).map_err(|err| err.to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Context, CreateBasicExecutor, DeclarationError, ParameterRange, ParameterValueError,
    };
    use std::time::Duration;

    /// Conversion for `std::time::Duration` stored as a number of seconds
    fn seconds() -> ParameterConversion<Duration> {
        ParameterConversion::double(Duration::as_secs_f64, Duration::try_from_secs_f64)
    }

    fn node(name: &str) -> crate::Node {
        Context::default()
            .create_basic_executor()
            .create_node(name)
            .unwrap()
    }

    /// `Duration` belongs to `std` and so can never implement `ParameterVariant`. Saying how it
    /// is represented is enough to declare it, to read and write it as itself through a handle,
    /// and to reach it through the undeclared API, which has no type to dispatch on either.
    #[test]
    fn test_a_foreign_type_can_be_a_parameter() {
        let node = node("foreign_type");
        let timeout = node
            .declare_parameter_with("timeout", seconds())
            .default(Duration::from_millis(500))
            .mandatory()
            .unwrap();

        assert_eq!(timeout.get(), Duration::from_millis(500));
        timeout.set(Duration::from_secs(2)).unwrap();
        assert_eq!(timeout.get(), Duration::from_secs(2));

        let undeclared = node.use_undeclared_parameters();
        // What is stored is the representation, which is what a parameter file would contain and
        // what `ros2 param get` reports.
        assert_eq!(undeclared.get::<f64>("timeout"), Some(2.0));

        // Given the conversion, the undeclared API reads and writes it as a `Duration` too.
        assert_eq!(
            undeclared.get_with("timeout", &seconds()),
            Some(Duration::from_secs(2))
        );
        undeclared
            .set_with("timeout", Duration::from_secs(3), &seconds())
            .unwrap();
        assert_eq!(timeout.get(), Duration::from_secs(3));
    }

    /// The conversion decides the range's units too, since the range constrains the stored value.
    #[test]
    fn test_a_range_applies_to_the_stored_value() {
        let node = node("foreign_range");
        let timeout = node
            .declare_parameter_with("timeout", seconds())
            .default(Duration::from_secs(1))
            // `Duration` has no `ParameterVariant` impl and so no `Range` of its own to name.
            // The bounds are written in what the conversion stores, which here is seconds.
            .stored_ranges(
                ParameterRange {
                    lower: Some(0.0),
                    upper: Some(5.0),
                    step: None,
                }
                .into(),
            )
            .mandatory()
            .unwrap();

        assert!(timeout.set(Duration::from_secs(4)).is_ok());
        assert!(matches!(
            timeout.set(Duration::from_secs(6)),
            Err(ParameterValueError::OutOfRange)
        ));
    }

    /// A value already set before the declaration has to pass through the conversion too. One
    /// that cannot fails the declaration rather than waiting to panic on the first read, which is
    /// what a negative number of seconds would otherwise do.
    #[test]
    fn test_a_prior_value_the_conversion_refuses() {
        let node = node("foreign_refused");
        node.use_undeclared_parameters()
            .set("timeout", -1.0)
            .unwrap();

        let err = node
            .declare_parameter_with("timeout", seconds())
            .default(Duration::from_secs(1))
            .mandatory()
            .err()
            .unwrap();
        assert_eq!(err, DeclarationError::PriorValueTypeMismatch);
    }

    /// A type that does implement `ParameterVariant` keeps working through the conversion it
    /// describes for itself, which is what `declare_parameter` uses.
    #[test]
    fn test_a_variant_supplies_its_own_conversion() {
        let conversion = ParameterConversion::<f64>::of_variant();
        assert_eq!(conversion.kind(), ParameterKind::Double);
        assert_eq!(conversion.to_value(&1.5), ParameterValue::Double(1.5));
        assert_eq!(conversion.from_value(ParameterValue::Double(1.5)), Ok(1.5));
        assert!(conversion.from_value(ParameterValue::Bool(true)).is_err());
    }
}
