use crate::{DeclarationError, ParameterValue, ParameterVariant};
use ros_env::rcl_interfaces::msg::rmw::{FloatingPointRange, IntegerRange};
use rosidl_runtime_rs::{seq, BoundedSequence};
use std::ops::{Range, RangeFrom, RangeFull, RangeInclusive, RangeTo, RangeToInclusive};

impl From<ParameterRange<f64>> for ParameterRanges {
    fn from(params: ParameterRange<f64>) -> Self {
        Self {
            float: Some(params),
            ..Default::default()
        }
    }
}

impl From<ParameterRange<i64>> for ParameterRanges {
    fn from(params: ParameterRange<i64>) -> Self {
        Self {
            integer: Some(params),
            ..Default::default()
        }
    }
}

impl From<()> for ParameterRanges {
    fn from(_empty: ()) -> Self {
        Self::default()
    }
}

/// Contains all the possible type of ranges that can be applied to a value.
/// Usually only one of these ranges will be applied, but all have to be stored since:
///
/// * A dynamic parameter can change its type at runtime, in which case a different range could be
///   applied.
/// * Introspection through service calls requires all the ranges to be reported to the user.
#[derive(Clone, Debug, Default)]
pub struct ParameterRanges {
    float: Option<ParameterRange<f64>>,
    integer: Option<ParameterRange<i64>>,
}

impl ParameterRanges {
    pub(crate) fn to_descriptor_ranges(
        &self,
    ) -> (
        BoundedSequence<IntegerRange, 1>,
        BoundedSequence<FloatingPointRange, 1>,
    ) {
        let int_range = self
            .integer
            .as_ref()
            .map(|range| {
                // Converting step to a positive value is safe because declaring a parameter with a
                // negative step is not allowed.
                // TODO(luca) explore changing step into a positive value in the generic definition to
                // make negative steps a compile error.
                if range.is_default() {
                    Default::default()
                } else {
                    seq![1 # IntegerRange {
                        from_value: range.lower.unwrap_or(i64::MIN),
                        to_value: range.upper.unwrap_or(i64::MAX),
                        step: range.step.unwrap_or(0).try_into().unwrap(),
                    }]
                }
            })
            .unwrap_or_default();
        let float_range = self
            .float
            .as_ref()
            .map(|range| {
                if range.is_default() {
                    Default::default()
                } else {
                    seq![1 # FloatingPointRange {
                        from_value: range.lower.unwrap_or(f64::NEG_INFINITY),
                        to_value: range.upper.unwrap_or(f64::INFINITY),
                        step: range.step.unwrap_or(0.0),
                    }]
                }
            })
            .unwrap_or_default();
        (int_range, float_range)
    }

    pub(crate) fn validate(&self) -> Result<(), DeclarationError> {
        if let Some(integer) = &self.integer {
            integer.validate()?;
        }
        if let Some(float) = &self.float {
            float.validate()?;
        }
        Ok(())
    }

    pub(crate) fn in_range(&self, value: &ParameterValue) -> bool {
        match value {
            ParameterValue::Integer(v) => {
                if let Some(range) = &self.integer {
                    if !range.in_range(*v) {
                        return false;
                    }
                }
            }
            ParameterValue::Double(v) => {
                if let Some(range) = &self.float {
                    if !range.in_range(*v) {
                        return false;
                    }
                }
            }
            _ => {}
        }
        true
    }
}

/// Describes the range for paramter type T.
#[derive(Clone, Debug, Default)]
pub struct ParameterRange<T: ParameterVariant + PartialOrd> {
    /// Lower limit, if set the parameter must be >= l.
    pub lower: Option<T>,
    /// Upper limit, if set the parameter must be <= u.
    pub upper: Option<T>,
    /// Step size, if set and `lower` is set the parameter must be within an integer number of
    /// steps of size `step` from `lower`, or equal to the upper limit if set.
    /// Example:
    /// If lower is `Some(0)`, upper is `Some(10)` and step is `Some(3)`, acceptable values are:
    /// `[0, 3, 6, 9, 10]`.
    pub step: Option<T>,
}

impl<T: ParameterVariant + PartialOrd> ParameterRange<T> {
    /// Sets the step, the spacing of the valid values within the range.
    ///
    /// No Rust range carries a step, so this is how one is added to a range written as `a..=b`.
    pub fn with_step(mut self, step: T) -> Self {
        self.step = Some(step);
        self
    }
}

impl<T: ParameterVariant + PartialOrd + Default> ParameterRange<T> {
    fn is_default(&self) -> bool {
        self.lower.is_none() && self.upper.is_none() && self.step.is_none()
    }

    fn inside_boundary(&self, value: &T) -> bool {
        if self.lower.as_ref().is_some_and(|l| value < l) {
            return false;
        }
        if self.upper.as_ref().is_some_and(|u| value > u) {
            return false;
        }
        true
    }

    fn validate(&self) -> Result<(), DeclarationError> {
        if self
            .lower
            .as_ref()
            .zip(self.upper.as_ref())
            .is_some_and(|(l, u)| l > u)
        {
            return Err(DeclarationError::InvalidRange);
        }
        if self.step.as_ref().is_some_and(|s| s <= &T::default()) {
            return Err(DeclarationError::InvalidRange);
        }
        Ok(())
    }
}

impl ParameterRange<i64> {
    fn in_range(&self, value: i64) -> bool {
        if !self.inside_boundary(&value) {
            return false;
        }
        if self.upper.is_some_and(|u| u == value) {
            return true;
        }
        if let (Some(l), Some(s)) = (self.lower, self.step) {
            if (value - l) % s != 0 {
                return false;
            }
        }
        true
    }
}

impl ParameterRange<f64> {
    // Same comparison function as rclcpp.
    fn are_close(v1: f64, v2: f64) -> bool {
        const ULP_TOL: f64 = 100.0;
        (v1 - v2).abs() <= (f64::EPSILON * (v1 + v2).abs() * ULP_TOL)
    }

    fn in_range(&self, value: f64) -> bool {
        if self.upper.is_some_and(|u| Self::are_close(u, value))
            || self.lower.is_some_and(|l| Self::are_close(l, value))
        {
            return true;
        }
        if !self.inside_boundary(&value) {
            return false;
        }
        if let (Some(l), Some(s)) = (self.lower, self.step) {
            if !Self::are_close(((value - l) / s).round() * s + l, value) {
                return false;
            }
        }
        true
    }
}

// A parameter range is written as a Rust range, so that a bound reads the way it does everywhere
// else in the language. ROS 2 ranges are inclusive of both ends, which is what decides which of
// the standard range types can be converted and how.

impl<T: ParameterVariant + PartialOrd> From<RangeInclusive<T>> for ParameterRange<T> {
    fn from(range: RangeInclusive<T>) -> Self {
        let (lower, upper) = range.into_inner();
        Self {
            lower: Some(lower),
            upper: Some(upper),
            step: None,
        }
    }
}

impl<T: ParameterVariant + PartialOrd> From<RangeFrom<T>> for ParameterRange<T> {
    fn from(range: RangeFrom<T>) -> Self {
        Self {
            lower: Some(range.start),
            upper: None,
            step: None,
        }
    }
}

impl<T: ParameterVariant + PartialOrd> From<RangeToInclusive<T>> for ParameterRange<T> {
    fn from(range: RangeToInclusive<T>) -> Self {
        Self {
            lower: None,
            upper: Some(range.end),
            step: None,
        }
    }
}

impl<T: ParameterVariant + PartialOrd> From<RangeFull> for ParameterRange<T> {
    fn from(_: RangeFull) -> Self {
        Self {
            lower: None,
            upper: None,
            step: None,
        }
    }
}

/// An exclusive end names the value just below it, which every type here has: the previous
/// integer, or the previous representable float.
///
/// The float case is exact rather than approximate. The values a `f64` can hold are discrete, so
/// `0.0..10.0` admits exactly those up to and including `next_down_f64(10.0)`, and an inclusive bound
/// there describes the same set. The bound the descriptor reports looks unusual, but a ROS 2 range
/// is inclusive of both ends and this is what an exclusive one means.
impl From<Range<i64>> for ParameterRange<i64> {
    fn from(range: Range<i64>) -> Self {
        Self {
            lower: Some(range.start),
            upper: Some(next_down_i64(range.end)),
            step: None,
        }
    }
}

impl From<RangeTo<i64>> for ParameterRange<i64> {
    fn from(range: RangeTo<i64>) -> Self {
        Self {
            lower: None,
            upper: Some(next_down_i64(range.end)),
            step: None,
        }
    }
}

impl From<Range<f64>> for ParameterRange<f64> {
    fn from(range: Range<f64>) -> Self {
        Self {
            lower: Some(range.start),
            upper: Some(next_down_f64(range.end)),
            step: None,
        }
    }
}

impl From<RangeTo<f64>> for ParameterRange<f64> {
    fn from(range: RangeTo<f64>) -> Self {
        Self {
            lower: None,
            upper: Some(next_down_f64(range.end)),
            step: None,
        }
    }
}

/// The largest `f64` strictly below `value`.
///
/// `f64::next_down` does this, but only from Rust 1.86, and the crate supports 1.85. The bit
/// pattern of a float increases with its value within a sign, so stepping the bits by one steps to
/// the neighbouring value.
fn next_down_f64(value: f64) -> f64 {
    if value.is_nan() || value == f64::NEG_INFINITY {
        return value;
    }
    if value == 0.0 {
        // Both zeroes have the same neighbour below them.
        return -f64::from_bits(1);
    }
    if value > 0.0 {
        f64::from_bits(value.to_bits() - 1)
    } else {
        f64::from_bits(value.to_bits() + 1)
    }
}

/// The largest `i64` strictly below `value`.
///
/// There is nothing below `i64::MIN`, so the bound stays there. A range ending at `i64::MIN`
/// admits nothing either way, and a declaration that asks for one is rejected for having no value
/// in range rather than by arithmetic here.
fn next_down_i64(value: i64) -> i64 {
    value.saturating_sub(1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Context, CreateBasicExecutor, MandatoryParameter};

    #[test]
    fn test_rust_ranges_convert_to_parameter_ranges() {
        let inclusive: ParameterRange<f64> = (0.0..=1.0).into();
        assert_eq!((inclusive.lower, inclusive.upper), (Some(0.0), Some(1.0)));

        let from: ParameterRange<i64> = (1024..).into();
        assert_eq!((from.lower, from.upper), (Some(1024), None));

        let to: ParameterRange<f64> = (..=0.0).into();
        assert_eq!((to.lower, to.upper), (None, Some(0.0)));

        let full: ParameterRange<i64> = (..).into();
        assert_eq!((full.lower, full.upper), (None, None));

        // An exclusive integer end is one past the last value it admits.
        let exclusive: ParameterRange<i64> = (0..10).into();
        assert_eq!((exclusive.lower, exclusive.upper), (Some(0), Some(9)));

        let exclusive_to: ParameterRange<i64> = (..10).into();
        assert_eq!((exclusive_to.lower, exclusive_to.upper), (None, Some(9)));
    }

    /// An exclusive float end names the largest value below it, which is exact: the set of `f64`
    /// values below 10.0 is exactly those up to and including `next_down_f64(10.0)`.
    #[test]
    fn test_an_exclusive_float_range_names_the_value_below() {
        let range: ParameterRange<f64> = (0.0..10.0).into();
        let upper = range.upper.unwrap();
        assert!(upper < 10.0);
        // Nothing sits between the bound and the end it excludes.
        assert_eq!(f64::from_bits(upper.to_bits() + 1), 10.0);

        let to: ParameterRange<f64> = (..1.0).into();
        assert_eq!(f64::from_bits(to.upper.unwrap().to_bits() + 1), 1.0);
    }

    #[test]
    fn test_next_down_handles_the_awkward_values() {
        assert_eq!(next_down_f64(0.0), -f64::from_bits(1));
        assert_eq!(next_down_f64(-0.0), -f64::from_bits(1));
        assert!(next_down_f64(-1.0) < -1.0);
        assert!(next_down_f64(f64::NAN).is_nan());
        assert_eq!(next_down_f64(f64::NEG_INFINITY), f64::NEG_INFINITY);
    }

    /// A step is part of no Rust range, so it is added after the conversion.
    #[test]
    fn test_a_step_is_added_to_a_converted_range() {
        let range: ParameterRange<i64> = ParameterRange::from(0..=10).with_step(5);
        assert_eq!(
            (range.lower, range.upper, range.step),
            (Some(0), Some(10), Some(5))
        );
    }

    /// A range ending at the smallest value admits nothing, and there is nothing below it to name.
    #[test]
    fn test_an_empty_exclusive_range_does_not_wrap() {
        let empty: ParameterRange<i64> = (i64::MIN..i64::MIN).into();
        assert_eq!(empty.upper, Some(i64::MIN));
    }

    #[test]
    fn test_a_parameter_takes_a_rust_range() {
        let executor = Context::default().create_basic_executor();
        let node = executor.create_node("rust_ranges").unwrap();

        let speed: MandatoryParameter<f64> = node
            .declare_parameter("speed")
            .default(0.5)
            .range(0.0..=1.0)
            .mandatory()
            .unwrap();
        assert!(speed.set(1.0).is_ok());
        assert!(speed.set(1.5).is_err());

        let count: MandatoryParameter<i64> = node
            .declare_parameter("count")
            .default(5)
            .range(0..10)
            .mandatory()
            .unwrap();
        assert!(count.set(9).is_ok());
        assert!(count.set(10).is_err());

        // A `ParameterRange` still works, and is how a step is given.
        let stepped: MandatoryParameter<i64> = node
            .declare_parameter("stepped")
            .default(0)
            .range(ParameterRange {
                lower: Some(0),
                upper: Some(10),
                step: Some(5),
            })
            .mandatory()
            .unwrap();
        assert!(stepped.set(5).is_ok());
        assert!(stepped.set(6).is_err());
    }
}
