use std::ops::{Range, RangeFrom, RangeFull, RangeInclusive, RangeTo, RangeToInclusive};

use crate::{DeclarationError, ParameterValue, ParameterVariant};
use ros_env::rcl_interfaces::msg::rmw::{FloatingPointRange, IntegerRange};
use rosidl_runtime_rs::{seq, BoundedSequence};

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
    fn from(_range: RangeFull) -> Self {
        Self {
            lower: None,
            upper: None,
            step: None,
        }
    }
}

impl From<Range<i64>> for ParameterRange<i64> {
    fn from(range: Range<i64>) -> Self {
        Self::from_exclusive_bounds(Some(range.start), range.end)
    }
}

impl From<RangeTo<i64>> for ParameterRange<i64> {
    fn from(range: RangeTo<i64>) -> Self {
        Self::from_exclusive_bounds(None, range.end)
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
    fn from_exclusive_bounds(lower: Option<i64>, end: i64) -> Self {
        match end.checked_sub(1) {
            Some(upper) => Self {
                lower,
                upper: Some(upper),
                step: None,
            },
            None => Self {
                lower: Some(i64::MAX),
                upper: Some(i64::MIN),
                step: None,
            },
        }
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_ranges_convert_to_parameter_ranges() {
        let inclusive = ParameterRange::from(1_i64..=3);
        assert_eq!(inclusive.lower, Some(1));
        assert_eq!(inclusive.upper, Some(3));

        let from = ParameterRange::from(1.0_f64..);
        assert_eq!(from.lower, Some(1.0));
        assert_eq!(from.upper, None);

        let to_inclusive = ParameterRange::from(..=3.0_f64);
        assert_eq!(to_inclusive.lower, None);
        assert_eq!(to_inclusive.upper, Some(3.0));

        let full: ParameterRange<i64> = ParameterRange::from(..);
        assert!(full.is_default());
    }

    #[test]
    fn exclusive_integer_ranges_adjust_the_upper_bound() {
        let bounded = ParameterRange::from(1_i64..3);
        assert_eq!(bounded.lower, Some(1));
        assert_eq!(bounded.upper, Some(2));

        let to = ParameterRange::from(..3_i64);
        assert_eq!(to.lower, None);
        assert_eq!(to.upper, Some(2));
    }

    #[test]
    fn exclusive_integer_range_without_a_predecessor_is_invalid() {
        let range = ParameterRange::from(..i64::MIN);
        assert!(matches!(
            range.validate(),
            Err(DeclarationError::InvalidRange)
        ));
    }
}
