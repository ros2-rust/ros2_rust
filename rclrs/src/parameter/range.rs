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

/// Describes the range for parameter type T.
///
/// Standard Rust ranges can be converted into this type without a step size.
/// Inclusive and unbounded ranges support both integers and floating-point values;
/// exclusive upper bounds are supported only for integers. Empty or reversed integer
/// ranges are rejected when declaring the parameter, including exclusive bounds at
/// [`i64::MIN`].
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

impl From<Range<i64>> for ParameterRange<i64> {
    fn from(range: Range<i64>) -> Self {
        match range.end.checked_sub(1) {
            Some(upper) => Self {
                lower: Some(range.start),
                upper: Some(upper),
                step: None,
            },
            None => Self::empty(),
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

impl<T: ParameterVariant + PartialOrd> From<RangeFull> for ParameterRange<T> {
    fn from(_: RangeFull) -> Self {
        Self {
            lower: None,
            upper: None,
            step: None,
        }
    }
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

impl From<RangeTo<i64>> for ParameterRange<i64> {
    fn from(range: RangeTo<i64>) -> Self {
        match range.end.checked_sub(1) {
            Some(upper) => Self {
                lower: None,
                upper: Some(upper),
                step: None,
            },
            None => Self::empty(),
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
    fn empty() -> Self {
        // No integer is below i64::MIN. Keep an invalid range so declaration returns
        // InvalidRange through the existing validation path instead of panicking.
        Self {
            lower: Some(0),
            upper: Some(-1),
            step: None,
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
    fn standard_integer_ranges_preserve_membership() {
        let cases = [
            ((-2..=2).into(), vec![-2, 0, 2], vec![-3, 3]),
            ((-2..2).into(), vec![-2, 0, 1], vec![-3, 2]),
            ((-2..).into(), vec![-2, 0, i64::MAX], vec![-3]),
            ((..=2).into(), vec![i64::MIN, 0, 2], vec![3]),
            ((..2).into(), vec![i64::MIN, 0, 1], vec![2]),
            ((..).into(), vec![i64::MIN, 0, i64::MAX], vec![]),
            ((i64::MIN..=i64::MIN).into(), vec![i64::MIN], vec![0]),
            ((i64::MAX..=i64::MAX).into(), vec![i64::MAX], vec![0]),
            ((..i64::MAX).into(), vec![i64::MAX - 1], vec![i64::MAX]),
            ((..i64::MIN + 1).into(), vec![i64::MIN], vec![i64::MIN + 1]),
        ];
        for (range, accepted, rejected) in cases {
            let range: ParameterRange<i64> = range;
            assert!(range.validate().is_ok(), "{range:?}");
            assert_eq!(range.step, None);
            for value in accepted {
                assert!(range.in_range(value), "{value} in {range:?}");
            }
            for value in rejected {
                assert!(!range.in_range(value), "{value} outside {range:?}");
            }
        }
    }

    #[test]
    fn standard_float_ranges_preserve_membership() {
        let cases = [
            ((-2.0..=2.0).into(), vec![-2.0, 0.0, 2.0], vec![-3.0, 3.0]),
            ((-2.0..).into(), vec![-2.0, 0.0, f64::MAX], vec![-3.0]),
            ((..=2.0).into(), vec![f64::MIN, 0.0, 2.0], vec![3.0]),
            ((..).into(), vec![f64::MIN, 0.0, f64::MAX], vec![]),
        ];
        for (range, accepted, rejected) in cases {
            let range: ParameterRange<f64> = range;
            assert!(range.validate().is_ok(), "{range:?}");
            assert_eq!(range.step, None);
            for value in accepted {
                assert!(range.in_range(value), "{value} in {range:?}");
            }
            for value in rejected {
                assert!(!range.in_range(value), "{value} outside {range:?}");
            }
        }
    }

    #[test]
    fn empty_integer_ranges_are_invalid_without_overflow() {
        let bounds = [(0, 0), (2, -2), (i64::MIN, i64::MIN), (i64::MAX, i64::MIN)];
        let mut ranges: Vec<ParameterRange<i64>> = bounds
            .into_iter()
            .map(|(lower, upper)| (lower..upper).into())
            .collect();
        ranges.push((..i64::MIN).into());
        ranges.push(RangeInclusive::new(2, -2).into());
        for range in ranges {
            assert!(matches!(
                range.validate(),
                Err(DeclarationError::InvalidRange)
            ));
            for value in [i64::MIN, -1, 0, i64::MAX] {
                assert!(!range.in_range(value));
            }
        }
        let reversed: ParameterRange<f64> = (2.0..=-2.0).into();
        assert!(matches!(
            reversed.validate(),
            Err(DeclarationError::InvalidRange)
        ));
    }

    #[test]
    fn converted_ranges_report_ros_descriptor_bounds() {
        let range = ParameterRanges::from(ParameterRange::from(0..10));
        let (integer, float) = range.to_descriptor_ranges();
        assert_eq!(integer.len(), 1);
        assert_eq!(integer[0].from_value, 0);
        assert_eq!(integer[0].to_value, 9);
        assert_eq!(integer[0].step, 0);
        assert!(float.is_empty());

        let range = ParameterRanges::from(ParameterRange::from(..=0.0));
        let (integer, float) = range.to_descriptor_ranges();
        assert!(integer.is_empty());
        assert_eq!(float.len(), 1);
        assert_eq!(float[0].from_value, f64::NEG_INFINITY);
        assert_eq!(float[0].to_value, 0.0);
        assert_eq!(float[0].step, 0.0);

        let range = ParameterRanges::from(ParameterRange::<i64>::from(..));
        let (integer, float) = range.to_descriptor_ranges();
        assert!(integer.is_empty());
        assert!(float.is_empty());
    }
}
