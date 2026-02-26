//! Snap rounding utilities for floating-point precision handling.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/snap_rounding.hpp
//!            wagyu/include/mapbox/geometry/wagyu/util.hpp (rounding functions)
//!
//! This module provides functions for rounding floating-point coordinates to
//! integer grid points. This is essential for the Vatti polygon clipping algorithm
//! to handle floating-point precision issues and ensure consistent results.
//!
//! ## Key Functions
//!
//! - [`wround`]: Basic rounding (equivalent to `llround`)
//! - [`round_towards_min`]: Rounds towards minimum (0.5 rounds down)
//! - [`round_towards_max`]: Rounds towards maximum (0.5 rounds up)
//! - [`round_point`]: Rounds a floating-point point to integer coordinates
//! - [`values_are_equal`]: ULP-based floating-point comparison
//!
//! ## Hot Pixels
//!
//! The snap rounding algorithm uses "hot pixels" - grid points where edges intersect
//! or where special handling is needed. These are collected during a pre-processing
//! pass and used during the main clipping algorithm.

use crate::point::Point;
use geo_types::CoordNum;

/// Maximum ULPs (Units in the Last Place) for floating-point comparison.
///
/// This determines how close two floating-point numbers need to be to be
/// considered equal. A value of 4 is typically sufficient for geometry
/// operations where accumulated floating-point errors are minimal.
const MAX_ULPS: u64 = 4;

/// Compares two floating-point numbers for approximate equality using ULP comparison.
///
/// Two numbers are considered equal if their IEEE 754 bit representations are
/// within `MAX_ULPS` of each other. This handles edge cases like comparing
/// values near zero and accounts for floating-point representation errors.
///
/// # Arguments
///
/// * `x` - First floating-point value
/// * `y` - Second floating-point value
///
/// # Returns
///
/// `true` if the values are approximately equal, `false` otherwise.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::values_are_equal;
///
/// assert!(values_are_equal(1.0, 1.0));
/// assert!(values_are_equal(0.0, 0.0));
/// assert!(!values_are_equal(1.0, 2.0));
///
/// // Handles values very close together
/// let a = 0.1 + 0.2;
/// let b = 0.3;
/// // These may not be exactly equal due to floating-point representation
/// // but are considered equal by ULP comparison
/// ```
pub fn values_are_equal(x: f64, y: f64) -> bool {
    // Handle NaN - NaN is never equal to anything, including itself
    if x.is_nan() || y.is_nan() {
        return false;
    }

    let x_bits = x.to_bits();
    let y_bits = y.to_bits();

    // Convert to sign-and-magnitude representation for comparison
    let x_biased = sign_and_magnitude_to_biased(x_bits);
    let y_biased = sign_and_magnitude_to_biased(y_bits);

    // Calculate the distance in ULPs
    let distance = x_biased.abs_diff(y_biased);

    distance <= MAX_ULPS
}

/// Converts IEEE 754 sign-and-magnitude representation to a biased representation
/// for easier distance calculation.
///
/// In biased representation, negative numbers are mapped to small positive integers
/// and positive numbers are mapped to larger integers, making distance calculation
/// simpler.
#[inline]
fn sign_and_magnitude_to_biased(bits: u64) -> u64 {
    const SIGN_BIT_MASK: u64 = 1 << 63;

    if (bits & SIGN_BIT_MASK) != 0 {
        // Negative number: flip all bits and add 1 (two's complement)
        !bits + 1
    } else {
        // Positive number: set the sign bit
        bits | SIGN_BIT_MASK
    }
}

/// Returns `true` if the value is approximately zero.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::value_is_zero;
///
/// assert!(value_is_zero(0.0));
/// assert!(value_is_zero(-0.0));
/// assert!(!value_is_zero(1.0));
/// ```
#[inline]
pub fn value_is_zero(val: f64) -> bool {
    values_are_equal(val, 0.0)
}

/// Returns `true` if `x >= y` using approximate floating-point comparison.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::greater_than_or_equal;
///
/// assert!(greater_than_or_equal(2.0, 1.0));
/// assert!(greater_than_or_equal(1.0, 1.0));
/// assert!(!greater_than_or_equal(0.5, 1.0));
/// ```
#[inline]
pub fn greater_than_or_equal(x: f64, y: f64) -> bool {
    x > y || values_are_equal(x, y)
}

/// Returns `true` if `x > y` using approximate floating-point comparison.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::greater_than;
///
/// assert!(greater_than(2.0, 1.0));
/// assert!(!greater_than(1.0, 1.0));
/// assert!(!greater_than(0.5, 1.0));
/// ```
#[inline]
pub fn greater_than(x: f64, y: f64) -> bool {
    !values_are_equal(x, y) && x > y
}

/// Returns `true` if `x < y` using approximate floating-point comparison.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::less_than;
///
/// assert!(less_than(1.0, 2.0));
/// assert!(!less_than(1.0, 1.0));
/// assert!(!less_than(2.0, 1.0));
/// ```
#[inline]
pub fn less_than(x: f64, y: f64) -> bool {
    !values_are_equal(x, y) && x < y
}

/// Rounds a floating-point value to the nearest integer.
///
/// This is equivalent to the C++ `llround` function - it rounds to the
/// nearest integer, with halfway cases (0.5) rounding away from zero.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::wround;
///
/// assert_eq!(wround(1.4), 1_i64);
/// assert_eq!(wround(1.5), 2_i64);
/// assert_eq!(wround(1.6), 2_i64);
/// assert_eq!(wround(-1.5), -2_i64);
/// ```
#[inline]
pub fn wround(value: f64) -> i64 {
    value.round() as i64
}

/// Rounds a floating-point value towards the minimum (floor direction for ties).
///
/// For values exactly at the halfway point (x.5), this rounds towards negative
/// infinity (floor). For other values, it uses standard rounding.
///
/// This is used when calculating minimum bounds of edges to ensure the
/// result is conservative (never overestimates).
///
/// # Rounding Behavior
///
/// - `0.5` rounds to `0`
/// - `0.0` rounds to `0`
/// - `-0.5` rounds to `-1`
/// - `1.4` rounds to `1`
/// - `1.6` rounds to `2`
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::round_towards_min;
///
/// assert_eq!(round_towards_min(0.5), 0_i64);
/// assert_eq!(round_towards_min(0.0), 0_i64);
/// assert_eq!(round_towards_min(-0.5), -1_i64);
/// assert_eq!(round_towards_min(1.4), 1_i64);
/// assert_eq!(round_towards_min(1.6), 2_i64);
/// ```
pub fn round_towards_min(val: f64) -> i64 {
    let half = val.floor() + 0.5;
    if values_are_equal(val, half) {
        val.floor() as i64
    } else {
        val.round() as i64
    }
}

/// Rounds a floating-point value towards the maximum (ceiling direction for ties).
///
/// For values exactly at the halfway point (x.5), this rounds towards positive
/// infinity (ceiling). For other values, it uses standard rounding.
///
/// This is used when calculating maximum bounds of edges to ensure the
/// result is conservative (never underestimates).
///
/// # Rounding Behavior
///
/// - `0.5` rounds to `1`
/// - `0.0` rounds to `0`
/// - `-0.5` rounds to `0`
/// - `1.4` rounds to `1`
/// - `1.6` rounds to `2`
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::round_towards_max;
///
/// assert_eq!(round_towards_max(0.5), 1_i64);
/// assert_eq!(round_towards_max(0.0), 0_i64);
/// assert_eq!(round_towards_max(-0.5), 0_i64);
/// assert_eq!(round_towards_max(1.4), 1_i64);
/// assert_eq!(round_towards_max(1.6), 2_i64);
/// ```
pub fn round_towards_max(val: f64) -> i64 {
    let half = val.floor() + 0.5;
    if values_are_equal(val, half) {
        val.ceil() as i64
    } else {
        val.round() as i64
    }
}

/// Rounds a floating-point point to integer coordinates.
///
/// Uses `round_towards_max` for both x and y coordinates to ensure consistent
/// snapping behavior at grid boundaries.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::round_point;
/// use wagyu_rs::Point;
///
/// let float_pt: Point<f64> = Point::new(1.5, 2.5);
/// let int_pt: Point<i64> = round_point(&float_pt);
/// assert_eq!(int_pt.x, 2);
/// assert_eq!(int_pt.y, 3);
/// ```
pub fn round_point<T>(pt: &Point<f64>) -> Point<T>
where
    T: CoordNum + From<i64>,
{
    Point::new(
        <T as From<i64>>::from(round_towards_max(pt.x)),
        <T as From<i64>>::from(round_towards_max(pt.y)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // values_are_equal tests
    // =========================================================================

    #[test]
    fn test_values_are_equal_identical_values() {
        assert!(values_are_equal(1.0, 1.0));
        assert!(values_are_equal(0.0, 0.0));
        assert!(values_are_equal(-1.0, -1.0));
        assert!(values_are_equal(123456.789, 123456.789));
    }

    #[test]
    fn test_values_are_equal_positive_and_negative_zero() {
        assert!(values_are_equal(0.0, -0.0));
        assert!(values_are_equal(-0.0, 0.0));
    }

    #[test]
    fn test_values_are_equal_nan_is_never_equal() {
        assert!(!values_are_equal(f64::NAN, f64::NAN));
        assert!(!values_are_equal(f64::NAN, 0.0));
        assert!(!values_are_equal(0.0, f64::NAN));
    }

    #[test]
    fn test_values_are_equal_different_values() {
        assert!(!values_are_equal(1.0, 2.0));
        assert!(!values_are_equal(0.0, 1.0));
        assert!(!values_are_equal(-1.0, 1.0));
    }

    #[test]
    fn test_values_are_equal_very_close_values() {
        // Values that differ by only a few ULPs should be equal
        let a: f64 = 1.0;
        let b = f64::from_bits(a.to_bits() + 1); // 1 ULP difference
        assert!(values_are_equal(a, b));

        let c = f64::from_bits(a.to_bits() + 4); // 4 ULPs difference
        assert!(values_are_equal(a, c));
    }

    #[test]
    fn test_values_are_equal_beyond_ulp_threshold() {
        let a: f64 = 1.0;
        let b = f64::from_bits(a.to_bits() + 10); // 10 ULPs difference
        assert!(!values_are_equal(a, b));
    }

    #[test]
    fn test_values_are_equal_infinity() {
        assert!(values_are_equal(f64::INFINITY, f64::INFINITY));
        assert!(values_are_equal(f64::NEG_INFINITY, f64::NEG_INFINITY));
        assert!(!values_are_equal(f64::INFINITY, f64::NEG_INFINITY));
    }

    // =========================================================================
    // value_is_zero tests
    // =========================================================================

    #[test]
    fn test_value_is_zero_positive_zero() {
        assert!(value_is_zero(0.0));
    }

    #[test]
    fn test_value_is_zero_negative_zero() {
        assert!(value_is_zero(-0.0));
    }

    #[test]
    fn test_value_is_zero_non_zero_values() {
        assert!(!value_is_zero(1.0));
        assert!(!value_is_zero(-1.0));
        assert!(!value_is_zero(0.1));
        assert!(!value_is_zero(1e-100));
    }

    // =========================================================================
    // greater_than_or_equal tests
    // =========================================================================

    #[test]
    fn test_greater_than_or_equal_greater() {
        assert!(greater_than_or_equal(2.0, 1.0));
        assert!(greater_than_or_equal(1.0, 0.0));
        assert!(greater_than_or_equal(0.0, -1.0));
    }

    #[test]
    fn test_greater_than_or_equal_equal() {
        assert!(greater_than_or_equal(1.0, 1.0));
        assert!(greater_than_or_equal(0.0, 0.0));
        assert!(greater_than_or_equal(-1.0, -1.0));
    }

    #[test]
    fn test_greater_than_or_equal_less() {
        assert!(!greater_than_or_equal(1.0, 2.0));
        assert!(!greater_than_or_equal(0.0, 1.0));
        assert!(!greater_than_or_equal(-1.0, 0.0));
    }

    // =========================================================================
    // greater_than tests
    // =========================================================================

    #[test]
    fn test_greater_than_greater() {
        assert!(greater_than(2.0, 1.0));
        assert!(greater_than(1.0, 0.0));
        assert!(greater_than(0.0, -1.0));
    }

    #[test]
    fn test_greater_than_equal_returns_false() {
        assert!(!greater_than(1.0, 1.0));
        assert!(!greater_than(0.0, 0.0));
    }

    #[test]
    fn test_greater_than_less_returns_false() {
        assert!(!greater_than(1.0, 2.0));
        assert!(!greater_than(0.0, 1.0));
    }

    // =========================================================================
    // less_than tests
    // =========================================================================

    #[test]
    fn test_less_than_less() {
        assert!(less_than(1.0, 2.0));
        assert!(less_than(0.0, 1.0));
        assert!(less_than(-1.0, 0.0));
    }

    #[test]
    fn test_less_than_equal_returns_false() {
        assert!(!less_than(1.0, 1.0));
        assert!(!less_than(0.0, 0.0));
    }

    #[test]
    fn test_less_than_greater_returns_false() {
        assert!(!less_than(2.0, 1.0));
        assert!(!less_than(1.0, 0.0));
    }

    // =========================================================================
    // wround tests
    // =========================================================================

    #[test]
    fn test_wround_positive_values() {
        assert_eq!(wround(1.4), 1);
        assert_eq!(wround(1.5), 2);
        assert_eq!(wround(1.6), 2);
        assert_eq!(wround(2.5), 3);
    }

    #[test]
    fn test_wround_negative_values() {
        assert_eq!(wround(-1.4), -1);
        assert_eq!(wround(-1.5), -2);
        assert_eq!(wround(-1.6), -2);
        assert_eq!(wround(-2.5), -3);
    }

    #[test]
    fn test_wround_zero() {
        assert_eq!(wround(0.0), 0);
        assert_eq!(wround(-0.0), 0);
    }

    #[test]
    fn test_wround_whole_numbers() {
        assert_eq!(wround(5.0), 5);
        assert_eq!(wround(-5.0), -5);
        assert_eq!(wround(100.0), 100);
    }

    // =========================================================================
    // round_towards_min tests
    // =========================================================================

    #[test]
    fn test_round_towards_min_half_rounds_down() {
        // Key behavior: 0.5 rounds towards floor
        assert_eq!(round_towards_min(0.5), 0);
        assert_eq!(round_towards_min(1.5), 1);
        assert_eq!(round_towards_min(2.5), 2);
    }

    #[test]
    fn test_round_towards_min_negative_half_rounds_down() {
        // -0.5 should round to -1 (floor)
        assert_eq!(round_towards_min(-0.5), -1);
        assert_eq!(round_towards_min(-1.5), -2);
        assert_eq!(round_towards_min(-2.5), -3);
    }

    #[test]
    fn test_round_towards_min_non_half_uses_standard_rounding() {
        assert_eq!(round_towards_min(1.4), 1);
        assert_eq!(round_towards_min(1.6), 2);
        assert_eq!(round_towards_min(-1.4), -1);
        assert_eq!(round_towards_min(-1.6), -2);
    }

    #[test]
    fn test_round_towards_min_zero() {
        assert_eq!(round_towards_min(0.0), 0);
        assert_eq!(round_towards_min(-0.0), 0);
    }

    // =========================================================================
    // round_towards_max tests
    // =========================================================================

    #[test]
    fn test_round_towards_max_half_rounds_up() {
        // Key behavior: 0.5 rounds towards ceiling
        assert_eq!(round_towards_max(0.5), 1);
        assert_eq!(round_towards_max(1.5), 2);
        assert_eq!(round_towards_max(2.5), 3);
    }

    #[test]
    fn test_round_towards_max_negative_half_rounds_up() {
        // -0.5 should round to 0 (ceiling)
        assert_eq!(round_towards_max(-0.5), 0);
        assert_eq!(round_towards_max(-1.5), -1);
        assert_eq!(round_towards_max(-2.5), -2);
    }

    #[test]
    fn test_round_towards_max_non_half_uses_standard_rounding() {
        assert_eq!(round_towards_max(1.4), 1);
        assert_eq!(round_towards_max(1.6), 2);
        assert_eq!(round_towards_max(-1.4), -1);
        assert_eq!(round_towards_max(-1.6), -2);
    }

    #[test]
    fn test_round_towards_max_zero() {
        assert_eq!(round_towards_max(0.0), 0);
        assert_eq!(round_towards_max(-0.0), 0);
    }

    // =========================================================================
    // round_point tests
    // =========================================================================

    #[test]
    fn test_round_point_positive_coordinates() {
        let pt: Point<f64> = Point::new(1.5, 2.5);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 2); // 1.5 -> 2 (round_towards_max)
        assert_eq!(rounded.y, 3); // 2.5 -> 3 (round_towards_max)
    }

    #[test]
    fn test_round_point_negative_coordinates() {
        let pt: Point<f64> = Point::new(-1.5, -2.5);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, -1); // -1.5 -> -1 (round_towards_max)
        assert_eq!(rounded.y, -2); // -2.5 -> -2 (round_towards_max)
    }

    #[test]
    fn test_round_point_mixed_coordinates() {
        let pt: Point<f64> = Point::new(-0.5, 0.5);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 0); // -0.5 -> 0 (round_towards_max)
        assert_eq!(rounded.y, 1); // 0.5 -> 1 (round_towards_max)
    }

    #[test]
    fn test_round_point_whole_numbers() {
        let pt: Point<f64> = Point::new(5.0, 10.0);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 5);
        assert_eq!(rounded.y, 10);
    }

    #[test]
    fn test_round_point_non_half_values() {
        let pt: Point<f64> = Point::new(1.4, 2.6);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 1); // standard rounding
        assert_eq!(rounded.y, 3); // standard rounding
    }

    // =========================================================================
    // Symmetry tests (round_towards_min vs round_towards_max)
    // =========================================================================

    #[test]
    fn test_rounding_symmetry_for_half_values() {
        // For x.5, round_towards_min should give floor
        // and round_towards_max should give ceiling
        assert_eq!(round_towards_min(1.5), 1);
        assert_eq!(round_towards_max(1.5), 2);

        assert_eq!(round_towards_min(-1.5), -2);
        assert_eq!(round_towards_max(-1.5), -1);
    }

    #[test]
    fn test_rounding_same_for_non_half_values() {
        // For non-half values, both should give the same result
        assert_eq!(round_towards_min(1.3), round_towards_max(1.3));
        assert_eq!(round_towards_min(1.7), round_towards_max(1.7));
        assert_eq!(round_towards_min(-1.3), round_towards_max(-1.3));
        assert_eq!(round_towards_min(-1.7), round_towards_max(-1.7));
    }

    // =========================================================================
    // Edge case tests
    // =========================================================================

    #[test]
    fn test_large_values() {
        let large = 1e15;
        assert_eq!(wround(large), large as i64);
        assert_eq!(round_towards_min(large), large as i64);
        assert_eq!(round_towards_max(large), large as i64);
    }

    #[test]
    fn test_small_values_near_zero() {
        assert_eq!(wround(0.001), 0);
        assert_eq!(wround(-0.001), 0);
        assert_eq!(round_towards_min(0.001), 0);
        assert_eq!(round_towards_max(0.001), 0);
    }

    #[test]
    fn test_values_very_close_to_half() {
        // Values just below 0.5 should round to 0
        let just_below = 0.5 - 1e-10;
        assert_eq!(round_towards_min(just_below), 0);
        assert_eq!(round_towards_max(just_below), 0);

        // Values just above 0.5 should round to 1
        let just_above = 0.5 + 1e-10;
        assert_eq!(round_towards_min(just_above), 1);
        assert_eq!(round_towards_max(just_above), 1);
    }
}
