//! ULP-based floating-point comparison utilities.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/almost_equal.hpp
//!
//! This module provides floating-point comparison using ULPs (Units in the Last Place),
//! adapted from Google Test. Two floating-point numbers are considered equal if they
//! are within a certain number of ULPs (default: 4) of each other.
//!
//! # IEEE 754 Floating-Point Format
//!
//! A floating-point number is stored as: `sign_bit | exponent_bits | fraction_bits`
//! - f32: 1 sign + 8 exponent + 23 fraction = 32 bits
//! - f64: 1 sign + 11 exponent + 52 fraction = 64 bits
//!
//! # License
//!
//! Original C++ code Copyright 2005, Google Inc. (BSD License)
//! Authors: wan@google.com (Zhanyong Wan), eefacm@gmail.com (Sean Mcafee)

/// Maximum ULPs (Units in Last Place) for floating-point comparison.
///
/// The maximum error of a single floating-point operation is 0.5 ULP.
/// On Intel CPUs, all floating-point calculations use 80-bit precision,
/// while f64 has 64 bits. Therefore, 4 should be enough for ordinary use.
pub const MAX_ULPS: u64 = 4;

/// Computes the bit representation of a f64.
#[inline]
fn to_bits(value: f64) -> u64 {
    value.to_bits()
}

/// Creates a f64 from its bit representation.
#[inline]
fn from_bits(bits: u64) -> f64 {
    f64::from_bits(bits)
}

// Constants for f64
const BIT_COUNT: usize = 64;
const FRACTION_BIT_COUNT: usize = 52;
const SIGN_BIT_MASK: u64 = 1 << (BIT_COUNT - 1);
const FRACTION_BIT_MASK: u64 = (1_u64 << FRACTION_BIT_COUNT) - 1;
// Exponent bits are: all bits except sign and fraction
const EXPONENT_BIT_MASK: u64 = !SIGN_BIT_MASK & !FRACTION_BIT_MASK;

/// Returns the exponent bits of a floating-point number.
#[inline]
fn exponent_bits(bits: u64) -> u64 {
    bits & EXPONENT_BIT_MASK
}

/// Returns the fraction bits of a floating-point number.
#[inline]
fn fraction_bits(bits: u64) -> u64 {
    bits & FRACTION_BIT_MASK
}

/// Returns true if the given bits represent NaN.
///
/// A number is NaN if the exponent bits are all ones and the fraction bits
/// are not entirely zeros.
#[inline]
fn is_nan_bits(bits: u64) -> bool {
    exponent_bits(bits) == EXPONENT_BIT_MASK && fraction_bits(bits) != 0
}

/// Converts a number from sign-and-magnitude representation to biased representation.
///
/// Let N be 2^(kBitCount - 1). An integer x is represented by the unsigned number x + N.
///
/// For instance:
/// - `-N + 1` (most negative) is represented by 1
/// - `0` is represented by N
/// - `N - 1` (most positive) is represented by 2N - 1
#[inline]
fn sign_and_magnitude_to_biased(sam: u64) -> u64 {
    if (SIGN_BIT_MASK & sam) != 0 {
        // sam represents a negative number
        !sam + 1
    } else {
        // sam represents a positive number
        SIGN_BIT_MASK | sam
    }
}

/// Given two numbers in sign-and-magnitude representation,
/// returns the distance between them as an unsigned number.
#[inline]
fn distance_between_sign_and_magnitude_numbers(sam1: u64, sam2: u64) -> u64 {
    let biased1 = sign_and_magnitude_to_biased(sam1);
    let biased2 = sign_and_magnitude_to_biased(sam2);
    biased1.abs_diff(biased2)
}

/// Checks if two f64 values are almost equal within the default ULP tolerance.
///
/// Two numbers are considered almost equal if they are within `MAX_ULPS` (4)
/// units in the last place of each other.
///
/// # Returns
///
/// - `false` if either number is NaN (IEEE standard: NaN comparisons return false)
/// - `true` if the numbers are within the ULP tolerance
/// - `false` otherwise
///
/// # Special Cases
///
/// - +0.0 and -0.0 are considered 0 ULPs apart (equal)
/// - Very large numbers are considered almost equal to infinity
///
/// # Examples
///
/// ```
/// use wagyu_rs::almost_equal::almost_equal;
///
/// assert!(almost_equal(1.0, 1.0));
/// assert!(almost_equal(0.0, -0.0)); // +0 and -0 are equal
/// assert!(!almost_equal(1.0, 2.0));
/// assert!(!almost_equal(f64::NAN, f64::NAN)); // NaN is never equal
/// ```
#[inline]
pub fn almost_equal(x: f64, y: f64) -> bool {
    almost_equal_ulps(x, y, MAX_ULPS)
}

/// Checks if two f64 values are almost equal within a specified ULP tolerance.
///
/// This is the generic version that allows specifying the maximum ULP distance.
///
/// # Arguments
///
/// * `x` - First floating-point value
/// * `y` - Second floating-point value
/// * `max_ulps` - Maximum allowed ULP distance (0 means exact equality required)
///
/// # Examples
///
/// ```
/// use wagyu_rs::almost_equal::almost_equal_ulps;
///
/// // Exact comparison (0 ULPs)
/// assert!(almost_equal_ulps(1.0, 1.0, 0));
///
/// // Allow some tolerance
/// assert!(almost_equal_ulps(1.0, 1.0000000000000002, 1));
/// ```
#[inline]
pub fn almost_equal_ulps(x: f64, y: f64, max_ulps: u64) -> bool {
    let x_bits = to_bits(x);
    let y_bits = to_bits(y);

    // IEEE standard: any comparison involving NaN must return false
    if is_nan_bits(x_bits) || is_nan_bits(y_bits) {
        return false;
    }

    distance_between_sign_and_magnitude_numbers(x_bits, y_bits) <= max_ulps
}

/// Returns positive infinity for f64.
#[inline]
pub fn infinity() -> f64 {
    from_bits(EXPONENT_BIT_MASK)
}

// =============================================================================
// f32 support
// =============================================================================

// Constants for f32
const F32_BIT_COUNT: usize = 32;
const F32_FRACTION_BIT_COUNT: usize = 23;
const F32_SIGN_BIT_MASK: u32 = 1 << (F32_BIT_COUNT - 1);
const F32_FRACTION_BIT_MASK: u32 = (1_u32 << F32_FRACTION_BIT_COUNT) - 1;
const F32_EXPONENT_BIT_MASK: u32 = !F32_SIGN_BIT_MASK & !F32_FRACTION_BIT_MASK;

/// Maximum ULPs for f32 comparison.
pub const MAX_ULPS_F32: u32 = 4;

#[inline]
fn is_nan_bits_f32(bits: u32) -> bool {
    (bits & F32_EXPONENT_BIT_MASK) == F32_EXPONENT_BIT_MASK && (bits & F32_FRACTION_BIT_MASK) != 0
}

#[inline]
fn sign_and_magnitude_to_biased_f32(sam: u32) -> u32 {
    if (F32_SIGN_BIT_MASK & sam) != 0 {
        !sam + 1
    } else {
        F32_SIGN_BIT_MASK | sam
    }
}

#[inline]
fn distance_between_sign_and_magnitude_numbers_f32(sam1: u32, sam2: u32) -> u32 {
    let biased1 = sign_and_magnitude_to_biased_f32(sam1);
    let biased2 = sign_and_magnitude_to_biased_f32(sam2);
    biased1.abs_diff(biased2)
}

/// Checks if two f32 values are almost equal within the default ULP tolerance.
#[inline]
pub fn almost_equal_f32(x: f32, y: f32) -> bool {
    almost_equal_ulps_f32(x, y, MAX_ULPS_F32)
}

/// Checks if two f32 values are almost equal within a specified ULP tolerance.
#[inline]
pub fn almost_equal_ulps_f32(x: f32, y: f32, max_ulps: u32) -> bool {
    let x_bits = x.to_bits();
    let y_bits = y.to_bits();

    if is_nan_bits_f32(x_bits) || is_nan_bits_f32(y_bits) {
        return false;
    }

    distance_between_sign_and_magnitude_numbers_f32(x_bits, y_bits) <= max_ulps
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // Test: Equal values should be almost equal
    // =========================================================================

    #[test]
    fn test_equal_values_are_almost_equal() {
        assert!(almost_equal(1.0, 1.0));
        assert!(almost_equal(0.0, 0.0));
        assert!(almost_equal(-1.0, -1.0));
        assert!(almost_equal(1e10, 1e10));
        assert!(almost_equal(1e-10, 1e-10));
    }

    // =========================================================================
    // Test: Positive and negative zero are equal
    // =========================================================================

    #[test]
    fn test_positive_and_negative_zero_are_equal() {
        assert!(almost_equal(0.0, -0.0));
        assert!(almost_equal(-0.0, 0.0));
    }

    // =========================================================================
    // Test: NaN comparisons
    // =========================================================================

    #[test]
    fn test_nan_is_not_almost_equal_to_nan() {
        assert!(!almost_equal(f64::NAN, f64::NAN));
    }

    #[test]
    fn test_nan_is_not_almost_equal_to_any_number() {
        assert!(!almost_equal(f64::NAN, 0.0));
        assert!(!almost_equal(0.0, f64::NAN));
        assert!(!almost_equal(f64::NAN, 1.0));
        assert!(!almost_equal(1.0, f64::NAN));
        assert!(!almost_equal(f64::NAN, f64::INFINITY));
    }

    // =========================================================================
    // Test: Infinity comparisons
    // =========================================================================

    #[test]
    fn test_infinity_equals_infinity() {
        assert!(almost_equal(f64::INFINITY, f64::INFINITY));
        assert!(almost_equal(f64::NEG_INFINITY, f64::NEG_INFINITY));
    }

    #[test]
    fn test_positive_infinity_not_equal_to_negative_infinity() {
        assert!(!almost_equal(f64::INFINITY, f64::NEG_INFINITY));
    }

    #[test]
    fn test_infinity_helper_function() {
        assert_eq!(infinity(), f64::INFINITY);
    }

    // =========================================================================
    // Test: Different values are not almost equal
    // =========================================================================

    #[test]
    fn test_clearly_different_values_are_not_almost_equal() {
        assert!(!almost_equal(1.0, 2.0));
        assert!(!almost_equal(0.0, 1.0));
        assert!(!almost_equal(-1.0, 1.0));
    }

    // =========================================================================
    // Test: Very close values within ULP tolerance
    // =========================================================================

    #[test]
    fn test_values_within_ulp_tolerance() {
        // The next representable f64 after 1.0
        let one = 1.0_f64;
        let one_plus_ulp = f64::from_bits(one.to_bits() + 1);
        let one_plus_4ulp = f64::from_bits(one.to_bits() + 4);

        assert!(almost_equal(one, one_plus_ulp));
        assert!(almost_equal(one, one_plus_4ulp));
    }

    #[test]
    fn test_values_beyond_ulp_tolerance() {
        let one = 1.0_f64;
        let one_plus_5ulp = f64::from_bits(one.to_bits() + 5);

        assert!(!almost_equal(one, one_plus_5ulp));
    }

    // =========================================================================
    // Test: Custom ULP tolerance
    // =========================================================================

    #[test]
    fn test_exact_comparison_with_zero_ulps() {
        assert!(almost_equal_ulps(1.0, 1.0, 0));
        assert!(almost_equal_ulps(0.0, -0.0, 0)); // +0 and -0 are the same
    }

    #[test]
    fn test_custom_ulp_tolerance() {
        let one = 1.0_f64;
        let one_plus_10ulp = f64::from_bits(one.to_bits() + 10);

        assert!(!almost_equal_ulps(one, one_plus_10ulp, 4));
        assert!(almost_equal_ulps(one, one_plus_10ulp, 10));
        assert!(almost_equal_ulps(one, one_plus_10ulp, 20));
    }

    // =========================================================================
    // Test: Negative numbers near zero
    // =========================================================================

    #[test]
    fn test_small_negative_numbers() {
        let neg_small = -1e-15_f64;
        let neg_small_close = f64::from_bits(neg_small.to_bits() + 1);

        assert!(almost_equal(neg_small, neg_small_close));
    }

    // =========================================================================
    // Test: f32 support
    // =========================================================================

    #[test]
    fn test_f32_equal_values() {
        assert!(almost_equal_f32(1.0_f32, 1.0_f32));
        assert!(almost_equal_f32(0.0_f32, -0.0_f32));
    }

    #[test]
    fn test_f32_nan() {
        assert!(!almost_equal_f32(f32::NAN, f32::NAN));
        assert!(!almost_equal_f32(f32::NAN, 1.0_f32));
    }

    #[test]
    fn test_f32_infinity() {
        assert!(almost_equal_f32(f32::INFINITY, f32::INFINITY));
        assert!(!almost_equal_f32(f32::INFINITY, f32::NEG_INFINITY));
    }

    #[test]
    fn test_f32_within_ulp_tolerance() {
        let one = 1.0_f32;
        let one_plus_ulp = f32::from_bits(one.to_bits() + 1);
        let one_plus_4ulp = f32::from_bits(one.to_bits() + 4);

        assert!(almost_equal_f32(one, one_plus_ulp));
        assert!(almost_equal_f32(one, one_plus_4ulp));
    }

    #[test]
    fn test_f32_beyond_ulp_tolerance() {
        let one = 1.0_f32;
        let one_plus_5ulp = f32::from_bits(one.to_bits() + 5);

        assert!(!almost_equal_f32(one, one_plus_5ulp));
    }

    // =========================================================================
    // Test: Internal helper functions
    // =========================================================================

    #[test]
    fn test_sign_and_magnitude_conversion() {
        // For positive numbers, biased = SIGN_BIT_MASK | sam
        let pos_bits = 1.0_f64.to_bits();
        let biased_pos = sign_and_magnitude_to_biased(pos_bits);
        assert_eq!(biased_pos, SIGN_BIT_MASK | pos_bits);

        // For negative numbers, biased = !sam + 1 (two's complement negation)
        let neg_bits = (-1.0_f64).to_bits();
        let biased_neg = sign_and_magnitude_to_biased(neg_bits);
        assert_eq!(biased_neg, !neg_bits + 1);
    }

    #[test]
    fn test_distance_between_equal_numbers() {
        let bits = 1.0_f64.to_bits();
        assert_eq!(distance_between_sign_and_magnitude_numbers(bits, bits), 0);
    }

    #[test]
    fn test_distance_between_adjacent_numbers() {
        let bits1 = 1.0_f64.to_bits();
        let bits2 = bits1 + 1;
        assert_eq!(distance_between_sign_and_magnitude_numbers(bits1, bits2), 1);
    }
}
