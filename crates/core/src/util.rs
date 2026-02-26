//! General utility functions for wagyu.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/util.hpp
//!
//! This module provides:
//! - `area()` - Compute signed area of a ring (Shoelace formula)
//! - `values_are_equal()` - ULP-based floating point comparison
//! - `value_is_zero()` - Check if value is approximately zero
//! - Comparison functions: `greater_than`, `less_than`, `greater_than_or_equal`
//! - `slopes_equal()` - Collinearity check using exact integer arithmetic
//! - `wround()` - Rounding helper for f64 to integer conversion

use crate::almost_equal::almost_equal;
use crate::point::Point;
use geo_types::CoordNum;
use num_traits::AsPrimitive;

// =============================================================================
// Floating-point comparison utilities
// =============================================================================

/// Checks if two f64 values are almost equal using ULP comparison.
///
/// This is a convenience wrapper around `almost_equal::almost_equal`.
///
/// # Examples
///
/// ```
/// use wagyu_rs::util::values_are_equal;
///
/// assert!(values_are_equal(1.0, 1.0));
/// assert!(values_are_equal(0.0, -0.0));
/// assert!(!values_are_equal(1.0, 2.0));
/// ```
#[inline]
pub fn values_are_equal(x: f64, y: f64) -> bool {
    almost_equal(x, y)
}

/// Checks if a f64 value is approximately zero.
///
/// # Examples
///
/// ```
/// use wagyu_rs::util::value_is_zero;
///
/// assert!(value_is_zero(0.0));
/// assert!(value_is_zero(-0.0));
/// assert!(!value_is_zero(1.0));
/// ```
#[inline]
pub fn value_is_zero(val: f64) -> bool {
    values_are_equal(val, 0.0)
}

/// Checks if x is greater than or equal to y, with floating-point tolerance.
///
/// Returns true if x > y OR if x and y are almost equal.
#[inline]
pub fn greater_than_or_equal(x: f64, y: f64) -> bool {
    x > y || values_are_equal(x, y)
}

/// Checks if x is strictly greater than y, with floating-point tolerance.
///
/// Returns true only if x > y AND x and y are NOT almost equal.
#[inline]
pub fn greater_than(x: f64, y: f64) -> bool {
    !values_are_equal(x, y) && x > y
}

/// Checks if x is strictly less than y, with floating-point tolerance.
///
/// Returns true only if x < y AND x and y are NOT almost equal.
#[inline]
pub fn less_than(x: f64, y: f64) -> bool {
    !values_are_equal(x, y) && x < y
}

// =============================================================================
// Area calculation (Shoelace formula)
// =============================================================================

/// Computes the signed area of a linear ring using the Shoelace formula.
///
/// The sign indicates winding direction:
/// - Positive area = counter-clockwise winding (in screen coordinates where Y increases downward)
/// - Negative area = clockwise winding
///
/// For a polygon with holes, outer rings have one sign and holes have the opposite.
///
/// # Arguments
///
/// * `ring` - A slice of points representing a closed linear ring
///
/// # Returns
///
/// The signed area of the ring, or 0.0 if the ring has fewer than 3 points.
///
/// # Examples
///
/// ```
/// use wagyu_rs::point::Point;
/// use wagyu_rs::util::area;
///
/// // Counter-clockwise square (area = 100)
/// let ccw_square: Vec<Point<i64>> = vec![
///     Point::new(0, 0),
///     Point::new(10, 0),
///     Point::new(10, 10),
///     Point::new(0, 10),
///     Point::new(0, 0),
/// ];
/// let a = area(&ccw_square);
/// assert!(a > 0.0);
///
/// // Clockwise square (area = -100)
/// let cw_square: Vec<Point<i64>> = vec![
///     Point::new(0, 0),
///     Point::new(0, 10),
///     Point::new(10, 10),
///     Point::new(10, 0),
///     Point::new(0, 0),
/// ];
/// let a = area(&cw_square);
/// assert!(a < 0.0);
/// ```
pub fn area<T: CoordNum>(ring: &[Point<T>]) -> f64 {
    let size = ring.len();
    if size < 3 {
        return 0.0;
    }

    let mut a = 0.0;

    // Start with the edge from last point to first point
    let last = &ring[size - 1];
    let first = &ring[0];
    let last_x = last.x.to_f64().unwrap_or(0.0);
    let last_y = last.y.to_f64().unwrap_or(0.0);
    let first_x = first.x.to_f64().unwrap_or(0.0);
    let first_y = first.y.to_f64().unwrap_or(0.0);

    a += (last_x + first_x) * (last_y - first_y);

    // Process remaining edges
    for i in 1..size {
        let prev = &ring[i - 1];
        let curr = &ring[i];
        let prev_x = prev.x.to_f64().unwrap_or(0.0);
        let prev_y = prev.y.to_f64().unwrap_or(0.0);
        let curr_x = curr.x.to_f64().unwrap_or(0.0);
        let curr_y = curr.y.to_f64().unwrap_or(0.0);

        a += (prev_x + curr_x) * (prev_y - curr_y);
    }

    -a * 0.5
}

// =============================================================================
// Slopes equal (collinearity check)
// =============================================================================

/// Checks if three points are collinear using exact integer arithmetic.
///
/// This avoids floating-point errors by using the cross product:
/// `(y1-y2)*(x2-x3) == (x1-x2)*(y2-y3)`
///
/// The computation uses i64 to avoid overflow for typical i32 inputs.
///
/// # Examples
///
/// ```
/// use wagyu_rs::point::Point;
/// use wagyu_rs::util::slopes_equal_3pts;
///
/// // Collinear points on the line y = x
/// let p1: Point<i64> = Point::new(0, 0);
/// let p2: Point<i64> = Point::new(5, 5);
/// let p3: Point<i64> = Point::new(10, 10);
/// assert!(slopes_equal_3pts(&p1, &p2, &p3));
///
/// // Non-collinear points
/// let p4: Point<i64> = Point::new(0, 0);
/// let p5: Point<i64> = Point::new(5, 5);
/// let p6: Point<i64> = Point::new(10, 0);
/// assert!(!slopes_equal_3pts(&p4, &p5, &p6));
/// ```
pub fn slopes_equal_3pts<T>(pt1: &Point<T>, pt2: &Point<T>, pt3: &Point<T>) -> bool
where
    T: CoordNum + AsPrimitive<i64>,
{
    let y1: i64 = pt1.y.as_();
    let y2: i64 = pt2.y.as_();
    let y3: i64 = pt3.y.as_();
    let x1: i64 = pt1.x.as_();
    let x2: i64 = pt2.x.as_();
    let x3: i64 = pt3.x.as_();

    (y1 - y2) * (x2 - x3) == (x1 - x2) * (y2 - y3)
}

/// Checks if two line segments have equal slopes using exact integer arithmetic.
///
/// Segment 1: pt1 -> pt2
/// Segment 2: pt3 -> pt4
///
/// Uses cross product: `(y1-y2)*(x3-x4) == (x1-x2)*(y3-y4)`
///
/// # Examples
///
/// ```
/// use wagyu_rs::point::Point;
/// use wagyu_rs::util::slopes_equal_4pts;
///
/// // Parallel lines
/// let p1: Point<i64> = Point::new(0, 0);
/// let p2: Point<i64> = Point::new(10, 10);
/// let p3: Point<i64> = Point::new(0, 1);
/// let p4: Point<i64> = Point::new(10, 11);
/// assert!(slopes_equal_4pts(&p1, &p2, &p3, &p4));
/// ```
pub fn slopes_equal_4pts<T>(pt1: &Point<T>, pt2: &Point<T>, pt3: &Point<T>, pt4: &Point<T>) -> bool
where
    T: CoordNum + AsPrimitive<i64>,
{
    let y1: i64 = pt1.y.as_();
    let y2: i64 = pt2.y.as_();
    let y3: i64 = pt3.y.as_();
    let y4: i64 = pt4.y.as_();
    let x1: i64 = pt1.x.as_();
    let x2: i64 = pt2.x.as_();
    let x3: i64 = pt3.x.as_();
    let x4: i64 = pt4.x.as_();

    (y1 - y2) * (x3 - x4) == (x1 - x2) * (y3 - y4)
}

// =============================================================================
// Rounding utility
// =============================================================================

/// Rounds a f64 value to the nearest integer of type T.
///
/// This is equivalent to `llround` in C++ for integer types.
///
/// # Examples
///
/// ```
/// use wagyu_rs::util::wround;
///
/// assert_eq!(wround::<i64>(1.5), 2);
/// assert_eq!(wround::<i64>(1.4), 1);
/// assert_eq!(wround::<i64>(-1.5), -2);
/// assert_eq!(wround::<i32>(2.7), 3);
/// ```
#[inline]
pub fn wround<T>(value: f64) -> T
where
    T: CoordNum + 'static,
    i64: AsPrimitive<T>,
{
    let rounded = value.round() as i64;
    rounded.as_()
}

/// Specialized version for f64 that returns an f64 (no conversion needed).
#[inline]
pub fn wround_f64(value: f64) -> f64 {
    value.round()
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // values_are_equal tests
    // =========================================================================

    #[test]
    fn test_values_are_equal_same_values() {
        assert!(values_are_equal(1.0, 1.0));
        assert!(values_are_equal(0.0, 0.0));
        assert!(values_are_equal(-1.0, -1.0));
    }

    #[test]
    fn test_values_are_equal_positive_negative_zero() {
        assert!(values_are_equal(0.0, -0.0));
        assert!(values_are_equal(-0.0, 0.0));
    }

    #[test]
    fn test_values_are_equal_different_values() {
        assert!(!values_are_equal(1.0, 2.0));
        assert!(!values_are_equal(0.0, 1.0));
    }

    // =========================================================================
    // value_is_zero tests
    // =========================================================================

    #[test]
    fn test_value_is_zero_for_zero() {
        assert!(value_is_zero(0.0));
        assert!(value_is_zero(-0.0));
    }

    #[test]
    fn test_value_is_zero_for_nonzero() {
        assert!(!value_is_zero(1.0));
        assert!(!value_is_zero(-1.0));
        assert!(!value_is_zero(0.001));
    }

    // =========================================================================
    // greater_than_or_equal tests
    // =========================================================================

    #[test]
    fn test_greater_than_or_equal_greater() {
        assert!(greater_than_or_equal(2.0, 1.0));
    }

    #[test]
    fn test_greater_than_or_equal_equal() {
        assert!(greater_than_or_equal(1.0, 1.0));
    }

    #[test]
    fn test_greater_than_or_equal_less() {
        assert!(!greater_than_or_equal(1.0, 2.0));
    }

    // =========================================================================
    // greater_than tests
    // =========================================================================

    #[test]
    fn test_greater_than_true() {
        assert!(greater_than(2.0, 1.0));
    }

    #[test]
    fn test_greater_than_equal_is_false() {
        assert!(!greater_than(1.0, 1.0));
    }

    #[test]
    fn test_greater_than_less_is_false() {
        assert!(!greater_than(1.0, 2.0));
    }

    // =========================================================================
    // less_than tests
    // =========================================================================

    #[test]
    fn test_less_than_true() {
        assert!(less_than(1.0, 2.0));
    }

    #[test]
    fn test_less_than_equal_is_false() {
        assert!(!less_than(1.0, 1.0));
    }

    #[test]
    fn test_less_than_greater_is_false() {
        assert!(!less_than(2.0, 1.0));
    }

    // =========================================================================
    // area tests
    // =========================================================================

    #[test]
    fn test_area_empty_ring() {
        let ring: Vec<Point<i64>> = vec![];
        assert_eq!(area(&ring), 0.0);
    }

    #[test]
    fn test_area_line_not_a_ring() {
        let ring: Vec<Point<i64>> = vec![Point::new(0, 0), Point::new(1, 1)];
        assert_eq!(area(&ring), 0.0);
    }

    #[test]
    fn test_area_unit_square_ccw() {
        // Counter-clockwise square
        let ring: Vec<Point<i64>> = vec![
            Point::new(0, 0),
            Point::new(1, 0),
            Point::new(1, 1),
            Point::new(0, 1),
            Point::new(0, 0),
        ];
        let a = area(&ring);
        assert!((a - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_area_unit_square_cw() {
        // Clockwise square
        let ring: Vec<Point<i64>> = vec![
            Point::new(0, 0),
            Point::new(0, 1),
            Point::new(1, 1),
            Point::new(1, 0),
            Point::new(0, 0),
        ];
        let a = area(&ring);
        assert!((a + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_area_larger_square() {
        // 10x10 square
        let ring: Vec<Point<i64>> = vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 10),
            Point::new(0, 10),
            Point::new(0, 0),
        ];
        let a = area(&ring);
        assert!((a - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_area_triangle() {
        // Triangle with vertices (0,0), (4,0), (0,3) => area = 6
        let ring: Vec<Point<i64>> = vec![
            Point::new(0, 0),
            Point::new(4, 0),
            Point::new(0, 3),
            Point::new(0, 0),
        ];
        let a = area(&ring);
        assert!((a - 6.0).abs() < 1e-10);
    }

    // =========================================================================
    // slopes_equal_3pts tests
    // =========================================================================

    #[test]
    fn test_slopes_equal_collinear_horizontal() {
        let p1: Point<i64> = Point::new(0, 5);
        let p2: Point<i64> = Point::new(5, 5);
        let p3: Point<i64> = Point::new(10, 5);
        assert!(slopes_equal_3pts(&p1, &p2, &p3));
    }

    #[test]
    fn test_slopes_equal_collinear_vertical() {
        let p1: Point<i64> = Point::new(5, 0);
        let p2: Point<i64> = Point::new(5, 5);
        let p3: Point<i64> = Point::new(5, 10);
        assert!(slopes_equal_3pts(&p1, &p2, &p3));
    }

    #[test]
    fn test_slopes_equal_collinear_diagonal() {
        let p1: Point<i64> = Point::new(0, 0);
        let p2: Point<i64> = Point::new(5, 5);
        let p3: Point<i64> = Point::new(10, 10);
        assert!(slopes_equal_3pts(&p1, &p2, &p3));
    }

    #[test]
    fn test_slopes_equal_not_collinear() {
        let p1: Point<i64> = Point::new(0, 0);
        let p2: Point<i64> = Point::new(5, 5);
        let p3: Point<i64> = Point::new(10, 0);
        assert!(!slopes_equal_3pts(&p1, &p2, &p3));
    }

    #[test]
    fn test_slopes_equal_3pts_with_i32() {
        let p1: Point<i32> = Point::new(0, 0);
        let p2: Point<i32> = Point::new(5, 5);
        let p3: Point<i32> = Point::new(10, 10);
        assert!(slopes_equal_3pts(&p1, &p2, &p3));
    }

    // =========================================================================
    // slopes_equal_4pts tests
    // =========================================================================

    #[test]
    fn test_slopes_equal_parallel_lines() {
        let p1: Point<i64> = Point::new(0, 0);
        let p2: Point<i64> = Point::new(10, 10);
        let p3: Point<i64> = Point::new(0, 1);
        let p4: Point<i64> = Point::new(10, 11);
        assert!(slopes_equal_4pts(&p1, &p2, &p3, &p4));
    }

    #[test]
    fn test_slopes_equal_same_line() {
        let p1: Point<i64> = Point::new(0, 0);
        let p2: Point<i64> = Point::new(10, 10);
        let p3: Point<i64> = Point::new(5, 5);
        let p4: Point<i64> = Point::new(15, 15);
        assert!(slopes_equal_4pts(&p1, &p2, &p3, &p4));
    }

    #[test]
    fn test_slopes_equal_different_slopes() {
        let p1: Point<i64> = Point::new(0, 0);
        let p2: Point<i64> = Point::new(10, 10);
        let p3: Point<i64> = Point::new(0, 0);
        let p4: Point<i64> = Point::new(10, 5);
        assert!(!slopes_equal_4pts(&p1, &p2, &p3, &p4));
    }

    // From C++ test: "test edge slope calculation - int32_t with possible overflow"
    #[test]
    fn test_slopes_equal_no_overflow() {
        let p1: Point<i32> = Point::new(1, 0);
        let p2: Point<i32> = Point::new(0, 100000);
        let p3: Point<i32> = Point::new(-1000000, 0);
        let p4: Point<i32> = Point::new(1100000, 453397504);

        // In the case of an overflow in i32, the calculation would incorrectly
        // say these slopes are equal. By casting to i64, we avoid overflow.
        assert!(!slopes_equal_4pts(&p1, &p2, &p3, &p4));
    }

    // =========================================================================
    // wround tests
    // =========================================================================

    #[test]
    fn test_wround_positive() {
        assert_eq!(wround::<i64>(1.5), 2);
        assert_eq!(wround::<i64>(1.4), 1);
        assert_eq!(wround::<i64>(2.6), 3);
    }

    #[test]
    fn test_wround_negative() {
        assert_eq!(wround::<i64>(-1.5), -2);
        assert_eq!(wround::<i64>(-1.4), -1);
        assert_eq!(wround::<i64>(-2.6), -3);
    }

    #[test]
    fn test_wround_zero() {
        assert_eq!(wround::<i64>(0.0), 0);
        assert_eq!(wround::<i64>(0.4), 0);
    }

    #[test]
    fn test_wround_i32() {
        assert_eq!(wround::<i32>(2.7), 3);
        assert_eq!(wround::<i32>(-2.7), -3);
    }

    #[test]
    fn test_wround_f64() {
        assert_eq!(wround_f64(2.7), 3.0);
        assert_eq!(wround_f64(-2.7), -3.0);
    }
}
