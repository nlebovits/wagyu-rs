//! Point type for wagyu geometry operations.
//!
//! This module provides a generic `Point<T>` struct that represents a 2D point
//! with x and y coordinates. It supports conversion from `geo_types::Coord<T>`
//! for interoperability with the geo ecosystem.

use geo_types::{Coord, CoordNum};

/// A 2D point with x and y coordinates.
///
/// This is the fundamental point type used throughout wagyu for representing
/// vertices in polygons and other geometric operations.
///
/// # Type Parameters
///
/// * `T` - The coordinate type, typically `i64` for integer coordinates or
///   `f64` for floating-point coordinates.
///
/// # Examples
///
/// ```
/// use wagyu_core::Point;
///
/// // Create a point using new()
/// let p1 = Point::new(10, 20);
///
/// // Create a point from a tuple
/// let p2: Point<i64> = (5, 10).into();
///
/// // Access coordinates
/// assert_eq!(p1.x, 10);
/// assert_eq!(p1.y, 20);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point<T: CoordNum> {
    /// The x coordinate
    pub x: T,
    /// The y coordinate
    pub y: T,
}

impl<T: CoordNum> Point<T> {
    /// Creates a new point with the given x and y coordinates.
    ///
    /// # Examples
    ///
    /// ```
    /// use wagyu_core::Point;
    ///
    /// let p = Point::new(3, 4);
    /// assert_eq!(p.x, 3);
    /// assert_eq!(p.y, 4);
    /// ```
    #[inline]
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl<T: CoordNum> Default for Point<T> {
    /// Returns the origin point (0, 0).
    #[inline]
    fn default() -> Self {
        Self {
            x: T::zero(),
            y: T::zero(),
        }
    }
}

impl<T: CoordNum> From<(T, T)> for Point<T> {
    /// Creates a point from a tuple of (x, y) coordinates.
    #[inline]
    fn from((x, y): (T, T)) -> Self {
        Self { x, y }
    }
}

impl<T: CoordNum> From<Point<T>> for (T, T) {
    /// Converts a point into a tuple of (x, y) coordinates.
    #[inline]
    fn from(point: Point<T>) -> Self {
        (point.x, point.y)
    }
}

impl<T: CoordNum> From<Coord<T>> for Point<T> {
    /// Creates a point from a `geo_types::Coord`.
    #[inline]
    fn from(coord: Coord<T>) -> Self {
        Self {
            x: coord.x,
            y: coord.y,
        }
    }
}

impl<T: CoordNum> From<Point<T>> for Coord<T> {
    /// Converts a point into a `geo_types::Coord`.
    #[inline]
    fn from(point: Point<T>) -> Self {
        Coord {
            x: point.x,
            y: point.y,
        }
    }
}

/// Type alias for a point with `i64` coordinates.
///
/// This is the standard integer coordinate type used in wagyu for
/// exact geometric computations.
pub type Point64 = Point<i64>;

/// Type alias for a point with `f64` coordinates.
///
/// This is useful for input/output operations where floating-point
/// coordinates are needed.
pub type PointF64 = Point<f64>;

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // Construction tests
    // =========================================================================

    #[test]
    fn test_new_creates_point_with_given_coordinates() {
        let p: Point<i64> = Point::new(10, 20);
        assert_eq!(p.x, 10);
        assert_eq!(p.y, 20);
    }

    #[test]
    fn test_new_with_f64_coordinates() {
        let p: Point<f64> = Point::new(1.5, 2.5);
        assert_eq!(p.x, 1.5);
        assert_eq!(p.y, 2.5);
    }

    #[test]
    fn test_default_is_origin() {
        let p: Point<i64> = Point::default();
        assert_eq!(p.x, 0);
        assert_eq!(p.y, 0);
    }

    #[test]
    fn test_default_f64_is_origin() {
        let p: Point<f64> = Point::default();
        assert_eq!(p.x, 0.0);
        assert_eq!(p.y, 0.0);
    }

    #[test]
    fn test_from_tuple() {
        let p: Point<i64> = Point::from((5, 10));
        assert_eq!(p.x, 5);
        assert_eq!(p.y, 10);
    }

    #[test]
    fn test_from_tuple_f64() {
        let p: Point<f64> = Point::from((3.5, 2.5));
        assert_eq!(p.x, 3.5);
        assert_eq!(p.y, 2.5);
    }

    #[test]
    fn test_into_tuple() {
        let p: Point<i64> = Point::new(7, 8);
        let tuple: (i64, i64) = p.into();
        assert_eq!(tuple, (7, 8));
    }

    // =========================================================================
    // geo_types::Coord conversion tests
    // =========================================================================

    #[test]
    fn test_from_geo_coord_i64() {
        let coord = geo_types::Coord { x: 100, y: 200 };
        let p: Point<i64> = Point::from(coord);
        assert_eq!(p.x, 100);
        assert_eq!(p.y, 200);
    }

    #[test]
    fn test_from_geo_coord_f64() {
        let coord = geo_types::Coord { x: 1.23, y: 4.56 };
        let p: Point<f64> = Point::from(coord);
        assert_eq!(p.x, 1.23);
        assert_eq!(p.y, 4.56);
    }

    #[test]
    fn test_into_geo_coord() {
        let p: Point<f64> = Point::new(9.0, 10.0);
        let coord: geo_types::Coord<f64> = p.into();
        assert_eq!(coord.x, 9.0);
        assert_eq!(coord.y, 10.0);
    }

    // =========================================================================
    // Equality tests
    // =========================================================================

    #[test]
    fn test_equality_same_points() {
        let p1: Point<i64> = Point::new(5, 10);
        let p2: Point<i64> = Point::new(5, 10);
        assert_eq!(p1, p2);
    }

    #[test]
    fn test_inequality_different_x() {
        let p1: Point<i64> = Point::new(5, 10);
        let p2: Point<i64> = Point::new(6, 10);
        assert_ne!(p1, p2);
    }

    #[test]
    fn test_inequality_different_y() {
        let p1: Point<i64> = Point::new(5, 10);
        let p2: Point<i64> = Point::new(5, 11);
        assert_ne!(p1, p2);
    }

    #[test]
    fn test_equality_f64_points() {
        let p1: Point<f64> = Point::new(1.5, 2.5);
        let p2: Point<f64> = Point::new(1.5, 2.5);
        assert_eq!(p1, p2);
    }

    // =========================================================================
    // Clone/Copy tests
    // =========================================================================

    #[test]
    #[allow(clippy::clone_on_copy)]
    fn test_clone() {
        let p1: Point<i64> = Point::new(1, 2);
        // Deliberately using clone() to test that Clone is implemented
        let p2 = p1.clone();
        assert_eq!(p1, p2);
    }

    #[test]
    fn test_copy() {
        let p1: Point<i64> = Point::new(1, 2);
        let p2 = p1; // Copy
        assert_eq!(p1, p2); // p1 is still valid because Point is Copy
    }

    // =========================================================================
    // Debug tests
    // =========================================================================

    #[test]
    fn test_debug_format() {
        let p: Point<i64> = Point::new(42, 99);
        let debug_str = format!("{:?}", p);
        assert!(debug_str.contains("42"));
        assert!(debug_str.contains("99"));
    }

    // =========================================================================
    // Type alias tests
    // =========================================================================

    #[test]
    fn test_point64_alias() {
        let p: Point64 = Point64::new(1, 2);
        assert_eq!(p.x, 1_i64);
        assert_eq!(p.y, 2_i64);
    }

    #[test]
    fn test_pointf64_alias() {
        let p: PointF64 = PointF64::new(1.0, 2.0);
        assert_eq!(p.x, 1.0_f64);
        assert_eq!(p.y, 2.0_f64);
    }

    // =========================================================================
    // Negative coordinate tests
    // =========================================================================

    #[test]
    fn test_negative_coordinates() {
        let p: Point<i64> = Point::new(-10, -20);
        assert_eq!(p.x, -10);
        assert_eq!(p.y, -20);
    }

    #[test]
    fn test_negative_f64_coordinates() {
        let p: Point<f64> = Point::new(-1.5, -2.5);
        assert_eq!(p.x, -1.5);
        assert_eq!(p.y, -2.5);
    }
}
