//! Ring Utilities - Functions for ring manipulation in the clipping algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp
//!
//! This module provides utility functions for working with rings during
//! the Vatti polygon clipping algorithm:
//!
//! - Rounding utilities for snap rounding (`round_towards_min`, `round_towards_max`)
//! - Point-in-polygon testing
//! - Ring comparison and manipulation
//! - Containment checks

use geo_types::{Coord, CoordFloat, CoordNum};

// ============================================================================
// Floating Point Comparison Utilities
// ============================================================================

/// Tolerance for floating point comparisons.
///
/// From C++: `constexpr double SCALED_EPSILON = 1.490116119385e-08`
const SCALED_EPSILON: f64 = 1.490116119385e-08;

/// Check if two floating point values are approximately equal.
///
/// From C++: `values_are_equal` in util.hpp
#[inline]
pub fn values_are_equal(a: f64, b: f64) -> bool {
    (a - b).abs() < SCALED_EPSILON
}

/// Check if a floating point value is approximately zero.
///
/// From C++: `value_is_zero` in util.hpp
#[inline]
pub fn value_is_zero(val: f64) -> bool {
    val.abs() < SCALED_EPSILON
}

/// Check if a is greater than or equal to b, accounting for floating point tolerance.
///
/// From C++: `greater_than_or_equal` in util.hpp
#[inline]
pub fn greater_than_or_equal(a: f64, b: f64) -> bool {
    a > b || values_are_equal(a, b)
}

// ============================================================================
// Rounding Utilities
// ============================================================================

/// Round towards the minimum (floor on ties).
///
/// From C++: `round_towards_min<T>(double val)`
///
/// - 0.5 rounds to 0
/// - 0.0 rounds to 0
/// - -0.5 rounds to -1
///
/// This is used in snap rounding to ensure consistent behavior at half-integer values.
pub fn round_towards_min(val: f64) -> i64 {
    // 0.5 rounds to 0
    // 0.0 rounds to 0
    // -0.5 rounds to -1
    let half = val.floor() + 0.5;
    if values_are_equal(val, half) {
        val.floor() as i64
    } else {
        val.round() as i64
    }
}

/// Round towards the maximum (ceil on ties).
///
/// From C++: `round_towards_max<T>(double val)`
///
/// - 0.5 rounds to 1
/// - 0.0 rounds to 0
/// - -0.5 rounds to 0
///
/// This is used in snap rounding to ensure consistent behavior at half-integer values.
pub fn round_towards_max(val: f64) -> i64 {
    // 0.5 rounds to 1
    // 0.0 rounds to 0
    // -0.5 rounds to 0
    let half = val.floor() + 0.5;
    if values_are_equal(val, half) {
        val.ceil() as i64
    } else {
        val.round() as i64
    }
}

// ============================================================================
// Slope Utilities
// ============================================================================

/// Calculate the inverse slope (dx/dy) between two points.
///
/// From C++: `get_dx(point<T> const& pt1, point<T> const& pt2)`
///
/// Returns infinity if the points have the same y coordinate (horizontal line).
pub fn get_dx<T: CoordNum>(pt1: &Coord<T>, pt2: &Coord<T>) -> f64 {
    let y1 = pt1.y.to_f64().unwrap_or(0.0);
    let y2 = pt2.y.to_f64().unwrap_or(0.0);
    let x1 = pt1.x.to_f64().unwrap_or(0.0);
    let x2 = pt2.x.to_f64().unwrap_or(0.0);

    if (y1 - y2).abs() < f64::EPSILON {
        f64::INFINITY
    } else {
        (x2 - x1) / (y2 - y1)
    }
}

// ============================================================================
// Point-in-Polygon Testing
// ============================================================================

/// Result of a point-in-polygon test.
///
/// From C++: `point_in_polygon_result` enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointInPolygonResult {
    /// Point lies on the polygon boundary
    OnPolygon = -1,
    /// Point is inside the polygon
    Inside = 0,
    /// Point is outside the polygon
    Outside = 1,
}

/// Test if a point is inside, outside, or on a polygon ring.
///
/// From C++: `point_in_polygon(point<T> const& pt, point_ptr<T> op)`
///
/// Uses the ray casting algorithm. A ray is cast from the test point
/// horizontally to the right. Each crossing of the polygon boundary
/// toggles inside/outside state.
///
/// # Arguments
/// * `pt` - The point to test
/// * `ring_points` - The points forming the polygon ring
///
/// # Returns
/// * `OnPolygon` if the point lies on the boundary
/// * `Inside` if the point is inside
/// * `Outside` if the point is outside
pub fn point_in_polygon<T: CoordNum>(
    pt: &Coord<T>,
    ring_points: &[Coord<T>],
) -> PointInPolygonResult {
    if ring_points.is_empty() {
        return PointInPolygonResult::Outside;
    }

    let pt_x = pt.x.to_f64().unwrap_or(0.0);
    let pt_y = pt.y.to_f64().unwrap_or(0.0);

    let mut result = PointInPolygonResult::Outside;

    let n = ring_points.len();
    for i in 0..n {
        let j = (i + 1) % n;

        let op_x = ring_points[i].x.to_f64().unwrap_or(0.0);
        let op_y = ring_points[i].y.to_f64().unwrap_or(0.0);
        let op_next_x = ring_points[j].x.to_f64().unwrap_or(0.0);
        let op_next_y = ring_points[j].y.to_f64().unwrap_or(0.0);

        // Check if point is on horizontal edge
        if values_are_equal(op_next_y, pt_y)
            && (values_are_equal(op_next_x, pt_x)
                || (values_are_equal(op_y, pt_y) && ((op_next_x > pt_x) == (op_x < pt_x))))
        {
            return PointInPolygonResult::OnPolygon;
        }

        // Ray casting: count crossings of horizontal ray from pt going right
        if (op_y < pt_y) != (op_next_y < pt_y) {
            if greater_than_or_equal(op_x, pt_x) {
                if op_next_x > pt_x {
                    // Edge clearly crosses ray
                    result = if result == PointInPolygonResult::Outside {
                        PointInPolygonResult::Inside
                    } else {
                        PointInPolygonResult::Outside
                    };
                } else {
                    // Need to check cross product for precise determination
                    let d = (op_x - pt_x) * (op_next_y - pt_y) - (op_next_x - pt_x) * (op_y - pt_y);
                    if value_is_zero(d) {
                        return PointInPolygonResult::OnPolygon;
                    }
                    if (d > 0.0) == (op_next_y > op_y) {
                        result = if result == PointInPolygonResult::Outside {
                            PointInPolygonResult::Inside
                        } else {
                            PointInPolygonResult::Outside
                        };
                    }
                }
            } else if op_next_x > pt_x {
                // Need to check cross product
                let d = (op_x - pt_x) * (op_next_y - pt_y) - (op_next_x - pt_x) * (op_y - pt_y);
                if value_is_zero(d) {
                    return PointInPolygonResult::OnPolygon;
                }
                if (d > 0.0) == (op_next_y > op_y) {
                    result = if result == PointInPolygonResult::Outside {
                        PointInPolygonResult::Inside
                    } else {
                        PointInPolygonResult::Outside
                    };
                }
            }
        }
    }

    result
}

// ============================================================================
// Convexity and Geometry Utilities
// ============================================================================

/// Check if the point at given index forms a convex vertex in the ring.
///
/// From C++: `is_convex(point_ptr<T> edge)`
///
/// A vertex is convex if the cross product of the vectors formed with
/// its neighbors has the appropriate sign relative to the ring's winding.
///
/// # Arguments
/// * `ring_points` - The points forming the ring
/// * `index` - Index of the vertex to check
/// * `ring_area_positive` - True if the ring has positive area (CCW winding)
///
/// # Returns
/// True if the vertex is convex
pub fn is_convex<T: CoordNum>(
    ring_points: &[Coord<T>],
    index: usize,
    ring_area_positive: bool,
) -> bool {
    if ring_points.len() < 3 {
        return false;
    }

    let n = ring_points.len();
    let prev_idx = if index == 0 { n - 1 } else { index - 1 };
    let next_idx = (index + 1) % n;

    let prev = &ring_points[prev_idx];
    let curr = &ring_points[index];
    let next = &ring_points[next_idx];

    let v1x = curr.x.to_f64().unwrap_or(0.0) - prev.x.to_f64().unwrap_or(0.0);
    let v1y = curr.y.to_f64().unwrap_or(0.0) - prev.y.to_f64().unwrap_or(0.0);
    let v2x = next.x.to_f64().unwrap_or(0.0) - curr.x.to_f64().unwrap_or(0.0);
    let v2y = next.y.to_f64().unwrap_or(0.0) - curr.y.to_f64().unwrap_or(0.0);

    let cross = v1x * v2y - v2x * v1y;

    // Return true if the vertex is "convex" in wagyu's sense
    // (which actually means reflex - where the triangle points inward)
    (cross < 0.0 && ring_area_positive) || (cross > 0.0 && !ring_area_positive)
}

/// Calculate the centroid of three consecutive points.
///
/// From C++: `centroid_of_points(point_ptr<T> edge)`
///
/// Returns the geometric center of the triangle formed by the point
/// at the given index and its two neighbors.
pub fn centroid_of_three_points<T: CoordNum>(ring_points: &[Coord<T>], index: usize) -> (f64, f64) {
    if ring_points.len() < 3 {
        return (0.0, 0.0);
    }

    let n = ring_points.len();
    let prev_idx = if index == 0 { n - 1 } else { index - 1 };
    let next_idx = (index + 1) % n;

    let prev = &ring_points[prev_idx];
    let curr = &ring_points[index];
    let next = &ring_points[next_idx];

    let x = (prev.x.to_f64().unwrap_or(0.0)
        + curr.x.to_f64().unwrap_or(0.0)
        + next.x.to_f64().unwrap_or(0.0))
        / 3.0;
    let y = (prev.y.to_f64().unwrap_or(0.0)
        + curr.y.to_f64().unwrap_or(0.0)
        + next.y.to_f64().unwrap_or(0.0))
        / 3.0;

    (x, y)
}

// ============================================================================
// Bounding Box Utilities
// ============================================================================

/// A simple 2D bounding box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BBox<T: CoordNum> {
    pub min: Coord<T>,
    pub max: Coord<T>,
}

impl<T: CoordNum> BBox<T> {
    /// Create a new bounding box from min and max coordinates.
    pub fn new(min: Coord<T>, max: Coord<T>) -> Self {
        Self { min, max }
    }
}

impl<T: CoordFloat> BBox<T> {
    /// Compute the bounding box of a ring.
    pub fn from_ring(ring_points: &[Coord<T>]) -> Option<Self> {
        if ring_points.is_empty() {
            return None;
        }

        let mut min_x = ring_points[0].x;
        let mut min_y = ring_points[0].y;
        let mut max_x = ring_points[0].x;
        let mut max_y = ring_points[0].y;

        for pt in ring_points.iter().skip(1) {
            if pt.x < min_x {
                min_x = pt.x;
            }
            if pt.y < min_y {
                min_y = pt.y;
            }
            if pt.x > max_x {
                max_x = pt.x;
            }
            if pt.y > max_y {
                max_y = pt.y;
            }
        }

        Some(BBox {
            min: Coord { x: min_x, y: min_y },
            max: Coord { x: max_x, y: max_y },
        })
    }
}

/// Check if box2 contains box1.
///
/// From C++: `box2_contains_box1`
///
/// Returns true if box2 fully contains box1.
pub fn box2_contains_box1<T: CoordNum + PartialOrd>(box1: &BBox<T>, box2: &BBox<T>) -> bool {
    box2.max.x >= box1.max.x
        && box2.max.y >= box1.max.y
        && box2.min.x <= box1.min.x
        && box2.min.y <= box1.min.y
}

// ============================================================================
// Ring Comparison Utilities
// ============================================================================

/// Calculate the signed area of a ring using the shoelace formula.
///
/// Positive area indicates counter-clockwise winding.
/// Negative area indicates clockwise winding.
pub fn ring_area<T: CoordNum>(ring_points: &[Coord<T>]) -> f64 {
    if ring_points.len() < 3 {
        return 0.0;
    }

    let mut sum = 0.0;
    let n = ring_points.len();

    for i in 0..n {
        let j = (i + 1) % n;
        let xi = ring_points[i].x.to_f64().unwrap_or(0.0);
        let yi = ring_points[i].y.to_f64().unwrap_or(0.0);
        let xj = ring_points[j].x.to_f64().unwrap_or(0.0);
        let yj = ring_points[j].y.to_f64().unwrap_or(0.0);
        sum += xi * yj - xj * yi;
    }

    sum * 0.5
}

/// Find the index of the bottom-most point in a ring.
///
/// From C++: `get_bottom_point(point_ptr<T> pp)`
///
/// The bottom point is the one with:
/// 1. Maximum y coordinate (lowest on screen)
/// 2. Among those, minimum x coordinate (leftmost)
///
/// This is used for determining ring orientation and for comparing rings.
pub fn get_bottom_point_index<T: CoordNum>(ring_points: &[Coord<T>]) -> Option<usize> {
    if ring_points.is_empty() {
        return None;
    }

    let mut best_idx = 0;
    let mut best_y = ring_points[0].y.to_f64().unwrap_or(0.0);
    let mut best_x = ring_points[0].x.to_f64().unwrap_or(0.0);

    for (i, pt) in ring_points.iter().enumerate().skip(1) {
        let y = pt.y.to_f64().unwrap_or(0.0);
        let x = pt.x.to_f64().unwrap_or(0.0);

        if y > best_y || (values_are_equal(y, best_y) && x < best_x) {
            best_y = y;
            best_x = x;
            best_idx = i;
        }
    }

    Some(best_idx)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Floating Point Comparison Tests ====================

    #[test]
    fn values_are_equal_returns_true_for_same_values() {
        assert!(values_are_equal(1.0, 1.0));
        assert!(values_are_equal(0.0, 0.0));
        assert!(values_are_equal(-5.0, -5.0));
    }

    #[test]
    fn values_are_equal_returns_true_for_close_values() {
        // Values within SCALED_EPSILON should be considered equal
        let epsilon = SCALED_EPSILON / 2.0;
        assert!(values_are_equal(1.0, 1.0 + epsilon));
        assert!(values_are_equal(1.0, 1.0 - epsilon));
    }

    #[test]
    fn values_are_equal_returns_false_for_different_values() {
        assert!(!values_are_equal(1.0, 2.0));
        assert!(!values_are_equal(0.0, 0.001));
    }

    #[test]
    fn value_is_zero_returns_true_for_zero() {
        assert!(value_is_zero(0.0));
    }

    #[test]
    fn value_is_zero_returns_true_for_near_zero() {
        let epsilon = SCALED_EPSILON / 2.0;
        assert!(value_is_zero(epsilon));
        assert!(value_is_zero(-epsilon));
    }

    #[test]
    fn value_is_zero_returns_false_for_nonzero() {
        assert!(!value_is_zero(1.0));
        assert!(!value_is_zero(-1.0));
        assert!(!value_is_zero(0.001));
    }

    // ==================== Rounding Tests ====================

    #[test]
    fn round_towards_min_rounds_half_to_floor() {
        // 0.5 should round to 0 (floor)
        assert_eq!(round_towards_min(0.5), 0);
        // 1.5 should round to 1 (floor)
        assert_eq!(round_towards_min(1.5), 1);
        // 2.5 should round to 2 (floor)
        assert_eq!(round_towards_min(2.5), 2);
    }

    #[test]
    fn round_towards_min_rounds_negative_half_to_floor() {
        // -0.5 should round to -1 (floor)
        assert_eq!(round_towards_min(-0.5), -1);
        // -1.5 should round to -2 (floor)
        assert_eq!(round_towards_min(-1.5), -2);
    }

    #[test]
    fn round_towards_min_rounds_non_half_normally() {
        // Values not at 0.5 should round normally
        assert_eq!(round_towards_min(0.3), 0);
        assert_eq!(round_towards_min(0.7), 1);
        assert_eq!(round_towards_min(-0.3), 0);
        assert_eq!(round_towards_min(-0.7), -1);
    }

    #[test]
    fn round_towards_max_rounds_half_to_ceil() {
        // 0.5 should round to 1 (ceil)
        assert_eq!(round_towards_max(0.5), 1);
        // 1.5 should round to 2 (ceil)
        assert_eq!(round_towards_max(1.5), 2);
        // 2.5 should round to 3 (ceil)
        assert_eq!(round_towards_max(2.5), 3);
    }

    #[test]
    fn round_towards_max_rounds_negative_half_to_ceil() {
        // -0.5 should round to 0 (ceil)
        assert_eq!(round_towards_max(-0.5), 0);
        // -1.5 should round to -1 (ceil)
        assert_eq!(round_towards_max(-1.5), -1);
    }

    #[test]
    fn round_towards_max_rounds_non_half_normally() {
        // Values not at 0.5 should round normally
        assert_eq!(round_towards_max(0.3), 0);
        assert_eq!(round_towards_max(0.7), 1);
        assert_eq!(round_towards_max(-0.3), 0);
        assert_eq!(round_towards_max(-0.7), -1);
    }

    // ==================== Get DX Tests ====================

    #[test]
    fn get_dx_returns_inverse_slope() {
        // Line from (0,0) to (10,20): dx/dy = 10/20 = 0.5
        let pt1 = Coord { x: 0.0, y: 0.0 };
        let pt2 = Coord { x: 10.0, y: 20.0 };
        assert!((get_dx(&pt1, &pt2) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn get_dx_returns_infinity_for_horizontal_line() {
        // Horizontal line: same y, different x
        let pt1 = Coord { x: 0.0, y: 5.0 };
        let pt2 = Coord { x: 10.0, y: 5.0 };
        assert!(get_dx(&pt1, &pt2).is_infinite());
    }

    #[test]
    fn get_dx_returns_zero_for_vertical_line() {
        // Vertical line: same x, different y -> dx = 0
        let pt1 = Coord { x: 5.0, y: 0.0 };
        let pt2 = Coord { x: 5.0, y: 10.0 };
        assert!((get_dx(&pt1, &pt2) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn get_dx_negative_slope() {
        // Line going from bottom-right to top-left
        let pt1 = Coord { x: 20.0, y: 0.0 };
        let pt2 = Coord { x: 0.0, y: 10.0 };
        // dx = (0 - 20) / (10 - 0) = -2.0
        assert!((get_dx(&pt1, &pt2) - (-2.0)).abs() < 1e-10);
    }

    // ==================== Point-in-Polygon Tests ====================

    fn make_unit_square() -> Vec<Coord<f64>> {
        vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ]
    }

    #[test]
    fn point_in_polygon_empty_ring_returns_outside() {
        let pt = Coord { x: 5.0, y: 5.0 };
        let ring: Vec<Coord<f64>> = vec![];
        assert_eq!(point_in_polygon(&pt, &ring), PointInPolygonResult::Outside);
    }

    #[test]
    fn point_in_polygon_inside_returns_inside() {
        let ring = make_unit_square();
        let pt = Coord { x: 5.0, y: 5.0 };
        assert_eq!(point_in_polygon(&pt, &ring), PointInPolygonResult::Inside);
    }

    #[test]
    fn point_in_polygon_outside_returns_outside() {
        let ring = make_unit_square();
        let pt = Coord { x: 15.0, y: 5.0 };
        assert_eq!(point_in_polygon(&pt, &ring), PointInPolygonResult::Outside);
    }

    #[test]
    fn point_in_polygon_on_vertex_returns_on_polygon() {
        let ring = make_unit_square();
        let pt = Coord { x: 0.0, y: 0.0 };
        assert_eq!(
            point_in_polygon(&pt, &ring),
            PointInPolygonResult::OnPolygon
        );
    }

    #[test]
    fn point_in_polygon_on_edge_returns_on_polygon() {
        let ring = make_unit_square();
        let pt = Coord { x: 5.0, y: 0.0 }; // On bottom edge
        assert_eq!(
            point_in_polygon(&pt, &ring),
            PointInPolygonResult::OnPolygon
        );
    }

    #[test]
    fn point_in_polygon_on_vertical_edge_returns_on_polygon() {
        let ring = make_unit_square();
        let pt = Coord { x: 10.0, y: 5.0 }; // On right edge
        assert_eq!(
            point_in_polygon(&pt, &ring),
            PointInPolygonResult::OnPolygon
        );
    }

    #[test]
    fn point_in_polygon_just_outside_corner() {
        let ring = make_unit_square();
        let pt = Coord { x: -0.1, y: -0.1 };
        assert_eq!(point_in_polygon(&pt, &ring), PointInPolygonResult::Outside);
    }

    #[test]
    fn point_in_polygon_triangle() {
        // Triangle: (0,0), (10,0), (5,10)
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 5.0, y: 10.0 },
        ];

        // Center of triangle should be inside
        let center = Coord { x: 5.0, y: 3.0 };
        assert_eq!(
            point_in_polygon(&center, &ring),
            PointInPolygonResult::Inside
        );

        // Point outside
        let outside = Coord { x: 0.0, y: 10.0 };
        assert_eq!(
            point_in_polygon(&outside, &ring),
            PointInPolygonResult::Outside
        );
    }

    // ==================== Is Convex Tests ====================
    //
    // NOTE: The is_convex function in wagyu has counterintuitive naming.
    // It returns TRUE for vertices where the triangle formed by the vertex
    // and its neighbors points INTO the polygon (centroid inside).
    // For a simple convex polygon like a square, all triangles point outward,
    // so is_convex returns FALSE for all vertices.
    // It returns TRUE only for REFLEX vertices in non-convex polygons.

    #[test]
    fn is_convex_returns_false_for_small_ring() {
        let ring = vec![Coord { x: 0.0, y: 0.0 }, Coord { x: 1.0, y: 0.0 }];
        assert!(!is_convex(&ring, 0, true));
    }

    #[test]
    fn is_convex_simple_convex_polygon_all_vertices_return_false() {
        // CCW square: all triangles point outward, so is_convex returns false
        // This is the expected behavior - is_convex is looking for reflex vertices
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        // For a simple convex polygon, is_convex returns false for all vertices
        assert!(!is_convex(&ring, 0, true));
        assert!(!is_convex(&ring, 1, true));
        assert!(!is_convex(&ring, 2, true));
        assert!(!is_convex(&ring, 3, true));
    }

    #[test]
    fn is_convex_reflex_vertex_returns_true() {
        // L-shaped polygon with a reflex vertex
        // CCW ordering: (0,0) -> (10,0) -> (10,5) -> (5,5) -> (5,10) -> (0,10)
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 5.0 },
            Coord { x: 5.0, y: 5.0 }, // Reflex vertex - triangle points inward
            Coord { x: 5.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        // Vertex 3 (5,5) is reflex in a CCW ring - is_convex should return TRUE
        // because its triangle centroid is inside the polygon
        assert!(is_convex(&ring, 3, true));
        // Other vertices are convex (outward pointing) - is_convex returns FALSE
        assert!(!is_convex(&ring, 0, true));
        assert!(!is_convex(&ring, 1, true));
    }

    // ==================== Centroid Tests ====================

    #[test]
    fn centroid_of_three_points_returns_center() {
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 6.0, y: 0.0 },
            Coord { x: 3.0, y: 6.0 },
        ];
        // Centroid of triangle (0,0), (6,0), (3,6) is ((0+6+3)/3, (0+0+6)/3) = (3, 2)
        let (cx, cy) = centroid_of_three_points(&ring, 0);
        // At index 0, neighbors are index 2 (prev) and index 1 (next)
        // So points are (3,6), (0,0), (6,0) -> centroid = (3,2)
        assert!((cx - 3.0).abs() < 1e-10);
        assert!((cy - 2.0).abs() < 1e-10);
    }

    #[test]
    fn centroid_of_three_points_small_ring_returns_zero() {
        let ring = vec![Coord { x: 1.0, y: 2.0 }];
        let (cx, cy) = centroid_of_three_points(&ring, 0);
        assert!((cx - 0.0).abs() < 1e-10);
        assert!((cy - 0.0).abs() < 1e-10);
    }

    // ==================== BBox Tests ====================

    #[test]
    fn bbox_from_ring_returns_correct_bounds() {
        let ring: Vec<Coord<f64>> = vec![
            Coord { x: 1.0, y: 2.0 },
            Coord { x: 5.0, y: 1.0 },
            Coord { x: 3.0, y: 8.0 },
        ];
        let bbox = BBox::from_ring(&ring).unwrap();
        assert!((bbox.min.x - 1.0_f64).abs() < 1e-10);
        assert!((bbox.min.y - 1.0_f64).abs() < 1e-10);
        assert!((bbox.max.x - 5.0_f64).abs() < 1e-10);
        assert!((bbox.max.y - 8.0_f64).abs() < 1e-10);
    }

    #[test]
    fn bbox_from_empty_ring_returns_none() {
        let ring: Vec<Coord<f64>> = vec![];
        assert!(BBox::from_ring(&ring).is_none());
    }

    #[test]
    fn box2_contains_box1_true_when_fully_contained() {
        let inner = BBox::new(Coord { x: 2.0, y: 2.0 }, Coord { x: 8.0, y: 8.0 });
        let outer = BBox::new(Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 10.0 });
        assert!(box2_contains_box1(&inner, &outer));
    }

    #[test]
    fn box2_contains_box1_false_when_not_contained() {
        let inner = BBox::new(Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 10.0 });
        let outer = BBox::new(Coord { x: 2.0, y: 2.0 }, Coord { x: 8.0, y: 8.0 });
        assert!(!box2_contains_box1(&inner, &outer));
    }

    #[test]
    fn box2_contains_box1_true_when_equal() {
        let box1 = BBox::new(Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 10.0 });
        let box2 = BBox::new(Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 10.0 });
        assert!(box2_contains_box1(&box1, &box2));
    }

    // ==================== Ring Area Tests ====================

    #[test]
    fn ring_area_ccw_square_is_positive() {
        // CCW square: (0,0) -> (10,0) -> (10,10) -> (0,10)
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        let area = ring_area(&ring);
        assert!(area > 0.0);
        assert!((area - 100.0).abs() < 1e-10);
    }

    #[test]
    fn ring_area_cw_square_is_negative() {
        // CW square: (0,0) -> (0,10) -> (10,10) -> (10,0)
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 0.0, y: 10.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 10.0, y: 0.0 },
        ];
        let area = ring_area(&ring);
        assert!(area < 0.0);
        assert!((area - (-100.0)).abs() < 1e-10);
    }

    #[test]
    fn ring_area_empty_is_zero() {
        let ring: Vec<Coord<f64>> = vec![];
        assert!((ring_area(&ring) - 0.0).abs() < 1e-10);
    }

    // ==================== Get Bottom Point Tests ====================

    #[test]
    fn get_bottom_point_index_returns_max_y() {
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 }, // This has max y
            Coord { x: 0.0, y: 10.0 },  // Same y, smaller x -> this wins
        ];
        // Max y is 10, and among those, min x is 0 (index 3)
        assert_eq!(get_bottom_point_index(&ring), Some(3));
    }

    #[test]
    fn get_bottom_point_index_prefers_smaller_x_on_tie() {
        let ring = vec![
            Coord { x: 5.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 }, // Same y=10, but x=0 < x=5
            Coord { x: 10.0, y: 10.0 },
        ];
        assert_eq!(get_bottom_point_index(&ring), Some(1));
    }

    #[test]
    fn get_bottom_point_index_empty_returns_none() {
        let ring: Vec<Coord<f64>> = vec![];
        assert!(get_bottom_point_index(&ring).is_none());
    }

    #[test]
    fn get_bottom_point_index_single_point() {
        let ring = vec![Coord { x: 5.0, y: 5.0 }];
        assert_eq!(get_bottom_point_index(&ring), Some(0));
    }
}
