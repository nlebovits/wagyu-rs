//! Quick clipping using Sutherland-Hodgman algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/quick_clip.hpp
//!
//! This module provides fast O(n) clipping of polygons against rectangular
//! bounding boxes. It's used as a pre-processing step before the full Vatti
//! clipping algorithm to reduce the number of edges.
//!
//! # Algorithm
//!
//! The Sutherland-Hodgman algorithm clips a polygon against each edge of the
//! bounding box in sequence (top, right, bottom, left). For each edge:
//!
//! 1. Walk through each edge of the input polygon
//! 2. Classify the endpoints as inside or outside the clip edge
//! 3. Output vertices based on the four cases:
//!    - Both inside: output the end vertex
//!    - Start inside, end outside: output the intersection
//!    - Start outside, end inside: output the intersection AND the end vertex
//!    - Both outside: output nothing
//!
//! # Note
//!
//! The output may contain degenerate edges or repeated points. These are
//! handled by the full wagyu clipper when processing the result.

use crate::point::Point;
use crate::util::wround;
use geo_types::CoordNum;
use num_traits::AsPrimitive;

/// A rectangular bounding box defined by minimum and maximum corners.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox<T: CoordNum> {
    /// Minimum corner (bottom-left)
    pub min: Point<T>,
    /// Maximum corner (top-right)
    pub max: Point<T>,
}

impl<T: CoordNum> BoundingBox<T> {
    /// Creates a new bounding box from min and max corners.
    pub fn new(min: Point<T>, max: Point<T>) -> Self {
        Self { min, max }
    }
}

/// Edge identifiers for the four sides of a bounding box.
///
/// Used internally to determine which box edge to clip against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
enum BoxEdge {
    /// Top edge (y = min.y in screen coords where Y increases downward)
    Top = 0,
    /// Right edge (x = max.x)
    Right = 1,
    /// Bottom edge (y = max.y in screen coords)
    Bottom = 2,
    /// Left edge (x = min.x)
    Left = 3,
}

impl BoxEdge {
    fn from_index(index: usize) -> Self {
        match index {
            0 => BoxEdge::Top,
            1 => BoxEdge::Right,
            2 => BoxEdge::Bottom,
            _ => BoxEdge::Left,
        }
    }
}

/// Computes the intersection point of line segment a->b with a box edge.
///
/// # Arguments
///
/// * `a` - Start point of the line segment
/// * `b` - End point of the line segment
/// * `edge` - Which edge of the box to intersect with
/// * `bbox` - The bounding box
///
/// # Returns
///
/// The intersection point, with coordinates rounded to the nearest integer.
fn intersect<T>(a: &Point<T>, b: &Point<T>, edge: BoxEdge, bbox: &BoundingBox<T>) -> Point<T>
where
    T: CoordNum + AsPrimitive<f64> + 'static,
    i64: AsPrimitive<T>,
{
    let ax: f64 = a.x.as_();
    let ay: f64 = a.y.as_();
    let bx: f64 = b.x.as_();
    let by: f64 = b.y.as_();

    match edge {
        BoxEdge::Top => {
            // Intersection with y = min.y
            let min_y: f64 = bbox.min.y.as_();
            let t = (min_y - ay) / (by - ay);
            let x = wround::<T>(ax + (bx - ax) * t);
            Point::new(x, bbox.min.y)
        }
        BoxEdge::Right => {
            // Intersection with x = max.x
            let max_x: f64 = bbox.max.x.as_();
            let t = (max_x - ax) / (bx - ax);
            let y = wround::<T>(ay + (by - ay) * t);
            Point::new(bbox.max.x, y)
        }
        BoxEdge::Bottom => {
            // Intersection with y = max.y
            let max_y: f64 = bbox.max.y.as_();
            let t = (max_y - ay) / (by - ay);
            let x = wround::<T>(ax + (bx - ax) * t);
            Point::new(x, bbox.max.y)
        }
        BoxEdge::Left => {
            // Intersection with x = min.x
            let min_x: f64 = bbox.min.x.as_();
            let t = (min_x - ax) / (bx - ax);
            let y = wround::<T>(ay + (by - ay) * t);
            Point::new(bbox.min.x, y)
        }
    }
}

/// Checks if a point is inside the bounding box relative to a specific edge.
///
/// "Inside" means on the interior side of that particular edge:
/// - Top edge: y > min.y
/// - Right edge: x < max.x
/// - Bottom edge: y < max.y
/// - Left edge: x > min.x
fn inside<T: CoordNum + PartialOrd>(p: &Point<T>, edge: BoxEdge, bbox: &BoundingBox<T>) -> bool {
    match edge {
        BoxEdge::Top => p.y > bbox.min.y,
        BoxEdge::Right => p.x < bbox.max.x,
        BoxEdge::Bottom => p.y < bbox.max.y,
        BoxEdge::Left => p.x > bbox.min.x,
    }
}

/// Clips a linear ring against a bounding box using Sutherland-Hodgman algorithm.
///
/// This is the core quick-clipping function. It clips the input ring against
/// all four edges of the bounding box.
///
/// # Arguments
///
/// * `ring` - The input linear ring (closed polygon)
/// * `bbox` - The bounding box to clip against
///
/// # Returns
///
/// The clipped ring. If the entire ring is outside the box, returns an empty vector.
/// The returned ring is closed (first point == last point) if non-empty.
///
/// # Examples
///
/// ```
/// use wagyu_rs::quick_clip::{quick_lr_clip, BoundingBox};
/// use wagyu_rs::point::Point;
///
/// // Bounding box from (0,0) to (100,100)
/// let bbox = BoundingBox::new(Point::new(0_i64, 0), Point::new(100, 100));
///
/// // Square that overlaps the right edge
/// let ring = vec![
///     Point::new(50, 25),
///     Point::new(150, 25),
///     Point::new(150, 75),
///     Point::new(50, 75),
///     Point::new(50, 25),
/// ];
///
/// let clipped = quick_lr_clip(&ring, &bbox);
///
/// // Result is clipped at x=100
/// assert!(!clipped.is_empty());
/// ```
pub fn quick_lr_clip<T>(ring: &[Point<T>], bbox: &BoundingBox<T>) -> Vec<Point<T>>
where
    T: CoordNum + PartialOrd + AsPrimitive<f64> + 'static,
    i64: AsPrimitive<T>,
{
    if ring.is_empty() {
        return Vec::new();
    }

    let mut out: Vec<Point<T>> = ring.to_vec();

    // Clip against each of the four edges
    for edge_idx in 0..4 {
        if out.is_empty() {
            break;
        }

        let edge = BoxEdge::from_index(edge_idx);
        let input = out;
        out = Vec::new();

        // Get the last point to start the walk
        let mut s = input[input.len() - 1];

        for e in &input {
            if inside(e, edge, bbox) {
                // E is inside
                if !inside(&s, edge, bbox) {
                    // S is outside, E is inside: output intersection then E
                    out.push(intersect(&s, e, edge, bbox));
                }
                out.push(*e);
            } else if inside(&s, edge, bbox) {
                // S is inside, E is outside: output intersection
                out.push(intersect(&s, e, edge, bbox));
            }
            // Both outside: output nothing

            s = *e;
        }
    }

    // Handle degenerate results
    if out.len() < 3 {
        return Vec::new();
    }

    // Close the ring if needed (the first/last point might have been outside)
    if let (Some(first), Some(last)) = (out.first(), out.last()) {
        if first != last {
            out.push(*first);
        }
    }

    out
}

/// Clips a polygon (represented as a vector of rings) against a bounding box.
///
/// The first ring is the outer boundary, subsequent rings are holes.
///
/// # Arguments
///
/// * `polygon` - Vector of rings (first is outer, rest are holes)
/// * `bbox` - The bounding box to clip against
///
/// # Returns
///
/// Vector of clipped rings (empty rings are removed)
pub fn quick_polygon_clip<T>(polygon: &[Vec<Point<T>>], bbox: &BoundingBox<T>) -> Vec<Vec<Point<T>>>
where
    T: CoordNum + PartialOrd + AsPrimitive<f64> + 'static,
    i64: AsPrimitive<T>,
{
    polygon
        .iter()
        .map(|ring| quick_lr_clip(ring, bbox))
        .filter(|ring| !ring.is_empty())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    type T = i64;

    fn make_bbox(x1: T, y1: T, x2: T, y2: T) -> BoundingBox<T> {
        BoundingBox::new(Point::new(x1, y1), Point::new(x2, y2))
    }

    // =========================================================================
    // Test: square entirely within bbox
    // From C++: TEST_CASE("square entirely within bbox")
    // =========================================================================

    #[test]
    fn test_square_entirely_within_bbox() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(25, 25),
            Point::new(75, 25),
            Point::new(75, 75),
            Point::new(25, 75),
            Point::new(25, 25),
        ];

        let out = quick_lr_clip(&ring, &bbox);
        assert_eq!(out, ring);
    }

    // =========================================================================
    // Test: square cut at right
    // From C++: TEST_CASE("square cut at right")
    // =========================================================================

    #[test]
    fn test_square_cut_at_right() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(25, 25),
            Point::new(175, 25),
            Point::new(175, 75),
            Point::new(25, 75),
            Point::new(25, 25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(25, 25),
            Point::new(100, 25),
            Point::new(100, 75),
            Point::new(25, 75),
            Point::new(25, 25),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Test: square cut at left
    // From C++: TEST_CASE("square cut at left")
    // =========================================================================

    #[test]
    fn test_square_cut_at_left() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(-25, 25),
            Point::new(75, 25),
            Point::new(75, 75),
            Point::new(-25, 75),
            Point::new(-25, 25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(0, 25),
            Point::new(75, 25),
            Point::new(75, 75),
            Point::new(0, 75),
            Point::new(0, 25),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Test: square cut at top
    // From C++: TEST_CASE("square cut at top")
    // =========================================================================

    #[test]
    fn test_square_cut_at_top() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(25, 25),
            Point::new(75, 25),
            Point::new(75, 175),
            Point::new(25, 175),
            Point::new(25, 25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(25, 25),
            Point::new(75, 25),
            Point::new(75, 100),
            Point::new(25, 100),
            Point::new(25, 25),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Test: square cut at bottom
    // From C++: TEST_CASE("square cut at bottom")
    // This test in C++ tests cut at left, so I'm using the correct interpretation
    // =========================================================================

    #[test]
    fn test_square_cut_at_bottom() {
        let bbox = make_bbox(0, 0, 100, 100);

        // Square that extends below y=0
        let ring = vec![
            Point::new(25, -25),
            Point::new(75, -25),
            Point::new(75, 75),
            Point::new(25, 75),
            Point::new(25, -25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(75, 0),
            Point::new(75, 75),
            Point::new(25, 75),
            Point::new(25, 0),
            Point::new(75, 0),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Test: square cut at top right
    // From C++: TEST_CASE("square cut at top right")
    // =========================================================================

    #[test]
    fn test_square_cut_at_top_right() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(25, 25),
            Point::new(175, 25),
            Point::new(175, 175),
            Point::new(25, 175),
            Point::new(25, 25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(25, 25),
            Point::new(100, 25),
            Point::new(100, 100),
            Point::new(25, 100),
            Point::new(25, 25),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Test: square cut at top and bottom right
    // From C++: TEST_CASE("square cut at top and bottom right")
    // =========================================================================

    #[test]
    fn test_square_cut_at_top_and_bottom_right() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(25, -25),
            Point::new(175, -25),
            Point::new(175, 175),
            Point::new(25, 175),
            Point::new(25, -25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(25, 100),
            Point::new(25, 0),
            Point::new(100, 0),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Test: square entirely out of bounds
    // From C++: TEST_CASE("square entirely out of bounds")
    // =========================================================================

    #[test]
    fn test_square_entirely_out_of_bounds() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(125, 125),
            Point::new(175, 125),
            Point::new(175, 175),
            Point::new(125, 175),
            Point::new(125, 125),
        ];

        let out = quick_lr_clip(&ring, &bbox);
        assert!(out.is_empty());
    }

    // =========================================================================
    // Test: square entirely enclosing bbox
    // From C++: TEST_CASE("square entirely enclosing bbox")
    // =========================================================================

    #[test]
    fn test_square_entirely_enclosing_bbox() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(-25, -25),
            Point::new(175, -25),
            Point::new(175, 175),
            Point::new(-25, 175),
            Point::new(-25, -25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
            Point::new(0, 0),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Test: sticking out and back in (complex case)
    // From C++: TEST_CASE("sticking out and back in")
    // =========================================================================

    #[test]
    fn test_sticking_out_and_back_in() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(25, 25),
            Point::new(150, 25),
            Point::new(150, 150),
            Point::new(25, 150),
            Point::new(25, 90),
            Point::new(75, 90),
            Point::new(75, 125),
            Point::new(125, 125),
            Point::new(125, 75),
            Point::new(25, 75),
            Point::new(25, 25),
        ];

        let out = quick_lr_clip(&ring, &bbox);

        let want = vec![
            Point::new(25, 25),
            Point::new(100, 25),
            Point::new(100, 100),
            Point::new(25, 100),
            Point::new(25, 90),
            Point::new(75, 90),
            Point::new(75, 100),
            Point::new(100, 100),
            Point::new(100, 75),
            Point::new(25, 75),
            Point::new(25, 25),
        ];

        assert_eq!(out, want);
    }

    // =========================================================================
    // Additional edge case tests
    // =========================================================================

    #[test]
    fn test_empty_ring() {
        let bbox = make_bbox(0, 0, 100, 100);
        let ring: Vec<Point<T>> = vec![];
        let out = quick_lr_clip(&ring, &bbox);
        assert!(out.is_empty());
    }

    #[test]
    fn test_degenerate_line() {
        let bbox = make_bbox(0, 0, 100, 100);
        let ring = vec![Point::new(50, 50), Point::new(50, 60)];
        let out = quick_lr_clip(&ring, &bbox);
        // Less than 3 points = empty result
        assert!(out.is_empty());
    }

    #[test]
    fn test_triangle_inside() {
        let bbox = make_bbox(0, 0, 100, 100);

        let ring = vec![
            Point::new(50, 25),
            Point::new(75, 75),
            Point::new(25, 75),
            Point::new(50, 25),
        ];

        let out = quick_lr_clip(&ring, &bbox);
        assert_eq!(out, ring);
    }

    // =========================================================================
    // Test polygon clipping
    // =========================================================================

    #[test]
    fn test_quick_polygon_clip() {
        let bbox = make_bbox(0, 0, 100, 100);

        // Outer ring that extends past right edge
        let outer = vec![
            Point::new(25, 25),
            Point::new(150, 25),
            Point::new(150, 75),
            Point::new(25, 75),
            Point::new(25, 25),
        ];

        // Inner hole entirely inside
        let hole = vec![
            Point::new(40, 40),
            Point::new(40, 60),
            Point::new(60, 60),
            Point::new(60, 40),
            Point::new(40, 40),
        ];

        // Another ring entirely outside (should be removed)
        let outside = vec![
            Point::new(200, 200),
            Point::new(250, 200),
            Point::new(250, 250),
            Point::new(200, 250),
            Point::new(200, 200),
        ];

        let polygon = vec![outer, hole.clone(), outside];
        let result = quick_polygon_clip(&polygon, &bbox);

        // Should have 2 rings (outer clipped, hole unchanged)
        // The outside ring should be removed
        assert_eq!(result.len(), 2);

        // Outer ring should be clipped
        let clipped_outer = vec![
            Point::new(25, 25),
            Point::new(100, 25),
            Point::new(100, 75),
            Point::new(25, 75),
            Point::new(25, 25),
        ];
        assert_eq!(result[0], clipped_outer);

        // Hole should be unchanged
        assert_eq!(result[1], hole);
    }

    // =========================================================================
    // Internal function tests
    // =========================================================================

    #[test]
    fn test_inside_top_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        assert!(!inside(&Point::new(50, 0), BoxEdge::Top, &bbox)); // on edge = outside
        assert!(inside(&Point::new(50, 1), BoxEdge::Top, &bbox));
        assert!(!inside(&Point::new(50, -1), BoxEdge::Top, &bbox));
    }

    #[test]
    fn test_inside_right_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        assert!(!inside(&Point::new(100, 50), BoxEdge::Right, &bbox)); // on edge = outside
        assert!(inside(&Point::new(99, 50), BoxEdge::Right, &bbox));
        assert!(!inside(&Point::new(101, 50), BoxEdge::Right, &bbox));
    }

    #[test]
    fn test_inside_bottom_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        assert!(!inside(&Point::new(50, 100), BoxEdge::Bottom, &bbox)); // on edge = outside
        assert!(inside(&Point::new(50, 99), BoxEdge::Bottom, &bbox));
        assert!(!inside(&Point::new(50, 101), BoxEdge::Bottom, &bbox));
    }

    #[test]
    fn test_inside_left_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        assert!(!inside(&Point::new(0, 50), BoxEdge::Left, &bbox)); // on edge = outside
        assert!(inside(&Point::new(1, 50), BoxEdge::Left, &bbox));
        assert!(!inside(&Point::new(-1, 50), BoxEdge::Left, &bbox));
    }

    #[test]
    fn test_intersect_top_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        let a = Point::new(50, -50);
        let b = Point::new(50, 50);
        let result = intersect(&a, &b, BoxEdge::Top, &bbox);
        assert_eq!(result, Point::new(50, 0));
    }

    #[test]
    fn test_intersect_right_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        let a = Point::new(50, 50);
        let b = Point::new(150, 50);
        let result = intersect(&a, &b, BoxEdge::Right, &bbox);
        assert_eq!(result, Point::new(100, 50));
    }

    #[test]
    fn test_intersect_bottom_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        let a = Point::new(50, 50);
        let b = Point::new(50, 150);
        let result = intersect(&a, &b, BoxEdge::Bottom, &bbox);
        assert_eq!(result, Point::new(50, 100));
    }

    #[test]
    fn test_intersect_left_edge() {
        let bbox = make_bbox(0, 0, 100, 100);
        let a = Point::new(50, 50);
        let b = Point::new(-50, 50);
        let result = intersect(&a, &b, BoxEdge::Left, &bbox);
        assert_eq!(result, Point::new(0, 50));
    }
}
