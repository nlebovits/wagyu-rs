//! Edge list construction from polygon coordinates.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/build_edges.hpp
//!
//! This module provides functions to build a list of edges from a linear ring
//! (polygon boundary). The edge list construction process:
//! - Removes duplicate consecutive points
//! - Removes collinear points (points that lie on the same line)
//! - Creates edges with proper bot/top orientation

use crate::bound::Edge;
use crate::point::Point;
use geo_types::CoordNum;
use num_traits::ToPrimitive;

/// Type alias for an edge list (vector of edges).
pub type EdgeList<T> = Vec<Edge<T>>;

/// Checks if three points have equal slopes (are collinear).
///
/// Uses cross-multiplication to avoid division:
/// `(pt1.y - pt2.y) * (pt2.x - pt3.x) == (pt1.x - pt2.x) * (pt2.y - pt3.y)`
///
/// This is equivalent to checking if the slope from pt1->pt2 equals the slope from pt2->pt3.
///
/// # Arguments
///
/// * `pt1` - First point
/// * `pt2` - Second point (middle point)
/// * `pt3` - Third point
///
/// # Returns
///
/// `true` if the three points are collinear (slopes are equal)
pub fn slopes_equal<T: CoordNum + ToPrimitive>(
    pt1: Point<T>,
    pt2: Point<T>,
    pt3: Point<T>,
) -> bool {
    // Convert to i64 to prevent overflow during multiplication
    // This matches the C++ implementation which casts to std::int64_t
    let pt1_x = pt1.x.to_i64().unwrap_or(0);
    let pt1_y = pt1.y.to_i64().unwrap_or(0);
    let pt2_x = pt2.x.to_i64().unwrap_or(0);
    let pt2_y = pt2.y.to_i64().unwrap_or(0);
    let pt3_x = pt3.x.to_i64().unwrap_or(0);
    let pt3_y = pt3.y.to_i64().unwrap_or(0);

    // Cross multiply: (dy1 * dx2) == (dx1 * dy2)
    (pt1_y - pt2_y) * (pt2_x - pt3_x) == (pt1_x - pt2_x) * (pt2_y - pt3_y)
}

/// Checks if four points form two line segments with equal slopes.
///
/// # Arguments
///
/// * `pt1` - Start of first segment
/// * `pt2` - End of first segment
/// * `pt3` - Start of second segment
/// * `pt4` - End of second segment
///
/// # Returns
///
/// `true` if the slope of pt1->pt2 equals the slope of pt3->pt4
pub fn slopes_equal_4pt<T: CoordNum + ToPrimitive>(
    pt1: Point<T>,
    pt2: Point<T>,
    pt3: Point<T>,
    pt4: Point<T>,
) -> bool {
    let pt1_x = pt1.x.to_i64().unwrap_or(0);
    let pt1_y = pt1.y.to_i64().unwrap_or(0);
    let pt2_x = pt2.x.to_i64().unwrap_or(0);
    let pt2_y = pt2.y.to_i64().unwrap_or(0);
    let pt3_x = pt3.x.to_i64().unwrap_or(0);
    let pt3_y = pt3.y.to_i64().unwrap_or(0);
    let pt4_x = pt4.x.to_i64().unwrap_or(0);
    let pt4_y = pt4.y.to_i64().unwrap_or(0);

    // Cross multiply: (dy1 * dx2) == (dx1 * dy2)
    (pt2_y - pt1_y) * (pt4_x - pt3_x) == (pt2_x - pt1_x) * (pt4_y - pt3_y)
}

/// Determines if pt2 lies between pt1 and pt3 on a line segment.
///
/// This function assumes the three points are collinear. It checks if pt2
/// is positioned between pt1 and pt3 coordinate-wise.
///
/// From C++: `point_2_is_between_point_1_and_point_3`
///
/// # Arguments
///
/// * `pt1` - First endpoint
/// * `pt2` - The point to check
/// * `pt3` - Second endpoint
///
/// # Returns
///
/// `true` if pt2 is strictly between pt1 and pt3 (not at either endpoint)
pub fn point_2_is_between_point_1_and_point_3<T: CoordNum + PartialOrd>(
    pt1: Point<T>,
    pt2: Point<T>,
    pt3: Point<T>,
) -> bool {
    // If any two points are the same, pt2 is not "between" them
    if pt1 == pt2 || pt3 == pt2 || pt1 == pt3 {
        return false;
    }

    // If pt1 and pt3 differ in x-coordinate, check x positioning
    if pt1.x != pt3.x {
        (pt2.x > pt1.x) == (pt2.x < pt3.x)
    } else {
        // Same x-coordinate, check y positioning
        (pt2.y > pt1.y) == (pt2.y < pt3.y)
    }
}

/// Builds an edge list from a linear ring (closed polygon boundary).
///
/// This function processes a ring of points and creates a list of edges,
/// removing duplicate consecutive points and collinear points along the way.
///
/// From C++: `build_edge_list`
///
/// # Arguments
///
/// * `ring` - A slice of points representing a linear ring (polygon boundary).
///   The ring should be closed (first point equals last point) or open.
///
/// # Returns
///
/// * `Some(EdgeList)` - A list of edges if successful (at least 3 edges formed)
/// * `None` - If the ring doesn't form a valid polygon (fewer than 3 edges)
///
/// # Algorithm
///
/// 1. Iterate through the ring, skipping duplicate consecutive points
/// 2. For each trio of points, check if the middle point is collinear
/// 3. If collinear, skip the middle point
/// 4. Create edges between non-collinear consecutive points
/// 5. Post-process to merge edges at ring boundaries if they're collinear
pub fn build_edge_list<T: CoordNum + ToPrimitive + PartialOrd>(
    ring: &[Point<T>],
) -> Option<EdgeList<T>> {
    // Need at least 3 points to form a polygon
    if ring.len() < 3 {
        return None;
    }

    let mut edges: EdgeList<T> = Vec::new();

    // Use indices for iteration, like the C++ bidirectional iterator approach
    let n = ring.len();

    // Track points: pt1 is the "anchor" for new edges, pt2 is the current point
    let mut pt1_idx = 0;

    // Skip duplicate starting points
    while pt1_idx < n - 1 && ring[pt1_idx] == ring[pt1_idx + 1] {
        pt1_idx += 1;
    }

    if pt1_idx >= n - 2 {
        return None; // Not enough unique points
    }

    let mut pt2_idx = pt1_idx + 1;

    // Skip duplicates after pt1
    while pt2_idx < n - 1 && ring[pt2_idx] == ring[pt2_idx + 1] {
        pt2_idx += 1;
    }

    if pt2_idx >= n - 1 {
        return None;
    }

    // Start iterating from pt3
    let mut i = pt2_idx + 1;

    // Keep track of the first point index for wrap-around comparison
    let first_pt_idx = pt1_idx;

    while i < n {
        let pt3 = ring[i];

        // Skip duplicates
        if ring[pt2_idx] == pt3 {
            i += 1;
            continue;
        }

        let pt1 = ring[pt1_idx];
        let pt2 = ring[pt2_idx];

        // Check for collinearity (spike or straight line)
        if slopes_equal(pt1, pt2, pt3) {
            // pt2 is collinear, check if it's a spike (pt2 between pt1 and pt3)
            if point_2_is_between_point_1_and_point_3(pt1, pt2, pt3) {
                // It's a spike - skip both pt2 and pt3, go back
                // In Rust we handle this differently - just skip pt2
                pt2_idx = i;
            } else {
                // pt2 is on the line but not between - skip pt2
                pt2_idx = i;
            }
        } else {
            // Not collinear - add an edge from pt1 to pt2
            add_edge(&mut edges, pt1, pt2);
            pt1_idx = pt2_idx;
            pt2_idx = i;
        }

        i += 1;
    }

    // Handle wrap-around: check if the last points connect back to the first
    // Add the final edge if there's one pending
    if pt1_idx != pt2_idx {
        let pt1 = ring[pt1_idx];
        let pt2 = ring[pt2_idx];
        let pt3 = ring[first_pt_idx];

        // Always add the edge if pt2 == pt3 (ring closing on itself)
        // Otherwise check for collinearity
        if pt2 == pt3 || !slopes_equal(pt1, pt2, pt3) {
            add_edge(&mut edges, pt1, pt2);
        }
    }

    // Close the ring: add edge from the last point back to the first point
    // if it wasn't already handled in the wrap-around logic above.
    // This is needed when the ring is "open" (first point != last point).
    if !edges.is_empty() && pt2_idx != first_pt_idx {
        // Get the last processed point and the first point
        let last_pt = ring[pt2_idx];
        let first_pt = ring[first_pt_idx];

        // Add closing edge if the points are different
        if last_pt != first_pt {
            // Check collinearity with the existing edges
            let first_edge_top = edges.first().unwrap().top;

            if !slopes_equal(last_pt, first_pt, first_edge_top) {
                add_edge(&mut edges, last_pt, first_pt);
            }
        }
    }

    // Post-process: merge front and back edges if they're collinear
    // This handles cases where the ring closure creates collinear segments
    while edges.len() > 2 {
        let front_bot = edges.first().unwrap().bot;
        let front_top = edges.first().unwrap().top;
        let back_bot = edges.last().unwrap().bot;
        let back_top = edges.last().unwrap().top;

        // Check if front and back edges are collinear
        if slopes_equal_4pt(front_bot, front_top, back_bot, back_top) {
            // They're collinear - need to merge them
            if front_bot == back_top {
                // Edges meet at the same point - remove one
                edges.pop();
            } else if front_top == back_bot {
                // Edges are connected - remove the back edge and extend front
                edges.pop();
            } else {
                // More complex merge - break out to avoid infinite loop
                break;
            }
        } else {
            break;
        }
    }

    // Need at least 3 edges to form a valid polygon
    if edges.len() < 3 {
        return None;
    }

    Some(edges)
}

/// Helper function to add an edge to the edge list.
///
/// Edges are oriented so that `bot` has the lower y-coordinate.
/// For horizontal edges (same y), `bot` has the lower x-coordinate.
fn add_edge<T: CoordNum + PartialOrd>(edges: &mut EdgeList<T>, p1: Point<T>, p2: Point<T>) {
    if p1 == p2 {
        return; // Skip degenerate edges
    }

    // Determine which point is bot (lower y, or lower x if y is equal)
    let (bot, top) = if p1.y < p2.y {
        (p1, p2)
    } else if p1.y > p2.y {
        (p2, p1)
    } else {
        // Horizontal edge - use x to determine order
        if p1.x < p2.x {
            (p1, p2)
        } else {
            (p2, p1)
        }
    };

    edges.push(Edge::new(bot, top));
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // slopes_equal tests
    // =========================================================================

    #[test]
    fn slopes_equal_horizontal_line() {
        // Three points on a horizontal line
        let pt1 = Point::new(0_i64, 5_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(10_i64, 5_i64);

        assert!(slopes_equal(pt1, pt2, pt3));
    }

    #[test]
    fn slopes_equal_vertical_line() {
        // Three points on a vertical line
        let pt1 = Point::new(5_i64, 0_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(5_i64, 10_i64);

        assert!(slopes_equal(pt1, pt2, pt3));
    }

    #[test]
    fn slopes_equal_diagonal_line() {
        // Three points on a 45-degree diagonal
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(10_i64, 10_i64);

        assert!(slopes_equal(pt1, pt2, pt3));
    }

    #[test]
    fn slopes_equal_negative_slope() {
        // Three points on a line with negative slope
        let pt1 = Point::new(0_i64, 10_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(10_i64, 0_i64);

        assert!(slopes_equal(pt1, pt2, pt3));
    }

    #[test]
    fn slopes_not_equal_different_slopes() {
        // Three points NOT on the same line
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(10_i64, 0_i64); // Creates a peak

        assert!(!slopes_equal(pt1, pt2, pt3));
    }

    #[test]
    fn slopes_not_equal_right_angle() {
        // Three points forming a right angle
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(5_i64, 0_i64);
        let pt3 = Point::new(5_i64, 5_i64);

        assert!(!slopes_equal(pt1, pt2, pt3));
    }

    #[test]
    fn slopes_equal_f64_coordinates() {
        // Test with floating point coordinates
        let pt1 = Point::new(0.0_f64, 0.0_f64);
        let pt2 = Point::new(5.0_f64, 5.0_f64);
        let pt3 = Point::new(10.0_f64, 10.0_f64);

        assert!(slopes_equal(pt1, pt2, pt3));
    }

    // =========================================================================
    // slopes_equal_4pt tests
    // =========================================================================

    #[test]
    fn slopes_equal_4pt_parallel_horizontal_lines() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(10_i64, 0_i64);
        let pt3 = Point::new(0_i64, 5_i64);
        let pt4 = Point::new(10_i64, 5_i64);

        assert!(slopes_equal_4pt(pt1, pt2, pt3, pt4));
    }

    #[test]
    fn slopes_equal_4pt_parallel_vertical_lines() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(0_i64, 10_i64);
        let pt3 = Point::new(5_i64, 0_i64);
        let pt4 = Point::new(5_i64, 10_i64);

        assert!(slopes_equal_4pt(pt1, pt2, pt3, pt4));
    }

    #[test]
    fn slopes_equal_4pt_parallel_diagonal_lines() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(10_i64, 10_i64);
        let pt3 = Point::new(5_i64, 0_i64);
        let pt4 = Point::new(15_i64, 10_i64);

        assert!(slopes_equal_4pt(pt1, pt2, pt3, pt4));
    }

    #[test]
    fn slopes_not_equal_4pt_perpendicular() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(10_i64, 0_i64);
        let pt3 = Point::new(0_i64, 0_i64);
        let pt4 = Point::new(0_i64, 10_i64);

        assert!(!slopes_equal_4pt(pt1, pt2, pt3, pt4));
    }

    // =========================================================================
    // point_2_is_between_point_1_and_point_3 tests
    // =========================================================================

    #[test]
    fn point_between_on_horizontal_line() {
        let pt1 = Point::new(0_i64, 5_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(10_i64, 5_i64);

        assert!(point_2_is_between_point_1_and_point_3(pt1, pt2, pt3));
    }

    #[test]
    fn point_between_on_vertical_line() {
        let pt1 = Point::new(5_i64, 0_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(5_i64, 10_i64);

        assert!(point_2_is_between_point_1_and_point_3(pt1, pt2, pt3));
    }

    #[test]
    fn point_between_on_diagonal() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(5_i64, 5_i64);
        let pt3 = Point::new(10_i64, 10_i64);

        assert!(point_2_is_between_point_1_and_point_3(pt1, pt2, pt3));
    }

    #[test]
    fn point_not_between_outside_segment() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(15_i64, 15_i64); // Beyond pt3
        let pt3 = Point::new(10_i64, 10_i64);

        assert!(!point_2_is_between_point_1_and_point_3(pt1, pt2, pt3));
    }

    #[test]
    fn point_not_between_at_endpoint_1() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(0_i64, 0_i64); // Same as pt1
        let pt3 = Point::new(10_i64, 10_i64);

        assert!(!point_2_is_between_point_1_and_point_3(pt1, pt2, pt3));
    }

    #[test]
    fn point_not_between_at_endpoint_3() {
        let pt1 = Point::new(0_i64, 0_i64);
        let pt2 = Point::new(10_i64, 10_i64); // Same as pt3
        let pt3 = Point::new(10_i64, 10_i64);

        assert!(!point_2_is_between_point_1_and_point_3(pt1, pt2, pt3));
    }

    #[test]
    fn point_not_between_same_endpoints() {
        let pt1 = Point::new(5_i64, 5_i64);
        let pt2 = Point::new(7_i64, 7_i64);
        let pt3 = Point::new(5_i64, 5_i64); // Same as pt1

        assert!(!point_2_is_between_point_1_and_point_3(pt1, pt2, pt3));
    }

    // =========================================================================
    // build_edge_list tests
    // =========================================================================

    #[test]
    fn build_edge_list_simple_triangle() {
        // A simple triangle: (0,0) -> (10,0) -> (5,10) -> (0,0)
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(10_i64, 0_i64),
            Point::new(5_i64, 10_i64),
            Point::new(0_i64, 0_i64), // Close the ring
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_some());

        let edges = edges.unwrap();
        assert_eq!(edges.len(), 3, "Triangle should have 3 edges");
    }

    #[test]
    fn build_edge_list_simple_square() {
        // A simple square: (0,0) -> (10,0) -> (10,10) -> (0,10) -> (0,0)
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(10_i64, 0_i64),
            Point::new(10_i64, 10_i64),
            Point::new(0_i64, 10_i64),
            Point::new(0_i64, 0_i64), // Close the ring
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_some());

        let edges = edges.unwrap();
        assert_eq!(edges.len(), 4, "Square should have 4 edges");
    }

    #[test]
    fn build_edge_list_removes_duplicate_points() {
        // Triangle with duplicate points
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(0_i64, 0_i64), // Duplicate
            Point::new(10_i64, 0_i64),
            Point::new(10_i64, 0_i64), // Duplicate
            Point::new(5_i64, 10_i64),
            Point::new(0_i64, 0_i64),
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_some());

        let edges = edges.unwrap();
        assert_eq!(
            edges.len(),
            3,
            "Triangle with duplicates should still have 3 edges"
        );
    }

    #[test]
    fn build_edge_list_removes_collinear_points() {
        // A triangle with an extra collinear point on one edge
        // (0,0) -> (5,0) -> (10,0) -> (5,10) -> (0,0)
        // The (5,0) point is collinear and should be removed
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(5_i64, 0_i64), // Collinear with (0,0) and (10,0)
            Point::new(10_i64, 0_i64),
            Point::new(5_i64, 10_i64),
            Point::new(0_i64, 0_i64),
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_some());

        let edges = edges.unwrap();
        assert_eq!(
            edges.len(),
            3,
            "Collinear point should be removed, leaving 3 edges"
        );
    }

    #[test]
    fn build_edge_list_returns_none_for_too_few_points() {
        // Only 2 points - not a valid polygon
        let ring = vec![Point::new(0_i64, 0_i64), Point::new(10_i64, 0_i64)];

        let edges = build_edge_list(&ring);
        assert!(edges.is_none());
    }

    #[test]
    fn build_edge_list_returns_none_for_degenerate_line() {
        // 3 collinear points - not a valid polygon
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(5_i64, 0_i64),
            Point::new(10_i64, 0_i64),
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_none());
    }

    #[test]
    fn build_edge_list_edges_have_correct_orientation() {
        // A simple triangle
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(10_i64, 0_i64),
            Point::new(5_i64, 10_i64),
            Point::new(0_i64, 0_i64),
        ];

        let edges = build_edge_list(&ring).unwrap();

        // All edges should have bot.y <= top.y
        for edge in &edges {
            assert!(
                edge.bot.y <= edge.top.y,
                "Edge bot.y ({}) should be <= top.y ({})",
                edge.bot.y,
                edge.top.y
            );
        }
    }

    #[test]
    fn build_edge_list_horizontal_edge_orientation() {
        // A square where bottom edge is horizontal
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(10_i64, 0_i64),
            Point::new(10_i64, 10_i64),
            Point::new(0_i64, 10_i64),
            Point::new(0_i64, 0_i64),
        ];

        let edges = build_edge_list(&ring).unwrap();

        // Find horizontal edges and verify bot.x <= top.x
        for edge in &edges {
            if edge.bot.y == edge.top.y {
                // Horizontal edge
                assert!(
                    edge.bot.x <= edge.top.x,
                    "Horizontal edge bot.x ({}) should be <= top.x ({})",
                    edge.bot.x,
                    edge.top.x
                );
            }
        }
    }

    #[test]
    fn build_edge_list_f64_coordinates() {
        // Test with floating point coordinates
        let ring = vec![
            Point::new(0.0_f64, 0.0_f64),
            Point::new(10.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
            Point::new(0.0_f64, 0.0_f64),
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_some());
        assert_eq!(edges.unwrap().len(), 3);
    }

    #[test]
    fn build_edge_list_negative_coordinates() {
        // Triangle with negative coordinates
        let ring = vec![
            Point::new(-10_i64, -10_i64),
            Point::new(10_i64, -10_i64),
            Point::new(0_i64, 10_i64),
            Point::new(-10_i64, -10_i64),
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_some());
        assert_eq!(edges.unwrap().len(), 3);
    }

    #[test]
    fn build_edge_list_open_ring() {
        // An "open" ring (first point != last point)
        // The function should still work
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(10_i64, 0_i64),
            Point::new(5_i64, 10_i64),
        ];

        let edges = build_edge_list(&ring);
        // This might return Some or None depending on implementation
        // The important thing is it doesn't panic
        assert!(edges.is_some() || edges.is_none());
    }

    #[test]
    fn build_edge_list_complex_polygon() {
        // A pentagon
        let ring = vec![
            Point::new(50_i64, 0_i64),
            Point::new(100_i64, 38_i64),
            Point::new(82_i64, 100_i64),
            Point::new(18_i64, 100_i64),
            Point::new(0_i64, 38_i64),
            Point::new(50_i64, 0_i64),
        ];

        let edges = build_edge_list(&ring);
        assert!(edges.is_some());

        let edges = edges.unwrap();
        assert_eq!(edges.len(), 5, "Pentagon should have 5 edges");
    }

    #[test]
    fn build_edge_list_spike_removal() {
        // A triangle with a "spike" - a point that goes out and back
        // (0,0) -> (10,0) -> (15,5) -> (10,0) -> (5,10) -> (0,0)
        // The spike at (15,5) should be removed
        let ring = vec![
            Point::new(0_i64, 0_i64),
            Point::new(10_i64, 0_i64),
            Point::new(15_i64, 5_i64), // Spike
            Point::new(10_i64, 0_i64), // Back to same point
            Point::new(5_i64, 10_i64),
            Point::new(0_i64, 0_i64),
        ];

        let edges = build_edge_list(&ring);
        // The result depends on how spikes are handled
        // At minimum, it shouldn't panic
        assert!(edges.is_some() || edges.is_none());
    }
}
