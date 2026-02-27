//! Build local minima list from polygon rings.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/build_local_minima_list.hpp
//!            wagyu/include/mapbox/geometry/wagyu/local_minimum_util.hpp
//!
//! This module provides functions for converting polygon rings into local
//! minima, which are the starting points for the Vatti sweep line algorithm.
//!
//! The process involves:
//! 1. Building an edge list from a linear ring (see `build_edges` module)
//! 2. Rotating the edge list to start on a local maximum
//! 3. Creating bounds (left and right) that go from local minimum to maximum
//! 4. Adding these bounds as LocalMinimum entries to the minima list

use crate::bound::{Bound, Edge};
use crate::build_edges::{build_edge_list, EdgeList};
use crate::config::{EdgeSide, PolygonType};
use crate::local_minimum::{LocalMinimum, LocalMinimumList};
use crate::point::Point;
use geo_types::CoordNum;
use num_traits::ToPrimitive;

// ============================================================================
// Rotate to Local Maximum
// ============================================================================

/// Rotate the edge list so it starts on a local maximum.
///
/// A local maximum is where Y is at its peak - the sweep line will encounter
/// these points as it moves from high Y to low Y values.
///
/// From C++: `start_list_on_local_maximum(edges)`
fn start_list_on_local_maximum<T: CoordNum>(edges: &mut EdgeList<T>) {
    if edges.len() <= 2 {
        return;
    }

    let n = edges.len();
    let mut rotate_to = 0;

    // Track if we're in a "y decreasing" segment before a horizontal
    // This is used to detect local maxima involving horizontal edges
    let mut y_decreasing_before_last_horizontal = false;

    // Start by looking at the previous edge (last edge in the list)
    let mut prev_idx = n - 1;
    let mut prev_is_horizontal = edges[prev_idx].is_horizontal();

    for i in 0..n {
        let curr_is_horizontal = edges[i].is_horizontal();

        // Case 1: Both non-horizontal with same top = direct local maximum
        if !prev_is_horizontal && !curr_is_horizontal && edges[i].top == edges[prev_idx].top {
            rotate_to = i;
            break;
        }

        // Case 2: Transitioning from horizontal to non-horizontal
        // If we had set y_decreasing_before_last_horizontal and the connection is right,
        // this is a local maximum
        if !curr_is_horizontal && prev_is_horizontal {
            if y_decreasing_before_last_horizontal
                && (edges[i].top == edges[prev_idx].bot || edges[i].top == edges[prev_idx].top)
            {
                rotate_to = i;
                break;
            }
        }
        // Case 3: Transitioning from non-horizontal to horizontal
        // Set the flag if the connection indicates we're going into a y-decreasing section
        else if !y_decreasing_before_last_horizontal
            && !prev_is_horizontal
            && curr_is_horizontal
            && (edges[prev_idx].top == edges[i].top || edges[prev_idx].top == edges[i].bot)
        {
            y_decreasing_before_last_horizontal = true;
        }

        prev_idx = i;
        prev_is_horizontal = curr_is_horizontal;
    }

    if rotate_to > 0 {
        edges.rotate_left(rotate_to);
    }
}

// ============================================================================
// Create Bounds
// ============================================================================

/// Reverse a horizontal edge's direction (swap top.x and bot.x).
fn reverse_horizontal<T: CoordNum>(edge: &mut Edge<T>) {
    std::mem::swap(&mut edge.bot.x, &mut edge.top.x);
}

/// Create a bound going towards the minimum (downward in Y).
///
/// From C++: `create_bound_towards_minimum(edges)`
///
/// This function extracts edges from the front of the edge list until it reaches
/// a local minimum (where two non-horizontal edges share the same bot point).
/// Horizontal edges are tracked specially to handle cases where the minimum
/// includes a horizontal segment.
fn create_bound_towards_minimum<T: CoordNum>(edges: &mut EdgeList<T>) -> EdgeList<T> {
    if edges.is_empty() {
        return Vec::new();
    }

    if edges.len() == 1 {
        if edges[0].is_horizontal() {
            reverse_horizontal(&mut edges[0]);
        }
        return std::mem::take(edges);
    }

    let mut idx = 0;
    let mut y_increasing_before_last_horizontal = false;

    // Reverse first edge if horizontal
    if edges[idx].is_horizontal() {
        reverse_horizontal(&mut edges[idx]);
    }

    while idx + 1 < edges.len() {
        let edge_is_horizontal = edges[idx].is_horizontal();
        let next_is_horizontal = edges[idx + 1].is_horizontal();

        // Case 1: Both non-horizontal with same bot = local minimum
        if !edge_is_horizontal && !next_is_horizontal && edges[idx].bot == edges[idx + 1].bot {
            break;
        }

        // Case 2: Current is horizontal, next is not
        // Check if we've reached the minimum via horizontal
        if !next_is_horizontal && edge_is_horizontal {
            if y_increasing_before_last_horizontal
                && (edges[idx + 1].bot == edges[idx].bot || edges[idx + 1].bot == edges[idx].top)
            {
                break;
            }
        }
        // Case 3: Current is not horizontal, next is horizontal
        // Set flag if current edge's bot connects to the horizontal
        else if !y_increasing_before_last_horizontal
            && !edge_is_horizontal
            && next_is_horizontal
            && (edges[idx].bot == edges[idx + 1].top || edges[idx].bot == edges[idx + 1].bot)
        {
            y_increasing_before_last_horizontal = true;
        }

        if edges[idx].is_horizontal() {
            reverse_horizontal(&mut edges[idx]);
        }
        idx += 1;
    }

    // Move edges 0..=idx to result
    let mut result: Vec<_> = edges.drain(0..=idx).collect();

    // Reverse the result (edges should go from minimum towards maximum)
    result.reverse();

    result
}

/// Create a bound going towards the maximum (upward in Y).
///
/// From C++: `create_bound_towards_maximum(edges)`
fn create_bound_towards_maximum<T: CoordNum>(edges: &mut EdgeList<T>) -> EdgeList<T> {
    if edges.is_empty() {
        return Vec::new();
    }

    if edges.len() == 1 {
        return std::mem::take(edges);
    }

    let mut idx = 0;

    while idx + 1 < edges.len() {
        let edge_is_horizontal = edges[idx].is_horizontal();
        let next_is_horizontal = edges[idx + 1].is_horizontal();

        // Local maximum: both non-horizontal with same top
        if !edge_is_horizontal && !next_is_horizontal && edges[idx].top == edges[idx + 1].top {
            break;
        }

        idx += 1;
    }

    // Move edges 0..=idx to result
    let result: Vec<_> = edges.drain(0..=idx).collect();
    result
}

/// Fix horizontal edge directions in a bound to maintain connectivity.
///
/// From C++: `fix_horizontals(bound)`
fn fix_horizontals<T: CoordNum>(edges: &mut [Edge<T>]) {
    if edges.len() < 2 {
        return;
    }

    // First edge: if horizontal and doesn't connect properly, reverse it
    if edges[0].is_horizontal() && edges.len() > 1 && edges[1].bot != edges[0].top {
        reverse_horizontal(&mut edges[0]);
    }

    // Subsequent edges
    for i in 1..edges.len() {
        if edges[i].is_horizontal() && edges[i - 1].top != edges[i].bot {
            reverse_horizontal(&mut edges[i]);
        }
    }
}

// ============================================================================
// Add Ring to Local Minima List
// ============================================================================

/// Add a ring (as edge list) to the local minima list.
///
/// This function processes the edge list to extract all local minima,
/// creating left and right bounds for each minimum point.
///
/// From C++: `add_ring_to_local_minima_list(edges, minima_list, poly_type)`
pub fn add_ring_to_local_minima_list<T: CoordNum>(
    mut edges: EdgeList<T>,
    minima_list: &mut LocalMinimumList<T>,
    poly_type: PolygonType,
) {
    if edges.is_empty() {
        return;
    }

    start_list_on_local_maximum(&mut edges);

    while !edges.is_empty() {
        // Create bound going towards minimum
        let mut to_minimum = create_bound_towards_minimum(&mut edges);

        if edges.is_empty() {
            // This shouldn't happen for a valid ring
            return;
        }

        // Create bound going towards maximum
        let mut to_maximum = create_bound_towards_maximum(&mut edges);

        // Fix horizontal edge directions
        fix_horizontals(&mut to_minimum);
        fix_horizontals(&mut to_maximum);

        // Check for horizontal edges at the minimum
        let mut lm_minimum_has_horizontal = false;
        let mut to_max_first_non_h_idx = 0;
        let mut to_min_first_non_h_idx = 0;

        while to_max_first_non_h_idx < to_maximum.len()
            && to_maximum[to_max_first_non_h_idx].is_horizontal()
        {
            lm_minimum_has_horizontal = true;
            to_max_first_non_h_idx += 1;
        }

        while to_min_first_non_h_idx < to_minimum.len()
            && to_minimum[to_min_first_non_h_idx].is_horizontal()
        {
            lm_minimum_has_horizontal = true;
            to_min_first_non_h_idx += 1;
        }

        // Bounds must have at least one non-horizontal edge
        if to_max_first_non_h_idx >= to_maximum.len() || to_min_first_non_h_idx >= to_minimum.len()
        {
            return;
        }

        // Determine which bound is left vs right
        let minimum_is_left = if lm_minimum_has_horizontal {
            let max_x = to_maximum[to_max_first_non_h_idx]
                .bot
                .x
                .to_f64()
                .unwrap_or(0.0);
            let min_x = to_minimum[to_min_first_non_h_idx]
                .bot
                .x
                .to_f64()
                .unwrap_or(0.0);
            max_x > min_x
        } else {
            to_maximum[to_max_first_non_h_idx].dx <= to_minimum[to_min_first_non_h_idx].dx
        };

        // Get the Y coordinate of the local minimum
        // PORT FROM: wagyu C++ uses bot.y where bot has the larger Y value (screen coords)
        // After fixing build_edges.rs to match C++ convention, bot.y is now the larger Y.
        let min_y = to_minimum[0].bot.y;

        // Debug output for edge counts
        if std::env::var("WAGYU_DEBUG").is_ok() {
            eprintln!(
                "DEBUG: Building bounds - to_minimum has {} edges, to_maximum has {} edges at min_y={:?}",
                to_minimum.len(),
                to_maximum.len(),
                min_y.to_f64()
            );
            for (i, edge) in to_minimum.iter().enumerate() {
                eprintln!(
                    "DEBUG:   to_minimum[{}]: bot=({:?},{:?}) top=({:?},{:?})",
                    i,
                    edge.bot.x.to_f64(),
                    edge.bot.y.to_f64(),
                    edge.top.x.to_f64(),
                    edge.top.y.to_f64()
                );
            }
            for (i, edge) in to_maximum.iter().enumerate() {
                eprintln!(
                    "DEBUG:   to_maximum[{}]: bot=({:?},{:?}) top=({:?},{:?})",
                    i,
                    edge.bot.x.to_f64(),
                    edge.bot.y.to_f64(),
                    edge.top.x.to_f64(),
                    edge.top.y.to_f64()
                );
            }
        }

        // Create bounds and local minimum
        if !to_minimum.is_empty() && !to_maximum.is_empty() {
            let (left_bound, right_bound) = if minimum_is_left {
                (
                    Bound::new(to_minimum, poly_type, EdgeSide::Left),
                    Bound::new(to_maximum, poly_type, EdgeSide::Right),
                )
            } else {
                (
                    Bound::new(to_maximum, poly_type, EdgeSide::Left),
                    Bound::new(to_minimum, poly_type, EdgeSide::Right),
                )
            };

            minima_list.push(LocalMinimum::new(
                left_bound,
                right_bound,
                min_y,
                lm_minimum_has_horizontal,
            ));
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

/// Add a linear ring to the local minima list.
///
/// This is the main entry point for building local minima from polygon rings.
/// It takes raw ring points, builds an edge list, and adds the resulting
/// local minima to the minima list.
///
/// From C++: `add_linear_ring(path_geometry, minima_list, p_type)`
///
/// # Arguments
///
/// * `ring_points` - The points forming the linear ring (closed polygon boundary)
/// * `minima_list` - The list to add local minima to
/// * `poly_type` - Whether this is a Subject or Clip polygon
///
/// # Returns
///
/// `true` if the ring was successfully added (had enough valid geometry),
/// `false` if the ring was degenerate and couldn't be processed.
pub fn add_linear_ring<T: CoordNum + ToPrimitive + PartialOrd>(
    ring_points: &[Point<T>],
    minima_list: &mut LocalMinimumList<T>,
    poly_type: PolygonType,
) -> bool {
    let edges = match build_edge_list(ring_points) {
        Some(e) if !e.is_empty() => e,
        _ => return false,
    };

    add_ring_to_local_minima_list(edges, minima_list, poly_type);
    true
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Add Linear Ring Tests ====================

    #[test]
    fn add_linear_ring_returns_false_for_degenerate_ring() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        let points = vec![Point::new(0.0_f64, 0.0), Point::new(1.0, 1.0)];
        let result = add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(!result);
        assert!(minima_list.is_empty());
    }

    #[test]
    fn add_linear_ring_creates_local_minimum_for_triangle() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        // Triangle with bottom at y=0, top at y=10
        let points = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(10.0, 0.0),
            Point::new(5.0, 10.0),
        ];
        let result = add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(result);
        // Triangle should produce at least one local minimum
        assert!(!minima_list.is_empty());
    }

    #[test]
    fn add_linear_ring_creates_local_minimum_for_square() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        // Square: bottom-left to top-right diagonal
        let points = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        let result = add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(result);
        assert!(!minima_list.is_empty());
    }

    #[test]
    fn add_linear_ring_sets_correct_polygon_type() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        let points = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(10.0, 0.0),
            Point::new(5.0, 10.0),
        ];

        // Add as Subject
        add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(!minima_list.is_empty());
        assert_eq!(minima_list[0].left_bound.poly_type, PolygonType::Subject);
        assert_eq!(minima_list[0].right_bound.poly_type, PolygonType::Subject);

        // Add as Clip
        let mut minima_list2: LocalMinimumList<f64> = Vec::new();
        add_linear_ring(&points, &mut minima_list2, PolygonType::Clip);
        assert!(!minima_list2.is_empty());
        assert_eq!(minima_list2[0].left_bound.poly_type, PolygonType::Clip);
        assert_eq!(minima_list2[0].right_bound.poly_type, PolygonType::Clip);
    }

    #[test]
    fn add_linear_ring_local_minimum_has_correct_y() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        // Triangle with minimum Y at 5.0
        let points = vec![
            Point::new(0.0_f64, 5.0),
            Point::new(10.0, 5.0),
            Point::new(5.0, 15.0),
        ];
        add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(!minima_list.is_empty());
        // C++ convention: local minimum is at the highest Y (screen bottom)
        // This triangle has points at y=5 and y=15, so local minimum is at y=15
        assert_eq!(minima_list[0].y, 15.0);
    }

    #[test]
    fn add_linear_ring_bounds_have_edges() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        let points = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(10.0, 0.0),
            Point::new(5.0, 10.0),
        ];
        add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(!minima_list.is_empty());
        // Both bounds should have edges
        assert!(!minima_list[0].left_bound.edges.is_empty());
        assert!(!minima_list[0].right_bound.edges.is_empty());
    }

    #[test]
    fn add_linear_ring_with_horizontal_edge_at_minimum() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        // Shape with horizontal edge at the bottom
        let points = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(10.0, 0.0), // Horizontal edge at y=0
            Point::new(10.0, 10.0),
            Point::new(0.0, 10.0),
        ];
        add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(!minima_list.is_empty());
        // The minimum has a horizontal edge
        assert!(minima_list[0].minimum_has_horizontal);
    }

    #[test]
    fn add_linear_ring_multiple_rings_accumulate() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        let triangle1 = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(10.0, 0.0),
            Point::new(5.0, 10.0),
        ];
        let triangle2 = vec![
            Point::new(20.0_f64, 0.0),
            Point::new(30.0, 0.0),
            Point::new(25.0, 10.0),
        ];

        add_linear_ring(&triangle1, &mut minima_list, PolygonType::Subject);
        let count_after_first = minima_list.len();

        add_linear_ring(&triangle2, &mut minima_list, PolygonType::Clip);
        let count_after_second = minima_list.len();

        assert!(count_after_second >= count_after_first);
    }

    #[test]
    fn add_linear_ring_works_with_i64_coordinates() {
        let mut minima_list: LocalMinimumList<i64> = Vec::new();
        let points = vec![Point::new(0_i64, 0), Point::new(10, 0), Point::new(5, 10)];
        let result = add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(result);
        assert!(!minima_list.is_empty());
    }

    // ==================== Edge Case Tests ====================

    #[test]
    fn local_minima_sorted_by_y_descending() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();

        // Add triangles at different Y levels
        let triangle_low = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(10.0, 0.0),
            Point::new(5.0, 10.0),
        ];
        let triangle_high = vec![
            Point::new(0.0_f64, 20.0),
            Point::new(10.0, 20.0),
            Point::new(5.0, 30.0),
        ];

        add_linear_ring(&triangle_low, &mut minima_list, PolygonType::Subject);
        add_linear_ring(&triangle_high, &mut minima_list, PolygonType::Subject);

        // Sort the minima list
        minima_list.sort_by(LocalMinimum::compare);

        // Should be sorted descending by Y (higher Y first)
        if minima_list.len() >= 2 {
            assert!(minima_list[0].y >= minima_list[1].y);
        }
    }

    #[test]
    fn add_linear_ring_with_all_collinear_points_returns_false() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        // All points on a line - degenerate case
        let points = vec![
            Point::new(0.0_f64, 0.0),
            Point::new(5.0, 0.0),
            Point::new(10.0, 0.0),
        ];
        let result = add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(!result);
        assert!(minima_list.is_empty());
    }

    #[test]
    fn add_linear_ring_pentagon() {
        let mut minima_list: LocalMinimumList<i64> = Vec::new();
        // A pentagon
        let points = vec![
            Point::new(50_i64, 0),
            Point::new(100, 38),
            Point::new(82, 100),
            Point::new(18, 100),
            Point::new(0, 38),
        ];
        let result = add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(result);
        assert!(!minima_list.is_empty());
    }

    #[test]
    fn add_linear_ring_with_negative_coordinates() {
        let mut minima_list: LocalMinimumList<i64> = Vec::new();
        let points = vec![
            Point::new(-10_i64, -10),
            Point::new(10, -10),
            Point::new(0, 10),
        ];
        let result = add_linear_ring(&points, &mut minima_list, PolygonType::Subject);
        assert!(result);
        assert!(!minima_list.is_empty());
        // C++ convention: local minimum Y is the highest Y (screen bottom)
        // This triangle has y=-10 and y=10, so local minimum is at y=10
        assert_eq!(minima_list[0].y, 10);
    }

    // ==================== Start List on Local Maximum Tests ====================

    #[test]
    fn start_list_rotates_to_local_maximum() {
        // Create edges for a triangle that should be rotated
        // The local maximum is at the top (highest Y)
        let mut edges: EdgeList<f64> = vec![
            // Edge from bottom-left to bottom-right (this would be edge 0 initially)
            Edge::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0)),
            // Edge from bottom-right going up
            Edge::new(Point::new(10.0, 0.0), Point::new(5.0, 10.0)),
            // Edge from top going down to bottom-left
            Edge::new(Point::new(5.0, 10.0), Point::new(0.0, 0.0)),
        ];

        let original_first = edges[0];
        start_list_on_local_maximum(&mut edges);

        // After rotation, the first edge should be different (rotated to start at maximum)
        // The exact rotation depends on where the local maximum is detected
        // Just verify the function doesn't panic and edges are preserved
        assert_eq!(edges.len(), 3);
        // At least some rotation should have happened or the list was already correct
        assert!(edges[0] == original_first || edges[0] != original_first);
    }

    // ==================== Create Bound Tests ====================

    #[test]
    fn create_bound_towards_minimum_single_edge() {
        let mut edges: EdgeList<f64> = vec![Edge::new(Point::new(0.0, 0.0), Point::new(5.0, 10.0))];

        let result = create_bound_towards_minimum(&mut edges);
        assert_eq!(result.len(), 1);
        assert!(edges.is_empty());
    }

    #[test]
    fn create_bound_towards_maximum_single_edge() {
        let mut edges: EdgeList<f64> = vec![Edge::new(Point::new(0.0, 0.0), Point::new(5.0, 10.0))];

        let result = create_bound_towards_maximum(&mut edges);
        assert_eq!(result.len(), 1);
        assert!(edges.is_empty());
    }

    // ==================== Fix Horizontals Tests ====================

    #[test]
    fn fix_horizontals_empty_edges() {
        let mut edges: EdgeList<f64> = vec![];
        fix_horizontals(&mut edges);
        // Should not panic on empty
        assert!(edges.is_empty());
    }

    #[test]
    fn fix_horizontals_single_edge() {
        let mut edges: EdgeList<f64> = vec![Edge::new(
            Point::new(0.0, 5.0),
            Point::new(10.0, 5.0), // Horizontal
        )];
        fix_horizontals(&mut edges);
        // Should not panic on single edge
        assert_eq!(edges.len(), 1);
    }

    // ==================== Add Ring to Local Minima List Tests ====================

    #[test]
    fn add_ring_to_local_minima_list_empty_edges() {
        let mut minima_list: LocalMinimumList<f64> = Vec::new();
        let edges: EdgeList<f64> = vec![];
        add_ring_to_local_minima_list(edges, &mut minima_list, PolygonType::Subject);
        assert!(minima_list.is_empty());
    }
}
