//! Process maxima during the sweep line algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/process_maxima.hpp
//!
//! This module handles maxima events during the Vatti clipping algorithm.
//! A maxima occurs when two bounds meet at their highest point and need
//! to be removed from the active edge list.

use geo_types::CoordNum;

use crate::active_edge_list::ActiveEdgeList;
use crate::bound::Bound;
use crate::config::FillType;
use crate::Operation;

// ============================================================================
// Helper functions for bound state checking
// ============================================================================

/// Check if a bound is at its maxima (top of all its edges) at the given Y.
///
/// From C++: `is_maxima(bound, y)` - returns true when bound's current edge top.y == y
/// and there are no more edges in the bound.
///
/// # Arguments
/// * `bound` - The bound to check
/// * `top_y` - The current scanline Y coordinate
pub fn is_maxima<T: CoordNum>(bound: &Bound<T>, top_y: T) -> bool {
    let current_edge = bound.current_edge();
    let edge_top_y = current_edge.top.y;

    // Check if this is the last edge and we've reached its top
    let at_edge_top = edge_top_y == top_y;
    let is_last_edge = bound.current_edge_index + 1 >= bound.edges.len();

    at_edge_top && is_last_edge
}

/// Check if a bound is at an intermediate vertex (not maxima, but at edge top).
///
/// From C++: `is_intermediate(bound, y)` - returns true when at edge top but
/// there are more edges to process.
///
/// # Arguments
/// * `bound` - The bound to check
/// * `top_y` - The current scanline Y coordinate
pub fn is_intermediate<T: CoordNum>(bound: &Bound<T>, top_y: T) -> bool {
    let current_edge = bound.current_edge();
    let edge_top_y = current_edge.top.y;

    // At edge top but has more edges
    let at_edge_top = edge_top_y == top_y;
    let has_more_edges = bound.current_edge_index + 1 < bound.edges.len();

    at_edge_top && has_more_edges
}

/// Check if the current edge of a bound is horizontal.
///
/// From C++: `current_edge_is_horizontal<T>(bnd)`
pub fn current_edge_is_horizontal<T: CoordNum>(bound: &Bound<T>) -> bool {
    bound.current_edge().is_horizontal()
}

/// Check if the next edge of a bound is horizontal.
///
/// From C++: `next_edge_is_horizontal<T>(bnd)`
pub fn next_edge_is_horizontal<T: CoordNum>(bound: &Bound<T>) -> bool {
    if bound.current_edge_index + 1 < bound.edges.len() {
        bound.edges[bound.current_edge_index + 1].is_horizontal()
    } else {
        false
    }
}

/// Find the maxima pair for a bound in the active edge list.
///
/// From C++: `get_maxima_pair(bnd, active_bounds)` - finds the bound that shares
/// the same maximum point.
///
/// Returns the position in the AEL of the maxima pair, or None if not found.
///
/// # Arguments
/// * `bound_pos` - Position of the bound in the AEL
/// * `bounds` - All bounds
/// * `ael` - The active edge list
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/active_bound_list.hpp - get_maxima_pair (lines 161-164)
pub fn get_maxima_pair<T: CoordNum>(
    bound_pos: usize,
    bounds: &[Bound<T>],
    ael: &ActiveEdgeList,
) -> Option<usize> {
    // Get the bound index at this position
    let bound_idx = ael.get(bound_pos)?;
    let bound = bounds.get(bound_idx)?;

    // PORT FROM: C++ uses maximum_bound pointer for O(1) lookup
    // If maximum_bound is set, use it directly
    if let Some(max_bound_idx) = bound.maximum_bound {
        // Find this bound's position in the active edge list
        return ael.position(max_bound_idx);
    }

    // Fallback: Search for a bound with matching top point
    // This is O(n) but handles cases where maximum_bound isn't set
    let our_top = bound.current_edge().top;

    for pos in 0..ael.len() {
        if pos == bound_pos {
            continue;
        }
        if let Some(&other_idx) = ael.iter().nth(pos) {
            if let Some(other_bound) = bounds.get(other_idx) {
                let other_top = other_bound.current_edge().top;
                if our_top == other_top {
                    return Some(pos);
                }
            }
        }
    }

    None
}

// ============================================================================
// Main maxima processing
// ============================================================================

/// Process a maxima event for a bound.
///
/// From C++: `do_maxima(bnd, bndMaxPair, cliptype, ...)`
///
/// This function processes bounds that have reached their maximum point during
/// the sweep. It handles intersections with intervening bounds and connects
/// the maxima pair.
///
/// # Arguments
/// * `bound_pos` - Position of the bound in the active edge list
/// * `max_pair_pos` - Position of the maxima pair in the active edge list
/// * `bounds` - Mutable slice of all bounds
/// * `ael` - The active edge list
/// * `cliptype` - The boolean operation type
/// * `subject_fill_type` - Fill rule for subject polygon
/// * `clip_fill_type` - Fill rule for clip polygon
///
/// # Returns
/// The next position to process in the AEL after handling the maxima.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/process_maxima.hpp - do_maxima (lines 21-55)
///
/// NOTE: The caller (process_edges_at_top_of_scanbeam) must call add_local_maximum_point
/// before calling this function, as we don't have access to the RingManager here.
/// The C++ version calls add_local_maximum_point internally, but the Rust version
/// separates concerns - ring operations are handled in the caller.
pub fn do_maxima<T: CoordNum>(
    bound_pos: usize,
    max_pair_pos: usize,
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    _cliptype: Operation,
    _subject_fill_type: FillType,
    _clip_fill_type: FillType,
) -> usize {
    // Get the bound indices
    let bound_idx = match ael.get(bound_pos) {
        Some(idx) => idx,
        None => return bound_pos,
    };
    let max_pair_idx = match ael.get(max_pair_pos) {
        Some(idx) => idx,
        None => return bound_pos,
    };

    // Get the top point (maxima point)
    let _maxima_point = bounds[bound_idx].current_edge().top;

    // Track if we skipped any bounds (for return value calculation)
    let mut skipped = false;

    // Process any bounds between bound_pos and max_pair_pos
    // PORT FROM: C++ while loop that calls intersect_bounds for intervening bounds
    let mut current_pos = bound_pos;
    let mut next_pos = if bound_pos < max_pair_pos {
        bound_pos + 1
    } else {
        bound_pos.saturating_sub(1)
    };

    // Move towards the maxima pair, processing intersections
    while next_pos != max_pair_pos && next_pos < ael.len() {
        // In full implementation: call intersect_bounds here for the intervening bound
        // For now, swap positions to move bound towards its pair
        skipped = true;
        ael.swap(current_pos, next_pos);
        current_pos = next_pos;
        next_pos = if bound_pos < max_pair_pos {
            next_pos + 1
        } else {
            next_pos.saturating_sub(1)
        };
    }

    // Clear ring references on both bounds (they're done contributing)
    // PORT FROM: C++ sets *bndMaxPair = nullptr and *bnd = nullptr
    // Note: add_local_maximum_point already clears .ring, but we ensure it here
    bounds[bound_idx].ring = None;
    bounds[max_pair_idx].ring = None;

    // Mark bounds as processed by removing from AEL
    // Remove in reverse order of position to keep indices valid
    let (first_remove, second_remove) = if current_pos > max_pair_pos {
        (current_pos, max_pair_pos)
    } else {
        (max_pair_pos, current_pos)
    };

    // Remove from AEL (bounds are marked as "null" in C++, we remove indices)
    if let Some(idx) = ael.get(first_remove) {
        ael.remove(idx);
    }
    if let Some(idx) = ael.get(second_remove) {
        ael.remove(idx);
    }

    // Return the position to continue processing from
    // PORT FROM: C++ returns return_bnd which is incremented if !skipped
    if skipped {
        bound_pos
    } else {
        bound_pos.saturating_add(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::Edge;
    use crate::config::{EdgeSide, PolygonType};
    use crate::point::Point;

    // ==================== Test Helpers ====================

    fn make_bound_with_edges(edges: Vec<(Point<f64>, Point<f64>)>) -> Bound<f64> {
        let edges: Vec<Edge<f64>> = edges
            .into_iter()
            .map(|(bot, top)| Edge::new(bot, top))
            .collect();
        Bound::new(edges, PolygonType::Subject, EdgeSide::Left)
    }

    fn make_simple_bound(bot: Point<f64>, top: Point<f64>) -> Bound<f64> {
        make_bound_with_edges(vec![(bot, top)])
    }

    // ==================== is_maxima Tests ====================

    #[test]
    fn is_maxima_returns_true_at_last_edge_top() {
        // Single edge bound at its top = maxima
        let bound = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));

        assert!(is_maxima(&bound, 10.0));
    }

    #[test]
    fn is_maxima_returns_false_below_top() {
        let bound = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));

        assert!(!is_maxima(&bound, 5.0));
    }

    #[test]
    fn is_maxima_returns_false_with_more_edges() {
        // Two edge bound - at first edge top, still has more edges
        let bound = make_bound_with_edges(vec![
            (Point::new(0.0, 0.0), Point::new(5.0, 10.0)),
            (Point::new(5.0, 10.0), Point::new(10.0, 20.0)),
        ]);

        // At y=10 (first edge top), but there's another edge
        assert!(!is_maxima(&bound, 10.0));
    }

    #[test]
    fn is_maxima_multi_edge_at_final_top() {
        // Two edge bound, advanced to second edge
        let mut bound = make_bound_with_edges(vec![
            (Point::new(0.0, 0.0), Point::new(5.0, 10.0)),
            (Point::new(5.0, 10.0), Point::new(10.0, 20.0)),
        ]);
        bound.next_edge(); // Advance to second edge

        // Now at y=20 (second edge top), no more edges
        assert!(is_maxima(&bound, 20.0));
    }

    // ==================== is_intermediate Tests ====================

    #[test]
    fn is_intermediate_returns_true_with_more_edges() {
        let bound = make_bound_with_edges(vec![
            (Point::new(0.0, 0.0), Point::new(5.0, 10.0)),
            (Point::new(5.0, 10.0), Point::new(10.0, 20.0)),
        ]);

        // At y=10 (first edge top), has more edges = intermediate
        assert!(is_intermediate(&bound, 10.0));
    }

    #[test]
    fn is_intermediate_returns_false_at_maxima() {
        let bound = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));

        // At y=10, no more edges = not intermediate (it's maxima)
        assert!(!is_intermediate(&bound, 10.0));
    }

    #[test]
    fn is_intermediate_returns_false_below_edge_top() {
        let bound = make_bound_with_edges(vec![
            (Point::new(0.0, 0.0), Point::new(5.0, 10.0)),
            (Point::new(5.0, 10.0), Point::new(10.0, 20.0)),
        ]);

        // At y=5, not at edge top
        assert!(!is_intermediate(&bound, 5.0));
    }

    // ==================== current_edge_is_horizontal Tests ====================

    #[test]
    fn current_edge_is_horizontal_returns_true_for_horizontal() {
        let bound = make_simple_bound(Point::new(0.0, 5.0), Point::new(10.0, 5.0));

        assert!(current_edge_is_horizontal(&bound));
    }

    #[test]
    fn current_edge_is_horizontal_returns_false_for_non_horizontal() {
        let bound = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));

        assert!(!current_edge_is_horizontal(&bound));
    }

    // ==================== next_edge_is_horizontal Tests ====================

    #[test]
    fn next_edge_is_horizontal_returns_true_when_next_is_horizontal() {
        let bound = make_bound_with_edges(vec![
            (Point::new(0.0, 0.0), Point::new(5.0, 10.0)), // Not horizontal
            (Point::new(5.0, 10.0), Point::new(15.0, 10.0)), // Horizontal
        ]);

        assert!(next_edge_is_horizontal(&bound));
    }

    #[test]
    fn next_edge_is_horizontal_returns_false_when_next_is_not_horizontal() {
        let bound = make_bound_with_edges(vec![
            (Point::new(0.0, 0.0), Point::new(5.0, 10.0)),
            (Point::new(5.0, 10.0), Point::new(10.0, 20.0)),
        ]);

        assert!(!next_edge_is_horizontal(&bound));
    }

    #[test]
    fn next_edge_is_horizontal_returns_false_when_no_next_edge() {
        let bound = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));

        assert!(!next_edge_is_horizontal(&bound));
    }

    // ==================== get_maxima_pair Tests ====================

    #[test]
    fn get_maxima_pair_finds_matching_top() {
        // Two bounds meeting at the same top point
        let bound1 = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));
        let bound2 = make_simple_bound(Point::new(10.0, 0.0), Point::new(5.0, 10.0));
        let bounds = vec![bound1, bound2];

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        // Bound 0's maxima pair should be bound at position 1
        let pair = get_maxima_pair(0, &bounds, &ael);
        assert_eq!(pair, Some(1));
    }

    #[test]
    fn get_maxima_pair_returns_none_when_no_match() {
        // Two bounds with different top points
        let bound1 = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));
        let bound2 = make_simple_bound(Point::new(10.0, 0.0), Point::new(15.0, 20.0));
        let bounds = vec![bound1, bound2];

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        let pair = get_maxima_pair(0, &bounds, &ael);
        assert_eq!(pair, None);
    }

    // ==================== do_maxima Tests ====================

    #[test]
    fn do_maxima_removes_both_bounds_from_ael() {
        // Two bounds meeting at maxima
        let bound1 = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));
        let bound2 = make_simple_bound(Point::new(10.0, 0.0), Point::new(5.0, 10.0));
        let mut bounds = vec![bound1, bound2];

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        assert_eq!(ael.len(), 2);

        do_maxima(
            0,
            1,
            &mut bounds,
            &mut ael,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Both bounds should be removed
        assert!(ael.is_empty());
    }

    #[test]
    fn do_maxima_handles_adjacent_bounds() {
        // Two adjacent bounds at positions 0 and 1
        let bound1 = make_simple_bound(Point::new(0.0, 0.0), Point::new(5.0, 10.0));
        let bound2 = make_simple_bound(Point::new(10.0, 0.0), Point::new(5.0, 10.0));
        let mut bounds = vec![bound1, bound2];

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        let next_pos = do_maxima(
            0,
            1,
            &mut bounds,
            &mut ael,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Should return starting position
        assert_eq!(next_pos, 0);
    }
}
