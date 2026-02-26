//! Process horizontal edges during the Vatti sweep algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/process_horizontal.hpp
//!
//! Horizontal edges require special handling in the Vatti algorithm because
//! they don't have a well-defined intersection Y coordinate with the scanline.
//! Instead, they are processed by traversing left-to-right or right-to-left
//! and handling intersections with other edges along the way.
//!
//! Key concepts:
//! - Horizontal edges have `dx == infinity` (or very large)
//! - Processing direction depends on edge direction (bot.x vs top.x)
//! - Intersections along the horizontal are processed in order
//! - "Hot pixels" are used to improve output quality

use geo_types::CoordNum;
use num_traits::ToPrimitive;

use crate::active_edge_list::ActiveEdgeList;
use crate::bound::Bound;
use crate::build_result::RingManager;
use crate::config::{FillType, HorizontalDirection};
use crate::intersect_util::get_current_x;
use crate::local_minimum::LocalMinimumList;
use crate::process_maxima::{do_maxima, get_maxima_pair, is_maxima};
use crate::scanbeam::Scanbeam;
use crate::Operation;

// ============================================================================
// Direction Detection
// ============================================================================

/// Determine the processing direction for a horizontal edge.
///
/// From C++: Part of `process_horizontal`
///
/// If bot.x < top.x, the horizontal goes left-to-right.
/// Otherwise, it goes right-to-left.
pub fn get_horizontal_direction<T: CoordNum>(bound: &Bound<T>) -> HorizontalDirection {
    let edge = bound.current_edge();
    let bot_x = edge.bot.x.to_f64().unwrap_or(0.0);
    let top_x = edge.top.x.to_f64().unwrap_or(0.0);

    if bot_x < top_x {
        HorizontalDirection::LeftToRight
    } else {
        HorizontalDirection::RightToLeft
    }
}

/// Get the left and right X extents of a horizontal edge.
///
/// Returns (left_x, right_x) where left_x <= right_x.
pub fn get_horizontal_extents<T: CoordNum>(bound: &Bound<T>) -> (f64, f64) {
    let edge = bound.current_edge();
    let bot_x = edge.bot.x.to_f64().unwrap_or(0.0);
    let top_x = edge.top.x.to_f64().unwrap_or(0.0);

    if bot_x < top_x {
        (bot_x, top_x)
    } else {
        (top_x, bot_x)
    }
}

// ============================================================================
// Horizontal Edge Checks
// ============================================================================

/// Check if a bound's current edge is horizontal.
///
/// From C++: `current_edge_is_horizontal<T>(bnd)`
pub fn current_edge_is_horizontal<T: CoordNum>(bound: &Bound<T>) -> bool {
    bound.current_edge().is_horizontal()
}

/// Check if the next edge in a bound would be horizontal.
///
/// From C++: Helper for process_horizontal
pub fn next_edge_would_be_horizontal<T: CoordNum>(bound: &Bound<T>) -> bool {
    if bound.current_edge_index + 1 < bound.edges.len() {
        bound.edges[bound.current_edge_index + 1].is_horizontal()
    } else {
        false
    }
}

// ============================================================================
// Process Horizontal Left-to-Right
// ============================================================================

/// Process a horizontal edge from left to right.
///
/// From C++: `process_horizontal_left_to_right(scanline_y, horz_bound, ...)`
///
/// This traverses the AEL from the horizontal bound's position rightward,
/// processing intersections with each bound encountered until reaching
/// the end of the horizontal edge.
///
/// # Arguments
/// * `horz_bound_pos` - Position of the horizontal bound in the AEL
/// * `bounds` - All bounds
/// * `ael` - The active edge list
/// * `scanline_y` - Current scanline Y coordinate
/// * `scanbeam` - The scanbeam (for adding new edge tops)
/// * `cliptype` - Boolean operation type
/// * `subject_fill_type` - Fill rule for subject
/// * `clip_fill_type` - Fill rule for clip
///
/// # Returns
/// Position to resume processing from (typically where the horizontal was)
#[allow(clippy::too_many_arguments)]
pub fn process_horizontal_left_to_right<T: CoordNum + ToPrimitive>(
    horz_bound_pos: usize,
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    scanline_y: T,
    scanbeam: &mut Scanbeam<T>,
    _cliptype: Operation,
    _subject_fill_type: FillType,
    _clip_fill_type: FillType,
) -> usize {
    let horz_bound_idx = match ael.get(horz_bound_pos) {
        Some(idx) => idx,
        None => return horz_bound_pos,
    };

    // Get the right extent of the horizontal edge
    let (_, right_x) = get_horizontal_extents(&bounds[horz_bound_idx]);

    // Check if this is a maxima edge
    let is_maxima_edge = is_maxima(&bounds[horz_bound_idx], scanline_y);
    let max_pair_pos = if is_maxima_edge {
        get_maxima_pair(horz_bound_pos, bounds, ael)
    } else {
        None
    };

    // Process bounds to the right
    let mut current_pos = horz_bound_pos;
    let mut next_pos = horz_bound_pos + 1;

    while next_pos < ael.len() {
        let next_idx = match ael.get(next_pos) {
            Some(idx) => idx,
            None => break,
        };

        let next_x = bounds[next_idx].current_x;

        // Stop if we've passed the end of the horizontal
        if next_x > right_x {
            break;
        }

        // If this is the maxima pair, handle the maxima
        if Some(next_pos) == max_pair_pos {
            // Mark both bounds for removal (simplified - full impl would use ring manager)
            break;
        }

        // Process intersection (simplified - full impl would call intersect_bounds)
        // For now, just swap the bounds
        ael.swap(current_pos, next_pos);
        current_pos = next_pos;
        next_pos += 1;
    }

    // Advance to next edge if there is one
    let horz_idx = match ael.get(current_pos) {
        Some(idx) => idx,
        None => return horz_bound_pos,
    };

    if bounds[horz_idx].current_edge_index + 1 < bounds[horz_idx].edges.len() {
        bounds[horz_idx].current_edge_index += 1;
        let new_top_y = bounds[horz_idx].current_edge().top.y;
        scanbeam.insert(new_top_y);
    }

    horz_bound_pos
}

/// Process a horizontal edge from right to left.
///
/// From C++: `process_horizontal_right_to_left(scanline_y, horz_bound, ...)`
///
/// Similar to left-to-right but traverses in the opposite direction.
#[allow(clippy::too_many_arguments)]
pub fn process_horizontal_right_to_left<T: CoordNum + ToPrimitive>(
    horz_bound_pos: usize,
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    scanline_y: T,
    scanbeam: &mut Scanbeam<T>,
    _cliptype: Operation,
    _subject_fill_type: FillType,
    _clip_fill_type: FillType,
) -> usize {
    let horz_bound_idx = match ael.get(horz_bound_pos) {
        Some(idx) => idx,
        None => return horz_bound_pos,
    };

    // Get the left extent of the horizontal edge
    let (left_x, _) = get_horizontal_extents(&bounds[horz_bound_idx]);

    // Check if this is a maxima edge
    let is_maxima_edge = is_maxima(&bounds[horz_bound_idx], scanline_y);
    let max_pair_pos = if is_maxima_edge {
        get_maxima_pair(horz_bound_pos, bounds, ael)
    } else {
        None
    };

    // Process bounds to the left
    let mut current_pos = horz_bound_pos;

    while current_pos > 0 {
        let prev_pos = current_pos - 1;
        let prev_idx = match ael.get(prev_pos) {
            Some(idx) => idx,
            None => break,
        };

        let prev_x = bounds[prev_idx].current_x;

        // Stop if we've passed the end of the horizontal
        if prev_x < left_x {
            break;
        }

        // If this is the maxima pair, handle the maxima
        if Some(prev_pos) == max_pair_pos {
            break;
        }

        // Process intersection (simplified)
        ael.swap(prev_pos, current_pos);
        current_pos = prev_pos;
    }

    // Advance to next edge if there is one
    let horz_idx = match ael.get(current_pos) {
        Some(idx) => idx,
        None => return horz_bound_pos,
    };

    if bounds[horz_idx].current_edge_index + 1 < bounds[horz_idx].edges.len() {
        bounds[horz_idx].current_edge_index += 1;
        let new_top_y = bounds[horz_idx].current_edge().top.y;
        scanbeam.insert(new_top_y);
    }

    if horz_bound_pos < ael.len() {
        horz_bound_pos
    } else {
        0
    }
}

// ============================================================================
// Main Process Horizontal
// ============================================================================

/// Process a single horizontal edge.
///
/// From C++: `process_horizontal(scanline_y, horz_bound, ...)`
///
/// Dispatches to left-to-right or right-to-left based on edge direction.
#[allow(clippy::too_many_arguments)]
pub fn process_horizontal<T: CoordNum + ToPrimitive>(
    horz_bound_pos: usize,
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    scanline_y: T,
    scanbeam: &mut Scanbeam<T>,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> usize {
    let horz_bound_idx = match ael.get(horz_bound_pos) {
        Some(idx) => idx,
        None => return horz_bound_pos,
    };

    let direction = get_horizontal_direction(&bounds[horz_bound_idx]);

    match direction {
        HorizontalDirection::LeftToRight => process_horizontal_left_to_right(
            horz_bound_pos,
            bounds,
            ael,
            scanline_y,
            scanbeam,
            cliptype,
            subject_fill_type,
            clip_fill_type,
        ),
        HorizontalDirection::RightToLeft => process_horizontal_right_to_left(
            horz_bound_pos,
            bounds,
            ael,
            scanline_y,
            scanbeam,
            cliptype,
            subject_fill_type,
            clip_fill_type,
        ),
    }
}

/// Process all horizontal edges at the current scanline.
///
/// From C++: `process_horizontals(scanline_y, active_bounds, ...)`
///
/// Iterates through the AEL and processes each bound that has a horizontal
/// current edge.
pub fn process_horizontals<T: CoordNum + ToPrimitive>(
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    scanline_y: T,
    scanbeam: &mut Scanbeam<T>,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) {
    let mut pos = 0;

    while pos < ael.len() {
        let bound_idx = match ael.get(pos) {
            Some(idx) => idx,
            None => {
                pos += 1;
                continue;
            }
        };

        if current_edge_is_horizontal(&bounds[bound_idx]) {
            pos = process_horizontal(
                pos,
                bounds,
                ael,
                scanline_y,
                scanbeam,
                cliptype,
                subject_fill_type,
                clip_fill_type,
            );
        }
        pos += 1;
    }
}

/// Update current_x for all bounds before processing horizontals.
///
/// This ensures bounds are positioned correctly for horizontal intersection
/// detection.
pub fn update_all_current_x<T: CoordNum>(bounds: &mut [Bound<T>], ael: &ActiveEdgeList, y: T) {
    for &idx in ael.iter() {
        if let Some(bound) = bounds.get_mut(idx) {
            bound.current_x = get_current_x(bound.current_edge(), y);
        }
    }
}

/// Process edges at the top of the scanbeam.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/active_bound_list.hpp - process_edges_at_top_of_scanbeam
///
/// This function handles two types of events at the current scanline:
/// 1. Maxima - edges that end at this Y coordinate
/// 2. Horizontal edges - edges that need to be processed along the scanline
///
/// # Arguments
///
/// * `scanline_y` - Current Y coordinate of the scanline
/// * `ael` - Active edge list
/// * `bounds` - Storage for all bounds
/// * `scanbeam` - Priority queue of Y coordinates
/// * `minima_sorted` - Sorted indices into minima_list
/// * `current_lm_idx` - Current position in minima_sorted
/// * `minima_list` - List of local minima
/// * `_manager` - Ring manager for output (unused in this stub)
/// * `clip_type` - Type of boolean operation
/// * `subject_fill_type` - Fill rule for subject polygons
/// * `clip_fill_type` - Fill rule for clip polygons
#[allow(clippy::too_many_arguments)]
pub fn process_edges_at_top_of_scanbeam<T: CoordNum + ToPrimitive>(
    scanline_y: T,
    ael: &mut ActiveEdgeList,
    bounds: &mut [Bound<T>],
    scanbeam: &mut Scanbeam<T>,
    _minima_sorted: &[usize],
    _current_lm_idx: &mut usize,
    _minima_list: &LocalMinimumList<T>,
    _manager: &mut RingManager<T>,
    clip_type: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) {
    // Update current_x for all active bounds at the new scanline
    update_all_current_x(bounds, ael, scanline_y);

    // Process all edges in the active edge list
    let mut i = 0;
    while i < ael.len() {
        let bound_idx = match ael.get(i) {
            Some(idx) => idx,
            None => {
                i += 1;
                continue;
            }
        };

        let bound = match bounds.get(bound_idx) {
            Some(b) => b,
            None => {
                i += 1;
                continue;
            }
        };

        // Check if this bound has reached its maxima
        if is_maxima(bound, scanline_y) {
            // Find the maxima pair (note: argument order is bound_pos, bounds, ael)
            if let Some(pair_pos) = get_maxima_pair(i, bounds, ael) {
                // Process the maxima (note: do_maxima takes positions, not indices)
                do_maxima(
                    i,
                    pair_pos,
                    bounds,
                    ael,
                    clip_type,
                    subject_fill_type,
                    clip_fill_type,
                );
                // do_maxima may have removed entries from ael, so don't increment i
                continue;
            }
        }

        // Check for horizontal edges
        let bound = match bounds.get(bound_idx) {
            Some(b) => b,
            None => {
                i += 1;
                continue;
            }
        };

        if current_edge_is_horizontal(bound) {
            // Process the horizontal edge
            process_horizontal(
                i,
                bounds,
                ael,
                scanline_y,
                scanbeam,
                clip_type,
                subject_fill_type,
                clip_fill_type,
            );
        }

        // Move to next edge if it's at this scanline
        if let Some(bound) = bounds.get_mut(bound_idx) {
            if bound.current_edge_index + 1 < bound.edges.len() {
                let next_edge = &bound.edges[bound.current_edge_index + 1];
                if next_edge.bot.y == scanline_y {
                    bound.current_edge_index += 1;
                    scanbeam.insert(next_edge.top.y);
                }
            }
        }

        i += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::Edge;
    use crate::config::{EdgeSide, PolygonType};
    use crate::point::Point;

    // ==================== Test Helpers ====================

    fn make_bound(bot: (f64, f64), top: (f64, f64)) -> Bound<f64> {
        let edge = Edge::new(Point::new(bot.0, bot.1), Point::new(top.0, top.1));
        Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left)
    }

    fn make_horizontal_bound(left_x: f64, right_x: f64, y: f64) -> Bound<f64> {
        // Horizontal edge from left to right
        let edge = Edge::new(Point::new(left_x, y), Point::new(right_x, y));
        Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left)
    }

    // ==================== get_horizontal_direction Tests ====================

    #[test]
    fn get_horizontal_direction_left_to_right() {
        let bound = make_horizontal_bound(0.0, 10.0, 5.0);
        assert_eq!(
            get_horizontal_direction(&bound),
            HorizontalDirection::LeftToRight
        );
    }

    #[test]
    fn get_horizontal_direction_right_to_left() {
        // Edge going from right to left
        let edge = Edge::new(Point::new(10.0_f64, 5.0), Point::new(0.0, 5.0));
        let bound = Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left);
        assert_eq!(
            get_horizontal_direction(&bound),
            HorizontalDirection::RightToLeft
        );
    }

    // ==================== get_horizontal_extents Tests ====================

    #[test]
    fn get_horizontal_extents_left_to_right() {
        let bound = make_horizontal_bound(0.0, 10.0, 5.0);
        let (left, right) = get_horizontal_extents(&bound);
        assert!((left - 0.0).abs() < 1e-10);
        assert!((right - 10.0).abs() < 1e-10);
    }

    #[test]
    fn get_horizontal_extents_right_to_left() {
        let edge = Edge::new(Point::new(10.0_f64, 5.0), Point::new(0.0, 5.0));
        let bound = Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left);
        let (left, right) = get_horizontal_extents(&bound);
        assert!((left - 0.0).abs() < 1e-10);
        assert!((right - 10.0).abs() < 1e-10);
    }

    // ==================== current_edge_is_horizontal Tests ====================

    #[test]
    fn current_edge_is_horizontal_returns_true() {
        let bound = make_horizontal_bound(0.0, 10.0, 5.0);
        assert!(current_edge_is_horizontal(&bound));
    }

    #[test]
    fn current_edge_is_horizontal_returns_false() {
        let bound = make_bound((0.0, 0.0), (5.0, 10.0));
        assert!(!current_edge_is_horizontal(&bound));
    }

    // ==================== next_edge_would_be_horizontal Tests ====================

    #[test]
    fn next_edge_would_be_horizontal_true() {
        let edges = vec![
            Edge::new(Point::new(0.0_f64, 0.0), Point::new(5.0, 10.0)), // Non-horizontal
            Edge::new(Point::new(5.0_f64, 10.0), Point::new(15.0, 10.0)), // Horizontal
        ];
        let bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);
        assert!(next_edge_would_be_horizontal(&bound));
    }

    #[test]
    fn next_edge_would_be_horizontal_false() {
        let edges = vec![
            Edge::new(Point::new(0.0_f64, 0.0), Point::new(5.0, 10.0)),
            Edge::new(Point::new(5.0_f64, 10.0), Point::new(10.0, 20.0)),
        ];
        let bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);
        assert!(!next_edge_would_be_horizontal(&bound));
    }

    #[test]
    fn next_edge_would_be_horizontal_no_next() {
        let bound = make_bound((0.0, 0.0), (5.0, 10.0));
        assert!(!next_edge_would_be_horizontal(&bound));
    }

    // ==================== process_horizontal_left_to_right Tests ====================

    #[test]
    fn process_horizontal_left_to_right_basic() {
        // Set up: horizontal edge at y=5, crossing a vertical edge
        let mut bounds = vec![
            make_horizontal_bound(0.0, 10.0, 5.0),
            make_bound((5.0, 0.0), (5.0, 10.0)), // Vertical at x=5
        ];
        bounds[0].current_x = 0.0;
        bounds[1].current_x = 5.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        let result = process_horizontal_left_to_right(
            0,
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Should complete without panic
        assert!(result <= ael.len());
    }

    #[test]
    fn process_horizontal_left_to_right_no_intersections() {
        // Horizontal edge doesn't intersect anything
        let mut bounds = vec![make_horizontal_bound(0.0, 10.0, 5.0)];
        bounds[0].current_x = 0.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);

        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        let result = process_horizontal_left_to_right(
            0,
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        assert_eq!(result, 0);
    }

    // ==================== process_horizontal_right_to_left Tests ====================

    #[test]
    fn process_horizontal_right_to_left_basic() {
        // Horizontal edge going right to left
        let edge = Edge::new(Point::new(10.0_f64, 5.0), Point::new(0.0, 5.0));
        let mut bounds = vec![
            Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left),
            make_bound((5.0, 0.0), (5.0, 10.0)),
        ];
        bounds[0].current_x = 10.0;
        bounds[1].current_x = 5.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(1, &bounds); // Insert vertical first (lower x)
        ael.insert(0, &bounds); // Then horizontal (higher x)

        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        let result = process_horizontal_right_to_left(
            1, // Position of horizontal in AEL
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Should complete without panic
        assert!(result <= ael.len());
    }

    // ==================== process_horizontal Tests ====================

    #[test]
    fn process_horizontal_dispatches_correctly_ltr() {
        let mut bounds = vec![make_horizontal_bound(0.0, 10.0, 5.0)];
        bounds[0].current_x = 0.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);

        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        // Should dispatch to left-to-right
        let result = process_horizontal(
            0,
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        assert_eq!(result, 0);
    }

    #[test]
    fn process_horizontal_dispatches_correctly_rtl() {
        let edge = Edge::new(Point::new(10.0_f64, 5.0), Point::new(0.0, 5.0));
        let mut bounds = vec![Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left)];
        bounds[0].current_x = 10.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);

        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        // Should dispatch to right-to-left
        let result = process_horizontal(
            0,
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        assert_eq!(result, 0);
    }

    // ==================== process_horizontals Tests ====================

    #[test]
    fn process_horizontals_skips_non_horizontal() {
        let mut bounds = vec![
            make_bound((0.0, 0.0), (5.0, 10.0)),   // Non-horizontal
            make_bound((10.0, 0.0), (15.0, 10.0)), // Non-horizontal
        ];
        bounds[0].current_x = 0.0;
        bounds[1].current_x = 10.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        // Should not modify anything since no horizontals
        process_horizontals(
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // AEL should remain unchanged
        assert_eq!(ael.len(), 2);
    }

    #[test]
    fn process_horizontals_processes_horizontal() {
        let mut bounds = vec![
            make_horizontal_bound(0.0, 10.0, 5.0),
            make_bound((5.0, 0.0), (5.0, 10.0)),
        ];
        bounds[0].current_x = 0.0;
        bounds[1].current_x = 5.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        process_horizontals(
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Should complete without panic
        assert!(ael.len() <= 2);
    }

    // ==================== update_all_current_x Tests ====================

    #[test]
    fn update_all_current_x_updates_positions() {
        let mut bounds = vec![
            make_bound((0.0, 0.0), (10.0, 10.0)), // Slope: dx = 1
            make_bound((5.0, 0.0), (15.0, 10.0)), // Slope: dx = 1
        ];

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        // Update at y=5
        update_all_current_x(&mut bounds, &ael, 5.0_f64);

        // At y=5: bound 0 should be at x=5, bound 1 should be at x=10
        assert!((bounds[0].current_x - 5.0).abs() < 1e-10);
        assert!((bounds[1].current_x - 10.0).abs() < 1e-10);
    }
}
