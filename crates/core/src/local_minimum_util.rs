//! Local minimum utilities for the Vatti clipping algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/local_minimum_util.hpp
//!
//! This module provides additional utility functions for working with local
//! minima during the sweep. Most local minimum construction is handled by
//! `build_local_minima_list`, but these utilities are needed during sweep
//! execution.
//!
//! Note: Much of local_minimum_util.hpp is already ported to build_local_minima_list.rs.
//! This module contains remaining utilities used during sweep execution.

use geo_types::CoordNum;

use crate::active_edge_list::ActiveEdgeList;
use crate::bound::Bound;
use crate::config::EdgeSide;
use crate::intersect_util::get_current_x;
use crate::local_minimum::LocalMinimum;
use crate::scanbeam::Scanbeam;

// ============================================================================
// Local Minimum Initialization
// ============================================================================

/// Initialize a local minimum's bounds for sweep processing.
///
/// From C++: `initialize_lm(lm)`
///
/// This sets up the current_x and resets winding counts for both bounds
/// before they are inserted into the active edge list.
///
/// # Arguments
/// * `lm` - The local minimum to initialize
pub fn initialize_lm<T: CoordNum>(lm: &mut LocalMinimum<T>) {
    // Initialize left bound
    if !lm.left_bound.edges.is_empty() {
        lm.left_bound.current_edge_index = 0;
        let first_edge = &lm.left_bound.edges[0];
        lm.left_bound.current_x = first_edge.bot.x.to_f64().unwrap_or(0.0);
        lm.left_bound.winding_count = 0;
        lm.left_bound.winding_count2 = 0;
        lm.left_bound.side = EdgeSide::Left;
        lm.left_bound.ring = None;
    }

    // Initialize right bound
    if !lm.right_bound.edges.is_empty() {
        lm.right_bound.current_edge_index = 0;
        let first_edge = &lm.right_bound.edges[0];
        lm.right_bound.current_x = first_edge.bot.x.to_f64().unwrap_or(0.0);
        lm.right_bound.winding_count = 0;
        lm.right_bound.winding_count2 = 0;
        lm.right_bound.side = EdgeSide::Right;
        lm.right_bound.ring = None;
    }
}

// ============================================================================
// Scanbeam Management
// ============================================================================

/// Add the next edge's top Y to the scanbeam.
///
/// From C++: `next_edge_in_bound(bound, scanbeam)`
///
/// Advances the bound to its next edge and adds the new edge's top Y
/// to the scanbeam for future processing.
///
/// # Arguments
/// * `bound` - The bound to advance
/// * `scanbeam` - The scanbeam to add the Y coordinate to
///
/// # Returns
/// `true` if successfully advanced to next edge, `false` if at last edge
pub fn next_edge_in_bound<T: CoordNum>(bound: &mut Bound<T>, scanbeam: &mut Scanbeam<T>) -> bool {
    if bound.next_edge() {
        // Add new edge's top Y to scanbeam
        let top_y = bound.current_edge().top.y;
        scanbeam.insert(top_y);
        true
    } else {
        false
    }
}

/// Insert a local minimum's bounds into the active edge list.
///
/// From C++: Part of `insert_local_minima_into_ABL`
///
/// Inserts both left and right bounds into the AEL, maintaining proper
/// sorted order.
///
/// # Arguments
/// * `lm` - The local minimum whose bounds to insert
/// * `bounds` - Vector of all bounds (will have lm's bounds appended)
/// * `ael` - The active edge list
/// * `scanbeam` - The scanbeam for adding edge top Y coordinates
///
/// # Returns
/// A tuple of (left_bound_index, right_bound_index) in the bounds vector
pub fn insert_local_minimum_into_ael<T: CoordNum>(
    lm: LocalMinimum<T>,
    bounds: &mut Vec<Bound<T>>,
    ael: &mut ActiveEdgeList,
    scanbeam: &mut Scanbeam<T>,
) -> (usize, usize) {
    // Add bounds to the bounds vector
    let left_idx = bounds.len();
    bounds.push(lm.left_bound);
    let right_idx = bounds.len();
    bounds.push(lm.right_bound);

    // Insert into AEL
    ael.insert_pair(left_idx, right_idx, bounds);

    // Add edge tops to scanbeam
    let left_top = bounds[left_idx].current_edge().top.y;
    let right_top = bounds[right_idx].current_edge().top.y;
    scanbeam.insert(left_top);
    scanbeam.insert(right_top);

    (left_idx, right_idx)
}

/// Check if a bound's current edge is at its top at the given Y.
///
/// From C++: Helper used in sweep processing
pub fn at_edge_top<T: CoordNum + PartialEq>(bound: &Bound<T>, y: T) -> bool {
    bound.current_edge().top.y == y
}

// ============================================================================
// Insert Local Minima into ABL (Active Bound List)
// ============================================================================

use crate::build_result::RingManager;
use crate::config::FillType;
use crate::local_minimum::LocalMinimumList;
use crate::ring_util;
use crate::winding;
use crate::Operation;
use num_traits::ToPrimitive;

/// Insert local minima at the current scanline into the active bound list.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/active_bound_list.hpp - insert_local_minima_into_ABL
///
/// This function processes all local minima that have their Y coordinate equal to bot_y,
/// initializing them and inserting their bounds into the active edge list.
///
/// # Arguments
/// * `bot_y` - The current scanline Y coordinate
/// * `minima_sorted` - Sorted list of local minima (by Y descending)
/// * `current_lm_idx` - Current index into minima_sorted (will be incremented)
/// * `bounds` - Storage for all bounds
/// * `ael` - Active edge list
/// * `manager` - Ring manager (used for contributing edges)
/// * `scanbeam` - Scanbeam for adding edge top Y coordinates
/// * `cliptype` - Boolean operation type
/// * `subject_fill_type` - Fill rule for subject polygons
/// * `clip_fill_type` - Fill rule for clip polygons
#[allow(clippy::too_many_arguments)]
pub fn insert_local_minima_into_abl<T: CoordNum + ToPrimitive>(
    bot_y: T,
    minima_sorted: &mut LocalMinimumList<T>,
    current_lm_idx: &mut usize,
    bounds: &mut Vec<Bound<T>>,
    ael: &mut ActiveEdgeList,
    manager: &mut RingManager<T>,
    scanbeam: &mut Scanbeam<T>,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) {
    let bot_y_f64 = bot_y.to_f64().unwrap_or(0.0);

    // Process all local minima at the current scanline Y
    while *current_lm_idx < minima_sorted.len() {
        let lm_y = minima_sorted[*current_lm_idx].y;
        let lm_y_f64 = lm_y.to_f64().unwrap_or(0.0);

        // Check if this LM is at the current scanline (bot_y == lm.y)
        if (lm_y_f64 - bot_y_f64).abs() > f64::EPSILON {
            break;
        }

        // Initialize the local minimum
        // From C++: initialize_lm<T>(current_lm)
        initialize_lm(&mut minima_sorted[*current_lm_idx]);

        // Take ownership of the bounds from the local minimum
        // In C++, bounds are referenced; in Rust we move them into the bounds vec
        let lm = &mut minima_sorted[*current_lm_idx];

        // Extract the bounds - we need to move them out
        // Create placeholder bounds and swap
        let left_bound = std::mem::replace(
            &mut lm.left_bound,
            Bound::new_empty(crate::config::PolygonType::Subject, EdgeSide::Left),
        );
        let right_bound = std::mem::replace(
            &mut lm.right_bound,
            Bound::new_empty(crate::config::PolygonType::Subject, EdgeSide::Right),
        );

        // Insert bounds into bounds vector
        let left_idx = bounds.len();
        bounds.push(left_bound);
        let right_idx = bounds.len();
        bounds.push(right_bound);

        // Link maximum_bound for simple cases where left and right meet at the same max
        // PORT FROM: wagyu/include/mapbox/geometry/wagyu/local_minimum_util.hpp
        // For simple polygons, the left and right bounds of a local minimum share
        // the same maximum point, so we link them together.
        // Note: For complex multi-minima rings, this needs additional linking logic.
        let left_max_top = bounds[left_idx].edges.last().map(|e| e.top);
        let right_max_top = bounds[right_idx].edges.last().map(|e| e.top);
        if left_max_top == right_max_top {
            bounds[left_idx].maximum_bound = Some(right_idx);
            bounds[right_idx].maximum_bound = Some(left_idx);
        }

        // Insert into AEL
        // From C++: insert_lm_left_and_right_bound(left_bound, right_bound, active_bounds, ...)
        ael.insert_pair(left_idx, right_idx, bounds);

        // Find position of left bound in AEL for winding count calculation
        let left_pos = ael.position(left_idx).unwrap_or(0);

        // PORT FROM: C++ insert_lm_left_and_right_bound (lines 340-345)
        // Set winding count for the left bound based on bounds to its left
        winding::set_winding_count(
            left_pos,
            ael.as_slice(),
            bounds,
            subject_fill_type,
            clip_fill_type,
        );

        // Copy winding counts to right bound (they share the same local minimum)
        // From C++: (*rb_abl_itr)->winding_count = (*lb_abl_itr)->winding_count;
        let (left_wc, left_wc2) = {
            let left = &bounds[left_idx];
            (left.winding_count, left.winding_count2)
        };
        bounds[right_idx].winding_count = left_wc;
        bounds[right_idx].winding_count2 = left_wc2;

        // DEBUG: Trace local minimum insertion
        if std::env::var("WAGYU_DEBUG").is_ok() {
            let bot = bounds[left_idx].current_edge().bot;
            let poly_type = bounds[left_idx].poly_type;
            let contributing = winding::is_contributing(
                &bounds[left_idx],
                cliptype,
                subject_fill_type,
                clip_fill_type,
            );
            eprintln!(
                "DEBUG: LM at ({},{}) poly_type={:?} wc={} wc2={} contributing={}",
                bot.x.to_f64().unwrap_or(0.0),
                bot.y.to_f64().unwrap_or(0.0),
                poly_type,
                left_wc,
                left_wc2,
                contributing
            );
            eprintln!("DEBUG: AEL state: {:?}", ael.as_slice());
            for (i, &idx) in ael.as_slice().iter().enumerate() {
                eprintln!(
                    "DEBUG:   [{}] bound {} at x={:.2} poly_type={:?}",
                    i, idx, bounds[idx].current_x, bounds[idx].poly_type
                );
            }
        }

        // Check if this local minimum contributes to output
        // From C++: if (is_contributing(left_bound, cliptype, subject_fill_type, clip_fill_type))
        if winding::is_contributing(
            &bounds[left_idx],
            cliptype,
            subject_fill_type,
            clip_fill_type,
        ) {
            // Create ring at this local minimum point
            // From C++: add_local_minimum_point(lb, rb, active_bounds, lb.current_edge->bot, rings)
            let pt = {
                let bot = bounds[left_idx].current_edge().bot;
                geo_types::Coord { x: bot.x, y: bot.y }
            };
            ring_util::add_local_minimum_point(
                left_idx,
                right_idx,
                bounds,
                ael.as_slice(),
                pt,
                manager,
            );
        }

        // Add edge tops to scanbeam
        // From C++: insert_sorted_scanbeam(scanbeam, (*lb_abl_itr)->current_edge->top.y)
        let left_top = bounds[left_idx].current_edge().top.y;
        let right_top = bounds[right_idx].current_edge().top.y;
        scanbeam.insert(left_top);

        // From C++: Only add right edge top if not horizontal
        // if (!current_edge_is_horizontal<T>(rb_abl_itr))
        if !bounds[right_idx].current_edge().is_horizontal() {
            scanbeam.insert(right_top);
        }

        // Move to next local minimum
        *current_lm_idx += 1;
    }
}

/// Insert horizontal local minima into the active edge list.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/active_bound_list.hpp - insert_horizontal_local_minima_into_ABL
///
/// This function processes local minima that have `minimum_has_horizontal = true`.
/// It must be called BEFORE `process_horizontals` so that horizontal edges from
/// newly inserted bounds can be processed at their correct scanline.
///
/// The key difference from `insert_local_minima_into_abl` is the additional check
/// for `minimum_has_horizontal`. Since local minima are sorted with horizontal
/// ones first at each Y, this function will process all horizontal LMs before
/// the regular `insert_local_minima_into_abl` sees any remaining non-horizontal ones.
///
/// # Arguments
/// Same as `insert_local_minima_into_abl`
#[allow(clippy::too_many_arguments)]
pub fn insert_horizontal_local_minima_into_abl<T: CoordNum + ToPrimitive>(
    top_y: T,
    minima_sorted: &mut LocalMinimumList<T>,
    current_lm_idx: &mut usize,
    bounds: &mut Vec<Bound<T>>,
    ael: &mut ActiveEdgeList,
    manager: &mut RingManager<T>,
    scanbeam: &mut Scanbeam<T>,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) {
    let top_y_f64 = top_y.to_f64().unwrap_or(0.0);

    // Process local minima that are at current Y AND have horizontal first edge
    // From C++: while (current_lm != minima_sorted.end() && top_y == (*current_lm)->y && (*current_lm)->minimum_has_horizontal)
    while *current_lm_idx < minima_sorted.len() {
        let lm = &minima_sorted[*current_lm_idx];
        let lm_y_f64 = lm.y.to_f64().unwrap_or(0.0);

        // Check if this LM is at the current scanline
        if (lm_y_f64 - top_y_f64).abs() > f64::EPSILON {
            break;
        }

        // Check if this LM has a horizontal first edge
        if !lm.minimum_has_horizontal {
            break;
        }

        // Initialize the local minimum
        initialize_lm(&mut minima_sorted[*current_lm_idx]);

        // Take ownership of the bounds from the local minimum
        let lm = &mut minima_sorted[*current_lm_idx];

        // Extract the bounds - we need to move them out
        let left_bound = std::mem::replace(
            &mut lm.left_bound,
            Bound::new_empty(crate::config::PolygonType::Subject, EdgeSide::Left),
        );
        let right_bound = std::mem::replace(
            &mut lm.right_bound,
            Bound::new_empty(crate::config::PolygonType::Subject, EdgeSide::Right),
        );

        // Insert bounds into bounds vector
        let left_idx = bounds.len();
        bounds.push(left_bound);
        let right_idx = bounds.len();
        bounds.push(right_bound);

        // Link maximum_bound for simple cases
        let left_max_top = bounds[left_idx].edges.last().map(|e| e.top);
        let right_max_top = bounds[right_idx].edges.last().map(|e| e.top);
        if left_max_top == right_max_top {
            bounds[left_idx].maximum_bound = Some(right_idx);
            bounds[right_idx].maximum_bound = Some(left_idx);
        }

        // Insert into AEL
        ael.insert_pair(left_idx, right_idx, bounds);

        // Find position of left bound in AEL for winding count calculation
        let left_pos = ael.position(left_idx).unwrap_or(0);

        // Set winding count for the left bound
        winding::set_winding_count(
            left_pos,
            ael.as_slice(),
            bounds,
            subject_fill_type,
            clip_fill_type,
        );

        // Copy winding counts to right bound
        let (left_wc, left_wc2) = {
            let left = &bounds[left_idx];
            (left.winding_count, left.winding_count2)
        };
        bounds[right_idx].winding_count = left_wc;
        bounds[right_idx].winding_count2 = left_wc2;

        // DEBUG: Trace horizontal local minimum insertion
        if std::env::var("WAGYU_DEBUG").is_ok() {
            let bot = bounds[left_idx].current_edge().bot;
            let poly_type = bounds[left_idx].poly_type;
            let contributing = winding::is_contributing(
                &bounds[left_idx],
                cliptype,
                subject_fill_type,
                clip_fill_type,
            );
            eprintln!(
                "DEBUG: Horizontal LM at ({},{}) poly_type={:?} wc={} wc2={} contributing={}",
                bot.x.to_f64().unwrap_or(0.0),
                bot.y.to_f64().unwrap_or(0.0),
                poly_type,
                left_wc,
                left_wc2,
                contributing
            );
        }

        // Check if this local minimum contributes to output
        if winding::is_contributing(
            &bounds[left_idx],
            cliptype,
            subject_fill_type,
            clip_fill_type,
        ) {
            // Create ring at this local minimum point
            let pt = {
                let bot = bounds[left_idx].current_edge().bot;
                geo_types::Coord { x: bot.x, y: bot.y }
            };
            ring_util::add_local_minimum_point(
                left_idx,
                right_idx,
                bounds,
                ael.as_slice(),
                pt,
                manager,
            );
        }

        // Add edge tops to scanbeam
        let left_top = bounds[left_idx].current_edge().top.y;
        let right_top = bounds[right_idx].current_edge().top.y;
        scanbeam.insert(left_top);

        // Only add right edge top if not horizontal
        if !bounds[right_idx].current_edge().is_horizontal() {
            scanbeam.insert(right_top);
        }

        // Move to next local minimum
        *current_lm_idx += 1;
    }
}

/// Update the current_x of a bound for a given Y coordinate.
///
/// Uses the edge's slope (dx) to calculate the x position at y.
pub fn update_bound_current_x<T: CoordNum>(bound: &mut Bound<T>, y: T) {
    bound.current_x = get_current_x(bound.current_edge(), y);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::Edge;
    use crate::config::PolygonType;
    use crate::point::Point;

    // ==================== Test Helpers ====================

    fn make_bound(bot: (f64, f64), top: (f64, f64)) -> Bound<f64> {
        let edge = Edge::new(Point::new(bot.0, bot.1), Point::new(top.0, top.1));
        Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left)
    }

    fn make_local_minimum(
        left_bot: (f64, f64),
        left_top: (f64, f64),
        right_bot: (f64, f64),
        right_top: (f64, f64),
    ) -> LocalMinimum<f64> {
        let left_edge = Edge::new(
            Point::new(left_bot.0, left_bot.1),
            Point::new(left_top.0, left_top.1),
        );
        let right_edge = Edge::new(
            Point::new(right_bot.0, right_bot.1),
            Point::new(right_top.0, right_top.1),
        );

        let left_bound = Bound::new(vec![left_edge], PolygonType::Subject, EdgeSide::Left);
        let right_bound = Bound::new(vec![right_edge], PolygonType::Subject, EdgeSide::Right);

        LocalMinimum::new(left_bound, right_bound, left_bot.1, false)
    }

    // ==================== initialize_lm Tests ====================

    #[test]
    fn initialize_lm_sets_current_x_from_edge_bot() {
        let mut lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 10.0));

        initialize_lm(&mut lm);

        assert!((lm.left_bound.current_x - 0.0).abs() < 1e-10);
        assert!((lm.right_bound.current_x - 0.0).abs() < 1e-10);
    }

    #[test]
    fn initialize_lm_resets_winding_counts() {
        let mut lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 10.0));

        // Set some arbitrary winding counts
        lm.left_bound.winding_count = 5;
        lm.left_bound.winding_count2 = 3;
        lm.right_bound.winding_count = 7;
        lm.right_bound.winding_count2 = 2;

        initialize_lm(&mut lm);

        assert_eq!(lm.left_bound.winding_count, 0);
        assert_eq!(lm.left_bound.winding_count2, 0);
        assert_eq!(lm.right_bound.winding_count, 0);
        assert_eq!(lm.right_bound.winding_count2, 0);
    }

    #[test]
    fn initialize_lm_sets_correct_sides() {
        let mut lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 10.0));

        // Change sides to something else
        lm.left_bound.side = EdgeSide::Right;
        lm.right_bound.side = EdgeSide::Left;

        initialize_lm(&mut lm);

        assert_eq!(lm.left_bound.side, EdgeSide::Left);
        assert_eq!(lm.right_bound.side, EdgeSide::Right);
    }

    #[test]
    fn initialize_lm_clears_ring_references() {
        let mut lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 10.0));

        lm.left_bound.ring = Some(5);
        lm.right_bound.ring = Some(10);

        initialize_lm(&mut lm);

        assert!(lm.left_bound.ring.is_none());
        assert!(lm.right_bound.ring.is_none());
    }

    #[test]
    fn initialize_lm_resets_current_edge_index() {
        let mut lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 10.0));

        lm.left_bound.current_edge_index = 5;
        lm.right_bound.current_edge_index = 3;

        initialize_lm(&mut lm);

        assert_eq!(lm.left_bound.current_edge_index, 0);
        assert_eq!(lm.right_bound.current_edge_index, 0);
    }

    // ==================== next_edge_in_bound Tests ====================

    #[test]
    fn next_edge_in_bound_advances_and_adds_to_scanbeam() {
        let edges = vec![
            Edge::new(Point::new(0.0_f64, 0.0), Point::new(5.0, 10.0)),
            Edge::new(Point::new(5.0_f64, 10.0), Point::new(10.0, 20.0)),
        ];
        let mut bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);
        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        let result = next_edge_in_bound(&mut bound, &mut scanbeam);

        assert!(result);
        assert_eq!(bound.current_edge_index, 1);
        // The new edge's top Y (20.0) should be in the scanbeam
        assert_eq!(scanbeam.peek(), Some(&20.0));
    }

    #[test]
    fn next_edge_in_bound_returns_false_at_last_edge() {
        let mut bound = make_bound((0.0, 0.0), (5.0, 10.0));
        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        let result = next_edge_in_bound(&mut bound, &mut scanbeam);

        assert!(!result);
    }

    // ==================== insert_local_minimum_into_ael Tests ====================

    #[test]
    fn insert_local_minimum_adds_bounds_to_vector() {
        let lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 10.0));
        let mut bounds: Vec<Bound<f64>> = Vec::new();
        let mut ael = ActiveEdgeList::new();
        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        let (left_idx, right_idx) =
            insert_local_minimum_into_ael(lm, &mut bounds, &mut ael, &mut scanbeam);

        assert_eq!(left_idx, 0);
        assert_eq!(right_idx, 1);
        assert_eq!(bounds.len(), 2);
    }

    #[test]
    fn insert_local_minimum_adds_to_ael() {
        let lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 10.0));
        let mut bounds: Vec<Bound<f64>> = Vec::new();
        let mut ael = ActiveEdgeList::new();
        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        insert_local_minimum_into_ael(lm, &mut bounds, &mut ael, &mut scanbeam);

        assert_eq!(ael.len(), 2);
    }

    #[test]
    fn insert_local_minimum_adds_tops_to_scanbeam() {
        let lm = make_local_minimum((0.0, 0.0), (-5.0, 10.0), (0.0, 0.0), (5.0, 15.0));
        let mut bounds: Vec<Bound<f64>> = Vec::new();
        let mut ael = ActiveEdgeList::new();
        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();

        insert_local_minimum_into_ael(lm, &mut bounds, &mut ael, &mut scanbeam);

        // Scanbeam should contain 10.0 and 15.0
        assert_eq!(scanbeam.len(), 2);
        assert_eq!(scanbeam.pop(), Some(15.0));
        assert_eq!(scanbeam.pop(), Some(10.0));
    }

    // ==================== at_edge_top Tests ====================

    #[test]
    fn at_edge_top_returns_true_at_top_y() {
        let bound = make_bound((0.0, 0.0), (5.0, 10.0));
        assert!(at_edge_top(&bound, 10.0));
    }

    #[test]
    fn at_edge_top_returns_false_below_top_y() {
        let bound = make_bound((0.0, 0.0), (5.0, 10.0));
        assert!(!at_edge_top(&bound, 5.0));
    }

    // ==================== update_bound_current_x Tests ====================

    #[test]
    fn update_bound_current_x_at_bottom() {
        let mut bound = make_bound((0.0, 0.0), (10.0, 10.0));
        update_bound_current_x(&mut bound, 0.0);
        assert!((bound.current_x - 0.0).abs() < 1e-10);
    }

    #[test]
    fn update_bound_current_x_at_midpoint() {
        let mut bound = make_bound((0.0, 0.0), (10.0, 10.0));
        update_bound_current_x(&mut bound, 5.0);
        assert!((bound.current_x - 5.0).abs() < 1e-10);
    }

    #[test]
    fn update_bound_current_x_at_top() {
        let mut bound = make_bound((0.0, 0.0), (10.0, 10.0));
        update_bound_current_x(&mut bound, 10.0);
        assert!((bound.current_x - 10.0).abs() < 1e-10);
    }
}
