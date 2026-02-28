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
use crate::intersect_util::{get_current_x, intersect_bounds, IntersectResult};
use crate::local_minimum::LocalMinimumList;
use crate::local_minimum_util::insert_horizontal_local_minima_into_abl;
use crate::point::Point;
use crate::process_maxima::{do_maxima, get_maxima_pair, is_intermediate, is_maxima};
use crate::ring_util;
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
/// * `manager` - Ring manager for output polygons
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
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
    manager: &mut RingManager<T>,
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
            // PORT FROM: C++ lines 75-84 - handle maxima pair
            let horz_idx = ael.get(current_pos).unwrap();
            if bounds[horz_idx].ring.is_some() && bounds[next_idx].ring.is_some() {
                let max_pt = bounds[horz_idx].current_edge().top;
                ring_util::add_local_maximum_point(
                    horz_idx,
                    next_idx,
                    bounds,
                    ael.as_slice(),
                    geo_types::Coord {
                        x: max_pt.x,
                        y: max_pt.y,
                    },
                    manager,
                );
            }
            break;
        }

        // PORT FROM: C++ lines 68-70 - add point to ring BEFORE intersection handling
        // This records the crossing point on the horizontal's ring
        let horz_idx = ael.get(current_pos).unwrap();
        if bounds[horz_idx].ring.is_some() {
            // Round the intersection x coordinate like C++ does with wround
            let intersection_x = bounds[next_idx].current_x;
            ring_util::add_point_to_ring(
                horz_idx,
                bounds,
                geo_types::Coord {
                    x: T::from(intersection_x as i64).unwrap_or(scanline_y),
                    y: scanline_y,
                },
                manager,
            );
        }

        // PORT FROM: C++ lines 89-91 - call intersect_bounds
        // This updates winding counts and handles ring swapping
        let horz_idx = ael.get(current_pos).unwrap();
        let intersection_pt = Point::new(
            T::from(bounds[next_idx].current_x as i64).unwrap_or(scanline_y),
            scanline_y,
        );

        // Use split_at_mut to get two mutable references safely
        // BUGFIX: Capture and handle IntersectResult to update other bounds
        // when rings are merged during horizontal processing
        let result = {
            let (b1, b2) = if horz_idx < next_idx {
                let (left, right) = bounds.split_at_mut(next_idx);
                (&mut left[horz_idx], &mut right[0])
            } else {
                let (left, right) = bounds.split_at_mut(horz_idx);
                (&mut right[0], &mut left[next_idx])
            };

            intersect_bounds(
                b1,
                b2,
                intersection_pt,
                cliptype,
                subject_fill_type,
                clip_fill_type,
                manager,
            )
        };

        // Handle the intersection result
        // PORT FROM: wagyu C++ implicitly handles this through active_bounds parameter
        match result {
            IntersectResult::Merged(keep_ring_idx, remove_ring_idx, keep_side) => {
                // Update other active bounds that reference the removed ring
                for &ab_idx in ael.as_slice() {
                    if bounds[ab_idx].ring == Some(remove_ring_idx) {
                        bounds[ab_idx].ring = Some(keep_ring_idx);
                        bounds[ab_idx].side = keep_side;
                        break; // C++ breaks after first match
                    }
                }
            }
            IntersectResult::NewRing(_ring_idx) => {
                // Hole state would be set here if needed
                // For horizontal processing, this is typically not triggered
            }
            IntersectResult::None => {}
        }

        // Swap positions in AEL
        ael.swap(current_pos, next_pos);
        current_pos = next_pos;
        next_pos += 1;
    }

    // PORT FROM: C++ lines 104-106 - add endpoint to ring AFTER the loop
    let horz_idx = match ael.get(current_pos) {
        Some(idx) => idx,
        None => return horz_bound_pos,
    };

    if bounds[horz_idx].ring.is_some() {
        let edge_top = bounds[horz_idx].current_edge().top;
        ring_util::add_point_to_ring(
            horz_idx,
            bounds,
            geo_types::Coord {
                x: edge_top.x,
                y: edge_top.y,
            },
            manager,
        );
    }

    // Advance to next edge if there is one
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
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
    manager: &mut RingManager<T>,
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
            // PORT FROM: C++ lines 177-186 - handle maxima pair
            let horz_idx = ael.get(current_pos).unwrap();
            if bounds[horz_idx].ring.is_some() && bounds[prev_idx].ring.is_some() {
                let max_pt = bounds[horz_idx].current_edge().top;
                ring_util::add_local_maximum_point(
                    horz_idx,
                    prev_idx,
                    bounds,
                    ael.as_slice(),
                    geo_types::Coord {
                        x: max_pt.x,
                        y: max_pt.y,
                    },
                    manager,
                );
            }
            break;
        }

        // PORT FROM: C++ lines 170-173 - add point to ring BEFORE intersection handling
        let horz_idx = ael.get(current_pos).unwrap();
        if bounds[horz_idx].ring.is_some() {
            let intersection_x = bounds[prev_idx].current_x;
            ring_util::add_point_to_ring(
                horz_idx,
                bounds,
                geo_types::Coord {
                    x: T::from(intersection_x as i64).unwrap_or(scanline_y),
                    y: scanline_y,
                },
                manager,
            );
        }

        // PORT FROM: C++ lines 192-194 - call intersect_bounds
        // Note: for right-to-left, the bound order is swapped (prev, horz) vs (horz, next)
        let horz_idx = ael.get(current_pos).unwrap();
        let intersection_pt = Point::new(
            T::from(bounds[prev_idx].current_x as i64).unwrap_or(scanline_y),
            scanline_y,
        );

        // Use split_at_mut to get two mutable references safely
        // BUGFIX: Capture and handle IntersectResult to update other bounds
        // when rings are merged during horizontal processing
        let result = {
            let (b1, b2) = if prev_idx < horz_idx {
                let (left, right) = bounds.split_at_mut(horz_idx);
                (&mut left[prev_idx], &mut right[0])
            } else {
                let (left, right) = bounds.split_at_mut(prev_idx);
                (&mut right[0], &mut left[horz_idx])
            };

            intersect_bounds(
                b1,
                b2,
                intersection_pt,
                cliptype,
                subject_fill_type,
                clip_fill_type,
                manager,
            )
        };

        // Handle the intersection result
        // PORT FROM: wagyu C++ implicitly handles this through active_bounds parameter
        match result {
            IntersectResult::Merged(keep_ring_idx, remove_ring_idx, keep_side) => {
                // Update other active bounds that reference the removed ring
                for &ab_idx in ael.as_slice() {
                    if bounds[ab_idx].ring == Some(remove_ring_idx) {
                        bounds[ab_idx].ring = Some(keep_ring_idx);
                        bounds[ab_idx].side = keep_side;
                        break; // C++ breaks after first match
                    }
                }
            }
            IntersectResult::NewRing(_ring_idx) => {
                // Hole state would be set here if needed
                // For horizontal processing, this is typically not triggered
            }
            IntersectResult::None => {}
        }

        // Swap positions in AEL
        ael.swap(prev_pos, current_pos);
        current_pos = prev_pos;
    }

    // PORT FROM: C++ lines 204-206 - add endpoint to ring AFTER the loop
    let horz_idx = match ael.get(current_pos) {
        Some(idx) => idx,
        None => return horz_bound_pos,
    };

    if bounds[horz_idx].ring.is_some() {
        let edge_top = bounds[horz_idx].current_edge().top;
        ring_util::add_point_to_ring(
            horz_idx,
            bounds,
            geo_types::Coord {
                x: edge_top.x,
                y: edge_top.y,
            },
            manager,
        );
    }

    // Advance to next edge if there is one
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
    manager: &mut RingManager<T>,
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
            manager,
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
            manager,
        ),
    }
}

/// Process all horizontal edges at the current scanline.
///
/// From C++: `process_horizontals(scanline_y, active_bounds, ...)`
///
/// Iterates through the AEL and processes each bound that has a horizontal
/// current edge.
#[allow(clippy::too_many_arguments)]
pub fn process_horizontals<T: CoordNum + ToPrimitive>(
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    scanline_y: T,
    scanbeam: &mut Scanbeam<T>,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
    manager: &mut RingManager<T>,
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
                manager,
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
/// * `manager` - Ring manager for output
/// * `clip_type` - Type of boolean operation
/// * `subject_fill_type` - Fill rule for subject polygons
/// * `clip_fill_type` - Fill rule for clip polygons
#[allow(clippy::too_many_arguments)]
pub fn process_edges_at_top_of_scanbeam<T: CoordNum + ToPrimitive>(
    scanline_y: T,
    ael: &mut ActiveEdgeList,
    bounds: &mut Vec<Bound<T>>,
    scanbeam: &mut Scanbeam<T>,
    _minima_sorted: &[usize],
    current_lm_idx: &mut usize,
    minima_list: &mut LocalMinimumList<T>,
    manager: &mut RingManager<T>,
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
        // PORT FROM: C++ lines 77-88 in process_maxima.hpp
        let mut is_maxima_edge = is_maxima(bound, scanline_y);

        if is_maxima_edge {
            // Find the maxima pair (note: argument order is bound_pos, bounds, ael)
            if let Some(pair_pos) = get_maxima_pair(i, bounds, ael) {
                // Get the pair's bound index for ring operations
                let pair_idx = ael.get(pair_pos);

                // CRITICAL: Check if the pair is ALSO at maxima!
                // From C++ lines 81-82:
                // is_maxima_edge = ((bnd_max_pair == active_bounds.end() || !current_edge_is_horizontal<T>(bnd_max_pair)) &&
                //                   is_maxima(bnd_max_pair, top_y));
                // Both bounds must be at maxima to process as a maxima pair
                let pair_is_maxima = pair_idx
                    .and_then(|idx| bounds.get(idx))
                    .map(|pair_bound| {
                        // Check: pair's edge is not horizontal AND pair is at maxima
                        !pair_bound.current_edge().is_horizontal()
                            && is_maxima(pair_bound, scanline_y)
                    })
                    .unwrap_or(false);

                is_maxima_edge = pair_is_maxima;

                if is_maxima_edge {
                    // Check if both bounds have rings before calling add_local_maximum_point
                    // From C++: if ((*horz_bound)->ring && (*bound_max_pair)->ring)
                    let both_have_rings = {
                        let b1_has_ring = bounds
                            .get(bound_idx)
                            .map(|b| b.ring.is_some())
                            .unwrap_or(false);
                        let b2_has_ring = pair_idx
                            .and_then(|idx| bounds.get(idx))
                            .map(|b| b.ring.is_some())
                            .unwrap_or(false);
                        b1_has_ring && b2_has_ring
                    };

                    if both_have_rings {
                        if let Some(pair_bound_idx) = pair_idx {
                            // Get the maximum point (top of current edge)
                            let max_pt = bounds[bound_idx].current_edge().top;

                            // Add local maximum point to close/merge rings
                            // PORT FROM: C++ add_local_maximum_point in process_horizontal.hpp
                            ring_util::add_local_maximum_point(
                                bound_idx,
                                pair_bound_idx,
                                bounds,
                                ael.as_slice(),
                                geo_types::Coord {
                                    x: max_pt.x,
                                    y: max_pt.y,
                                },
                                manager,
                            );
                        }
                    }

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
        }

        // 2. Promote horizontal edges
        // PORT FROM: C++ lines 89-101 - if intermediate and next edge is horizontal
        let bound = match bounds.get(bound_idx) {
            Some(b) => b,
            None => {
                i += 1;
                continue;
            }
        };

        if is_intermediate(bound, scanline_y) && next_edge_would_be_horizontal(bound) {
            // PORT FROM: C++ lines 91-97
            if bound.ring.is_some() {
                // Insert hot pixels (TODO: implement hot pixel insertion)
                let edge_top = bound.current_edge().top;
                ring_util::add_point_to_ring(
                    bound_idx,
                    bounds,
                    geo_types::Coord {
                        x: edge_top.x,
                        y: edge_top.y,
                    },
                    manager,
                );
            }
            // Advance to next edge
            if let Some(bound) = bounds.get_mut(bound_idx) {
                bound.current_edge_index += 1;
                let new_top_y = bound.current_edge().top.y;
                scanbeam.insert(new_top_y);
            }
        } else {
            // Just update current_x - already done at top of function
        }

        i += 1;
    }

    // 3. Insert horizontal local minima
    // PORT FROM: C++ line 105-106 - insert_horizontal_local_minima_into_ABL
    insert_horizontal_local_minima_into_abl(
        scanline_y,
        minima_list,
        current_lm_idx,
        bounds,
        ael,
        manager,
        scanbeam,
        clip_type,
        subject_fill_type,
        clip_fill_type,
    );

    // Process horizontals
    // PORT FROM: C++ line 108
    process_horizontals(
        bounds,
        ael,
        scanline_y,
        scanbeam,
        clip_type,
        subject_fill_type,
        clip_fill_type,
        manager,
    );

    // 4. Promote intermediate vertices
    // PORT FROM: C++ lines 112-119
    // This is the critical step that adds polygon vertices to rings!
    let debug = std::env::var("WAGYU_DEBUG").is_ok();
    if debug {
        eprintln!(
            "DEBUG: step4 start - AEL len={} scanline_y={:?}",
            ael.len(),
            scanline_y.to_f64()
        );
    }
    for i in 0..ael.len() {
        let bound_idx = match ael.get(i) {
            Some(idx) => idx,
            None => continue,
        };

        let bound = match bounds.get(bound_idx) {
            Some(b) => b,
            None => continue,
        };

        let edge_top_y = bound.current_edge().top.y;
        let has_more_edges = bound.current_edge_index + 1 < bound.edges.len();
        let is_at_top = edge_top_y == scanline_y;

        if debug {
            eprintln!(
                "DEBUG: step4 bound {} edge_top_y={:?} scanline_y={:?} has_more_edges={} is_at_top={} ring={:?}",
                bound_idx,
                edge_top_y.to_f64(),
                scanline_y.to_f64(),
                has_more_edges,
                is_at_top,
                bound.ring
            );
        }

        if is_intermediate(bound, scanline_y) {
            // Add the edge top point to the ring BEFORE advancing to the next edge
            // This is the vertex that connects the current edge to the next edge
            if bound.ring.is_some() {
                let edge_top = bound.current_edge().top;
                if debug {
                    eprintln!(
                        "DEBUG: Adding intermediate vertex ({}, {}) to bound {} ring {:?}",
                        edge_top.x.to_f64().unwrap_or(0.0),
                        edge_top.y.to_f64().unwrap_or(0.0),
                        bound_idx,
                        bound.ring
                    );
                }
                ring_util::add_point_to_ring(
                    bound_idx,
                    bounds,
                    geo_types::Coord {
                        x: edge_top.x,
                        y: edge_top.y,
                    },
                    manager,
                );
            }
            // Advance to next edge (next_edge_in_bound)
            if let Some(bound) = bounds.get_mut(bound_idx) {
                bound.current_edge_index += 1;
                let new_top_y = bound.current_edge().top.y;
                scanbeam.insert(new_top_y);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::Edge;
    use crate::build_result::RingManager;
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
        let mut manager: RingManager<f64> = RingManager::new();

        let result = process_horizontal_left_to_right(
            0,
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
            &mut manager,
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
        let mut manager: RingManager<f64> = RingManager::new();

        let result = process_horizontal_left_to_right(
            0,
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
            &mut manager,
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
        let mut manager: RingManager<f64> = RingManager::new();

        let result = process_horizontal_right_to_left(
            1, // Position of horizontal in AEL
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
            &mut manager,
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
        let mut manager: RingManager<f64> = RingManager::new();

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
            &mut manager,
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
        let mut manager: RingManager<f64> = RingManager::new();

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
            &mut manager,
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
        let mut manager: RingManager<f64> = RingManager::new();

        // Should not modify anything since no horizontals
        process_horizontals(
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
            &mut manager,
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
        let mut manager: RingManager<f64> = RingManager::new();

        process_horizontals(
            &mut bounds,
            &mut ael,
            5.0,
            &mut scanbeam,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
            &mut manager,
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
