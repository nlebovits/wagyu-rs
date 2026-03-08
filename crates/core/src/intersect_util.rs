//! Intersection utilities for the Vatti clipping algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp
//!
//! This module provides functions for calculating and processing edge
//! intersections during the sweep line algorithm. Key functionality includes:
//! - Edge intersection point calculation
//! - Building intersection lists from active bounds
//! - Processing intersection events
//! - Winding count updates at intersections

use geo_types::CoordNum;
use num_traits::ToPrimitive;

use crate::active_edge_list::ActiveEdgeList;
use crate::bound::{Bound, Edge};
use crate::build_edges::slopes_equal_4pt;
use crate::build_result::RingManager;
use crate::config::{EdgeSide, FillType, PolygonType};
use crate::intersect::{IntersectList, IntersectNode};
use crate::point::Point;
use crate::Operation;

/// Result of processing a bound intersection.
///
/// Provides information needed by the caller to update state after intersection.
#[derive(Debug, Clone, Copy)]
pub enum IntersectResult {
    /// No special action needed
    None,
    /// Rings were merged: (kept_ring_idx, removed_ring_idx, keep_side)
    Merged(usize, usize, EdgeSide),
    /// A new ring was created at this intersection
    NewRing(usize),
}

// ============================================================================
// Helper functions
// ============================================================================

/// Round a floating-point point to integer coordinates.
///
/// From C++: `round_point<T>(pt)` - rounds using `round_towards_max`
pub fn round_point<T>(pt: Point<f64>) -> Point<T>
where
    T: CoordNum + num_traits::NumCast,
{
    let x = T::from(pt.x.round()).unwrap_or_else(|| T::zero());
    let y = T::from(pt.y.round()).unwrap_or_else(|| T::zero());
    Point::new(x, y)
}

/// Swap the ring assignments between two bounds.
///
/// From C++: `swap_rings(b1, b2)`
pub fn swap_rings<T: CoordNum>(b1: &mut Bound<T>, b2: &mut Bound<T>) {
    std::mem::swap(&mut b1.ring, &mut b2.ring);
}

/// Swap the edge sides between two bounds.
///
/// From C++: `swap_sides(b1, b2)`
pub fn swap_sides<T: CoordNum>(b1: &mut Bound<T>, b2: &mut Bound<T>) {
    std::mem::swap(&mut b1.side, &mut b2.side);
}

/// Check if two floating-point values are approximately equal.
#[allow(dead_code)]
fn values_are_equal(x: f64, y: f64) -> bool {
    // Using a simple epsilon comparison (matching wagyu's almost_equal behavior)
    (x - y).abs() < 1e-10
}

// ============================================================================
// Edge intersection calculation
// ============================================================================

/// Calculate the intersection point between two edges.
///
/// From C++: `get_edge_intersection(e1, e2, pt)`
///
/// Uses parametric line-line intersection formula. Returns `Some(point)` if
/// edges intersect within their bounds, `None` otherwise.
///
/// # Arguments
/// * `e1` - First edge
/// * `e2` - Second edge
///
/// # Returns
/// The intersection point as `Point<f64>`, or `None` if edges don't intersect.
pub fn get_edge_intersection<T: CoordNum>(e1: &Edge<T>, e2: &Edge<T>) -> Option<Point<f64>> {
    let p0_x = e1.bot.x.to_f64()?;
    let p0_y = e1.bot.y.to_f64()?;
    let p1_x = e1.top.x.to_f64()?;
    let p1_y = e1.top.y.to_f64()?;
    let p2_x = e2.bot.x.to_f64()?;
    let p2_y = e2.bot.y.to_f64()?;
    let p3_x = e2.top.x.to_f64()?;
    let p3_y = e2.top.y.to_f64()?;

    let s1_x = p1_x - p0_x;
    let s1_y = p1_y - p0_y;
    let s2_x = p3_x - p2_x;
    let s2_y = p3_y - p2_y;

    let denom = -s2_x * s1_y + s1_x * s2_y;

    // Parallel lines check
    if denom.abs() < 1e-10 {
        return None;
    }

    let s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / denom;
    let t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / denom;

    // Check if intersection is within both line segments
    if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
        let x = p0_x + t * s1_x;
        let y = p0_y + t * s1_y;
        Some(Point::new(x, y))
    } else {
        None
    }
}

/// Get the x coordinate of an edge at a given y coordinate.
///
/// From C++: `get_current_x(edge, y)` in edge.hpp:81-87
///
/// IMPORTANT: When y == edge.top.y, we return edge.top.x directly.
/// This is critical for horizontal edges where bot.x != top.x, as the
/// linear interpolation formula would incorrectly return bot.x.
pub fn get_current_x<T: CoordNum>(edge: &Edge<T>, y: T) -> f64 {
    // C++ special case: when at the top of the edge, return top.x directly.
    // This is critical for horizontal edges where bot.x != top.x.
    if y == edge.top.y {
        return edge.top.x.to_f64().unwrap_or(0.0);
    }

    if edge.is_horizontal() {
        edge.bot.x.to_f64().unwrap_or(0.0)
    } else {
        let bot_x = edge.bot.x.to_f64().unwrap_or(0.0);
        let bot_y = edge.bot.y.to_f64().unwrap_or(0.0);
        let y_f64 = y.to_f64().unwrap_or(0.0);
        bot_x + edge.dx * (y_f64 - bot_y)
    }
}

// ============================================================================
// Intersection list building
// ============================================================================

/// Check if bounds need to be swapped based on their current_x values.
///
/// From C++: `intersection_compare` functor
/// Returns true if b1 should come before b2 (no swap needed).
fn intersection_compare<T: CoordNum + ToPrimitive>(b1: &Bound<T>, b2: &Bound<T>) -> bool {
    // Returns true if NOT (b1.current_x > b2.current_x AND slopes not equal)
    // i.e., returns false if b1.current_x > b2.current_x and edges aren't parallel
    b1.current_x <= b2.current_x || slopes_equal_edges(b1.current_edge(), b2.current_edge())
}

/// Check if two edges have equal slopes.
fn slopes_equal_edges<T: CoordNum + ToPrimitive>(e1: &Edge<T>, e2: &Edge<T>) -> bool {
    slopes_equal_4pt(e1.bot, e1.top, e2.bot, e2.top)
}

/// Build an intersection list by detecting crossings in the active bounds.
///
/// From C++: `build_intersect_list(active_bounds, intersects)`
///
/// This uses a bubble sort approach: when two adjacent bounds are out of order
/// (based on current_x), they must have crossed and we record the intersection.
///
/// # Arguments
/// * `ael` - The active edge list
/// * `bounds` - All bounds
/// * `intersects` - List to populate with intersection nodes
pub fn build_intersect_list<T>(
    ael: &ActiveEdgeList,
    bounds: &[Bound<T>],
    intersects: &mut IntersectList<T>,
) where
    T: CoordNum + ToPrimitive + num_traits::NumCast,
{
    let len = ael.len();
    if len < 2 {
        return;
    }

    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp
    // DIVERGENCE FROM WAGYU: The C++ version swaps during build_intersect_list
    // and then restores the original order before process_intersect_list.
    // We instead detect intersections without swapping, and let process_intersect_list
    // do all the swapping. This produces the same final result.

    // Create a copy of bound indices for simulation
    let mut simulated_order: Vec<usize> = ael.iter().copied().collect();

    // Bubble sort simulation with intersection detection
    let mut swapped = true;
    const MAX_BUBBLE_SORT_ITERATIONS: usize = 100_000;
    let mut bubble_iteration = 0;
    while swapped {
        bubble_iteration += 1;
        if bubble_iteration > MAX_BUBBLE_SORT_ITERATIONS {
            panic!(
                "INFINITE LOOP DETECTED in build_intersect_list bubble sort at iteration {}, simulated_order.len()={}",
                bubble_iteration,
                simulated_order.len()
            );
        }
        swapped = false;
        for i in 0..simulated_order.len() - 1 {
            let idx1 = simulated_order[i];
            let idx2 = simulated_order[i + 1];

            let b1 = &bounds[idx1];
            let b2 = &bounds[idx2];

            // Check if bounds are out of order
            if !intersection_compare(b1, b2) {
                // Bounds crossed - record intersection
                if let Some(pt) = get_edge_intersection(b1.current_edge(), b2.current_edge()) {
                    let rounded: Point<T> = round_point(pt);
                    crate::debug::log_intersect(
                        idx1,
                        idx2,
                        (
                            rounded.x.to_f64().unwrap_or(0.0),
                            rounded.y.to_f64().unwrap_or(0.0),
                        ),
                    );
                    // PORT FROM: C++ intersect_list_sorter uses winding_count2 sum for tie-breaking
                    let winding_count2_sum = b1.winding_count2 + b2.winding_count2;
                    intersects.push(IntersectNode::new(rounded, idx1, idx2, winding_count2_sum));
                }
                // Swap in simulated order (not in actual AEL)
                simulated_order.swap(i, i + 1);
                swapped = true;
            }
        }
    }
}

/// Update the current_x values for all bounds in the active edge list.
///
/// From C++: `update_current_x(active_bounds, top_y)`
pub fn update_current_x<T: CoordNum>(ael: &ActiveEdgeList, bounds: &mut [Bound<T>], top_y: T) {
    for &idx in ael.iter() {
        if let Some(bound) = bounds.get_mut(idx) {
            bound.current_x = get_current_x(bound.current_edge(), top_y);
        }
    }
}

// ============================================================================
// Winding count helpers
// ============================================================================

/// Check if a bound uses even-odd fill type.
///
/// From C++: `is_even_odd_fill_type(bound, subject_fill_type, clip_fill_type)`
pub fn is_even_odd_fill_type<T: CoordNum>(
    bound: &Bound<T>,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> bool {
    match bound.poly_type {
        PolygonType::Subject => subject_fill_type == FillType::EvenOdd,
        PolygonType::Clip => clip_fill_type == FillType::EvenOdd,
    }
}

/// Check if the alternate polygon type uses even-odd fill.
///
/// From C++: `is_even_odd_alt_fill_type(bound, subject_fill_type, clip_fill_type)`
///
/// This is used when updating winding_count2, which tracks the winding
/// relative to the OTHER polygon type.
pub fn is_even_odd_fill_type_alt<T: CoordNum>(
    bound: &Bound<T>,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> bool {
    match bound.poly_type {
        PolygonType::Subject => clip_fill_type == FillType::EvenOdd,
        PolygonType::Clip => subject_fill_type == FillType::EvenOdd,
    }
}

/// Get the effective winding count for a bound based on fill type.
fn get_winding_count(winding: i32, fill_type: FillType) -> i32 {
    match fill_type {
        FillType::Positive => winding,
        FillType::Negative => -winding,
        FillType::EvenOdd | FillType::NonZero => winding.abs(),
    }
}

/// Get the fill type for a bound.
fn get_fill_type<T: CoordNum>(
    bound: &Bound<T>,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> FillType {
    match bound.poly_type {
        PolygonType::Subject => subject_fill_type,
        PolygonType::Clip => clip_fill_type,
    }
}

/// Get the "other" fill type for a bound (the fill type of the other polygon type).
fn get_fill_type2<T: CoordNum>(
    bound: &Bound<T>,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> FillType {
    match bound.poly_type {
        PolygonType::Subject => clip_fill_type,
        PolygonType::Clip => subject_fill_type,
    }
}

// ============================================================================
// Intersection processing
// ============================================================================

/// Update winding counts when two bounds intersect.
///
/// From C++: Part of `intersect_bounds` - the winding count update logic.
///
/// Assumes b1 will be to the right of b2 ABOVE the intersection.
pub fn update_winding_counts<T: CoordNum>(
    b1: &mut Bound<T>,
    b2: &mut Bound<T>,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) {
    if b1.poly_type == b2.poly_type {
        // Same polygon type
        if is_even_odd_fill_type(b1, subject_fill_type, clip_fill_type) {
            std::mem::swap(&mut b1.winding_count, &mut b2.winding_count);
        } else {
            // Non-zero fill type
            // PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp lines 91-102
            // Use the stored winding_delta (invariant property of the bound direction)
            // NOT derived from side, which can change via swap_sides().
            let b1_delta = b1.winding_delta;
            let b2_delta = b2.winding_delta;

            if b1.winding_count + b2_delta == 0 {
                b1.winding_count = -b1.winding_count;
            } else {
                b1.winding_count += b2_delta;
            }
            if b2.winding_count - b1_delta == 0 {
                b2.winding_count = -b2.winding_count;
            } else {
                b2.winding_count -= b1_delta;
            }
        }
    } else {
        // Different polygon types
        // PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp lines 103-116
        // Use stored winding_delta instead of deriving from side
        if !is_even_odd_fill_type(b2, subject_fill_type, clip_fill_type) {
            b1.winding_count2 += b2.winding_delta;
        } else {
            b1.winding_count2 = if b1.winding_count2 == 0 { 1 } else { 0 };
        }
        if !is_even_odd_fill_type(b1, subject_fill_type, clip_fill_type) {
            b2.winding_count2 -= b1.winding_delta;
        } else {
            b2.winding_count2 = if b2.winding_count2 == 0 { 1 } else { 0 };
        }
    }
}

/// Add a point to a bound's ring at an intersection.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_point
///
/// This adds the intersection point to the bound's ring if one exists.
/// The bound's side determines where the point is inserted:
/// - Left side: insert at front (prepend)
/// - Right side: insert at back (append)
fn add_point<T: CoordNum>(bound: &mut Bound<T>, pt: Point<T>, manager: &mut RingManager<T>) {
    if let Some(ring_idx) = bound.ring {
        // Check if this point differs from last_point
        if pt.x != bound.last_point.x || pt.y != bound.last_point.y {
            let coord = geo_types::Coord { x: pt.x, y: pt.y };
            let to_front = bound.side == EdgeSide::Left;

            if let Some(ring) = manager.get_mut(ring_idx) {
                // Check for duplicate at insertion position
                if to_front {
                    if ring.first() == Some(&coord) {
                        return;
                    }
                    ring.insert_at_front(coord);
                } else {
                    if ring.points().last() == Some(&coord) {
                        return;
                    }
                    ring.push_point(coord);
                }
            }
            bound.last_point = pt;
        }
    }
}

/// Add a local maximum point where two bounds meet at an intersection.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_local_maximum_point
///
/// When two contributing bounds intersect in a way that closes/merges rings,
/// this handles the ring operations.
///
/// # Returns
/// If rings were merged, returns `Some((kept_ring_idx, removed_ring_idx, keep_side))`
/// so the caller can update other bounds that reference the removed ring.
fn add_local_maximum_point_at_intersection<T: CoordNum + Copy>(
    b1: &mut Bound<T>,
    b2: &mut Bound<T>,
    pt: Point<T>,
    manager: &mut RingManager<T>,
) -> Option<(usize, usize, EdgeSide)> {
    // Add point to b1's ring
    add_point(b1, pt, manager);

    // Check if both bounds share the same ring
    if b1.ring == b2.ring {
        // Close the ring - both bounds reference the same ring
        b1.ring = None;
        b2.ring = None;
        None
    } else if let (Some(ring1_idx), Some(ring2_idx)) = (b1.ring, b2.ring) {
        // Different rings - need to merge them
        // PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - append_ring
        let merge_info = merge_rings_at_intersection(b1, b2, ring1_idx, ring2_idx, pt, manager);
        Some(merge_info)
    } else {
        None
    }
}

/// Merge two rings when bounds meet at an intersection.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - append_ring (simplified)
///
/// This is a simplified version that works with just two bounds at an intersection.
/// Returns `(kept_ring_idx, removed_ring_idx, keep_side)` so the caller can
/// update other bounds that reference the removed ring.
fn merge_rings_at_intersection<T: CoordNum + Copy>(
    b1: &mut Bound<T>,
    b2: &mut Bound<T>,
    ring1_idx: usize,
    ring2_idx: usize,
    pt: Point<T>,
    manager: &mut RingManager<T>,
) -> (usize, usize, EdgeSide) {
    // DEBUG: Log which bounds/rings are being merged
    if crate::debug::debug_enabled() {
        eprintln!(
            "[MERGE_RINGS] b1.ring={:?} b2.ring={:?}, ring1_idx={} ring2_idx={}",
            b1.ring, b2.ring, ring1_idx, ring2_idx
        );
    }
    // Determine which ring to keep (lower index = created first)
    // C++ uses get_lower_most_ring based on bottom point, but for simplicity
    // we use ring index ordering (matches C++ append_ring fallback behavior)
    let (keep_idx, remove_idx, keep_side, remove_side) = if ring1_idx < ring2_idx {
        (ring1_idx, ring2_idx, b1.side, b2.side)
    } else {
        (ring2_idx, ring1_idx, b2.side, b1.side)
    };

    // Get points from the ring to remove
    let remove_points: Vec<geo_types::Coord<T>> = match manager.get(remove_idx) {
        Some(r) => r.points().to_vec(),
        None => Vec::new(),
    };

    // Merge points based on sides
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp lines 544-576
    if let Some(keep_ring) = manager.get_mut(keep_idx) {
        let keep_points = keep_ring.points_mut();

        // DIVERGENCE FROM WAGYU: The C++ uses a circular linked list where nodes
        // are reconnected without duplication. In Rust with Vecs, we must check
        // for duplicate points at the join and skip them to avoid creating
        // spurious duplicates that would be incorrectly removed by topology correction.
        match (keep_side, remove_side) {
            (EdgeSide::Left, EdgeSide::Left) => {
                // C++: reverse remove, prepend to keep (z y x a b c)
                let mut reversed: Vec<_> = remove_points.into_iter().rev().collect();
                // Skip duplicate at join point (last of reversed == first of keep)
                if let (Some(rev_last), Some(keep_first)) = (reversed.last(), keep_points.first()) {
                    if rev_last == keep_first {
                        reversed.pop();
                    }
                }
                reversed.append(keep_points);
                *keep_points = reversed;
            }
            (EdgeSide::Left, EdgeSide::Right) => {
                // C++: prepend remove to keep (x y z a b c)
                let mut new_points = remove_points;
                // Skip duplicate at join point (last of remove == first of keep)
                if let (Some(rem_last), Some(keep_first)) = (new_points.last(), keep_points.first()) {
                    if rem_last == keep_first {
                        new_points.pop();
                    }
                }
                new_points.append(keep_points);
                *keep_points = new_points;
            }
            (EdgeSide::Right, EdgeSide::Right) => {
                // C++: reverse remove, append to keep (a b c z y x)
                let mut reversed: Vec<_> = remove_points.into_iter().rev().collect();
                // Skip duplicate at join point (last of keep == first of reversed)
                if let (Some(keep_last), Some(rev_first)) = (keep_points.last(), reversed.first()) {
                    if keep_last == rev_first {
                        reversed.remove(0);
                    }
                }
                keep_points.extend(reversed);
            }
            (EdgeSide::Right, EdgeSide::Left) => {
                // C++: append remove to keep (a b c x y z)
                let mut remove = remove_points;
                // Skip duplicate at join point (last of keep == first of remove)
                if let (Some(keep_last), Some(rem_first)) = (keep_points.last(), remove.first()) {
                    if keep_last == rem_first {
                        remove.remove(0);
                    }
                }
                keep_points.extend(remove);
            }
        }
    }

    // Transfer children from removed ring to kept ring
    if let Some(remove_ring) = manager.get(remove_idx) {
        let children: Vec<usize> = remove_ring.children().to_vec();

        // Update children's parent to point to kept ring
        for &child_idx in &children {
            if let Some(child) = manager.get_mut(child_idx) {
                child.set_parent(Some(keep_idx));
            }
        }

        // Add children to kept ring
        if let Some(keep_ring) = manager.get_mut(keep_idx) {
            for &child_idx in &children {
                keep_ring.add_child(child_idx);
            }
        }
    }

    // PORT FROM: wagyu C++ append_ring (ring_util.hpp:582)
    // C++ sets: remove_ring->points = nullptr; remove_ring->bottom_point = nullptr;
    //
    // NOTE: We do NOT clear the removed ring's points here during the Vatti sweep
    // because other bounds may still reference this ring. Instead, we mark the
    // ring as merged, and its points will be cleared at the start of topology
    // correction to prevent collinear edge correction bugs.
    manager.mark_as_merged(remove_idx);

    // Clear ring references on both bounds (they meet at max, so done contributing)
    b1.ring = None;
    b2.ring = None;

    // Record the coordinates where these bounds' rings were cleared
    // They can create new rings at the SAME point (corner-touching) but not
    // at DIFFERENT points on the same scanline (spurious ring creation)
    b1.ring_cleared_at = Some((pt.x, pt.y));
    b2.ring_cleared_at = Some((pt.x, pt.y));

    // Return merge info for caller to update other bounds
    (keep_idx, remove_idx, keep_side)
}

/// Add a local minimum point where two non-contributing bounds become contributing.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_local_minimum_point
///
/// When two non-contributing bounds of different polygon types intersect
/// in a way that starts output, this creates a new ring.
///
/// Returns the index of the newly created ring so the caller can set hole state.
fn add_local_minimum_point_at_intersection<T: CoordNum>(
    b1: &mut Bound<T>,
    b2: &mut Bound<T>,
    pt: Point<T>,
    manager: &mut RingManager<T>,
) -> usize {
    // Create a new ring and add the point
    let ring = crate::Ring::new(vec![geo_types::Coord { x: pt.x, y: pt.y }]);
    let ring_idx = manager.add_ring(ring);

    // Log the new ring creation (for debug parity with add_first_point in ring_util.rs)
    if crate::debug::debug_enabled() {
        crate::debug::log_ring_new(
            ring_idx,
            (pt.x.to_f64().unwrap_or(0.0), pt.y.to_f64().unwrap_or(0.0)),
        );
    }

    // Determine which bound gets left side based on dx
    // PORT FROM: C++ ring_util.hpp line 360:
    // if (is_horizontal(*b2.current_edge) || (b1.current_edge->dx > b2.current_edge->dx))
    let b2_horizontal = b2.is_horizontal();
    let b1_dx = b1.current_edge().dx;
    let b2_dx = b2.current_edge().dx;

    if b2_horizontal || b1_dx > b2_dx {
        b1.ring = Some(ring_idx);
        b1.side = EdgeSide::Left;
        b1.last_point = pt;

        b2.ring = Some(ring_idx);
        b2.side = EdgeSide::Right;
        b2.last_point = pt;
    } else {
        b1.ring = Some(ring_idx);
        b1.side = EdgeSide::Right;
        b1.last_point = pt;

        b2.ring = Some(ring_idx);
        b2.side = EdgeSide::Left;
        b2.last_point = pt;
    }

    ring_idx
}

/// Process an intersection between two bounds.
///
/// From C++: `intersect_bounds(b1, b2, pt, cliptype, ...)`
///
/// This handles winding count updates, side/ring swapping, and
/// adding intersection points to rings.
///
/// # Arguments
/// * `b1` - First bound
/// * `b2` - Second bound
/// * `pt` - Intersection point
/// * `cliptype` - Boolean operation type
/// * `subject_fill_type` - Fill rule for subject
/// * `clip_fill_type` - Fill rule for clip
/// * `manager` - Ring manager for output
///
/// # Returns
/// `IntersectResult` indicating what happened:
/// - `None`: No special action needed
/// - `Merged`: Rings were merged, caller should update other bounds
/// - `NewRing`: A new ring was created, caller should set hole state
pub fn intersect_bounds<T: CoordNum>(
    b1: &mut Bound<T>,
    b2: &mut Bound<T>,
    pt: Point<T>,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
    manager: &mut RingManager<T>,
) -> IntersectResult {
    // Update winding counts
    update_winding_counts(b1, b2, subject_fill_type, clip_fill_type);

    // Get effective winding counts for decision making
    let b1_fill = get_fill_type(b1, subject_fill_type, clip_fill_type);
    let b2_fill = get_fill_type(b2, subject_fill_type, clip_fill_type);
    let b1_wc = get_winding_count(b1.winding_count, b1_fill);
    let b2_wc = get_winding_count(b2.winding_count, b2_fill);

    let b1_contributing = b1.ring.is_some();
    let b2_contributing = b2.ring.is_some();

    // Handle intersection based on contribution status
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp lines 192-201
    if b1_contributing && b2_contributing {
        // Check if we need to merge rings at this intersection:
        // 1. Unusual winding counts (not 0 or 1), OR
        // 2. Different polygon types (subject vs clip) and NOT doing XOR
        //
        // This is CRITICAL for difference operations where subject and clip
        // bounds intersect - their rings must merge into one continuous boundary.
        let unusual_winding = (b1_wc != 0 && b1_wc != 1) || (b2_wc != 0 && b2_wc != 1);
        let different_poly_types_not_xor =
            b1.poly_type != b2.poly_type && cliptype != Operation::Xor;

        if unusual_winding || different_poly_types_not_xor {
            // Add local maximum point - rings meet and close/merge
            if let Some((keep, remove, side)) =
                add_local_maximum_point_at_intersection(b1, b2, pt, manager)
            {
                crate::debug::log_ring_merge(remove, keep);
                return IntersectResult::Merged(keep, remove, side);
            }
            return IntersectResult::None;
        } else {
            // Add point to both rings before swapping
            add_point(b1, pt, manager);
            add_point(b2, pt, manager);

            // Swap sides and rings
            swap_sides(b1, b2);
            swap_rings(b1, b2);
        }
    } else if b1_contributing {
        if b2_wc == 0 || b2_wc == 1 {
            // Add point to b1's ring before swapping
            add_point(b1, pt, manager);
            // PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp line 205
            // Also update b2's last_point even though it's not contributing
            b2.last_point = pt;

            swap_sides(b1, b2);
            swap_rings(b1, b2);
        }
    } else if b2_contributing {
        if b1_wc == 0 || b1_wc == 1 {
            // PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp line 211
            // Update b1's last_point even though it's not contributing
            b1.last_point = pt;
            // Add point to b2's ring before swapping
            add_point(b2, pt, manager);

            swap_sides(b1, b2);
            swap_rings(b1, b2);
        }
    } else if (b1_wc == 0 || b1_wc == 1) && (b2_wc == 0 || b2_wc == 1) {
        // Neither contributing - may start a new output region
        // PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp lines 217-270
        if b1.poly_type != b2.poly_type {
            // FIX #54: Check if either bound's ring was cleared at a DIFFERENT point
            // on the same scanline. This prevents spurious ring creation while allowing
            // legitimate corner-touching cases.
            //
            // - Spurious: merge at (10,10), new ring at (0,10) - BLOCK (different X)
            // - Legitimate: merge at (1,1), new ring at (1,1) - ALLOW (same point)
            let b1_cleared_elsewhere = b1
                .ring_cleared_at
                .map_or(false, |(x, y)| y == pt.y && x != pt.x);
            let b2_cleared_elsewhere = b2
                .ring_cleared_at
                .map_or(false, |(x, y)| y == pt.y && x != pt.x);

            if b1_cleared_elsewhere || b2_cleared_elsewhere {
                // Ring was cleared at a different X on this scanline - spurious
                swap_sides(b1, b2);
                swap_rings(b1, b2);
                return IntersectResult::None;
            }

            // Different polygon types - add local minimum point
            let ring_idx = add_local_minimum_point_at_intersection(b1, b2, pt, manager);
            return IntersectResult::NewRing(ring_idx);
        } else if b1_wc == 1 && b2_wc == 1 {
            // Same polygon type, both with winding count 1
            // Calculate effective winding_count2 based on fill type
            let b1_fill_type2 = get_fill_type2(b1, subject_fill_type, clip_fill_type);
            let b2_fill_type2 = get_fill_type2(b2, subject_fill_type, clip_fill_type);
            let b1_wc2 = get_winding_count(b1.winding_count2, b1_fill_type2);
            let b2_wc2 = get_winding_count(b2.winding_count2, b2_fill_type2);

            match cliptype {
                Operation::Intersection => {
                    if b1_wc2 > 0 && b2_wc2 > 0 {
                        let ring_idx = add_local_minimum_point_at_intersection(b1, b2, pt, manager);
                        return IntersectResult::NewRing(ring_idx);
                    }
                }
                Operation::Union => {
                    if b1_wc2 <= 0 && b2_wc2 <= 0 {
                        let ring_idx = add_local_minimum_point_at_intersection(b1, b2, pt, manager);
                        return IntersectResult::NewRing(ring_idx);
                    }
                }
                Operation::Difference => {
                    // For difference: depends on polygon type and winding_count2
                    let should_add = match b1.poly_type {
                        PolygonType::Clip => b1_wc2 > 0 && b2_wc2 > 0,
                        PolygonType::Subject => b1_wc2 <= 0 && b2_wc2 <= 0,
                    };
                    if should_add {
                        let ring_idx = add_local_minimum_point_at_intersection(b1, b2, pt, manager);
                        return IntersectResult::NewRing(ring_idx);
                    }
                }
                Operation::Xor => {
                    // XOR always starts a new ring for same-type bounds at wc=1
                    let ring_idx = add_local_minimum_point_at_intersection(b1, b2, pt, manager);
                    return IntersectResult::NewRing(ring_idx);
                }
            }
        } else {
            // b1_wc != 1 || b2_wc != 1, just swap sides
            swap_sides(b1, b2);
        }
    }
    IntersectResult::None
}

/// Process all intersections in the list.
///
/// From C++: `process_intersect_list(intersects, cliptype, ...)`
///
/// Processes each intersection node, calling intersect_bounds and swapping
/// bounds in the active edge list.
pub fn process_intersect_list<T: CoordNum + ToPrimitive>(
    intersects: &IntersectList<T>,
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
    manager: &mut RingManager<T>,
) {
    for node in intersects.iter() {
        let idx1 = node.bound1_index;
        let idx2 = node.bound2_index;

        // Find positions in AEL
        let pos1 = ael.position(idx1);
        let pos2 = ael.position(idx2);

        if let (Some(p1), Some(p2)) = (pos1, pos2) {
            // Process intersection
            // Safety: we're using indices from the intersection list which
            // were valid when built
            let result = {
                let (b1, b2) = if idx1 < idx2 {
                    let (left, right) = bounds.split_at_mut(idx2);
                    (&mut left[idx1], &mut right[0])
                } else {
                    let (left, right) = bounds.split_at_mut(idx1);
                    (&mut right[0], &mut left[idx2])
                };

                intersect_bounds(
                    b1,
                    b2,
                    node.point,
                    cliptype,
                    subject_fill_type,
                    clip_fill_type,
                    manager,
                )
            };

            match result {
                IntersectResult::Merged(keep_ring_idx, remove_ring_idx, keep_side) => {
                    // Update other active bounds that reference the removed ring
                    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - append_ring (lines 597-606)
                    if crate::debug::debug_enabled() {
                        eprintln!(
                            "[MERGE_SEARCH] Looking for bounds with ring={}, AEL has {} bounds",
                            remove_ring_idx, ael.as_slice().len()
                        );
                        for &ab_idx in ael.as_slice() {
                            eprintln!(
                                "  [AEL_BOUND] idx={} ring={:?} side={:?}",
                                ab_idx, bounds[ab_idx].ring, bounds[ab_idx].side
                            );
                        }
                    }
                    for &ab_idx in ael.as_slice() {
                        if bounds[ab_idx].ring == Some(remove_ring_idx) {
                            if crate::debug::debug_enabled() {
                                eprintln!(
                                    "[BOUND_UPDATE] intersect_merge: ab_idx={} ring: {} -> {}, side: {:?}",
                                    ab_idx, remove_ring_idx, keep_ring_idx, keep_side
                                );
                            }
                            bounds[ab_idx].ring = Some(keep_ring_idx);
                            bounds[ab_idx].side = keep_side;
                            // FIX #53: Don't break - update ALL bounds with removed ring
                            // C++ breaks because pointer comparison is unique.
                            // In Rust, multiple bounds can share the same ring index.
                        }
                    }
                }
                IntersectResult::NewRing(ring_idx) => {
                    // Set hole state for newly created ring
                    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - set_hole_state (lines 31-57)
                    //
                    // Find which bound owns this ring (the one with left side)
                    let owner_pos = if bounds[idx1].ring == Some(ring_idx)
                        && bounds[idx1].side == EdgeSide::Left
                    {
                        ael.position(idx1)
                    } else if bounds[idx2].ring == Some(ring_idx)
                        && bounds[idx2].side == EdgeSide::Left
                    {
                        ael.position(idx2)
                    } else {
                        // Just use the first one
                        Some(p1.min(p2))
                    };

                    if let Some(owner_pos) = owner_pos {
                        // Look leftward in the AEL to find parent ring
                        // C++: finds first ring to the left, canceling pairs with same ring
                        let mut tmp_ring: Option<usize> = None;

                        for i in (0..owner_pos).rev() {
                            let ab_idx = ael.as_slice()[i];
                            if let Some(other_ring) = bounds[ab_idx].ring {
                                if other_ring == ring_idx {
                                    continue; // Skip our own ring
                                }
                                if tmp_ring.is_none() {
                                    tmp_ring = Some(other_ring);
                                } else if tmp_ring == Some(other_ring) {
                                    tmp_ring = None; // Cancel out paired bounds
                                }
                            }
                        }

                        if let Some(parent_idx) = tmp_ring {
                            manager.set_parent(ring_idx, parent_idx);
                            if let Some(ring) = manager.get_mut(ring_idx) {
                                ring.set_hole(true);
                            }
                        }
                    }
                }
                IntersectResult::None => {}
            }

            // Swap positions in AEL
            ael.swap(p1, p2);
        }
    }
}

/// Main entry point for intersection processing at a scanline.
///
/// From C++: `process_intersections(top_y, active_bounds, cliptype, ...)`
pub fn process_intersections<T>(
    top_y: T,
    bounds: &mut [Bound<T>],
    ael: &mut ActiveEdgeList,
    cliptype: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
    manager: &mut RingManager<T>,
) where
    T: CoordNum + ToPrimitive + num_traits::NumCast,
{
    if ael.is_empty() {
        return;
    }

    // Update current_x for all bounds at this scanline
    update_current_x(ael, bounds, top_y);

    // Build list of intersections (simulates swaps without modifying AEL)
    let mut intersects: IntersectList<T> = IntersectList::new();
    build_intersect_list(&*ael, bounds, &mut intersects);

    if intersects.is_empty() {
        return;
    }

    // Sort intersections
    intersects.sort();

    // Process all intersections
    process_intersect_list(
        &intersects,
        bounds,
        ael,
        cliptype,
        subject_fill_type,
        clip_fill_type,
        manager,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Test Helpers ====================

    fn make_edge(bot: (f64, f64), top: (f64, f64)) -> Edge<f64> {
        Edge::new(Point::new(bot.0, bot.1), Point::new(top.0, top.1))
    }

    fn make_bound(bot: (f64, f64), top: (f64, f64)) -> Bound<f64> {
        let edge = make_edge(bot, top);
        Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left)
    }

    // ==================== round_point Tests ====================

    #[test]
    fn round_point_rounds_to_nearest_integer() {
        let pt: Point<i64> = round_point(Point::new(1.4, 2.6));
        assert_eq!(pt.x, 1);
        assert_eq!(pt.y, 3);
    }

    #[test]
    fn round_point_handles_negative() {
        let pt: Point<i64> = round_point(Point::new(-1.4, -2.6));
        assert_eq!(pt.x, -1);
        assert_eq!(pt.y, -3);
    }

    #[test]
    fn round_point_handles_halfway() {
        // Rust's f64::round() rounds away from zero for .5
        let pt: Point<i64> = round_point(Point::new(1.5, -1.5));
        assert_eq!(pt.x, 2);
        assert_eq!(pt.y, -2);
    }

    // ==================== swap_rings Tests ====================

    #[test]
    fn swap_rings_exchanges_ring_indices() {
        let mut b1 = make_bound((0.0, 0.0), (5.0, 10.0));
        let mut b2 = make_bound((10.0, 0.0), (5.0, 10.0));

        b1.ring = Some(1);
        b2.ring = Some(2);

        swap_rings(&mut b1, &mut b2);

        assert_eq!(b1.ring, Some(2));
        assert_eq!(b2.ring, Some(1));
    }

    #[test]
    fn swap_rings_handles_none() {
        let mut b1 = make_bound((0.0, 0.0), (5.0, 10.0));
        let mut b2 = make_bound((10.0, 0.0), (5.0, 10.0));

        b1.ring = Some(1);
        b2.ring = None;

        swap_rings(&mut b1, &mut b2);

        assert_eq!(b1.ring, None);
        assert_eq!(b2.ring, Some(1));
    }

    // ==================== swap_sides Tests ====================

    #[test]
    fn swap_sides_exchanges_edge_sides() {
        let mut b1 = make_bound((0.0, 0.0), (5.0, 10.0));
        let mut b2 = make_bound((10.0, 0.0), (5.0, 10.0));

        b1.side = EdgeSide::Left;
        b2.side = EdgeSide::Right;

        swap_sides(&mut b1, &mut b2);

        assert_eq!(b1.side, EdgeSide::Right);
        assert_eq!(b2.side, EdgeSide::Left);
    }

    // ==================== get_edge_intersection Tests ====================

    #[test]
    fn get_edge_intersection_finds_crossing() {
        // Two edges that cross at (5, 5)
        // Edge 1: (0, 0) -> (10, 10)
        // Edge 2: (0, 10) -> (10, 0)
        let e1 = make_edge((0.0, 0.0), (10.0, 10.0));
        let e2 = make_edge((0.0, 10.0), (10.0, 0.0));

        let result = get_edge_intersection(&e1, &e2);

        assert!(result.is_some());
        let pt = result.unwrap();
        assert!((pt.x - 5.0).abs() < 1e-10);
        assert!((pt.y - 5.0).abs() < 1e-10);
    }

    #[test]
    fn get_edge_intersection_returns_none_for_parallel() {
        // Two parallel edges
        let e1 = make_edge((0.0, 0.0), (10.0, 10.0));
        let e2 = make_edge((1.0, 0.0), (11.0, 10.0));

        let result = get_edge_intersection(&e1, &e2);

        assert!(result.is_none());
    }

    #[test]
    fn get_edge_intersection_returns_none_when_outside_segments() {
        // Edges that would intersect if extended, but not within their bounds
        let e1 = make_edge((0.0, 0.0), (5.0, 5.0));
        let e2 = make_edge((10.0, 10.0), (15.0, 5.0));

        let result = get_edge_intersection(&e1, &e2);

        assert!(result.is_none());
    }

    #[test]
    fn get_edge_intersection_handles_horizontal() {
        // Horizontal edge crossed by sloped edge
        let e1 = make_edge((0.0, 5.0), (10.0, 5.0)); // horizontal at y=5
        let e2 = make_edge((5.0, 0.0), (5.0, 10.0)); // vertical at x=5

        let result = get_edge_intersection(&e1, &e2);

        assert!(result.is_some());
        let pt = result.unwrap();
        assert!((pt.x - 5.0).abs() < 1e-10);
        assert!((pt.y - 5.0).abs() < 1e-10);
    }

    // ==================== get_current_x Tests ====================

    #[test]
    fn get_current_x_at_bottom() {
        let edge = make_edge((0.0, 0.0), (10.0, 10.0));
        let x = get_current_x(&edge, 0.0_f64);
        assert!((x - 0.0).abs() < 1e-10);
    }

    #[test]
    fn get_current_x_at_top() {
        let edge = make_edge((0.0, 0.0), (10.0, 10.0));
        let x = get_current_x(&edge, 10.0_f64);
        assert!((x - 10.0).abs() < 1e-10);
    }

    #[test]
    fn get_current_x_at_midpoint() {
        let edge = make_edge((0.0, 0.0), (10.0, 10.0));
        let x = get_current_x(&edge, 5.0_f64);
        assert!((x - 5.0).abs() < 1e-10);
    }

    #[test]
    fn get_current_x_horizontal_returns_top_x() {
        // Horizontal edge: bot=(5.0, 10.0), top=(15.0, 10.0)
        // At y=10.0 (which equals top.y), C++ returns top.x = 15.0
        // This was previously incorrectly testing for bot.x = 5.0
        let edge = make_edge((5.0, 10.0), (15.0, 10.0)); // horizontal
        let x = get_current_x(&edge, 10.0_f64);
        assert!(
            (x - 15.0).abs() < 1e-10,
            "Horizontal edge at y==top.y should return top.x (15.0), got {}",
            x
        );
    }

    /// Test that get_current_x returns top.x when y == top.y (C++ special case).
    ///
    /// This is critical for horizontal edges where bot.x != top.x.
    /// C++ code from edge.hpp:81-87:
    /// ```cpp
    /// if (current_y == edge.top.y) {
    ///     return static_cast<double>(edge.top.x);
    /// }
    /// ```
    #[test]
    fn get_current_x_at_top_y_returns_top_x() {
        // Horizontal edge from (2500, -2500) to (-2500, -2500)
        // bot = (2500, -2500), top = (-2500, -2500) based on Edge::new logic
        // At y = -2500 (top.y), C++ returns top.x = -2500
        let edge = make_edge((2500.0, -2500.0), (-2500.0, -2500.0));
        let x = get_current_x(&edge, -2500.0_f64);
        // C++ returns top.x = -2500.0, not bot.x = 2500.0
        assert!(
            (x - (-2500.0)).abs() < 1e-10,
            "Expected top.x (-2500.0) when y == top.y, got {} (likely bot.x)",
            x
        );
    }

    /// Test that non-horizontal edges also return top.x at top.y (edge case).
    #[test]
    fn get_current_x_non_horizontal_at_top_y() {
        // Diagonal edge from (0, 0) to (10, 10)
        let edge = make_edge((0.0, 0.0), (10.0, 10.0));
        let x = get_current_x(&edge, 10.0_f64);
        // At top.y=10, should return top.x=10
        assert!((x - 10.0).abs() < 1e-10);
    }

    // ==================== is_even_odd_fill_type Tests ====================

    #[test]
    fn is_even_odd_fill_type_subject_even_odd() {
        let bound = make_bound((0.0, 0.0), (5.0, 10.0));
        assert!(is_even_odd_fill_type(
            &bound,
            FillType::EvenOdd,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_even_odd_fill_type_subject_non_zero() {
        let bound = make_bound((0.0, 0.0), (5.0, 10.0));
        assert!(!is_even_odd_fill_type(
            &bound,
            FillType::NonZero,
            FillType::EvenOdd
        ));
    }

    #[test]
    fn is_even_odd_fill_type_clip() {
        let edge = make_edge((0.0, 0.0), (5.0, 10.0));
        let bound = Bound::new(vec![edge], PolygonType::Clip, EdgeSide::Left);

        assert!(is_even_odd_fill_type(
            &bound,
            FillType::NonZero,
            FillType::EvenOdd
        ));
    }

    // ==================== update_winding_counts Tests ====================

    #[test]
    fn update_winding_counts_same_type_even_odd_swaps() {
        let mut b1 = make_bound((0.0, 0.0), (5.0, 10.0));
        let mut b2 = make_bound((10.0, 0.0), (5.0, 10.0));

        b1.winding_count = 1;
        b2.winding_count = 2;

        update_winding_counts(&mut b1, &mut b2, FillType::EvenOdd, FillType::EvenOdd);

        assert_eq!(b1.winding_count, 2);
        assert_eq!(b2.winding_count, 1);
    }

    #[test]
    fn update_winding_counts_different_type_updates_count2() {
        let edge1 = make_edge((0.0, 0.0), (5.0, 10.0));
        let edge2 = make_edge((10.0, 0.0), (5.0, 10.0));
        let mut b1 = Bound::new(vec![edge1], PolygonType::Subject, EdgeSide::Left);
        let mut b2 = Bound::new(vec![edge2], PolygonType::Clip, EdgeSide::Left);

        b1.winding_count2 = 0;
        b2.winding_count2 = 0;

        update_winding_counts(&mut b1, &mut b2, FillType::NonZero, FillType::NonZero);

        // b1's count2 should increase, b2's count2 should decrease
        assert_eq!(b1.winding_count2, 1);
        assert_eq!(b2.winding_count2, -1);
    }

    // ==================== build_intersect_list Tests ====================

    #[test]
    fn build_intersect_list_empty_ael() {
        let ael = ActiveEdgeList::new();
        let bounds: Vec<Bound<f64>> = vec![];
        let mut intersects: IntersectList<f64> = IntersectList::new();

        build_intersect_list(&ael.clone(), &bounds, &mut intersects);

        assert!(intersects.is_empty());
    }

    #[test]
    fn build_intersect_list_single_bound() {
        let bounds = vec![make_bound((0.0, 0.0), (5.0, 10.0))];
        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        let mut intersects: IntersectList<f64> = IntersectList::new();

        build_intersect_list(&ael, &bounds, &mut intersects);

        assert!(intersects.is_empty());
    }

    #[test]
    fn build_intersect_list_non_crossing_bounds() {
        // Two bounds that don't cross (parallel)
        let mut bounds = vec![
            make_bound((0.0, 0.0), (0.0, 10.0)), // vertical at x=0
            make_bound((5.0, 0.0), (5.0, 10.0)), // vertical at x=5
        ];

        // Set current_x values
        bounds[0].current_x = 0.0;
        bounds[1].current_x = 5.0;

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        let mut intersects: IntersectList<f64> = IntersectList::new();

        build_intersect_list(&ael, &bounds, &mut intersects);

        // Parallel bounds don't intersect
        assert!(intersects.is_empty());
    }

    #[test]
    fn build_intersect_list_crossing_bounds() {
        // Two bounds that cross - set up so they're out of order
        let mut bounds = vec![
            make_bound((0.0, 0.0), (10.0, 10.0)), // goes up-right
            make_bound((10.0, 0.0), (0.0, 10.0)), // goes up-left
        ];

        // At some scanline, their x positions are swapped
        // Bound 0 (starts at x=0) is now at x=7
        // Bound 1 (starts at x=10) is now at x=3
        bounds[0].current_x = 7.0;
        bounds[1].current_x = 3.0;

        // Create AEL manually with indices in "wrong" order based on current_x
        // We want bound 0 (current_x=7) to be before bound 1 (current_x=3) in AEL
        // This simulates the state after the sweep has moved up past the intersection
        let mut ael = ActiveEdgeList::new();

        // Insert bound 1 first (lower current_x), then bound 0 (higher current_x)
        // But actually, we want them in the "wrong" order to trigger detection
        // The issue: insert uses bounds' current_x for positioning
        // Let me manually construct the AEL state

        // For a proper test, we need to have them in the wrong order in the AEL
        // Let's use insert with initial setup, then modify current_x after
        bounds[0].current_x = 0.0; // Initial state
        bounds[1].current_x = 10.0;

        ael.insert(0, &bounds); // Inserted at correct position based on initial x
        ael.insert(1, &bounds);

        // Now simulate that the sweep moved up and they crossed
        // Update current_x to the new positions
        bounds[0].current_x = 7.0; // Now on the right side
        bounds[1].current_x = 3.0; // Now on the left side

        // The bounds are now out of order (bound 0 has higher x but comes first)
        let mut intersects: IntersectList<f64> = IntersectList::new();

        build_intersect_list(&ael, &bounds, &mut intersects);

        // Should detect the crossing
        assert!(!intersects.is_empty());
    }

    // ==================== intersect_bounds Tests ====================

    #[test]
    fn intersect_bounds_swaps_sides_when_both_contributing() {
        let mut b1 = make_bound((0.0, 0.0), (5.0, 10.0));
        let mut b2 = make_bound((10.0, 0.0), (5.0, 10.0));

        b1.ring = Some(0);
        b2.ring = Some(1);
        b1.side = EdgeSide::Left;
        b2.side = EdgeSide::Right;

        let mut manager: RingManager<f64> = RingManager::new();
        // Add placeholder rings so the indices are valid
        manager.add_ring(crate::Ring::empty());
        manager.add_ring(crate::Ring::empty());

        intersect_bounds(
            &mut b1,
            &mut b2,
            Point::new(5.0, 5.0),
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
            &mut manager,
        );

        // Sides should be swapped
        assert_eq!(b1.side, EdgeSide::Right);
        assert_eq!(b2.side, EdgeSide::Left);
    }

    // ==================== update_current_x Tests ====================

    #[test]
    fn update_current_x_updates_all_bounds() {
        let mut bounds = vec![
            make_bound((0.0, 0.0), (10.0, 10.0)),
            make_bound((5.0, 0.0), (15.0, 10.0)),
        ];

        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        // Update at y=5
        update_current_x(&ael, &mut bounds, 5.0_f64);

        // Edge 1: at y=5, x should be 5
        assert!((bounds[0].current_x - 5.0).abs() < 1e-10);
        // Edge 2: at y=5, x should be 10
        assert!((bounds[1].current_x - 10.0).abs() < 1e-10);
    }

    // ==================== MINIMAL CROSSING TEST ====================
    // This test creates the SIMPLEST possible case where two edges cross.
    // It uses two edges forming an X pattern and verifies intersection detection.

    #[test]
    fn minimal_x_pattern_crossing_detection() {
        // Two edges that CLEARLY cross in an X pattern:
        //
        //    y=10:   *           *
        //            |\         /|
        //            | \       / |
        //    y=5:    |  \  X  /  |    <- crossing point at (5, 5)
        //            |   \   /   |
        //            |    \ /    |
        //    y=0:    *     *     *
        //           x=0   x=5   x=10
        //
        // Edge A: (0, 10) -> (10, 0)  -- goes down-right
        // Edge B: (10, 10) -> (0, 0)  -- goes down-left
        //
        // In wagyu convention: bot.y >= top.y
        // So Edge A: bot=(0, 10), top=(10, 0)
        //    Edge B: bot=(10, 10), top=(0, 0)

        // First, verify the edge geometry is correct
        let edge_a = Edge::new(Point::new(0.0_f64, 10.0), Point::new(10.0_f64, 0.0));
        let edge_b = Edge::new(Point::new(10.0_f64, 10.0), Point::new(0.0_f64, 0.0));

        // Verify orientation (bot.y >= top.y)
        assert!(
            edge_a.bot.y >= edge_a.top.y,
            "Edge A should have bot.y >= top.y, but bot={:?}, top={:?}",
            edge_a.bot,
            edge_a.top
        );
        assert!(
            edge_b.bot.y >= edge_b.top.y,
            "Edge B should have bot.y >= top.y, but bot={:?}, top={:?}",
            edge_b.bot,
            edge_b.top
        );

        // Verify dx is computed correctly
        // For edge_a: dx = (top.x - bot.x) / (top.y - bot.y) = (10 - 0) / (0 - 10) = -1
        eprintln!(
            "Edge A: bot={:?}, top={:?}, dx={}",
            edge_a.bot, edge_a.top, edge_a.dx
        );
        eprintln!(
            "Edge B: bot={:?}, top={:?}, dx={}",
            edge_b.bot, edge_b.top, edge_b.dx
        );
        assert!(
            (edge_a.dx - (-1.0)).abs() < 1e-10,
            "Edge A dx should be -1, got {}",
            edge_a.dx
        );
        // For edge_b: dx = (top.x - bot.x) / (top.y - bot.y) = (0 - 10) / (0 - 10) = 1
        assert!(
            (edge_b.dx - 1.0).abs() < 1e-10,
            "Edge B dx should be 1, got {}",
            edge_b.dx
        );

        // Create bounds from these edges
        let mut bounds = vec![
            Bound::new(vec![edge_a], PolygonType::Subject, EdgeSide::Left),
            Bound::new(vec![edge_b], PolygonType::Subject, EdgeSide::Right),
        ];

        // At y=10 (the starting scanline), edges start at:
        // - Edge A: x = 0 (from bot.x)
        // - Edge B: x = 10 (from bot.x)
        bounds[0].current_x = get_current_x(bounds[0].current_edge(), 10.0_f64);
        bounds[1].current_x = get_current_x(bounds[1].current_edge(), 10.0_f64);

        eprintln!(
            "At y=10: bound[0].x = {}, bound[1].x = {}",
            bounds[0].current_x, bounds[1].current_x
        );
        assert!(
            (bounds[0].current_x - 0.0).abs() < 1e-10,
            "At y=10, edge A should be at x=0"
        );
        assert!(
            (bounds[1].current_x - 10.0).abs() < 1e-10,
            "At y=10, edge B should be at x=10"
        );

        // Insert into AEL in initial order (bound 0 first because it has lower x)
        let mut ael = ActiveEdgeList::new();
        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        // Now advance to y=0 (bottom scanline)
        // At y=0:
        // - Edge A: x = 0 + dx * (0 - 10) = 0 + (-1) * (-10) = 10
        // - Edge B: x = 10 + dx * (0 - 10) = 10 + 1 * (-10) = 0
        update_current_x(&ael, &mut bounds, 0.0_f64);

        eprintln!(
            "At y=0: bound[0].x = {}, bound[1].x = {}",
            bounds[0].current_x, bounds[1].current_x
        );
        assert!(
            (bounds[0].current_x - 10.0).abs() < 1e-10,
            "At y=0, edge A should be at x=10, got {}",
            bounds[0].current_x
        );
        assert!(
            (bounds[1].current_x - 0.0).abs() < 1e-10,
            "At y=0, edge B should be at x=0, got {}",
            bounds[1].current_x
        );

        // The edges have CROSSED! Bound 0 is now at x=10, bound 1 is at x=0
        // But in the AEL, bound 0 comes BEFORE bound 1.
        // This means bound 0 (x=10) comes before bound 1 (x=0) -- they're out of order!

        // Now build_intersect_list should detect this crossing
        let mut intersects: IntersectList<f64> = IntersectList::new();
        build_intersect_list(&ael, &bounds, &mut intersects);

        // This is the critical assertion: intersection MUST be detected
        assert!(
            !intersects.is_empty(),
            "Two clearly crossing edges should produce an intersection! \
            Bound 0 (x={}) comes before Bound 1 (x={}) in AEL but has higher x.",
            bounds[0].current_x,
            bounds[1].current_x
        );

        // Verify the intersection point is correct (should be at (5, 5))
        let node = &intersects[0];
        let pt = get_edge_intersection(
            bounds[node.bound1_index].current_edge(),
            bounds[node.bound2_index].current_edge(),
        );
        assert!(
            pt.is_some(),
            "get_edge_intersection should find the crossing"
        );
        let pt = pt.unwrap();
        assert!(
            (pt.x - 5.0).abs() < 1e-10,
            "Intersection x should be 5, got {}",
            pt.x
        );
        assert!(
            (pt.y - 5.0).abs() < 1e-10,
            "Intersection y should be 5, got {}",
            pt.y
        );
    }

    // ==================== Issue #53: Chained Merge Bug Tests ====================

    /// Test for issue #53: Verifies that when multiple bounds reference the same
    /// ring index, ALL of them get updated after a merge (not just the first).
    ///
    /// This tests the FIXED behavior where we don't break after the first match.
    #[test]
    fn merge_update_loop_should_update_all_bounds_with_removed_ring() {
        // Setup: Three bounds where TWO reference the same ring
        let mut bounds = [
            make_bound((0.0, 0.0), (5.0, 10.0)),
            make_bound((5.0, 0.0), (10.0, 10.0)),
            make_bound((10.0, 0.0), (15.0, 10.0)),
        ];

        // bounds[0] has ring 0, bounds[1] and bounds[2] BOTH have ring 1
        // This can happen when two edges of the same ring are both active
        bounds[0].ring = Some(0);
        bounds[1].ring = Some(1); // Left edge of ring 1
        bounds[2].ring = Some(1); // Right edge of ring 1

        bounds[0].side = EdgeSide::Left;
        bounds[1].side = EdgeSide::Left;
        bounds[2].side = EdgeSide::Right;

        // Simulate: Ring 1 is merged into Ring 0
        // This is what happens in process_intersect_list after IntersectResult::Merged
        let keep_ring_idx = 0usize;
        let remove_ring_idx = 1usize;
        let keep_side = EdgeSide::Left;

        // This is the FIXED update loop - no break, updates ALL matching bounds
        let ael_indices = [0usize, 1, 2];
        for &ab_idx in &ael_indices {
            if bounds[ab_idx].ring == Some(remove_ring_idx) {
                bounds[ab_idx].ring = Some(keep_ring_idx);
                bounds[ab_idx].side = keep_side;
                // FIX #53: No break - continue to update all matching bounds
            }
        }

        // bounds[1] should be updated to ring 0 (first match, updated)
        assert_eq!(
            bounds[1].ring,
            Some(0),
            "Bound 1 should be updated to ring 0"
        );

        // With the fix, bounds[2] should ALSO be updated to ring 0
        assert_eq!(
            bounds[2].ring,
            Some(0),
            "Bound 2 should also be updated to ring 0 (fix for issue #53)"
        );
    }
}
