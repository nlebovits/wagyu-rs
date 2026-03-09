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
// Ring Creation and Manipulation Functions
// ============================================================================
// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp (lines 306-625)
//
// These functions handle ring creation, point addition, and ring merging
// during the Vatti clipping algorithm.
//
// DIVERGENCE FROM WAGYU:
// - C++ uses a circular doubly-linked list of points; Rust uses Vec<Coord<T>>
// - C++ uses raw pointers for ring/bound references; Rust uses indices
// - C++ mutates linked list in place; Rust appends to Vec and may reverse
// - The "side" (Left/Right) concept is simplified since we don't have a linked list

use crate::bound::Bound;
use crate::build_result::RingManager;
use crate::config::EdgeSide;
use crate::Ring;

/// Determine the hole state for a new ring based on active bounds.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - set_hole_state (lines 31-57)
///
/// This function scans active bounds to the left of the current bound to find
/// the nearest non-null ring. If found, the new ring becomes a child of that ring.
/// If not found, the new ring is a top-level ring (added to manager.children).
///
/// # Arguments
/// * `bound_idx` - Index of the bound whose ring needs hole state set
/// * `active_bounds` - List of active bound indices (rightmost first after reverse iteration)
/// * `bounds` - Slice of all bounds
/// * `rings` - The ring manager
///
/// DIVERGENCE FROM WAGYU:
/// - C++ iterates reverse from bound position; we iterate from the bound's position to start
/// - C++ uses nullptr checks; Rust uses Option<usize> for ring indices
pub fn set_hole_state<T: CoordNum>(
    bound_idx: usize,
    active_bounds: &[usize],
    bounds: &[Bound<T>],
    rings: &mut RingManager<T>,
) {
    // Find position of this bound in active_bounds
    let pos = active_bounds.iter().position(|&b| b == bound_idx);
    if pos.is_none() {
        return;
    }
    let pos = pos.unwrap();

    let ring_idx = match bounds[bound_idx].ring {
        Some(r) => r,
        None => return,
    };

    // Look leftward (earlier in the list) to find a non-null ring
    let mut bnd_tmp: Option<usize> = None;
    for &ab_idx in active_bounds[..pos].iter().rev() {
        if let Some(other_ring_idx) = bounds[ab_idx].ring {
            if bnd_tmp.is_none() {
                bnd_tmp = Some(ab_idx);
            } else if let Some(tmp_idx) = bnd_tmp {
                // If the same ring is encountered twice, it cancels out
                if bounds[tmp_idx].ring == Some(other_ring_idx) {
                    bnd_tmp = None;
                }
            }
        }
    }

    if let Some(tmp_idx) = bnd_tmp {
        // This ring is a child of the ring at bnd_tmp
        if let Some(parent_ring_idx) = bounds[tmp_idx].ring {
            rings.set_parent(ring_idx, parent_ring_idx);
        }
    }
    // If bnd_tmp is None, the ring is a top-level ring (no parent to set)
}

/// Create a new ring with the first point and set its hole state.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_first_point (lines 306-316)
///
/// This function:
/// 1. Creates a new ring in the ring manager
/// 2. Adds the initial point
/// 3. Links the ring to the bound
/// 4. Determines if this ring is a hole based on surrounding active bounds
///
/// # Arguments
/// * `bound_idx` - Index of the bound to associate with the new ring
/// * `bounds` - Mutable slice of all bounds
/// * `active_bounds` - List of active bound indices
/// * `pt` - The first point of the ring
/// * `rings` - The ring manager
///
/// # Returns
/// The index of the newly created ring
pub fn add_first_point<T: CoordNum + Copy>(
    bound_idx: usize,
    bounds: &mut [Bound<T>],
    active_bounds: &[usize],
    pt: Coord<T>,
    rings: &mut RingManager<T>,
) -> usize {
    // Create a new ring
    let mut new_ring = Ring::empty();
    new_ring.push_point(pt);

    // Add ring to manager and get its index
    let ring_idx = rings.add_ring(new_ring);

    // Link ring to bound
    bounds[bound_idx].ring = Some(ring_idx);

    // Determine hole state based on active bounds
    // We need to make a copy of bounds for the immutable reference
    let bounds_snapshot: Vec<_> = bounds.iter().map(|b| b.ring).collect();

    // For set_hole_state, we need to work with the current state
    // Since we can't borrow mutably and immutably at the same time,
    // we'll set the parent after analyzing the active bounds
    let mut parent_ring: Option<usize> = None;

    // Find position of this bound in active_bounds
    if let Some(pos) = active_bounds.iter().position(|&b| b == bound_idx) {
        // Look leftward to find a non-null ring
        let mut bnd_tmp: Option<usize> = None;
        for &ab_idx in active_bounds[..pos].iter().rev() {
            if let Some(other_ring_idx) = bounds_snapshot[ab_idx] {
                if bnd_tmp.is_none() {
                    bnd_tmp = Some(ab_idx);
                } else if let Some(tmp_idx) = bnd_tmp {
                    if bounds_snapshot[tmp_idx] == Some(other_ring_idx) {
                        bnd_tmp = None;
                    }
                }
            }
        }

        if let Some(tmp_idx) = bnd_tmp {
            parent_ring = bounds_snapshot[tmp_idx];
        }
    }

    // Set parent if found
    if let Some(parent_idx) = parent_ring {
        rings.set_parent(ring_idx, parent_idx);
        // If this ring has a parent, it's a hole
        if let Some(ring) = rings.get_mut(ring_idx) {
            ring.set_hole(true);
        }
    }

    // C++: bnd.last_point = pt; (line 315)
    // Track the last point added to this bound
    bounds[bound_idx].last_point = pt.into();

    crate::debug::log_ring_new(
        ring_idx,
        (pt.x.to_f64().unwrap_or(0.0), pt.y.to_f64().unwrap_or(0.0)),
    );

    ring_idx
}

/// Add a point to an existing ring.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_point_to_ring (lines 319-337)
///
/// DIVERGENCE FROM WAGYU:
/// - C++ inserts before/after based on side (left/right) due to linked list
/// - Rust simply appends to the Vec; final orientation is handled in build_result
/// - We skip duplicate point detection for now (can be added during final output)
///
/// # Arguments
/// * `bound_idx` - Index of the bound whose ring receives the point
/// * `bounds` - Slice of all bounds
/// * `pt` - The point to add
/// * `rings` - The ring manager
pub fn add_point_to_ring<T: CoordNum + Copy>(
    bound_idx: usize,
    bounds: &[Bound<T>],
    pt: Coord<T>,
    rings: &mut RingManager<T>,
) {
    let ring_idx = match bounds[bound_idx].ring {
        Some(r) => r,
        None => return,
    };

    // DEBUG: Check if ring actually exists
    if crate::debug::debug_enabled() && rings.get(ring_idx).is_none() {
        eprintln!(
            "[WARNING] add_point_to_ring: bound_idx={} has ring={} but ring does not exist! rings.len()={}",
            bound_idx, ring_idx, rings.len()
        );
    }

    // PORT FROM: wagyu C++ add_point_to_ring (ring_util.hpp:319-337)
    // C++ uses a circular linked list where:
    //   - Left side points are inserted at the FRONT (bnd.ring->points = new_point)
    //   - Right side points are inserted at the BACK (before head = after tail)
    // We replicate this with Vec operations.
    let side = bounds[bound_idx].side;
    let to_front = side == EdgeSide::Left;

    crate::debug::log_ring_point(
        ring_idx,
        (pt.x.to_f64().unwrap_or(0.0), pt.y.to_f64().unwrap_or(0.0)),
        to_front,
    );

    if let Some(ring) = rings.get_mut(ring_idx) {
        // Check for duplicate at the insertion position
        if to_front {
            if let Some(first) = ring.first() {
                if *first == pt {
                    return;
                }
            }
            ring.insert_at_front(pt);
        } else {
            if let Some(last) = ring.points().last() {
                if *last == pt {
                    return;
                }
            }
            ring.push_point(pt);
        }
    }
}

/// Add a point to a ring, creating the ring if necessary.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_point (lines 340-349)
///
/// If the bound has no ring, creates one with `add_first_point`.
/// Otherwise, adds the point to the existing ring.
///
/// # Arguments
/// * `bound_idx` - Index of the bound
/// * `bounds` - Mutable slice of all bounds
/// * `active_bounds` - List of active bound indices
/// * `pt` - The point to add
/// * `rings` - The ring manager
pub fn add_point<T: CoordNum + Copy>(
    bound_idx: usize,
    bounds: &mut [Bound<T>],
    active_bounds: &[usize],
    pt: Coord<T>,
    rings: &mut RingManager<T>,
) {
    if bounds[bound_idx].ring.is_none() {
        add_first_point(bound_idx, bounds, active_bounds, pt, rings);
    } else {
        add_point_to_ring(bound_idx, bounds, pt, rings);
    }
    // C++: insert_hot_pixels_in_path sets bnd.last_point = end_pt (ring_util.hpp:297)
    // We update last_point here since we don't have hot pixel interpolation yet
    bounds[bound_idx].last_point = pt.into();
}

/// Add the initial point at a local minimum, linking two bounds.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_local_minimum_point (lines 352-370)
///
/// At a local minimum, two bounds meet at their starting point. This function:
/// 1. Determines which bound should "own" the ring based on edge slopes
/// 2. Creates a ring on the owning bound
/// 3. Links both bounds to the same ring
/// 4. Sets the appropriate side (Left/Right) for each bound
///
/// # Arguments
/// * `b1_idx` - Index of the first bound
/// * `b2_idx` - Index of the second bound
/// * `bounds` - Mutable slice of all bounds
/// * `active_bounds` - List of active bound indices
/// * `pt` - The local minimum point
/// * `rings` - The ring manager
pub fn add_local_minimum_point<T: CoordNum + Copy>(
    b1_idx: usize,
    b2_idx: usize,
    bounds: &mut [Bound<T>],
    active_bounds: &[usize],
    pt: Coord<T>,
    rings: &mut RingManager<T>,
) {
    // Determine which bound should own the ring based on edge slopes
    // C++: if (is_horizontal(*b2.current_edge) || (b1.current_edge->dx > b2.current_edge->dx))
    let b2_horizontal = bounds[b2_idx].is_horizontal();
    let b1_dx = bounds[b1_idx].current_edge().dx;
    let b2_dx = bounds[b2_idx].current_edge().dx;

    if b2_horizontal || b1_dx > b2_dx {
        // b1 owns the ring
        add_point(b1_idx, bounds, active_bounds, pt, rings);
        // C++: b2.last_point = pt; (line 359)
        // The non-owning bound must also track the starting point
        bounds[b2_idx].last_point = pt.into();
        let ring_idx = bounds[b1_idx].ring;
        bounds[b2_idx].ring = ring_idx;
        bounds[b1_idx].side = EdgeSide::Left;
        bounds[b2_idx].side = EdgeSide::Right;
    } else {
        // b2 owns the ring
        add_point(b2_idx, bounds, active_bounds, pt, rings);
        // C++: b1.last_point = pt; (line 365)
        // The non-owning bound must also track the starting point
        bounds[b1_idx].last_point = pt.into();
        let ring_idx = bounds[b2_idx].ring;
        bounds[b1_idx].ring = ring_idx;
        bounds[b1_idx].side = EdgeSide::Right;
        bounds[b2_idx].side = EdgeSide::Left;
    }
}

/// Check if ring1 is a descendant of ring2 in the parent chain.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - ring1_child_below_ring2 (lines 484-492)
///
/// # Arguments
/// * `ring1_idx` - Index of the potential descendant ring
/// * `ring2_idx` - Index of the potential ancestor ring
/// * `rings` - The ring manager
///
/// # Returns
/// True if ring1 is a descendant of ring2
fn ring1_child_below_ring2<T: CoordNum>(
    ring1_idx: usize,
    ring2_idx: usize,
    rings: &RingManager<T>,
) -> bool {
    let mut current = ring1_idx;
    loop {
        let parent = match rings.get(current) {
            Some(r) => r.parent(),
            None => return false,
        };
        match parent {
            Some(p) if p == ring2_idx => return true,
            Some(p) => current = p,
            None => return false,
        }
    }
}

/// Determine which of two rings is the "lower most" based on their bottom points.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - get_lower_most_ring (lines 454-480)
///
/// This is used to determine which ring to keep when merging two rings.
/// The ring with the lower (larger y) bottom point is preferred.
/// If y values are equal, the ring with the smaller x is preferred.
///
/// # Arguments
/// * `ring1_idx` - Index of the first ring
/// * `ring2_idx` - Index of the second ring
/// * `rings` - The ring manager
///
/// # Returns
/// Index of the "lower most" ring, or ring1_idx if they're equal
fn get_lower_most_ring<T: CoordNum>(
    ring1_idx: usize,
    ring2_idx: usize,
    rings: &RingManager<T>,
) -> usize {
    // Get bottom points for both rings
    let ring1_points = match rings.get(ring1_idx) {
        Some(r) => r.points(),
        None => return ring2_idx,
    };
    let ring2_points = match rings.get(ring2_idx) {
        Some(r) => r.points(),
        None => return ring1_idx,
    };

    let bp1_idx = match get_bottom_point_index(ring1_points) {
        Some(idx) => idx,
        None => return ring2_idx,
    };
    let bp2_idx = match get_bottom_point_index(ring2_points) {
        Some(idx) => idx,
        None => return ring1_idx,
    };

    let pt1 = ring1_points[bp1_idx];
    let pt2 = ring2_points[bp2_idx];

    let y1 = pt1.y.to_f64().unwrap_or(0.0);
    let y2 = pt2.y.to_f64().unwrap_or(0.0);
    let x1 = pt1.x.to_f64().unwrap_or(0.0);
    let x2 = pt2.x.to_f64().unwrap_or(0.0);

    // Larger y = lower in coordinate system
    if y1 > y2 {
        return ring1_idx;
    } else if y1 < y2 {
        return ring2_idx;
    }

    // Same y: prefer smaller x
    if x1 < x2 {
        return ring1_idx;
    } else if x1 > x2 {
        return ring2_idx;
    }

    // Same point: fallback to first_is_bottom_point logic
    // For simplicity, we use area comparison as the C++ does for the final fallback
    let area1 = ring_area(ring1_points);
    let area2 = ring_area(ring2_points);

    // Larger absolute area = more significant ring
    if area1.abs() > area2.abs() {
        ring1_idx
    } else {
        ring2_idx
    }
}

/// Merge two rings when bounds meet at a local maximum.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - append_ring (lines 504-606)
///
/// When two bounds meet at a local maximum, their rings need to be merged.
/// This function:
/// 1. Determines which ring to keep based on hierarchy and bottom point
/// 2. Merges the points from the removed ring into the kept ring
/// 3. Updates parent/child relationships
/// 4. Updates any active bounds that reference the removed ring
///
/// DIVERGENCE FROM WAGYU:
/// - C++ performs complex linked list manipulations with reversal
/// - Rust merges Vec<Coord<T>> by concatenation (with potential reversal)
/// - The side-based merging logic is simplified
///
/// # Arguments
/// * `b1_idx` - Index of the first bound
/// * `b2_idx` - Index of the second bound
/// * `bounds` - Mutable slice of all bounds
/// * `active_bounds` - List of active bound indices
/// * `rings` - The ring manager
pub fn append_ring<T: CoordNum + Copy>(
    b1_idx: usize,
    b2_idx: usize,
    bounds: &mut [Bound<T>],
    active_bounds: &[usize],
    rings: &mut RingManager<T>,
) {
    // DEBUG: Log all bounds' ring assignments before merge
    if crate::debug::debug_enabled() {
        eprintln!("[APPEND_RING_START] b1={} b2={}", b1_idx, b2_idx);
        for &ab_idx in active_bounds {
            eprintln!(
                "  [BOUND_STATE] idx={} ring={:?} side={:?}",
                ab_idx, bounds[ab_idx].ring, bounds[ab_idx].side
            );
        }
    }

    let ring1_idx = match bounds[b1_idx].ring {
        Some(r) => r,
        None => return,
    };
    let ring2_idx = match bounds[b2_idx].ring {
        Some(r) => r,
        None => return,
    };

    if ring1_idx == ring2_idx {
        // Same ring - nothing to merge
        bounds[b1_idx].ring = None;
        bounds[b2_idx].ring = None;
        return;
    }

    // DEBUG: Log ring point counts before merge decision
    if crate::debug::debug_enabled() {
        let r1_pts = rings.get(ring1_idx).map(|r| r.points().len()).unwrap_or(0);
        let r2_pts = rings.get(ring2_idx).map(|r| r.points().len()).unwrap_or(0);
        eprintln!(
            "[APPEND_RING_DECIDE] ring1={} ({} pts) ring2={} ({} pts)",
            ring1_idx, r1_pts, ring2_idx, r2_pts
        );
    }

    // Determine which ring to keep based on hierarchy and bottom point position
    // PORT FROM: C++ append_ring logic (lines 510-530)
    let (keep_ring_idx, keep_bound_idx, remove_ring_idx, remove_bound_idx) =
        if ring1_child_below_ring2(ring1_idx, ring2_idx, rings) {
            if crate::debug::debug_enabled() {
                eprintln!("[APPEND_RING_DECIDE] ring1_child_below_ring2({}, {}) = true", ring1_idx, ring2_idx);
            }
            (ring2_idx, b2_idx, ring1_idx, b1_idx)
        } else if ring1_child_below_ring2(ring2_idx, ring1_idx, rings) {
            if crate::debug::debug_enabled() {
                eprintln!("[APPEND_RING_DECIDE] ring1_child_below_ring2({}, {}) = true", ring2_idx, ring1_idx);
            }
            (ring1_idx, b1_idx, ring2_idx, b2_idx)
        } else if ring1_idx == get_lower_most_ring(ring1_idx, ring2_idx, rings) {
            if crate::debug::debug_enabled() {
                eprintln!("[APPEND_RING_DECIDE] get_lower_most_ring({}, {}) = {}", ring1_idx, ring2_idx, ring1_idx);
            }
            // Use get_lower_most_ring to determine which ring to keep
            (ring1_idx, b1_idx, ring2_idx, b2_idx)
        } else {
            if crate::debug::debug_enabled() {
                eprintln!("[APPEND_RING_DECIDE] get_lower_most_ring({}, {}) = {}", ring1_idx, ring2_idx, ring2_idx);
            }
            (ring2_idx, b2_idx, ring1_idx, b1_idx)
        };

    // Get the points from the ring to remove
    let remove_points: Vec<Coord<T>> = match rings.get(remove_ring_idx) {
        Some(r) => r.points().to_vec(),
        None => Vec::new(),
    };

    // Get the sides for merging logic
    let keep_side = bounds[keep_bound_idx].side;
    let remove_side = bounds[remove_bound_idx].side;

    // Merge points based on sides
    // PORT FROM: C++ append_ring point merging (lines 544-579)
    // The side combination determines how points are concatenated
    if let Some(keep_ring) = rings.get_mut(keep_ring_idx) {
        let keep_points = keep_ring.points_mut();

        match (keep_side, remove_side) {
            (EdgeSide::Left, EdgeSide::Left) => {
                // C++: z y x a b c - reverse remove, prepend to keep
                let mut reversed: Vec<_> = remove_points.into_iter().rev().collect();
                reversed.append(keep_points);
                *keep_points = reversed;
            }
            (EdgeSide::Left, EdgeSide::Right) => {
                // C++: x y z a b c - prepend remove to keep
                let mut new_points = remove_points;
                new_points.append(keep_points);
                *keep_points = new_points;
            }
            (EdgeSide::Right, EdgeSide::Right) => {
                // C++: a b c z y x - reverse remove, append to keep
                let reversed: Vec<_> = remove_points.into_iter().rev().collect();
                keep_points.extend(reversed);
            }
            (EdgeSide::Right, EdgeSide::Left) => {
                // C++: a b c x y z - append remove to keep
                keep_points.extend(remove_points);
            }
        }
    }

    // Determine if rings are holes based on their area
    // PORT FROM: C++ append_ring (lines 581-587)
    let keep_is_hole = rings.ring_is_hole(keep_ring_idx);
    let remove_is_hole = rings.ring_is_hole(remove_ring_idx);

    // Get the kept ring's parent for hole replacement logic
    let keep_parent = rings.get(keep_ring_idx).and_then(|r| r.parent());

    // Use ring1_replaces_ring2 with proper hole handling
    // If the rings have different hole status, target the parent
    if keep_is_hole != remove_is_hole {
        rings.ring1_replaces_ring2(keep_parent, remove_ring_idx);
    } else {
        rings.ring1_replaces_ring2(Some(keep_ring_idx), remove_ring_idx);
    }

    // Clear ring references on both bounds
    // PORT FROM: C++ append_ring (lines 596-597)
    bounds[keep_bound_idx].ring = None;
    bounds[remove_bound_idx].ring = None;

    // Update any other active bounds that reference the removed ring
    for &ab_idx in active_bounds {
        if bounds[ab_idx].ring == Some(remove_ring_idx) {
            if crate::debug::debug_enabled() {
                eprintln!(
                    "[BOUND_UPDATE] append_ring: ab_idx={} ring: {} -> {}, side: {:?}",
                    ab_idx, remove_ring_idx, keep_ring_idx, keep_side
                );
            }
            bounds[ab_idx].ring = Some(keep_ring_idx);
            bounds[ab_idx].side = keep_side;
            // FIX #88: Restore C++ break - only update ONE bound per merge
            // The C++ break exists because at most one OTHER bound references
            // the removed ring (two bounds form one ring, two are already nulled).
            // Updating all causes cascading ring reassignments.
            break;
        }
    }

    // DEBUG: Log all bounds' ring assignments after merge
    if crate::debug::debug_enabled() {
        eprintln!("[APPEND_RING_END] keep={} remove={}", keep_ring_idx, remove_ring_idx);
        for &ab_idx in active_bounds {
            eprintln!(
                "  [BOUND_STATE] idx={} ring={:?} side={:?}",
                ab_idx, bounds[ab_idx].ring, bounds[ab_idx].side
            );
        }
    }
}

/// Handle a local maximum by adding the final point and merging rings.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - add_local_maximum_point (lines 609-625)
///
/// At a local maximum, two bounds meet at their ending point. This function:
/// 1. Adds the maximum point to the first bound
/// 2. If both bounds share the same ring, closes it
/// 3. Otherwise, merges the two rings with `append_ring`
///
/// # Arguments
/// * `b1_idx` - Index of the first bound
/// * `b2_idx` - Index of the second bound
/// * `bounds` - Mutable slice of all bounds
/// * `active_bounds` - List of active bound indices
/// * `pt` - The local maximum point
/// * `rings` - The ring manager
pub fn add_local_maximum_point<T: CoordNum + Copy>(
    b1_idx: usize,
    b2_idx: usize,
    bounds: &mut [Bound<T>],
    active_bounds: &[usize],
    pt: Coord<T>,
    rings: &mut RingManager<T>,
) {
    // Add point to first bound
    add_point(b1_idx, bounds, active_bounds, pt, rings);

    let ring1 = bounds[b1_idx].ring;
    let ring2 = bounds[b2_idx].ring;

    if ring1 == ring2 {
        // Same ring - just close it by clearing references
        bounds[b1_idx].ring = None;
        bounds[b2_idx].ring = None;
    } else {
        // Different rings - need to merge
        // Order by ring index (lower first)
        match (ring1, ring2) {
            (Some(r1), Some(r2)) if r1 < r2 => {
                append_ring(b1_idx, b2_idx, bounds, active_bounds, rings);
            }
            (Some(_), Some(_)) => {
                append_ring(b2_idx, b1_idx, bounds, active_bounds, rings);
            }
            _ => {
                // One or both rings are None - just clear
                bounds[b1_idx].ring = None;
                bounds[b2_idx].ring = None;
            }
        }
    }
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
