//! Snap rounding utilities for floating-point precision handling.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/snap_rounding.hpp
//!            wagyu/include/mapbox/geometry/wagyu/util.hpp (rounding functions)
//!
//! This module provides functions for rounding floating-point coordinates to
//! integer grid points. This is essential for the Vatti polygon clipping algorithm
//! to handle floating-point precision issues and ensure consistent results.
//!
//! ## Key Functions
//!
//! - [`wround`]: Basic rounding (equivalent to `llround`)
//! - [`round_towards_min`]: Rounds towards minimum (0.5 rounds down)
//! - [`round_towards_max`]: Rounds towards maximum (0.5 rounds up)
//! - [`round_point`]: Rounds a floating-point point to integer coordinates
//! - [`values_are_equal`]: ULP-based floating-point comparison
//!
//! ## Hot Pixels
//!
//! The snap rounding algorithm uses "hot pixels" - grid points where edges intersect
//! or where special handling is needed. These are collected during a pre-processing
//! pass and used during the main clipping algorithm.

use crate::point::Point;
use geo_types::CoordNum;

/// Maximum ULPs (Units in the Last Place) for floating-point comparison.
///
/// This determines how close two floating-point numbers need to be to be
/// considered equal. A value of 4 is typically sufficient for geometry
/// operations where accumulated floating-point errors are minimal.
const MAX_ULPS: u64 = 4;

/// Compares two floating-point numbers for approximate equality using ULP comparison.
///
/// Two numbers are considered equal if their IEEE 754 bit representations are
/// within `MAX_ULPS` of each other. This handles edge cases like comparing
/// values near zero and accounts for floating-point representation errors.
///
/// # Arguments
///
/// * `x` - First floating-point value
/// * `y` - Second floating-point value
///
/// # Returns
///
/// `true` if the values are approximately equal, `false` otherwise.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::values_are_equal;
///
/// assert!(values_are_equal(1.0, 1.0));
/// assert!(values_are_equal(0.0, 0.0));
/// assert!(!values_are_equal(1.0, 2.0));
///
/// // Handles values very close together
/// let a = 0.1 + 0.2;
/// let b = 0.3;
/// // These may not be exactly equal due to floating-point representation
/// // but are considered equal by ULP comparison
/// ```
pub fn values_are_equal(x: f64, y: f64) -> bool {
    // Handle NaN - NaN is never equal to anything, including itself
    if x.is_nan() || y.is_nan() {
        return false;
    }

    let x_bits = x.to_bits();
    let y_bits = y.to_bits();

    // Convert to sign-and-magnitude representation for comparison
    let x_biased = sign_and_magnitude_to_biased(x_bits);
    let y_biased = sign_and_magnitude_to_biased(y_bits);

    // Calculate the distance in ULPs
    let distance = x_biased.abs_diff(y_biased);

    distance <= MAX_ULPS
}

/// Converts IEEE 754 sign-and-magnitude representation to a biased representation
/// for easier distance calculation.
///
/// In biased representation, negative numbers are mapped to small positive integers
/// and positive numbers are mapped to larger integers, making distance calculation
/// simpler.
#[inline]
fn sign_and_magnitude_to_biased(bits: u64) -> u64 {
    const SIGN_BIT_MASK: u64 = 1 << 63;

    if (bits & SIGN_BIT_MASK) != 0 {
        // Negative number: flip all bits and add 1 (two's complement)
        !bits + 1
    } else {
        // Positive number: set the sign bit
        bits | SIGN_BIT_MASK
    }
}

/// Returns `true` if the value is approximately zero.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::value_is_zero;
///
/// assert!(value_is_zero(0.0));
/// assert!(value_is_zero(-0.0));
/// assert!(!value_is_zero(1.0));
/// ```
#[inline]
pub fn value_is_zero(val: f64) -> bool {
    values_are_equal(val, 0.0)
}

/// Returns `true` if `x >= y` using approximate floating-point comparison.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::greater_than_or_equal;
///
/// assert!(greater_than_or_equal(2.0, 1.0));
/// assert!(greater_than_or_equal(1.0, 1.0));
/// assert!(!greater_than_or_equal(0.5, 1.0));
/// ```
#[inline]
pub fn greater_than_or_equal(x: f64, y: f64) -> bool {
    x > y || values_are_equal(x, y)
}

/// Returns `true` if `x > y` using approximate floating-point comparison.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::greater_than;
///
/// assert!(greater_than(2.0, 1.0));
/// assert!(!greater_than(1.0, 1.0));
/// assert!(!greater_than(0.5, 1.0));
/// ```
#[inline]
pub fn greater_than(x: f64, y: f64) -> bool {
    !values_are_equal(x, y) && x > y
}

/// Returns `true` if `x < y` using approximate floating-point comparison.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::less_than;
///
/// assert!(less_than(1.0, 2.0));
/// assert!(!less_than(1.0, 1.0));
/// assert!(!less_than(2.0, 1.0));
/// ```
#[inline]
pub fn less_than(x: f64, y: f64) -> bool {
    !values_are_equal(x, y) && x < y
}

/// Rounds a floating-point value to the nearest integer.
///
/// This is equivalent to the C++ `llround` function - it rounds to the
/// nearest integer, with halfway cases (0.5) rounding away from zero.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::wround;
///
/// assert_eq!(wround(1.4), 1_i64);
/// assert_eq!(wround(1.5), 2_i64);
/// assert_eq!(wround(1.6), 2_i64);
/// assert_eq!(wround(-1.5), -2_i64);
/// ```
#[inline]
pub fn wround(value: f64) -> i64 {
    value.round() as i64
}

/// Rounds a floating-point value towards the minimum (floor direction for ties).
///
/// For values exactly at the halfway point (x.5), this rounds towards negative
/// infinity (floor). For other values, it uses standard rounding.
///
/// This is used when calculating minimum bounds of edges to ensure the
/// result is conservative (never overestimates).
///
/// # Rounding Behavior
///
/// - `0.5` rounds to `0`
/// - `0.0` rounds to `0`
/// - `-0.5` rounds to `-1`
/// - `1.4` rounds to `1`
/// - `1.6` rounds to `2`
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::round_towards_min;
///
/// assert_eq!(round_towards_min(0.5), 0_i64);
/// assert_eq!(round_towards_min(0.0), 0_i64);
/// assert_eq!(round_towards_min(-0.5), -1_i64);
/// assert_eq!(round_towards_min(1.4), 1_i64);
/// assert_eq!(round_towards_min(1.6), 2_i64);
/// ```
pub fn round_towards_min(val: f64) -> i64 {
    let half = val.floor() + 0.5;
    if values_are_equal(val, half) {
        val.floor() as i64
    } else {
        val.round() as i64
    }
}

/// Rounds a floating-point value towards the maximum (ceiling direction for ties).
///
/// For values exactly at the halfway point (x.5), this rounds towards positive
/// infinity (ceiling). For other values, it uses standard rounding.
///
/// This is used when calculating maximum bounds of edges to ensure the
/// result is conservative (never underestimates).
///
/// # Rounding Behavior
///
/// - `0.5` rounds to `1`
/// - `0.0` rounds to `0`
/// - `-0.5` rounds to `0`
/// - `1.4` rounds to `1`
/// - `1.6` rounds to `2`
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::round_towards_max;
///
/// assert_eq!(round_towards_max(0.5), 1_i64);
/// assert_eq!(round_towards_max(0.0), 0_i64);
/// assert_eq!(round_towards_max(-0.5), 0_i64);
/// assert_eq!(round_towards_max(1.4), 1_i64);
/// assert_eq!(round_towards_max(1.6), 2_i64);
/// ```
pub fn round_towards_max(val: f64) -> i64 {
    let half = val.floor() + 0.5;
    if values_are_equal(val, half) {
        val.ceil() as i64
    } else {
        val.round() as i64
    }
}

/// Rounds a floating-point point to integer coordinates.
///
/// Uses `round_towards_max` for both x and y coordinates to ensure consistent
/// snapping behavior at grid boundaries.
///
/// # Examples
///
/// ```
/// use wagyu_rs::snap_rounding::round_point;
/// use wagyu_rs::Point;
///
/// let float_pt: Point<f64> = Point::new(1.5, 2.5);
/// let int_pt: Point<i64> = round_point(&float_pt);
/// assert_eq!(int_pt.x, 2);
/// assert_eq!(int_pt.y, 3);
/// ```
pub fn round_point<T>(pt: &Point<f64>) -> Point<T>
where
    T: CoordNum + From<i64>,
{
    Point::new(
        <T as From<i64>>::from(round_towards_max(pt.x)),
        <T as From<i64>>::from(round_towards_max(pt.y)),
    )
}

// ============================================================================
// Hot Pixel Helper Functions
// PORT FROM: wagyu/include/mapbox/geometry/wagyu/snap_rounding.hpp
// ============================================================================

use crate::bound::Bound;
use crate::bubble_sort::bubble_sort;
use crate::intersect_util::get_edge_intersection;
use crate::local_minimum::LocalMinimum;
use crate::Scanbeam;
use num_traits::ToPrimitive;

/// Add a point to the hot pixels list.
///
/// PORT FROM: wagyu ring_util.hpp - add_to_hot_pixels
#[inline]
fn add_to_hot_pixels<T: CoordNum>(pt: Point<T>, manager: &mut crate::build_result::RingManager<T>) {
    manager.hot_pixels.push(pt);
}

/// Sort and deduplicate hot pixels.
///
/// PORT FROM: wagyu ring_util.hpp - sort_hot_pixels
///
/// Sorts by Y descending (higher Y first), then X ascending.
/// Removes duplicate points.
fn sort_hot_pixels<T: CoordNum>(manager: &mut crate::build_result::RingManager<T>) {
    // Sort: Y descending, X ascending
    manager.hot_pixels.sort_by(|a, b| {
        let a_y = a.y.to_f64().unwrap_or(0.0);
        let b_y = b.y.to_f64().unwrap_or(0.0);
        let a_x = a.x.to_f64().unwrap_or(0.0);
        let b_x = b.x.to_f64().unwrap_or(0.0);

        // Y descending (b.y compared to a.y for descending)
        match b_y.partial_cmp(&a_y) {
            Some(std::cmp::Ordering::Equal) => {
                // X ascending for same Y
                a_x.partial_cmp(&b_x).unwrap_or(std::cmp::Ordering::Equal)
            }
            Some(ord) => ord,
            None => std::cmp::Ordering::Equal,
        }
    });

    // Remove duplicates (std::unique equivalent)
    manager.hot_pixels.dedup_by(|a, b| {
        let a_x = a.x.to_f64().unwrap_or(0.0);
        let b_x = b.x.to_f64().unwrap_or(0.0);
        let a_y = a.y.to_f64().unwrap_or(0.0);
        let b_y = b.y.to_f64().unwrap_or(0.0);
        (a_x - b_x).abs() < f64::EPSILON && (a_y - b_y).abs() < f64::EPSILON
    });
}

/// Setup scanbeam with Y coordinates from local minima.
///
/// PORT FROM: wagyu local_minimum_util.hpp - setup_scanbeam
fn setup_scanbeam<T: CoordNum>(minima_list: &[LocalMinimum<T>], scanbeam: &mut Scanbeam<T>) {
    for lm in minima_list {
        scanbeam.insert(lm.y);
        // Add top Y of first edges in both bounds
        if !lm.left_bound.edges.is_empty() {
            scanbeam.insert(lm.left_bound.edges[0].top.y);
        }
        if !lm.right_bound.edges.is_empty() {
            scanbeam.insert(lm.right_bound.edges[0].top.y);
        }
    }
}

/// Bound state for hot pixel sweep (simplified from full Vatti bound).
///
/// PORT FROM: wagyu bound.hpp - relevant fields for hot pixel collection
#[derive(Debug, Clone)]
struct HotPixelBound<T: CoordNum> {
    /// Index of the current edge in the edges vector
    current_edge_idx: usize,
    /// All edges in this bound
    edges: Vec<crate::bound::Edge<T>>,
    /// Current X position at sweep line
    current_x: f64,
}

impl<T: CoordNum> HotPixelBound<T> {
    fn from_bound(bound: &Bound<T>) -> Self {
        let current_x = if !bound.edges.is_empty() {
            bound.edges[0].bot.x.to_f64().unwrap_or(0.0)
        } else {
            0.0
        };
        Self {
            current_edge_idx: 0,
            edges: bound.edges.clone(),
            current_x,
        }
    }

    fn current_edge(&self) -> Option<&crate::bound::Edge<T>> {
        self.edges.get(self.current_edge_idx)
    }

    fn is_at_end(&self) -> bool {
        self.current_edge_idx >= self.edges.len()
    }

    fn advance_edge(&mut self, scanbeam: &mut Scanbeam<T>) {
        if self.current_edge_idx + 1 < self.edges.len() {
            self.current_edge_idx += 1;
            if let Some(edge) = self.current_edge() {
                scanbeam.insert(edge.top.y);
            }
        } else {
            self.current_edge_idx = self.edges.len(); // Mark as ended
        }
    }
}

/// Update current_x for all active bounds at a given Y.
///
/// PORT FROM: wagyu active_bound_list.hpp - update_current_x
fn update_current_x<T: CoordNum>(bounds: &mut [HotPixelBound<T>], y: T) {
    let y_f64 = y.to_f64().unwrap_or(0.0);
    for bound in bounds.iter_mut() {
        if let Some(edge) = bound.current_edge() {
            if edge.is_horizontal() {
                bound.current_x = edge.bot.x.to_f64().unwrap_or(0.0);
            } else {
                let bot_x = edge.bot.x.to_f64().unwrap_or(0.0);
                let bot_y = edge.bot.y.to_f64().unwrap_or(0.0);
                bound.current_x = bot_x + edge.dx * (y_f64 - bot_y);
            }
        }
    }
}

/// Process intersections at current scanline using bubble sort.
///
/// PORT FROM: wagyu snap_rounding.hpp - process_hot_pixel_intersections
#[allow(clippy::ptr_arg)]
fn process_hot_pixel_intersections<T: CoordNum + ToPrimitive>(
    _top_y: T,
    active_bounds: &mut Vec<HotPixelBound<T>>,
    manager: &mut crate::build_result::RingManager<T>,
) {
    if active_bounds.is_empty() {
        return;
    }

    // Update current_x for all bounds
    update_current_x(active_bounds, _top_y);

    // Bubble sort, recording intersections when swapping
    // PORT FROM: hp_intersection_swap callback
    bubble_sort(
        active_bounds,
        |a, b| a.current_x < b.current_x,
        |b1, b2| {
            // When swapping, calculate intersection point and add to hot pixels
            if let (Some(e1), Some(e2)) = (b1.current_edge(), b2.current_edge()) {
                if let Some(pt) = get_edge_intersection(e1, e2) {
                    // Round to coordinate type
                    let x = T::from(pt.x.round()).unwrap_or_else(T::zero);
                    let y = T::from(pt.y.round()).unwrap_or_else(T::zero);
                    add_to_hot_pixels(Point::new(x, y), manager);
                }
            }
        },
    );
}

/// Insert local minima into active bounds at the current scanline Y.
///
/// PORT FROM: wagyu snap_rounding.hpp - insert_local_minima_into_ABL_hot_pixel
fn insert_local_minima_into_abl_hot_pixel<T: CoordNum>(
    top_y: T,
    minima_sorted: &[&LocalMinimum<T>],
    current_lm_idx: &mut usize,
    active_bounds: &mut Vec<HotPixelBound<T>>,
    manager: &mut crate::build_result::RingManager<T>,
    scanbeam: &mut Scanbeam<T>,
) {
    let top_y_f64 = top_y.to_f64().unwrap_or(0.0);

    while *current_lm_idx < minima_sorted.len() {
        let lm = minima_sorted[*current_lm_idx];
        let lm_y_f64 = lm.y.to_f64().unwrap_or(0.0);

        if (lm_y_f64 - top_y_f64).abs() > f64::EPSILON {
            break;
        }

        // Add the local minimum point to hot pixels
        if !lm.left_bound.edges.is_empty() {
            add_to_hot_pixels(lm.left_bound.edges[0].bot, manager);
        }

        // Create bound states
        let left_bound = HotPixelBound::from_bound(&lm.left_bound);
        let right_bound = HotPixelBound::from_bound(&lm.right_bound);

        // Add edge tops to scanbeam
        if let Some(edge) = left_bound.current_edge() {
            if !edge.is_horizontal() {
                scanbeam.insert(edge.top.y);
            }
        }
        if let Some(edge) = right_bound.current_edge() {
            if !edge.is_horizontal() {
                scanbeam.insert(edge.top.y);
            }
        }

        // Find insertion position and insert bounds
        let insert_pos = active_bounds
            .iter()
            .position(|b| b.current_x > left_bound.current_x)
            .unwrap_or(active_bounds.len());

        active_bounds.insert(insert_pos, left_bound);
        active_bounds.insert(insert_pos + 1, right_bound);

        *current_lm_idx += 1;
    }
}

/// Process edges at top of scanbeam.
///
/// PORT FROM: wagyu snap_rounding.hpp - process_hot_pixel_edges_at_top_of_scanbeam
fn process_hot_pixel_edges_at_top_of_scanbeam<T: CoordNum>(
    top_y: T,
    scanbeam: &mut Scanbeam<T>,
    active_bounds: &mut Vec<HotPixelBound<T>>,
    manager: &mut crate::build_result::RingManager<T>,
) {
    let top_y_f64 = top_y.to_f64().unwrap_or(0.0);

    // First pass: collect horizontal edge info and edge tops
    let mut horizontal_ranges: Vec<(usize, f64, f64)> = Vec::new(); // (bound_idx, min_x, max_x)
    let mut edge_tops_to_add: Vec<Point<T>> = Vec::new();

    for (idx, bound) in active_bounds.iter().enumerate() {
        if let Some(edge) = bound.current_edge() {
            let edge_top_y = edge.top.y.to_f64().unwrap_or(0.0);
            if (edge_top_y - top_y_f64).abs() <= f64::EPSILON {
                edge_tops_to_add.push(edge.top);

                if edge.is_horizontal() {
                    let edge_top_x = edge.top.x.to_f64().unwrap_or(0.0);
                    let edge_bot_x = edge.bot.x.to_f64().unwrap_or(0.0);
                    horizontal_ranges.push((
                        idx,
                        edge_bot_x.min(edge_top_x),
                        edge_bot_x.max(edge_top_x),
                    ));
                }
            }
        }
    }

    // Add edge tops to hot pixels
    for pt in edge_tops_to_add {
        add_to_hot_pixels(pt, manager);
    }

    // Handle horizontal edges - check intersections with other bounds
    for (horiz_idx, min_x, max_x) in &horizontal_ranges {
        for (other_idx, other_bound) in active_bounds.iter().enumerate() {
            if other_idx == *horiz_idx {
                continue;
            }
            if let Some(other_edge) = other_bound.current_edge() {
                let other_top_y = other_edge.top.y.to_f64().unwrap_or(0.0);
                let other_bot_y = other_edge.bot.y.to_f64().unwrap_or(0.0);
                // Skip if other edge starts or ends at this Y
                if (other_top_y - top_y_f64).abs() < f64::EPSILON
                    || (other_bot_y - top_y_f64).abs() < f64::EPSILON
                {
                    continue;
                }
                if other_bound.current_x >= *min_x && other_bound.current_x <= *max_x {
                    let x = T::from(other_bound.current_x.round()).unwrap_or_else(T::zero);
                    add_to_hot_pixels(Point::new(x, top_y), manager);
                }
            }
        }
    }

    // Second pass: advance edges and mark bounds for removal
    let mut to_remove = Vec::new();
    for (idx, bound) in active_bounds.iter_mut().enumerate() {
        while let Some(edge) = bound.current_edge() {
            let edge_top_y = edge.top.y.to_f64().unwrap_or(0.0);
            if (edge_top_y - top_y_f64).abs() > f64::EPSILON {
                break;
            }
            bound.advance_edge(scanbeam);
        }

        if bound.is_at_end() {
            to_remove.push(idx);
        }
    }

    // Remove completed bounds (in reverse order to preserve indices)
    for idx in to_remove.into_iter().rev() {
        active_bounds.remove(idx);
    }
}

/// Build the hot pixels list from local minima.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/snap_rounding.hpp - build_hot_pixels
///
/// This performs a full sweep-line simulation to collect all hot pixels:
/// - Vertex coordinates from edges
/// - Intersection points between edges
/// - Points where horizontal edges cross other edges
///
/// # Algorithm
///
/// 1. Sort local minima by Y coordinate (descending)
/// 2. Setup scanbeam with Y coordinates from all edges
/// 3. Process scanbeam from top to bottom:
///    - Find and record edge intersections
///    - Insert new local minima into active bounds
///    - Process edges reaching their top at current Y
/// 4. Sort and deduplicate hot pixels
///
/// # Arguments
///
/// * `minima_list` - List of local minima containing polygon edges
/// * `manager` - Ring manager to store hot pixels in
pub fn build_hot_pixels<T: CoordNum + ToPrimitive>(
    minima_list: &crate::local_minimum::LocalMinimumList<T>,
    manager: &mut crate::build_result::RingManager<T>,
) {
    manager.hot_pixels.clear();

    if minima_list.is_empty() {
        manager.current_hp_idx = 0;
        return;
    }

    // Create sorted list of minima references (by Y descending)
    // PORT FROM: local_minimum_sorter
    let mut minima_sorted: Vec<&LocalMinimum<T>> = minima_list.iter().collect();
    minima_sorted.sort_by(|a, b| LocalMinimum::compare(a, b));

    // Setup scanbeam
    let mut scanbeam: Scanbeam<T> = Scanbeam::new();
    setup_scanbeam(minima_list, &mut scanbeam);

    // Reserve estimated capacity
    let reserve: usize = minima_list
        .iter()
        .map(|lm| lm.left_bound.edges.len() + lm.right_bound.edges.len() + 4)
        .sum();
    manager.hot_pixels.reserve(reserve);

    // Active bounds list
    let mut active_bounds: Vec<HotPixelBound<T>> = Vec::new();
    let mut current_lm_idx = 0;

    // Main sweep loop
    while let Some(scanline_y) = scanbeam.pop() {
        // Process intersections at this scanline
        process_hot_pixel_intersections(scanline_y, &mut active_bounds, manager);

        // Insert local minima at this Y
        insert_local_minima_into_abl_hot_pixel(
            scanline_y,
            &minima_sorted,
            &mut current_lm_idx,
            &mut active_bounds,
            manager,
            &mut scanbeam,
        );

        // Process edges at top of scanbeam
        process_hot_pixel_edges_at_top_of_scanbeam(
            scanline_y,
            &mut scanbeam,
            &mut active_bounds,
            manager,
        );
    }

    // Process any remaining minima
    while current_lm_idx < minima_sorted.len() {
        let lm = minima_sorted[current_lm_idx];
        if !lm.left_bound.edges.is_empty() {
            add_to_hot_pixels(lm.left_bound.edges[0].bot, manager);
        }
        for edge in &lm.left_bound.edges {
            add_to_hot_pixels(edge.top, manager);
        }
        for edge in &lm.right_bound.edges {
            add_to_hot_pixels(edge.top, manager);
        }
        current_lm_idx += 1;
    }

    // Sort and deduplicate
    sort_hot_pixels(manager);

    // Reset index for sweep processing
    manager.current_hp_idx = 0;
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // values_are_equal tests
    // =========================================================================

    #[test]
    fn test_values_are_equal_identical_values() {
        assert!(values_are_equal(1.0, 1.0));
        assert!(values_are_equal(0.0, 0.0));
        assert!(values_are_equal(-1.0, -1.0));
        assert!(values_are_equal(123456.789, 123456.789));
    }

    #[test]
    fn test_values_are_equal_positive_and_negative_zero() {
        assert!(values_are_equal(0.0, -0.0));
        assert!(values_are_equal(-0.0, 0.0));
    }

    #[test]
    fn test_values_are_equal_nan_is_never_equal() {
        assert!(!values_are_equal(f64::NAN, f64::NAN));
        assert!(!values_are_equal(f64::NAN, 0.0));
        assert!(!values_are_equal(0.0, f64::NAN));
    }

    #[test]
    fn test_values_are_equal_different_values() {
        assert!(!values_are_equal(1.0, 2.0));
        assert!(!values_are_equal(0.0, 1.0));
        assert!(!values_are_equal(-1.0, 1.0));
    }

    #[test]
    fn test_values_are_equal_very_close_values() {
        // Values that differ by only a few ULPs should be equal
        let a: f64 = 1.0;
        let b = f64::from_bits(a.to_bits() + 1); // 1 ULP difference
        assert!(values_are_equal(a, b));

        let c = f64::from_bits(a.to_bits() + 4); // 4 ULPs difference
        assert!(values_are_equal(a, c));
    }

    #[test]
    fn test_values_are_equal_beyond_ulp_threshold() {
        let a: f64 = 1.0;
        let b = f64::from_bits(a.to_bits() + 10); // 10 ULPs difference
        assert!(!values_are_equal(a, b));
    }

    #[test]
    fn test_values_are_equal_infinity() {
        assert!(values_are_equal(f64::INFINITY, f64::INFINITY));
        assert!(values_are_equal(f64::NEG_INFINITY, f64::NEG_INFINITY));
        assert!(!values_are_equal(f64::INFINITY, f64::NEG_INFINITY));
    }

    // =========================================================================
    // value_is_zero tests
    // =========================================================================

    #[test]
    fn test_value_is_zero_positive_zero() {
        assert!(value_is_zero(0.0));
    }

    #[test]
    fn test_value_is_zero_negative_zero() {
        assert!(value_is_zero(-0.0));
    }

    #[test]
    fn test_value_is_zero_non_zero_values() {
        assert!(!value_is_zero(1.0));
        assert!(!value_is_zero(-1.0));
        assert!(!value_is_zero(0.1));
        assert!(!value_is_zero(1e-100));
    }

    // =========================================================================
    // greater_than_or_equal tests
    // =========================================================================

    #[test]
    fn test_greater_than_or_equal_greater() {
        assert!(greater_than_or_equal(2.0, 1.0));
        assert!(greater_than_or_equal(1.0, 0.0));
        assert!(greater_than_or_equal(0.0, -1.0));
    }

    #[test]
    fn test_greater_than_or_equal_equal() {
        assert!(greater_than_or_equal(1.0, 1.0));
        assert!(greater_than_or_equal(0.0, 0.0));
        assert!(greater_than_or_equal(-1.0, -1.0));
    }

    #[test]
    fn test_greater_than_or_equal_less() {
        assert!(!greater_than_or_equal(1.0, 2.0));
        assert!(!greater_than_or_equal(0.0, 1.0));
        assert!(!greater_than_or_equal(-1.0, 0.0));
    }

    // =========================================================================
    // greater_than tests
    // =========================================================================

    #[test]
    fn test_greater_than_greater() {
        assert!(greater_than(2.0, 1.0));
        assert!(greater_than(1.0, 0.0));
        assert!(greater_than(0.0, -1.0));
    }

    #[test]
    fn test_greater_than_equal_returns_false() {
        assert!(!greater_than(1.0, 1.0));
        assert!(!greater_than(0.0, 0.0));
    }

    #[test]
    fn test_greater_than_less_returns_false() {
        assert!(!greater_than(1.0, 2.0));
        assert!(!greater_than(0.0, 1.0));
    }

    // =========================================================================
    // less_than tests
    // =========================================================================

    #[test]
    fn test_less_than_less() {
        assert!(less_than(1.0, 2.0));
        assert!(less_than(0.0, 1.0));
        assert!(less_than(-1.0, 0.0));
    }

    #[test]
    fn test_less_than_equal_returns_false() {
        assert!(!less_than(1.0, 1.0));
        assert!(!less_than(0.0, 0.0));
    }

    #[test]
    fn test_less_than_greater_returns_false() {
        assert!(!less_than(2.0, 1.0));
        assert!(!less_than(1.0, 0.0));
    }

    // =========================================================================
    // wround tests
    // =========================================================================

    #[test]
    fn test_wround_positive_values() {
        assert_eq!(wround(1.4), 1);
        assert_eq!(wround(1.5), 2);
        assert_eq!(wround(1.6), 2);
        assert_eq!(wround(2.5), 3);
    }

    #[test]
    fn test_wround_negative_values() {
        assert_eq!(wround(-1.4), -1);
        assert_eq!(wround(-1.5), -2);
        assert_eq!(wround(-1.6), -2);
        assert_eq!(wround(-2.5), -3);
    }

    #[test]
    fn test_wround_zero() {
        assert_eq!(wround(0.0), 0);
        assert_eq!(wround(-0.0), 0);
    }

    #[test]
    fn test_wround_whole_numbers() {
        assert_eq!(wround(5.0), 5);
        assert_eq!(wround(-5.0), -5);
        assert_eq!(wround(100.0), 100);
    }

    // =========================================================================
    // round_towards_min tests
    // =========================================================================

    #[test]
    fn test_round_towards_min_half_rounds_down() {
        // Key behavior: 0.5 rounds towards floor
        assert_eq!(round_towards_min(0.5), 0);
        assert_eq!(round_towards_min(1.5), 1);
        assert_eq!(round_towards_min(2.5), 2);
    }

    #[test]
    fn test_round_towards_min_negative_half_rounds_down() {
        // -0.5 should round to -1 (floor)
        assert_eq!(round_towards_min(-0.5), -1);
        assert_eq!(round_towards_min(-1.5), -2);
        assert_eq!(round_towards_min(-2.5), -3);
    }

    #[test]
    fn test_round_towards_min_non_half_uses_standard_rounding() {
        assert_eq!(round_towards_min(1.4), 1);
        assert_eq!(round_towards_min(1.6), 2);
        assert_eq!(round_towards_min(-1.4), -1);
        assert_eq!(round_towards_min(-1.6), -2);
    }

    #[test]
    fn test_round_towards_min_zero() {
        assert_eq!(round_towards_min(0.0), 0);
        assert_eq!(round_towards_min(-0.0), 0);
    }

    // =========================================================================
    // round_towards_max tests
    // =========================================================================

    #[test]
    fn test_round_towards_max_half_rounds_up() {
        // Key behavior: 0.5 rounds towards ceiling
        assert_eq!(round_towards_max(0.5), 1);
        assert_eq!(round_towards_max(1.5), 2);
        assert_eq!(round_towards_max(2.5), 3);
    }

    #[test]
    fn test_round_towards_max_negative_half_rounds_up() {
        // -0.5 should round to 0 (ceiling)
        assert_eq!(round_towards_max(-0.5), 0);
        assert_eq!(round_towards_max(-1.5), -1);
        assert_eq!(round_towards_max(-2.5), -2);
    }

    #[test]
    fn test_round_towards_max_non_half_uses_standard_rounding() {
        assert_eq!(round_towards_max(1.4), 1);
        assert_eq!(round_towards_max(1.6), 2);
        assert_eq!(round_towards_max(-1.4), -1);
        assert_eq!(round_towards_max(-1.6), -2);
    }

    #[test]
    fn test_round_towards_max_zero() {
        assert_eq!(round_towards_max(0.0), 0);
        assert_eq!(round_towards_max(-0.0), 0);
    }

    // =========================================================================
    // round_point tests
    // =========================================================================

    #[test]
    fn test_round_point_positive_coordinates() {
        let pt: Point<f64> = Point::new(1.5, 2.5);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 2); // 1.5 -> 2 (round_towards_max)
        assert_eq!(rounded.y, 3); // 2.5 -> 3 (round_towards_max)
    }

    #[test]
    fn test_round_point_negative_coordinates() {
        let pt: Point<f64> = Point::new(-1.5, -2.5);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, -1); // -1.5 -> -1 (round_towards_max)
        assert_eq!(rounded.y, -2); // -2.5 -> -2 (round_towards_max)
    }

    #[test]
    fn test_round_point_mixed_coordinates() {
        let pt: Point<f64> = Point::new(-0.5, 0.5);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 0); // -0.5 -> 0 (round_towards_max)
        assert_eq!(rounded.y, 1); // 0.5 -> 1 (round_towards_max)
    }

    #[test]
    fn test_round_point_whole_numbers() {
        let pt: Point<f64> = Point::new(5.0, 10.0);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 5);
        assert_eq!(rounded.y, 10);
    }

    #[test]
    fn test_round_point_non_half_values() {
        let pt: Point<f64> = Point::new(1.4, 2.6);
        let rounded: Point<i64> = round_point(&pt);
        assert_eq!(rounded.x, 1); // standard rounding
        assert_eq!(rounded.y, 3); // standard rounding
    }

    // =========================================================================
    // Symmetry tests (round_towards_min vs round_towards_max)
    // =========================================================================

    #[test]
    fn test_rounding_symmetry_for_half_values() {
        // For x.5, round_towards_min should give floor
        // and round_towards_max should give ceiling
        assert_eq!(round_towards_min(1.5), 1);
        assert_eq!(round_towards_max(1.5), 2);

        assert_eq!(round_towards_min(-1.5), -2);
        assert_eq!(round_towards_max(-1.5), -1);
    }

    #[test]
    fn test_rounding_same_for_non_half_values() {
        // For non-half values, both should give the same result
        assert_eq!(round_towards_min(1.3), round_towards_max(1.3));
        assert_eq!(round_towards_min(1.7), round_towards_max(1.7));
        assert_eq!(round_towards_min(-1.3), round_towards_max(-1.3));
        assert_eq!(round_towards_min(-1.7), round_towards_max(-1.7));
    }

    // =========================================================================
    // Edge case tests
    // =========================================================================

    #[test]
    fn test_large_values() {
        let large = 1e15;
        assert_eq!(wround(large), large as i64);
        assert_eq!(round_towards_min(large), large as i64);
        assert_eq!(round_towards_max(large), large as i64);
    }

    #[test]
    fn test_small_values_near_zero() {
        assert_eq!(wround(0.001), 0);
        assert_eq!(wround(-0.001), 0);
        assert_eq!(round_towards_min(0.001), 0);
        assert_eq!(round_towards_max(0.001), 0);
    }

    #[test]
    fn test_values_very_close_to_half() {
        // Values just below 0.5 should round to 0
        let just_below = 0.5 - 1e-10;
        assert_eq!(round_towards_min(just_below), 0);
        assert_eq!(round_towards_max(just_below), 0);

        // Values just above 0.5 should round to 1
        let just_above = 0.5 + 1e-10;
        assert_eq!(round_towards_min(just_above), 1);
        assert_eq!(round_towards_max(just_above), 1);
    }

    // =========================================================================
    // build_hot_pixels tests
    // PORT FROM: wagyu C++ snap_rounding.hpp - build_hot_pixels
    // =========================================================================

    mod build_hot_pixels_tests {
        use super::super::build_hot_pixels;
        use crate::bound::{Bound, Edge};
        use crate::build_result::RingManager;
        use crate::config::{EdgeSide, PolygonType};
        use crate::local_minimum::{LocalMinimum, LocalMinimumList};
        use crate::point::Point;

        /// Helper to create a simple LocalMinimum with edges from bot to top.
        fn make_local_minimum(
            left_bot: Point<f64>,
            left_top: Point<f64>,
            right_bot: Point<f64>,
            right_top: Point<f64>,
            y: f64,
        ) -> LocalMinimum<f64> {
            let left_edges = vec![Edge::new(left_bot, left_top)];
            let right_edges = vec![Edge::new(right_bot, right_top)];
            let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
            let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);
            LocalMinimum::new(left_bound, right_bound, y, false)
        }

        #[test]
        fn build_hot_pixels_empty_minima_list_produces_empty_hot_pixels() {
            // PORT FROM: wagyu snap_rounding.hpp - build_hot_pixels with empty input
            // An empty local minima list should result in empty hot_pixels.
            let minima_list: LocalMinimumList<f64> = Vec::new();
            let mut manager: RingManager<f64> = RingManager::new();

            build_hot_pixels(&minima_list, &mut manager);

            assert!(
                manager.hot_pixels.is_empty(),
                "Hot pixels should be empty for empty minima list"
            );
        }

        #[test]
        fn build_hot_pixels_single_minimum_collects_all_edge_vertices() {
            // PORT FROM: wagyu snap_rounding.hpp - build_hot_pixels
            // A single local minimum should collect bot and top points from both bounds.
            //
            // Shape: V-shape with local minimum at (0, 0)
            //   Left edge: (0, 0) -> (-5, 10)
            //   Right edge: (0, 0) -> (5, 10)
            //
            // Hot pixels should include: (0, 0), (-5, 10), (5, 10)
            let lm = make_local_minimum(
                Point::new(0.0, 0.0),
                Point::new(-5.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(5.0, 10.0),
                0.0,
            );

            let minima_list: LocalMinimumList<f64> = vec![lm];
            let mut manager: RingManager<f64> = RingManager::new();

            build_hot_pixels(&minima_list, &mut manager);

            // Should have collected vertices (may have duplicates before dedup)
            // After sorting and dedup, expect 3 unique points
            assert!(
                !manager.hot_pixels.is_empty(),
                "Hot pixels should not be empty"
            );
            assert!(
                manager.hot_pixels.len() >= 3,
                "Expected at least 3 hot pixels, got {}",
                manager.hot_pixels.len()
            );

            // Verify the points are present (order may vary before sorting)
            let has_origin = manager.hot_pixels.iter().any(|p| p.x == 0.0 && p.y == 0.0);
            let has_left_top = manager
                .hot_pixels
                .iter()
                .any(|p| p.x == -5.0 && p.y == 10.0);
            let has_right_top = manager.hot_pixels.iter().any(|p| p.x == 5.0 && p.y == 10.0);

            assert!(has_origin, "Should contain origin (0, 0)");
            assert!(has_left_top, "Should contain left top (-5, 10)");
            assert!(has_right_top, "Should contain right top (5, 10)");
        }

        #[test]
        fn build_hot_pixels_sorts_by_y_descending_then_x_ascending() {
            // PORT FROM: wagyu ring_util.hpp - hot_pixel_sorter
            // Hot pixels should be sorted by Y descending, then X ascending.
            //
            // This matches the C++ sorter:
            // if (pt1.y == pt2.y) { return pt1.x < pt2.x; }
            // else { return pt1.y > pt2.y; }
            let lm = make_local_minimum(
                Point::new(0.0, 0.0),
                Point::new(-5.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(5.0, 10.0),
                0.0,
            );

            let minima_list: LocalMinimumList<f64> = vec![lm];
            let mut manager: RingManager<f64> = RingManager::new();

            build_hot_pixels(&minima_list, &mut manager);

            // Verify sorted order: higher Y first, then lower X first for same Y
            // Expected order: (-5, 10), (5, 10), (0, 0)
            // Because: y=10 > y=0, and at y=10: x=-5 < x=5
            assert!(manager.hot_pixels.len() >= 3);

            // Check that it's sorted: each element should satisfy sort criteria
            for i in 1..manager.hot_pixels.len() {
                let prev = &manager.hot_pixels[i - 1];
                let curr = &manager.hot_pixels[i];

                let is_valid_order = if (prev.y - curr.y).abs() < f64::EPSILON {
                    // Same Y: X should be ascending
                    prev.x <= curr.x
                } else {
                    // Different Y: Y should be descending
                    prev.y > curr.y
                };

                assert!(
                    is_valid_order,
                    "Hot pixels not sorted correctly at index {}: {:?} should come before {:?}",
                    i, prev, curr
                );
            }
        }

        #[test]
        fn build_hot_pixels_removes_duplicates() {
            // PORT FROM: wagyu ring_util.hpp - sort_hot_pixels uses std::unique
            // Duplicate points should be removed.
            //
            // When left and right bounds share the same bot point, it should
            // appear only once in the final hot_pixels list.
            let lm = make_local_minimum(
                Point::new(0.0, 0.0),
                Point::new(-5.0, 10.0),
                Point::new(0.0, 0.0), // Same as left bot - creates duplicate
                Point::new(5.0, 10.0),
                0.0,
            );

            let minima_list: LocalMinimumList<f64> = vec![lm];
            let mut manager: RingManager<f64> = RingManager::new();

            build_hot_pixels(&minima_list, &mut manager);

            // Count occurrences of (0, 0)
            let origin_count = manager
                .hot_pixels
                .iter()
                .filter(|p| p.x == 0.0 && p.y == 0.0)
                .count();

            assert_eq!(
                origin_count, 1,
                "Duplicate origin should be removed, found {} occurrences",
                origin_count
            );

            // Total should be 3 unique points: (0,0), (-5,10), (5,10)
            assert_eq!(
                manager.hot_pixels.len(),
                3,
                "Expected 3 unique hot pixels, got {}",
                manager.hot_pixels.len()
            );
        }

        #[test]
        fn build_hot_pixels_multiple_minima_collects_all_vertices() {
            // PORT FROM: wagyu snap_rounding.hpp - build_hot_pixels
            // Multiple local minima should have all their vertices collected.
            //
            // First V-shape at (0, 0)
            let lm1 = make_local_minimum(
                Point::new(0.0, 0.0),
                Point::new(-5.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(5.0, 10.0),
                0.0,
            );

            // Second V-shape at (20, 5)
            let lm2 = make_local_minimum(
                Point::new(20.0, 5.0),
                Point::new(15.0, 15.0),
                Point::new(20.0, 5.0),
                Point::new(25.0, 15.0),
                5.0,
            );

            let minima_list: LocalMinimumList<f64> = vec![lm1, lm2];
            let mut manager: RingManager<f64> = RingManager::new();

            build_hot_pixels(&minima_list, &mut manager);

            // Expected unique points:
            // From lm1: (0, 0), (-5, 10), (5, 10)
            // From lm2: (20, 5), (15, 15), (25, 15)
            // Total: 6 unique points
            assert_eq!(
                manager.hot_pixels.len(),
                6,
                "Expected 6 unique hot pixels from two minima, got {}",
                manager.hot_pixels.len()
            );
        }

        #[test]
        fn build_hot_pixels_multi_edge_bound_collects_all_vertices() {
            // PORT FROM: wagyu snap_rounding.hpp - build_hot_pixels
            // Bounds with multiple edges should have all edge vertices collected.
            //
            // Left bound: (0, 0) -> (-3, 5) -> (-5, 10)
            // Right bound: (0, 0) -> (3, 5) -> (5, 10)
            let left_edges = vec![
                Edge::new(Point::new(0.0_f64, 0.0_f64), Point::new(-3.0, 5.0)),
                Edge::new(Point::new(-3.0_f64, 5.0_f64), Point::new(-5.0, 10.0)),
            ];
            let right_edges = vec![
                Edge::new(Point::new(0.0_f64, 0.0_f64), Point::new(3.0, 5.0)),
                Edge::new(Point::new(3.0_f64, 5.0_f64), Point::new(5.0, 10.0)),
            ];

            let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
            let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);
            let lm = LocalMinimum::new(left_bound, right_bound, 0.0, false);

            let minima_list: LocalMinimumList<f64> = vec![lm];
            let mut manager: RingManager<f64> = RingManager::new();

            build_hot_pixels(&minima_list, &mut manager);

            // Expected unique points:
            // (0, 0), (-3, 5), (-5, 10), (3, 5), (5, 10)
            // Total: 5 unique points
            assert_eq!(
                manager.hot_pixels.len(),
                5,
                "Expected 5 unique hot pixels from multi-edge bounds, got {}",
                manager.hot_pixels.len()
            );

            // Verify intermediate points are present
            let has_left_mid = manager.hot_pixels.iter().any(|p| p.x == -3.0 && p.y == 5.0);
            let has_right_mid = manager.hot_pixels.iter().any(|p| p.x == 3.0 && p.y == 5.0);

            assert!(
                has_left_mid,
                "Should contain left intermediate point (-3, 5)"
            );
            assert!(
                has_right_mid,
                "Should contain right intermediate point (3, 5)"
            );
        }

        #[test]
        fn build_hot_pixels_resets_current_hp_idx() {
            // PORT FROM: wagyu snap_rounding.hpp - build_hot_pixels
            // The current_hp_idx should be reset to 0 after building hot pixels.
            let lm = make_local_minimum(
                Point::new(0.0, 0.0),
                Point::new(-5.0, 10.0),
                Point::new(0.0, 0.0),
                Point::new(5.0, 10.0),
                0.0,
            );

            let minima_list: LocalMinimumList<f64> = vec![lm];
            let mut manager: RingManager<f64> = RingManager::new();
            manager.current_hp_idx = 999; // Set to non-zero

            build_hot_pixels(&minima_list, &mut manager);

            assert_eq!(
                manager.current_hp_idx, 0,
                "current_hp_idx should be reset to 0"
            );
        }
    }
}
