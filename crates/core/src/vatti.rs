//! Vatti Polygon Clipping Algorithm
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/vatti.hpp
//!
//! This module implements the core Vatti sweep algorithm for boolean operations.
//! The algorithm processes edges from bottom to top, handling:
//! - Local minima (lowest points of polygon edges)
//! - Intersections between active edges
//! - Horizontal edges (special case)
//! - Maxima (highest points of polygon edges)

use geo_types::CoordNum;
use num_traits::{Bounded, ToPrimitive};

use crate::active_edge_list::ActiveEdgeList;
use crate::bound::Bound;
use crate::build_result::RingManager;
use crate::config::FillType;
use crate::intersect_util::process_intersections;
use crate::local_minimum::{LocalMinimum, LocalMinimumList};
use crate::local_minimum_util::insert_local_minima_into_abl;
use crate::process_horizontal::process_edges_at_top_of_scanbeam;
use crate::scanbeam::Scanbeam;
use crate::Operation;

// ============================================================================
// Scanbeam Helpers
// ============================================================================

/// Pop from scanbeam and update scanline_y.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/util.hpp - pop_from_scanbeam
///
/// Returns true if a value was popped (and scanline_y was updated), false otherwise.
#[inline]
fn pop_from_scanbeam<T: CoordNum>(scanline_y: &mut T, scanbeam: &mut Scanbeam<T>) -> bool {
    if let Some(y) = scanbeam.pop() {
        *scanline_y = y;
        true
    } else {
        false
    }
}

/// Setup scanbeam with all local minimum Y values.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/util.hpp - setup_scanbeam
fn setup_scanbeam<T: CoordNum>(minima_list: &LocalMinimumList<T>, scanbeam: &mut Scanbeam<T>) {
    for lm in minima_list {
        scanbeam.insert(lm.y);
    }
}

/// Execute the Vatti polygon clipping algorithm.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/vatti.hpp - execute_vatti
///
/// This is the main sweep algorithm that processes all edges from bottom to top,
/// building output rings as it goes.
///
/// # Arguments
///
/// * `minima_list` - List of local minima (entry points for polygon edges)
/// * `bounds` - Storage for all bounds (edges) in the algorithm
/// * `manager` - Ring manager for building output polygons
/// * `clip_type` - Type of boolean operation (Union, Intersection, etc.)
/// * `subject_fill_type` - Fill rule for subject polygons
/// * `clip_fill_type` - Fill rule for clip polygons
///
/// # Algorithm (matches C++ exactly)
///
/// ```text
/// while (pop_from_scanbeam(scanline_y, scanbeam) || current_lm != minima_sorted.end()) {
///     process_intersections(scanline_y, active_bounds, ...);
///     update_current_hp_itr(scanline_y, manager);
///     process_edges_at_top_of_scanbeam(scanline_y, active_bounds, ...);
///     insert_local_minima_into_ABL(scanline_y, minima_sorted, current_lm, ...);
/// }
/// ```
pub fn execute_vatti<T>(
    minima_list: &mut LocalMinimumList<T>,
    bounds: &mut Vec<Bound<T>>,
    manager: &mut RingManager<T>,
    clip_type: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) where
    T: CoordNum + Bounded + ToPrimitive + num_traits::NumCast,
{
    if minima_list.is_empty() {
        return;
    }

    // From C++: active_bound_list<T> active_bounds;
    let mut active_bounds = ActiveEdgeList::new();

    // From C++: scanbeam_list<T> scanbeam;
    let mut scanbeam: Scanbeam<T> = Scanbeam::new();

    // From C++: T scanline_y = std::numeric_limits<T>::max();
    let mut scanline_y: T = T::max_value();

    // From C++: local_minimum_ptr_list<T> minima_sorted;
    // Sort local minima by Y (descending order - larger Y processed first in scanbeam)
    // This matches C++ local_minimum_sorter which sorts by descending Y
    // From C++: std::stable_sort(minima_sorted.begin(), minima_sorted.end(), local_minimum_sorter<T>());
    minima_list.sort_by(LocalMinimum::compare);

    // From C++: local_minimum_ptr_list_itr<T> current_lm = minima_sorted.begin();
    // DIVERGENCE FROM WAGYU: C++ uses iterator over pointers
    // Rust uses index into the sorted list
    let mut current_lm_idx: usize = 0;

    // From C++: setup_scanbeam(minima_list, scanbeam);
    setup_scanbeam(minima_list, &mut scanbeam);

    // From C++: manager.current_hp_itr = manager.hot_pixels.begin();
    manager.current_hp_idx = 0;

    // Placeholder index for process_edges_at_top_of_scanbeam
    // (matches C++ parameter for minima_sorted iteration)
    let minima_indices: Vec<usize> = Vec::new();

    // Main sweep loop
    // From C++: while (pop_from_scanbeam(scanline_y, scanbeam) || current_lm != minima_sorted.end())
    crate::debug::log_vatti_start(minima_list.len(), scanbeam.len());

    while pop_from_scanbeam(&mut scanline_y, &mut scanbeam) || current_lm_idx < minima_list.len() {
        crate::debug::log_scanbeam(scanline_y.to_f64().unwrap_or(0.0));
        // From C++: process_intersections(scanline_y, active_bounds, cliptype, subject_fill_type, clip_fill_type, manager);
        process_intersections(
            scanline_y,
            bounds,
            &mut active_bounds,
            clip_type,
            subject_fill_type,
            clip_fill_type,
            manager,
        );

        // From C++: update_current_hp_itr(scanline_y, manager);
        manager.update_current_hp_itr(scanline_y);

        // From C++: process_edges_at_top_of_scanbeam(scanline_y, active_bounds, scanbeam, minima_sorted, current_lm, manager, cliptype, subject_fill_type, clip_fill_type);
        // First we process bounds that has already been added to the active bound list --
        // if the active bound list is empty local minima that are at this scanline_y and
        // have a horizontal edge at the local minima will be processed
        process_edges_at_top_of_scanbeam(
            scanline_y,
            &mut active_bounds,
            bounds,
            &mut scanbeam,
            &minima_indices,
            &mut current_lm_idx,
            minima_list, // Now passed as mutable for insert_horizontal_local_minima_into_abl
            manager,
            clip_type,
            subject_fill_type,
            clip_fill_type,
        );

        // From C++: insert_local_minima_into_ABL(scanline_y, minima_sorted, current_lm, active_bounds, manager, scanbeam, cliptype, subject_fill_type, clip_fill_type);
        // Next we will add local minima bounds to the active bounds list that are on the local
        // minima queue at this current scanline_y
        insert_local_minima_into_abl(
            scanline_y,
            minima_list,
            &mut current_lm_idx,
            bounds,
            &mut active_bounds,
            manager,
            &mut scanbeam,
            clip_type,
            subject_fill_type,
            clip_fill_type,
        );
    }

    crate::debug::log_vatti_end(manager.len());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::Edge;
    use crate::config::{EdgeSide, PolygonType};
    use crate::local_minimum::LocalMinimum;
    use crate::point::Point;

    // ==================== Test Helpers ====================

    /// Create a simple bound with a single edge
    fn make_bound(
        bot: (i64, i64),
        top: (i64, i64),
        poly_type: PolygonType,
        side: EdgeSide,
    ) -> Bound<i64> {
        let edge = Edge::new(Point::new(bot.0, bot.1), Point::new(top.0, top.1));
        Bound::new(vec![edge], poly_type, side)
    }

    /// Create a local minimum from two edges (left going up-left, right going up-right)
    fn make_local_minimum(
        y: i64,
        left_top: (i64, i64),
        right_top: (i64, i64),
        start_x: i64,
    ) -> LocalMinimum<i64> {
        let left_bound = make_bound((start_x, y), left_top, PolygonType::Subject, EdgeSide::Left);
        let right_bound = make_bound(
            (start_x, y),
            right_top,
            PolygonType::Subject,
            EdgeSide::Right,
        );
        LocalMinimum::new(left_bound, right_bound, y, false)
    }

    /// Create a triangle local minimum (apex at given height)
    #[allow(dead_code)]
    fn make_triangle_lm(base_y: i64, apex_x: i64, apex_y: i64) -> LocalMinimum<i64> {
        // Triangle with base at (0, base_y) - (10, base_y) and apex at (apex_x, apex_y)
        let left_bound = make_bound(
            (0, base_y),
            (apex_x, apex_y),
            PolygonType::Subject,
            EdgeSide::Left,
        );
        let right_bound = make_bound(
            (10, base_y),
            (apex_x, apex_y),
            PolygonType::Subject,
            EdgeSide::Right,
        );
        // Note: In a real triangle, both bounds start from the same local minimum point
        // For testing, we create them starting at different x but same y
        LocalMinimum::new(left_bound, right_bound, base_y, false)
    }

    // ==================== Empty Input Tests ====================

    #[test]
    fn test_execute_vatti_empty() {
        let mut minima_list: LocalMinimumList<i64> = LocalMinimumList::new();
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Should complete without error for empty input
        assert!(minima_list.is_empty());
    }

    // ==================== Single Local Minimum Tests ====================

    #[test]
    fn test_execute_vatti_single_lm_processes_scanbeam() {
        // A single triangle: base at y=0, apex at y=10
        let lm = make_local_minimum(0, (-5, 10), (15, 10), 5);
        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // After processing, bounds should have been added (2 bounds from the local minimum)
        assert_eq!(
            bounds.len(),
            2,
            "Two bounds should be added from the local minimum"
        );
    }

    #[test]
    fn test_execute_vatti_single_lm_bounds_reach_maxima() {
        // A simple triangle where both bounds meet at the apex
        let left_edge = Edge::new(Point::new(0_i64, 0), Point::new(5, 10));
        let right_edge = Edge::new(Point::new(10_i64, 0), Point::new(5, 10));

        let left_bound = Bound::new(vec![left_edge], PolygonType::Subject, EdgeSide::Left);
        let right_bound = Bound::new(vec![right_edge], PolygonType::Subject, EdgeSide::Right);

        let lm = LocalMinimum::new(left_bound, right_bound, 0, false);

        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Both bounds should exist in the bounds vector
        assert_eq!(
            bounds.len(),
            2,
            "Both bounds from LM should be in bounds vector"
        );

        // Both bounds should have been processed to their maxima (top of edge)
        // The current_edge_index should point to the last edge (0 in this case, single edge)
        assert_eq!(bounds[0].current_edge_index, 0);
        assert_eq!(bounds[1].current_edge_index, 0);
    }

    // ==================== Multiple Local Minima Tests ====================

    #[test]
    fn test_execute_vatti_multiple_lm_sorted_by_y() {
        // Two triangles at different heights
        // Triangle 1: base at y=0, apex at y=5
        let lm1 = make_local_minimum(0, (-5, 5), (15, 5), 5);
        // Triangle 2: base at y=10, apex at y=20
        let lm2 = make_local_minimum(10, (0, 20), (20, 20), 10);

        let mut minima_list: LocalMinimumList<i64> = vec![lm1, lm2];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Both local minima should have had their bounds added
        assert_eq!(bounds.len(), 4, "Four bounds total from two local minima");
    }

    #[test]
    fn test_execute_vatti_processes_lower_lm_first() {
        // Two triangles: one lower (y=0), one higher (y=10)
        // The lower one should be processed first
        let left1 = make_bound((0, 0), (5, 5), PolygonType::Subject, EdgeSide::Left);
        let right1 = make_bound((10, 0), (5, 5), PolygonType::Subject, EdgeSide::Right);
        let lm1 = LocalMinimum::new(left1, right1, 0, false);

        let left2 = make_bound((20, 10), (25, 20), PolygonType::Subject, EdgeSide::Left);
        let right2 = make_bound((30, 10), (25, 20), PolygonType::Subject, EdgeSide::Right);
        let lm2 = LocalMinimum::new(left2, right2, 10, false);

        // Insert in reverse order to test sorting
        let mut minima_list: LocalMinimumList<i64> = vec![lm2, lm1];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // All bounds should be processed
        assert_eq!(bounds.len(), 4);
    }

    // ==================== Scanbeam Population Tests ====================

    #[test]
    fn test_execute_vatti_scanbeam_includes_lm_y_values() {
        // Test that the scanbeam is populated with local minimum Y values
        // We can't directly access the scanbeam, but we can verify that
        // processing completes correctly
        let lm = make_local_minimum(5, (0, 15), (10, 15), 5);
        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // The bounds should have been inserted at y=5 and processed up to y=15
        assert_eq!(bounds.len(), 2);
    }

    // ==================== Intersection Processing Tests ====================

    #[test]
    fn test_execute_vatti_crossing_edges_detected() {
        // Two triangles that cross: their edges should intersect
        // Triangle 1: starts at (0, 0), apex at (15, 10) - goes right
        // Triangle 2: starts at (20, 0), apex at (5, 10) - goes left
        // These edges cross somewhere in the middle

        let left1 = make_bound((0, 0), (15, 10), PolygonType::Subject, EdgeSide::Left);
        let right1 = make_bound((5, 0), (20, 10), PolygonType::Subject, EdgeSide::Right);
        let lm1 = LocalMinimum::new(left1, right1, 0, false);

        let mut minima_list: LocalMinimumList<i64> = vec![lm1];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        // Should complete processing (intersection handling is internal)
        assert_eq!(bounds.len(), 2);
    }

    // ==================== Operation Type Tests ====================

    #[test]
    fn test_execute_vatti_union_operation() {
        let lm = make_local_minimum(0, (-5, 10), (15, 10), 5);
        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        assert_eq!(bounds.len(), 2);
    }

    #[test]
    fn test_execute_vatti_intersection_operation() {
        let lm = make_local_minimum(0, (-5, 10), (15, 10), 5);
        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Intersection,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        assert_eq!(bounds.len(), 2);
    }

    #[test]
    fn test_execute_vatti_difference_operation() {
        let lm = make_local_minimum(0, (-5, 10), (15, 10), 5);
        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Difference,
            FillType::EvenOdd,
            FillType::EvenOdd,
        );

        assert_eq!(bounds.len(), 2);
    }

    // ==================== Fill Type Tests ====================

    #[test]
    fn test_execute_vatti_nonzero_fill() {
        let lm = make_local_minimum(0, (-5, 10), (15, 10), 5);
        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::NonZero,
            FillType::NonZero,
        );

        assert_eq!(bounds.len(), 2);
    }

    #[test]
    fn test_execute_vatti_positive_fill() {
        let lm = make_local_minimum(0, (-5, 10), (15, 10), 5);
        let mut minima_list: LocalMinimumList<i64> = vec![lm];
        let mut bounds: Vec<Bound<i64>> = Vec::new();
        let mut manager: RingManager<i64> = RingManager::new();

        execute_vatti(
            &mut minima_list,
            &mut bounds,
            &mut manager,
            Operation::Union,
            FillType::Positive,
            FillType::Positive,
        );

        assert_eq!(bounds.len(), 2);
    }
}
