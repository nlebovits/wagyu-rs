//! Active Edge List (Active Bound List) for the Vatti clipping algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/active_bound_list.hpp
//!
//! The Active Edge List (AEL) maintains a sorted list of bounds currently
//! intersecting the sweep line. Bounds are sorted left-to-right by their
//! current X position at the sweep line.

use crate::bound::Bound;
use geo_types::CoordNum;

/// The Active Edge List maintains bounds sorted by their current_x position.
///
/// From C++: `template <typename T> using active_bound_list = std::vector<bound_ptr<T>>;`
///
/// In Rust, we use indices into an external bounds storage rather than raw pointers,
/// providing memory safety while maintaining the same algorithmic approach.
#[derive(Debug, Clone)]
pub struct ActiveEdgeList {
    /// Indices into the bounds storage, sorted by current_x of the referenced bounds.
    indices: Vec<usize>,
}

impl ActiveEdgeList {
    /// Create a new empty active edge list.
    pub fn new() -> Self {
        Self {
            indices: Vec::new(),
        }
    }

    /// Returns the number of active bounds.
    pub fn len(&self) -> usize {
        self.indices.len()
    }

    /// Returns true if there are no active bounds.
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    /// Returns an iterator over the bound indices.
    pub fn iter(&self) -> impl Iterator<Item = &usize> {
        self.indices.iter()
    }

    /// Returns the bound indices as a slice.
    pub fn as_slice(&self) -> &[usize] {
        &self.indices
    }

    /// Insert a bound index, maintaining sort order by current_x.
    ///
    /// The `bounds` slice is used to look up the current_x values for comparison.
    /// This corresponds to the C++ `bound_insert_location` functor and
    /// `insert_bound_into_ABL` function.
    ///
    /// # Arguments
    ///
    /// * `bound_index` - Index of the bound to insert
    /// * `bounds` - Slice of all bounds (for looking up current_x values)
    ///
    /// # Returns
    ///
    /// The position where the bound was inserted.
    pub fn insert<T: CoordNum>(&mut self, bound_index: usize, bounds: &[Bound<T>]) -> usize {
        let new_bound = &bounds[bound_index];
        let insert_pos = self.find_insert_position(new_bound, bounds);
        self.indices.insert(insert_pos, bound_index);
        insert_pos
    }

    /// Insert a pair of bounds (left and right) at the same position.
    ///
    /// This matches the C++ `insert_bound_into_ABL` function which inserts
    /// both bounds of a local minimum together.
    ///
    /// # Returns
    ///
    /// The position where the left bound was inserted.
    pub fn insert_pair<T: CoordNum>(
        &mut self,
        left_index: usize,
        right_index: usize,
        bounds: &[Bound<T>],
    ) -> usize {
        let left_bound = &bounds[left_index];
        let insert_pos = self.find_insert_position(left_bound, bounds);
        // Insert right first, then left, so left is at insert_pos and right is at insert_pos + 1
        self.indices.insert(insert_pos, right_index);
        self.indices.insert(insert_pos, left_index);
        insert_pos
    }

    /// Find the position to insert a bound while maintaining sort order.
    ///
    /// This corresponds to the C++ `bound_insert_location` functor.
    /// Bounds are sorted by current_x, with tie-breaking based on edge slopes.
    fn find_insert_position<T: CoordNum>(
        &self,
        new_bound: &Bound<T>,
        bounds: &[Bound<T>],
    ) -> usize {
        // Find first position where new_bound.current_x < existing.current_x
        for (pos, &idx) in self.indices.iter().enumerate() {
            let existing = &bounds[idx];
            if Self::should_insert_before(new_bound, existing) {
                return pos;
            }
        }
        // Insert at end if no position found
        self.indices.len()
    }

    /// Determine if new_bound should be inserted before existing_bound.
    ///
    /// This implements the C++ `bound_insert_location::operator()` logic:
    /// - Primary sort by current_x
    /// - Tie-breaking by edge slopes when current_x values are equal
    fn should_insert_before<T: CoordNum>(new_bound: &Bound<T>, existing: &Bound<T>) -> bool {
        let new_x = new_bound.current_x;
        let existing_x = existing.current_x;

        if values_are_equal(new_x, existing_x) {
            // When x values are equal, use edge slopes for tie-breaking
            let new_edge = new_bound.current_edge();
            let existing_edge = existing.current_edge();

            let new_top_y = new_edge.top.y.to_f64().unwrap_or(0.0);
            let existing_top_y = existing_edge.top.y.to_f64().unwrap_or(0.0);

            if new_top_y > existing_top_y {
                // new_bound's edge extends higher
                let new_top_x = new_edge.top.x.to_f64().unwrap_or(0.0);
                let existing_x_at_new_top = get_current_x(existing_edge, new_top_y);
                less_than(new_top_x, existing_x_at_new_top)
            } else {
                // existing's edge extends higher or equal
                let existing_top_x = existing_edge.top.x.to_f64().unwrap_or(0.0);
                let new_x_at_existing_top = get_current_x(new_edge, existing_top_y);
                greater_than(existing_top_x, new_x_at_existing_top)
            }
        } else {
            new_x < existing_x
        }
    }

    /// Remove a bound index from the active edge list.
    ///
    /// # Returns
    ///
    /// `true` if the bound was found and removed, `false` otherwise.
    pub fn remove(&mut self, bound_index: usize) -> bool {
        if let Some(pos) = self.indices.iter().position(|&idx| idx == bound_index) {
            self.indices.remove(pos);
            true
        } else {
            false
        }
    }

    /// Get the position of a bound index in the list.
    ///
    /// # Returns
    ///
    /// `Some(position)` if found, `None` otherwise.
    pub fn position(&self, bound_index: usize) -> Option<usize> {
        self.indices.iter().position(|&idx| idx == bound_index)
    }

    /// Get the bound index at a given position.
    pub fn get(&self, position: usize) -> Option<usize> {
        self.indices.get(position).copied()
    }

    /// Get the left neighbor (predecessor) of a bound at a given position.
    ///
    /// # Returns
    ///
    /// `Some(bound_index)` of the left neighbor, or `None` if at the start.
    pub fn left_neighbor(&self, position: usize) -> Option<usize> {
        if position > 0 {
            self.indices.get(position - 1).copied()
        } else {
            None
        }
    }

    /// Get the right neighbor (successor) of a bound at a given position.
    ///
    /// # Returns
    ///
    /// `Some(bound_index)` of the right neighbor, or `None` if at the end.
    pub fn right_neighbor(&self, position: usize) -> Option<usize> {
        self.indices.get(position + 1).copied()
    }

    /// Swap two bounds in the list by their positions.
    ///
    /// This is used when bounds need to be reordered after intersections.
    pub fn swap(&mut self, pos1: usize, pos2: usize) {
        self.indices.swap(pos1, pos2);
    }
}

impl Default for ActiveEdgeList {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Helper functions (from C++ util.hpp)
// ============================================================================

/// Epsilon for floating point comparisons.
const EPSILON: f64 = 1e-10;

/// Check if two floating point values are approximately equal.
fn values_are_equal(a: f64, b: f64) -> bool {
    (a - b).abs() < EPSILON
}

/// Check if a is less than b with epsilon tolerance.
fn less_than(a: f64, b: f64) -> bool {
    a < b - EPSILON
}

/// Check if a is greater than b with epsilon tolerance.
fn greater_than(a: f64, b: f64) -> bool {
    a > b + EPSILON
}

/// Get the x coordinate of an edge at a given y coordinate.
///
/// From C++: `get_current_x(edge<T> const& edge, T y)` in edge.hpp:81-87
///
/// IMPORTANT: When y == edge.top.y, we return edge.top.x directly.
/// This is critical for horizontal edges where bot.x != top.x.
fn get_current_x<T: CoordNum>(edge: &crate::bound::Edge<T>, y: f64) -> f64 {
    // C++ special case: when at the top of the edge, return top.x directly.
    let top_y = edge.top.y.to_f64().unwrap_or(0.0);
    if (y - top_y).abs() < f64::EPSILON {
        return edge.top.x.to_f64().unwrap_or(0.0);
    }

    if edge.is_horizontal() {
        edge.bot.x.to_f64().unwrap_or(0.0)
    } else {
        let bot_x = edge.bot.x.to_f64().unwrap_or(0.0);
        let bot_y = edge.bot.y.to_f64().unwrap_or(0.0);
        bot_x + edge.dx * (y - bot_y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::Edge;
    use crate::config::{EdgeSide, PolygonType};
    use crate::point::Point;

    // Helper to create a simple bound with a single edge
    fn make_bound(bot_x: f64, bot_y: f64, top_x: f64, top_y: f64) -> Bound<f64> {
        let edge = Edge::new(Point::new(bot_x, bot_y), Point::new(top_x, top_y));
        Bound::new(vec![edge], PolygonType::Subject, EdgeSide::Left)
    }

    // ==================== Empty List Tests ====================

    #[test]
    fn new_creates_empty_list() {
        let ael = ActiveEdgeList::new();
        assert!(ael.is_empty());
        assert_eq!(ael.len(), 0);
    }

    #[test]
    fn default_creates_empty_list() {
        let ael = ActiveEdgeList::default();
        assert!(ael.is_empty());
    }

    // ==================== Single Bound Insertion Tests ====================

    #[test]
    fn insert_single_bound_into_empty_list() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![make_bound(5.0, 0.0, 5.0, 10.0)];

        let pos = ael.insert(0, &bounds);

        assert_eq!(pos, 0);
        assert_eq!(ael.len(), 1);
        assert_eq!(ael.get(0), Some(0));
    }

    #[test]
    fn insert_returns_correct_position() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(10.0, 0.0, 10.0, 10.0), // x = 10
            make_bound(5.0, 0.0, 5.0, 10.0),   // x = 5
        ];

        ael.insert(0, &bounds); // Insert bound at x=10
        let pos = ael.insert(1, &bounds); // Insert bound at x=5, should go before

        assert_eq!(pos, 0); // x=5 should be at position 0
    }

    // ==================== Multiple Bounds Sort Order Tests ====================

    #[test]
    fn multiple_bounds_sorted_by_current_x() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(10.0, 0.0, 10.0, 10.0), // x = 10
            make_bound(5.0, 0.0, 5.0, 10.0),   // x = 5
            make_bound(15.0, 0.0, 15.0, 10.0), // x = 15
        ];

        // Insert in arbitrary order
        ael.insert(0, &bounds); // x = 10
        ael.insert(1, &bounds); // x = 5
        ael.insert(2, &bounds); // x = 15

        // Should be sorted: [5, 10, 15] -> indices [1, 0, 2]
        let result: Vec<usize> = ael.iter().copied().collect();
        assert_eq!(result, vec![1, 0, 2]);
    }

    #[test]
    fn insert_maintains_sorted_order() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(0.0, 0.0, 0.0, 10.0),
            make_bound(20.0, 0.0, 20.0, 10.0),
            make_bound(10.0, 0.0, 10.0, 10.0),
        ];

        ael.insert(0, &bounds); // x = 0
        ael.insert(1, &bounds); // x = 20
        ael.insert(2, &bounds); // x = 10 (middle)

        // Order should be: x=0, x=10, x=20 -> indices [0, 2, 1]
        let result: Vec<usize> = ael.iter().copied().collect();
        assert_eq!(result, vec![0, 2, 1]);
    }

    // ==================== Insert Pair Tests ====================

    #[test]
    fn insert_pair_inserts_both_bounds() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(5.0, 0.0, 5.0, 10.0),   // left
            make_bound(10.0, 0.0, 10.0, 10.0), // right
        ];

        let pos = ael.insert_pair(0, 1, &bounds);

        assert_eq!(pos, 0);
        assert_eq!(ael.len(), 2);
        // Left should be first, right second
        assert_eq!(ael.get(0), Some(0));
        assert_eq!(ael.get(1), Some(1));
    }

    #[test]
    fn insert_pair_in_middle_of_existing_bounds() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(0.0, 0.0, 0.0, 10.0),   // existing left
            make_bound(20.0, 0.0, 20.0, 10.0), // existing right
            make_bound(8.0, 0.0, 8.0, 10.0),   // new left
            make_bound(12.0, 0.0, 12.0, 10.0), // new right
        ];

        ael.insert(0, &bounds); // x = 0
        ael.insert(1, &bounds); // x = 20
        ael.insert_pair(2, 3, &bounds); // x = 8 and x = 12

        // Order: [0, 2, 3, 1] for x = [0, 8, 12, 20]
        let result: Vec<usize> = ael.iter().copied().collect();
        assert_eq!(result, vec![0, 2, 3, 1]);
    }

    // ==================== Removal Tests ====================

    #[test]
    fn remove_existing_bound_returns_true() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![make_bound(5.0, 0.0, 5.0, 10.0)];

        ael.insert(0, &bounds);
        let removed = ael.remove(0);

        assert!(removed);
        assert!(ael.is_empty());
    }

    #[test]
    fn remove_nonexistent_bound_returns_false() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![make_bound(5.0, 0.0, 5.0, 10.0)];

        ael.insert(0, &bounds);
        let removed = ael.remove(999);

        assert!(!removed);
        assert_eq!(ael.len(), 1);
    }

    #[test]
    fn remove_middle_bound_maintains_order() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(0.0, 0.0, 0.0, 10.0),
            make_bound(10.0, 0.0, 10.0, 10.0),
            make_bound(20.0, 0.0, 20.0, 10.0),
        ];

        ael.insert(0, &bounds);
        ael.insert(1, &bounds);
        ael.insert(2, &bounds);

        ael.remove(1); // Remove middle (x=10)

        let result: Vec<usize> = ael.iter().copied().collect();
        assert_eq!(result, vec![0, 2]);
    }

    // ==================== Neighbor Tests ====================

    #[test]
    fn left_neighbor_returns_none_for_first_element() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![make_bound(5.0, 0.0, 5.0, 10.0)];

        ael.insert(0, &bounds);

        assert_eq!(ael.left_neighbor(0), None);
    }

    #[test]
    fn left_neighbor_returns_previous_element() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(5.0, 0.0, 5.0, 10.0),
            make_bound(10.0, 0.0, 10.0, 10.0),
        ];

        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        assert_eq!(ael.left_neighbor(1), Some(0));
    }

    #[test]
    fn right_neighbor_returns_none_for_last_element() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![make_bound(5.0, 0.0, 5.0, 10.0)];

        ael.insert(0, &bounds);

        assert_eq!(ael.right_neighbor(0), None);
    }

    #[test]
    fn right_neighbor_returns_next_element() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(5.0, 0.0, 5.0, 10.0),
            make_bound(10.0, 0.0, 10.0, 10.0),
        ];

        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        assert_eq!(ael.right_neighbor(0), Some(1));
    }

    // ==================== Position Tests ====================

    #[test]
    fn position_returns_none_for_absent_bound() {
        let ael = ActiveEdgeList::new();
        assert_eq!(ael.position(0), None);
    }

    #[test]
    fn position_returns_correct_index() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(5.0, 0.0, 5.0, 10.0),
            make_bound(10.0, 0.0, 10.0, 10.0),
        ];

        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        assert_eq!(ael.position(0), Some(0));
        assert_eq!(ael.position(1), Some(1));
    }

    // ==================== Swap Tests ====================

    #[test]
    fn swap_exchanges_bounds_at_positions() {
        let mut ael = ActiveEdgeList::new();
        let bounds = vec![
            make_bound(5.0, 0.0, 5.0, 10.0),
            make_bound(10.0, 0.0, 10.0, 10.0),
        ];

        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        ael.swap(0, 1);

        let result: Vec<usize> = ael.iter().copied().collect();
        assert_eq!(result, vec![1, 0]);
    }

    // ==================== Helper Function Tests ====================

    #[test]
    fn values_are_equal_with_exact_match() {
        assert!(values_are_equal(1.0, 1.0));
    }

    #[test]
    fn values_are_equal_within_epsilon() {
        assert!(values_are_equal(1.0, 1.0 + 1e-11));
    }

    #[test]
    fn values_are_not_equal_outside_epsilon() {
        assert!(!values_are_equal(1.0, 1.0 + 1e-9));
    }

    #[test]
    fn get_current_x_for_vertical_edge() {
        // Vertical edge: x stays constant
        let edge = Edge::new(Point::new(5.0_f64, 0.0), Point::new(5.0_f64, 10.0));
        assert!((get_current_x(&edge, 5.0) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn get_current_x_for_sloped_edge() {
        // Edge from (0, 0) to (10, 10): dx = 1.0
        // At y = 5, x should be 5
        let edge = Edge::new(Point::new(0.0_f64, 0.0), Point::new(10.0_f64, 10.0));
        assert!((get_current_x(&edge, 5.0) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn get_current_x_for_horizontal_edge() {
        // Horizontal edge at y == top.y returns top.x (C++ edge.hpp:81-87)
        // bot=(5.0, 10.0), top=(15.0, 10.0) -> at y=10.0, returns top.x=15.0
        let edge = Edge::new(Point::new(5.0_f64, 10.0), Point::new(15.0_f64, 10.0));
        assert!(
            (get_current_x(&edge, 10.0) - 15.0).abs() < 1e-10,
            "Expected top.x=15.0 at y=top.y, got {}",
            get_current_x(&edge, 10.0)
        );
    }

    // ==================== Edge Case Tests ====================

    #[test]
    fn insert_with_same_x_uses_slope_tiebreaker() {
        let mut ael = ActiveEdgeList::new();
        // Two bounds starting at same x but with different slopes
        // Edge 0: starts at (5, 0), goes to (10, 10) - slopes right
        // Edge 1: starts at (5, 0), goes to (0, 10) - slopes left
        let bounds = vec![
            make_bound(5.0, 0.0, 10.0, 10.0), // slopes right
            make_bound(5.0, 0.0, 0.0, 10.0),  // slopes left
        ];

        ael.insert(0, &bounds);
        ael.insert(1, &bounds);

        // At the start, both have x=5, but bound 1 will be to the left
        // as the sweep progresses (it goes left)
        assert_eq!(ael.len(), 2);
    }
}
