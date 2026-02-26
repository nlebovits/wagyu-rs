//! Local minimum types for the Vatti clipping algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/local_minimum.hpp
//!
//! A `LocalMinimum` represents a Y-minimum point where the sweep line
//! algorithm starts processing a polygon edge. It stores bounds (left
//! and right edges going upward from the minimum point).

use crate::bound::Bound;
use geo_types::CoordNum;
use std::cmp::Ordering;

// ============================================================================
// LocalMinimum
// ============================================================================

/// Represents a local minimum in a polygon for the sweep line algorithm.
///
/// A local minimum is a point where the y-coordinate is at a minimum along
/// the polygon boundary. From this point, two bounds extend upward: one to
/// the left and one to the right. The sweep line algorithm processes these
/// local minima from bottom to top (or top to bottom, depending on coordinate
/// orientation).
///
/// From C++:
/// ```cpp
/// template <typename T>
/// struct local_minimum {
///     bound<T> left_bound;
///     bound<T> right_bound;
///     T y;
///     bool minimum_has_horizontal;
/// };
/// ```
#[derive(Debug, Clone)]
pub struct LocalMinimum<T: CoordNum> {
    /// The left bound extending upward from the minimum point.
    pub left_bound: Bound<T>,
    /// The right bound extending upward from the minimum point.
    pub right_bound: Bound<T>,
    /// The Y coordinate of the local minimum.
    pub y: T,
    /// Whether this local minimum has a horizontal edge at its base.
    pub minimum_has_horizontal: bool,
}

impl<T: CoordNum> LocalMinimum<T> {
    /// Create a new local minimum.
    ///
    /// # Arguments
    ///
    /// * `left_bound` - The left bound extending upward from the minimum
    /// * `right_bound` - The right bound extending upward from the minimum
    /// * `y` - The Y coordinate of the local minimum
    /// * `minimum_has_horizontal` - Whether this minimum has a horizontal edge
    pub fn new(
        left_bound: Bound<T>,
        right_bound: Bound<T>,
        y: T,
        minimum_has_horizontal: bool,
    ) -> Self {
        Self {
            left_bound,
            right_bound,
            y,
            minimum_has_horizontal,
        }
    }

    /// Compare two local minima for sorting.
    ///
    /// From C++: `local_minimum_sorter` sorts minima in descending Y order
    /// (larger Y values come first). When Y values are equal, minima with
    /// horizontal edges come before those without.
    ///
    /// This follows the C++ logic:
    /// ```cpp
    /// if (locMin2->y == locMin1->y) {
    ///     return locMin2->minimum_has_horizontal != locMin1->minimum_has_horizontal &&
    ///            locMin1->minimum_has_horizontal;
    /// }
    /// return locMin2->y < locMin1->y;
    /// ```
    pub fn compare(a: &Self, b: &Self) -> Ordering {
        // Convert to f64 for comparison
        let a_y = a.y.to_f64().unwrap_or(0.0);
        let b_y = b.y.to_f64().unwrap_or(0.0);

        if (a_y - b_y).abs() < f64::EPSILON {
            // Equal Y: horizontal minima come first
            // C++ returns true when a has horizontal and b doesn't
            match (a.minimum_has_horizontal, b.minimum_has_horizontal) {
                (true, false) => Ordering::Less,    // a comes first
                (false, true) => Ordering::Greater, // b comes first
                _ => Ordering::Equal,               // same horizontal status
            }
        } else if b_y < a_y {
            // b.y < a.y means a has larger y, so a should come first (descending)
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

// ============================================================================
// LocalMinimumList
// ============================================================================

/// A list of local minima.
///
/// From C++: `using local_minimum_list = std::deque<local_minimum<T>>;`
///
/// We use `Vec` instead of `VecDeque` for simplicity, as the main operations
/// are iteration and sorting rather than front/back insertions.
pub type LocalMinimumList<T> = Vec<LocalMinimum<T>>;

#[cfg(test)]
mod tests {
    // ==================== LocalMinimum Tests ====================

    #[test]
    fn local_minimum_new_creates_with_bounds_and_y() {
        // A LocalMinimum should store left_bound, right_bound, y, and minimum_has_horizontal
        // This test will fail until we implement LocalMinimum
        use super::LocalMinimum;
        use crate::bound::{Bound, Edge};
        use crate::config::{EdgeSide, PolygonType};
        use crate::point::Point;

        let left_edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(-5.0_f64, 10.0_f64),
        )];
        let right_edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
        )];

        let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
        let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);

        let lm = LocalMinimum::new(left_bound, right_bound, 0.0_f64, false);

        assert_eq!(lm.y, 0.0_f64);
        assert!(!lm.minimum_has_horizontal);
    }

    #[test]
    fn local_minimum_with_horizontal_flag_true() {
        use super::LocalMinimum;
        use crate::bound::{Bound, Edge};
        use crate::config::{EdgeSide, PolygonType};
        use crate::point::Point;

        let left_edges = vec![Edge::new(
            Point::new(0.0_f64, 5.0_f64),
            Point::new(-5.0_f64, 15.0_f64),
        )];
        let right_edges = vec![Edge::new(
            Point::new(0.0_f64, 5.0_f64),
            Point::new(5.0_f64, 15.0_f64),
        )];

        let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
        let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);

        let lm = LocalMinimum::new(left_bound, right_bound, 5.0_f64, true);

        assert_eq!(lm.y, 5.0_f64);
        assert!(lm.minimum_has_horizontal);
    }

    #[test]
    fn local_minimum_stores_bounds_correctly() {
        use super::LocalMinimum;
        use crate::bound::{Bound, Edge};
        use crate::config::{EdgeSide, PolygonType};
        use crate::point::Point;

        let left_edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(-5.0_f64, 10.0_f64),
        )];
        let right_edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
        )];

        let left_bound = Bound::new(left_edges.clone(), PolygonType::Subject, EdgeSide::Left);
        let right_bound = Bound::new(right_edges.clone(), PolygonType::Clip, EdgeSide::Right);

        let lm = LocalMinimum::new(left_bound, right_bound, 0.0_f64, false);

        assert_eq!(lm.left_bound.poly_type, PolygonType::Subject);
        assert_eq!(lm.right_bound.poly_type, PolygonType::Clip);
        assert_eq!(lm.left_bound.edges.len(), 1);
        assert_eq!(lm.right_bound.edges.len(), 1);
    }

    #[test]
    fn local_minimum_with_i64_coordinates() {
        use super::LocalMinimum;
        use crate::bound::{Bound, Edge};
        use crate::config::{EdgeSide, PolygonType};
        use crate::point::Point;

        let left_edges = vec![Edge::new(
            Point::new(0_i64, 0_i64),
            Point::new(-5_i64, 10_i64),
        )];
        let right_edges = vec![Edge::new(
            Point::new(0_i64, 0_i64),
            Point::new(5_i64, 10_i64),
        )];

        let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
        let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);

        let lm = LocalMinimum::new(left_bound, right_bound, 0_i64, false);

        assert_eq!(lm.y, 0_i64);
    }

    // ==================== LocalMinimum Sorting Tests ====================

    #[test]
    fn local_minimum_sorting_by_y_descending() {
        // From C++: local_minimum_sorter sorts by y descending (larger y first)
        // locMin2->y < locMin1->y means we want descending order
        use super::LocalMinimum;
        use crate::bound::{Bound, Edge};
        use crate::config::{EdgeSide, PolygonType};
        use crate::point::Point;

        fn make_lm(y: f64) -> LocalMinimum<f64> {
            let left_edges = vec![Edge::new(
                Point::new(0.0_f64, y),
                Point::new(-5.0_f64, y + 10.0),
            )];
            let right_edges = vec![Edge::new(
                Point::new(0.0_f64, y),
                Point::new(5.0_f64, y + 10.0),
            )];
            let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
            let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);
            LocalMinimum::new(left_bound, right_bound, y, false)
        }

        let mut minima = [make_lm(5.0), make_lm(10.0), make_lm(0.0), make_lm(7.0)];

        // Sort using the sorter (descending by y)
        minima.sort_by(LocalMinimum::compare);

        // Should be in descending y order: 10.0, 7.0, 5.0, 0.0
        assert_eq!(minima[0].y, 10.0);
        assert_eq!(minima[1].y, 7.0);
        assert_eq!(minima[2].y, 5.0);
        assert_eq!(minima[3].y, 0.0);
    }

    #[test]
    fn local_minimum_sorting_horizontal_comes_first_when_same_y() {
        // From C++: When y values are equal, horizontal minima come first
        // locMin2->minimum_has_horizontal != locMin1->minimum_has_horizontal && locMin1->minimum_has_horizontal
        use super::LocalMinimum;
        use crate::bound::{Bound, Edge};
        use crate::config::{EdgeSide, PolygonType};
        use crate::point::Point;

        fn make_lm(y: f64, has_horizontal: bool) -> LocalMinimum<f64> {
            let left_edges = vec![Edge::new(
                Point::new(0.0_f64, y),
                Point::new(-5.0_f64, y + 10.0),
            )];
            let right_edges = vec![Edge::new(
                Point::new(0.0_f64, y),
                Point::new(5.0_f64, y + 10.0),
            )];
            let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
            let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);
            LocalMinimum::new(left_bound, right_bound, y, has_horizontal)
        }

        let mut minima = [make_lm(5.0, false), make_lm(5.0, true), make_lm(5.0, false)];

        minima.sort_by(LocalMinimum::compare);

        // The one with horizontal should come first
        assert!(minima[0].minimum_has_horizontal);
        assert!(!minima[1].minimum_has_horizontal);
        assert!(!minima[2].minimum_has_horizontal);
    }

    // ==================== LocalMinimumList Tests ====================

    #[test]
    fn local_minimum_list_is_vec_of_local_minima() {
        use super::{LocalMinimum, LocalMinimumList};
        use crate::bound::{Bound, Edge};
        use crate::config::{EdgeSide, PolygonType};
        use crate::point::Point;

        fn make_lm(y: f64) -> LocalMinimum<f64> {
            let left_edges = vec![Edge::new(
                Point::new(0.0_f64, y),
                Point::new(-5.0_f64, y + 10.0),
            )];
            let right_edges = vec![Edge::new(
                Point::new(0.0_f64, y),
                Point::new(5.0_f64, y + 10.0),
            )];
            let left_bound = Bound::new(left_edges, PolygonType::Subject, EdgeSide::Left);
            let right_bound = Bound::new(right_edges, PolygonType::Subject, EdgeSide::Right);
            LocalMinimum::new(left_bound, right_bound, y, false)
        }

        let list: LocalMinimumList<f64> = vec![make_lm(0.0), make_lm(5.0)];

        assert_eq!(list.len(), 2);
        assert_eq!(list[0].y, 0.0);
        assert_eq!(list[1].y, 5.0);
    }
}
