//! Bound and Edge types for the Vatti clipping algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/bound.hpp
//!            wagyu/include/mapbox/geometry/wagyu/edge.hpp
//!
//! A `Bound` represents one side of a local minimum in the polygon,
//! containing a sequence of edges that go from the local minimum up
//! (or down) to a local maximum.

use crate::config::{EdgeSide, PolygonType};
use crate::point::Point;
use geo_types::CoordNum;

// ============================================================================
// Edge
// ============================================================================

/// An edge in a polygon, defined by its bottom and top points.
///
/// From C++: `struct edge { point<T> bot; point<T> top; double dx; }`
///
/// The `dx` field is the inverse slope (change in x per unit change in y),
/// used for efficient scanline intersection calculations. For horizontal edges,
/// `dx` is set to infinity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge<T: CoordNum> {
    /// The bottom point of the edge (lower y coordinate).
    pub bot: Point<T>,
    /// The top point of the edge (higher y coordinate).
    pub top: Point<T>,
    /// Inverse slope: (top.x - bot.x) / (top.y - bot.y).
    /// For horizontal edges (dy == 0), this is infinity.
    pub dx: f64,
}

impl<T: CoordNum> Edge<T> {
    /// Create a new edge from bottom to top point.
    ///
    /// Computes the inverse slope (dx) automatically.
    /// For horizontal edges (same y coordinate), dx is set to infinity.
    pub fn new(bot: Point<T>, top: Point<T>) -> Self {
        let bot_y = bot.y.to_f64().unwrap_or(0.0);
        let top_y = top.y.to_f64().unwrap_or(0.0);
        let bot_x = bot.x.to_f64().unwrap_or(0.0);
        let top_x = top.x.to_f64().unwrap_or(0.0);

        let dy = top_y - bot_y;
        let dx = if dy.abs() < f64::EPSILON {
            f64::INFINITY
        } else {
            (top_x - bot_x) / dy
        };

        Self { bot, top, dx }
    }

    /// Returns true if this edge is horizontal (dy == 0).
    ///
    /// From C++: `is_horizontal(edge<T> const& e) { return std::isinf(e.dx); }`
    pub fn is_horizontal(&self) -> bool {
        self.dx.is_infinite()
    }
}

// ============================================================================
// Bound
// ============================================================================

/// Represents one side of a local minimum in the sweep line algorithm.
///
/// A bound contains a sequence of edges that go from a local minimum
/// to a local maximum. During the sweep, bounds are processed from
/// bottom to top, tracking winding counts and building output rings.
///
/// From C++: `struct bound<T>` with fields for edges, current position,
/// winding counts, and output ring assignment.
#[derive(Debug, Clone)]
pub struct Bound<T: CoordNum> {
    /// The edges in this bound, ordered from local minimum to local maximum.
    pub edges: Vec<Edge<T>>,

    /// Index of the currently active edge in the `edges` vector.
    pub current_edge_index: usize,

    /// Current x position at the sweep line.
    /// Initially set to the x coordinate of the first edge's bottom point.
    pub current_x: f64,

    /// Whether this bound belongs to the subject or clip polygon.
    pub poly_type: PolygonType,

    /// Which side (left or right) this bound is on.
    pub side: EdgeSide,

    /// Winding count for this polygon type.
    pub winding_count: i32,

    /// Winding count for the other polygon type.
    pub winding_count2: i32,

    /// Winding delta: 1 or -1 depending on winding direction, 0 for linestrings.
    ///
    /// From C++: `std::int8_t winding_delta; // 1 or -1 depending on winding direction - 0 for linestrings`
    pub winding_delta: i8,

    /// Index to the output ring this bound is contributing to, if any.
    pub ring: Option<usize>,
}

impl<T: CoordNum> Bound<T> {
    /// Create a new bound from a list of edges.
    ///
    /// The bound starts at the first edge, with `current_x` set to the
    /// x coordinate of that edge's bottom point.
    ///
    /// # Panics
    ///
    /// Panics if `edges` is empty.
    pub fn new(edges: Vec<Edge<T>>, poly_type: PolygonType, side: EdgeSide) -> Self {
        assert!(!edges.is_empty(), "Bound must have at least one edge");

        let current_x = edges[0].bot.x.to_f64().unwrap_or(0.0);

        Self {
            edges,
            current_edge_index: 0,
            current_x,
            poly_type,
            side,
            winding_count: 0,
            winding_count2: 0,
            winding_delta: 0,
            ring: None,
        }
    }

    /// Create a new bound from a list of edges with a specified winding delta.
    ///
    /// The winding delta is typically 1 or -1 depending on the winding direction,
    /// or 0 for linestrings.
    ///
    /// # Panics
    ///
    /// Panics if `edges` is empty.
    pub fn new_with_delta(
        edges: Vec<Edge<T>>,
        poly_type: PolygonType,
        side: EdgeSide,
        winding_delta: i8,
    ) -> Self {
        assert!(!edges.is_empty(), "Bound must have at least one edge");

        let current_x = edges[0].bot.x.to_f64().unwrap_or(0.0);

        Self {
            edges,
            current_edge_index: 0,
            current_x,
            poly_type,
            side,
            winding_count: 0,
            winding_count2: 0,
            winding_delta,
            ring: None,
        }
    }

    /// Create a placeholder bound with a degenerate edge.
    ///
    /// This is used internally when we need to move bounds out of a LocalMinimum
    /// and need to leave a valid (but useless) bound in place.
    ///
    /// DIVERGENCE FROM WAGYU: C++ uses pointers/references and doesn't need this.
    /// In Rust, we need ownership transfer, so we create placeholders.
    pub fn new_empty(poly_type: PolygonType, side: EdgeSide) -> Self {
        // Create a degenerate edge at origin
        let origin = Point::new(T::zero(), T::zero());
        let degenerate_edge = Edge::new(origin, origin);

        Self {
            edges: vec![degenerate_edge],
            current_edge_index: 0,
            current_x: 0.0,
            poly_type,
            side,
            winding_count: 0,
            winding_count2: 0,
            winding_delta: 0,
            ring: None,
        }
    }

    /// Returns a reference to the currently active edge.
    pub fn current_edge(&self) -> &Edge<T> {
        &self.edges[self.current_edge_index]
    }

    /// Advance to the next edge in the bound.
    ///
    /// Returns `true` if there was a next edge to advance to,
    /// `false` if we're already at the last edge.
    pub fn next_edge(&mut self) -> bool {
        if self.current_edge_index + 1 < self.edges.len() {
            self.current_edge_index += 1;
            true
        } else {
            false
        }
    }

    /// Returns true if the current edge is horizontal.
    pub fn is_horizontal(&self) -> bool {
        self.current_edge().is_horizontal()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Edge Tests ====================

    #[test]
    fn edge_new_creates_edge_with_bot_and_top() {
        // An edge should store bot and top points
        let bot = Point::new(0.0_f64, 0.0_f64);
        let top = Point::new(10.0_f64, 20.0_f64);
        let edge = Edge::new(bot, top);

        assert_eq!(edge.bot, bot);
        assert_eq!(edge.top, top);
    }

    #[test]
    fn edge_computes_dx_as_inverse_slope() {
        // dx = (top.x - bot.x) / (top.y - bot.y)
        // For a line from (0,0) to (10,20): dx = 10/20 = 0.5
        let bot = Point::new(0.0_f64, 0.0_f64);
        let top = Point::new(10.0_f64, 20.0_f64);
        let edge = Edge::new(bot, top);

        assert!((edge.dx - 0.5).abs() < 1e-10);
    }

    #[test]
    fn edge_horizontal_has_infinite_dx() {
        // Horizontal edge: dy = 0, so dx = infinity
        let bot = Point::new(0.0_f64, 5.0_f64);
        let top = Point::new(10.0_f64, 5.0_f64);
        let edge = Edge::new(bot, top);

        assert!(edge.dx.is_infinite());
    }

    #[test]
    fn edge_is_horizontal_returns_true_for_horizontal_edge() {
        let bot = Point::new(0.0_f64, 5.0_f64);
        let top = Point::new(10.0_f64, 5.0_f64);
        let edge = Edge::new(bot, top);

        assert!(edge.is_horizontal());
    }

    #[test]
    fn edge_is_horizontal_returns_false_for_non_horizontal_edge() {
        let bot = Point::new(0.0_f64, 0.0_f64);
        let top = Point::new(10.0_f64, 20.0_f64);
        let edge = Edge::new(bot, top);

        assert!(!edge.is_horizontal());
    }

    #[test]
    fn edge_negative_slope() {
        // Line going from bottom-right to top-left
        // From (20, 0) to (0, 10): dx = (0 - 20) / (10 - 0) = -2.0
        let bot = Point::new(20.0_f64, 0.0_f64);
        let top = Point::new(0.0_f64, 10.0_f64);
        let edge = Edge::new(bot, top);

        assert!((edge.dx - (-2.0)).abs() < 1e-10);
    }

    #[test]
    fn edge_vertical_has_zero_dx() {
        // Vertical edge: dx = 0, dy != 0
        // From (5, 0) to (5, 10): dx = (5 - 5) / (10 - 0) = 0
        let bot = Point::new(5.0_f64, 0.0_f64);
        let top = Point::new(5.0_f64, 10.0_f64);
        let edge = Edge::new(bot, top);

        assert!((edge.dx - 0.0).abs() < 1e-10);
        assert!(!edge.is_horizontal());
    }

    #[test]
    fn edge_with_i64_coordinates() {
        // Test that Edge works with i64 coordinates
        let bot = Point::new(0_i64, 0_i64);
        let top = Point::new(10_i64, 20_i64);
        let edge = Edge::new(bot, top);

        assert!((edge.dx - 0.5).abs() < 1e-10);
    }

    // ==================== Bound Tests ====================

    #[test]
    fn bound_new_creates_bound_with_edges() {
        let edges = vec![
            Edge::new(Point::new(0.0_f64, 0.0_f64), Point::new(5.0_f64, 10.0_f64)),
            Edge::new(
                Point::new(5.0_f64, 10.0_f64),
                Point::new(10.0_f64, 20.0_f64),
            ),
        ];

        let bound = Bound::new(edges.clone(), PolygonType::Subject, EdgeSide::Left);

        assert_eq!(bound.edges.len(), 2);
        assert_eq!(bound.poly_type, PolygonType::Subject);
        assert_eq!(bound.side, EdgeSide::Left);
    }

    #[test]
    fn bound_current_edge_returns_first_edge_initially() {
        let edges = vec![
            Edge::new(Point::new(0.0_f64, 0.0_f64), Point::new(5.0_f64, 10.0_f64)),
            Edge::new(
                Point::new(5.0_f64, 10.0_f64),
                Point::new(10.0_f64, 20.0_f64),
            ),
        ];

        let bound = Bound::new(edges.clone(), PolygonType::Subject, EdgeSide::Left);

        assert_eq!(bound.current_edge().bot, edges[0].bot);
        assert_eq!(bound.current_edge().top, edges[0].top);
    }

    #[test]
    fn bound_next_edge_advances_to_next_edge() {
        let edges = vec![
            Edge::new(Point::new(0.0_f64, 0.0_f64), Point::new(5.0_f64, 10.0_f64)),
            Edge::new(
                Point::new(5.0_f64, 10.0_f64),
                Point::new(10.0_f64, 20.0_f64),
            ),
        ];

        let mut bound = Bound::new(edges.clone(), PolygonType::Subject, EdgeSide::Left);

        // Should return true and advance to second edge
        assert!(bound.next_edge());
        assert_eq!(bound.current_edge().bot, edges[1].bot);
        assert_eq!(bound.current_edge().top, edges[1].top);
    }

    #[test]
    fn bound_next_edge_returns_false_when_no_more_edges() {
        let edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
        )];

        let mut bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);

        // Only one edge, so next_edge should return false
        assert!(!bound.next_edge());
    }

    #[test]
    fn bound_is_horizontal_delegates_to_current_edge() {
        let horizontal_edge =
            Edge::new(Point::new(0.0_f64, 5.0_f64), Point::new(10.0_f64, 5.0_f64));
        let non_horizontal_edge =
            Edge::new(Point::new(0.0_f64, 0.0_f64), Point::new(5.0_f64, 10.0_f64));

        let bound_h = Bound::new(vec![horizontal_edge], PolygonType::Subject, EdgeSide::Left);
        let bound_nh = Bound::new(
            vec![non_horizontal_edge],
            PolygonType::Subject,
            EdgeSide::Left,
        );

        assert!(bound_h.is_horizontal());
        assert!(!bound_nh.is_horizontal());
    }

    #[test]
    fn bound_winding_counts_default_to_zero() {
        let edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
        )];

        let bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);

        assert_eq!(bound.winding_count, 0);
        assert_eq!(bound.winding_count2, 0);
    }

    #[test]
    fn bound_ring_defaults_to_none() {
        let edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
        )];

        let bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);

        assert!(bound.ring.is_none());
    }

    #[test]
    fn bound_current_x_defaults_to_bot_x_of_first_edge() {
        let edges = vec![Edge::new(
            Point::new(7.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
        )];

        let bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);

        assert!((bound.current_x - 7.0).abs() < 1e-10);
    }

    #[test]
    fn bound_with_clip_polygon_type() {
        let edges = vec![Edge::new(
            Point::new(0.0_f64, 0.0_f64),
            Point::new(5.0_f64, 10.0_f64),
        )];

        let bound = Bound::new(edges, PolygonType::Clip, EdgeSide::Right);

        assert_eq!(bound.poly_type, PolygonType::Clip);
        assert_eq!(bound.side, EdgeSide::Right);
    }

    #[test]
    fn bound_with_i64_coordinates() {
        // Test that Bound works with i64 coordinates
        let edges = vec![Edge::new(
            Point::new(0_i64, 0_i64),
            Point::new(5_i64, 10_i64),
        )];

        let bound = Bound::new(edges, PolygonType::Subject, EdgeSide::Left);

        assert_eq!(bound.current_x, 0.0);
    }
}
