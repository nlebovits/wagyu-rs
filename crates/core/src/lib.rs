//! Wagyu - Geometry Boolean Operations
//!
//! A Rust port of [Mapbox wagyu](https://github.com/mapbox/wagyu), providing:
//!
//! - Union
//! - Intersection
//! - Difference
//! - XOR
//!
//! Output geometry is guaranteed valid and simple per OGC standards.
//!
//! # Example
//!
//! ```rust,ignore
//! use wagyu_rs::{BooleanOp, Operation};
//!
//! let polygon_a = // ...
//! let polygon_b = // ...
//! let result = polygon_a.boolean_op(&polygon_b, Operation::Union);
//! ```

pub mod active_edge_list;
pub mod almost_equal;
pub mod bound;
pub mod bubble_sort;
pub mod build_edges;
pub mod build_local_minima_list;
pub mod build_result;
pub mod config;
pub mod error;
pub mod interrupt;
pub mod intersect;
pub mod intersect_util;
pub mod local_minimum;
pub mod local_minimum_util;
pub mod point;
pub mod process_horizontal;
pub mod process_maxima;
pub mod quick_clip;
pub mod ring;
pub mod ring_util;
pub mod scanbeam;
pub mod snap_rounding;
pub mod topology_correction;
pub mod util;
pub mod vatti;
pub mod wagyu;

pub use active_edge_list::ActiveEdgeList;
pub use bound::{Bound, Edge};
pub use build_edges::{build_edge_list, slopes_equal, EdgeList};
pub use build_local_minima_list::{add_linear_ring, add_ring_to_local_minima_list};
pub use build_result::{build_result, ring_to_linestring, RingManager};
pub use intersect::{IntersectList, IntersectNode};
pub use local_minimum::{LocalMinimum, LocalMinimumList};
pub use point::{Point, Point64, PointF64};
pub use ring::Ring;
pub use scanbeam::Scanbeam;

pub use ring_util::{
    box2_contains_box1, centroid_of_three_points, get_bottom_point_index, get_dx,
    greater_than_or_equal, is_convex, point_in_polygon, ring_area, round_towards_max,
    round_towards_min, value_is_zero, values_are_equal, BBox, PointInPolygonResult,
};
pub use topology_correction::{
    compare_points, find_collinear_sequences, find_duplicate_points, needs_orientation_reversal,
    points_are_collinear, poly2_contains_poly1, remove_collinear_points, reverse_ring,
    ring_has_self_intersection, segments_intersect, sort_rings_largest_to_smallest,
    sort_rings_smallest_to_largest, PointIndexPair,
};

// Golden test harness - only compiled for tests
#[cfg(test)]
mod golden;

pub use config::{FillType, PolygonType};
pub use error::WagyuError;
pub use vatti::execute_vatti;
pub use wagyu::{BoundingBox, Coord, MultiPolygon, Polygon, Wagyu};

/// Boolean operation types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Operation {
    /// Union of two polygons
    Union,
    /// Intersection of two polygons
    Intersection,
    /// Difference (A - B)
    Difference,
    /// Exclusive or (symmetric difference)
    Xor,
}
