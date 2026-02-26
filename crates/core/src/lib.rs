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
pub mod bound;
pub mod build_edges;
pub mod build_local_minima_list;
pub mod build_result;
pub mod config;
pub mod error;
pub mod intersect;
pub mod local_minimum;
pub mod point;
pub mod ring;
pub mod scanbeam;
pub mod snap_rounding;

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

// Golden test harness - only compiled for tests
#[cfg(test)]
mod golden;

pub use error::WagyuError;

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
