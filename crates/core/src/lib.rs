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
//! use wagyu_core::{BooleanOp, Operation};
//!
//! let polygon_a = // ...
//! let polygon_b = // ...
//! let result = polygon_a.boolean_op(&polygon_b, Operation::Union);
//! ```

pub mod bound;
pub mod config;
pub mod error;
pub mod point;
pub mod ring;

pub use bound::{Bound, Edge};
pub use point::{Point, Point64, PointF64};
pub use ring::Ring;

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
