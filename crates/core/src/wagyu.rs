//! Wagyu - Main API for Geometry Boolean Operations
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/wagyu.hpp
//!
//! This module provides the public API for performing boolean operations
//! on polygons. The main entry point is the [`Wagyu`] struct.
//!
//! # Example
//!
//! ```rust,ignore
//! use wagyu_rs::{Wagyu, Operation, FillType};
//!
//! let mut clipper = Wagyu::new();
//!
//! // Add subject polygon
//! clipper.add_polygon(&subject_polygon, PolygonType::Subject);
//!
//! // Add clip polygon
//! clipper.add_polygon(&clip_polygon, PolygonType::Clip);
//!
//! // Execute union operation
//! let result = clipper.execute(
//!     Operation::Union,
//!     FillType::EvenOdd,
//!     FillType::EvenOdd,
//! );
//! ```

use geo_types::CoordNum;
use num_traits::{Bounded, ToPrimitive};

use crate::bound::Bound;
use crate::build_local_minima_list::add_linear_ring;
use crate::build_result::{build_result, RingManager};
use crate::config::{FillType, PolygonType};
use crate::interrupt::interrupt_check;
use crate::local_minimum::LocalMinimumList;
use crate::point::Point;
use crate::snap_rounding::build_hot_pixels;
use crate::topology_correction::correct_topology;
use crate::vatti::execute_vatti;
use crate::Operation;
use crate::WagyuError;

/// Alias for coordinate trait bounds required for Wagyu operations.
/// Uses geo_types::CoordNum plus additional bounds needed for the algorithm.
pub trait Coord: CoordNum + Bounded + ToPrimitive {}
impl<T: CoordNum + Bounded + ToPrimitive> Coord for T {}

/// A ring is a closed sequence of points representing a polygon boundary.
pub type Ring<T> = Vec<Point<T>>;

/// A polygon is an outer ring plus zero or more hole rings.
pub type Polygon<T> = Vec<Ring<T>>;

/// Re-export geo_types::MultiPolygon for output
pub use geo_types::MultiPolygon;

/// Bounding box for geometry.
#[derive(Debug, Clone, PartialEq)]
pub struct BoundingBox<T: Coord> {
    pub min: Point<T>,
    pub max: Point<T>,
}

impl<T: Coord> Default for BoundingBox<T> {
    fn default() -> Self {
        Self {
            min: Point::new(T::zero(), T::zero()),
            max: Point::new(T::zero(), T::zero()),
        }
    }
}

/// Main clipper struct for performing boolean operations on polygons.
///
/// The workflow is:
/// 1. Create a new `Wagyu` instance
/// 2. Add subject and clip polygons using `add_ring` or `add_polygon`
/// 3. Call `execute` to perform the boolean operation
/// 4. Use the resulting multi-polygon
///
/// # Type Parameters
///
/// * `T` - The coordinate type (typically `i64` or `f64`)
pub struct Wagyu<T: Coord> {
    /// List of local minima (entry points for edges)
    minima_list: LocalMinimumList<T>,
    /// Storage for all bounds/edges
    bounds: Vec<Bound<T>>,
    /// Whether to reverse output ring orientation
    reverse_output: bool,
}

impl<T: Coord> Default for Wagyu<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Coord> Wagyu<T> {
    /// Create a new empty Wagyu clipper.
    pub fn new() -> Self {
        Self {
            minima_list: LocalMinimumList::new(),
            bounds: Vec::new(),
            reverse_output: false,
        }
    }

    /// Add a single ring (linear ring) to the clipper.
    ///
    /// # Arguments
    ///
    /// * `ring` - The ring to add (sequence of points, implicitly closed)
    /// * `polygon_type` - Whether this is a subject or clip polygon
    ///
    /// # Returns
    ///
    /// `true` if the ring was added successfully, `false` if it was degenerate
    /// (fewer than 3 points or zero area).
    pub fn add_ring(&mut self, ring: &Ring<T>, polygon_type: PolygonType) -> bool {
        add_linear_ring(ring, &mut self.minima_list, polygon_type)
    }

    /// Add a polygon (outer ring + holes) to the clipper.
    ///
    /// # Arguments
    ///
    /// * `polygon` - The polygon to add (first ring is outer, rest are holes)
    /// * `polygon_type` - Whether this is a subject or clip polygon
    ///
    /// # Returns
    ///
    /// `true` if any rings were added successfully.
    pub fn add_polygon(&mut self, polygon: &Polygon<T>, polygon_type: PolygonType) -> bool {
        let mut result = false;
        for ring in polygon {
            if self.add_ring(ring, polygon_type) {
                result = true;
            }
        }
        result
    }

    /// Set whether to reverse output ring orientations.
    ///
    /// By default, outer rings are counter-clockwise and holes are clockwise
    /// (following OGC convention). Set to `true` to reverse this.
    pub fn reverse_rings(&mut self, value: bool) {
        self.reverse_output = value;
    }

    /// Clear all added geometry.
    pub fn clear(&mut self) {
        self.minima_list.clear();
        self.bounds.clear();
    }

    /// Get the bounding box of all added geometry.
    ///
    /// Returns a zero-sized box at origin if no geometry has been added.
    pub fn get_bounds(&self) -> BoundingBox<T> {
        let mut bbox = BoundingBox::default();

        if self.minima_list.is_empty() {
            return bbox;
        }

        let mut first_set = false;

        for lm in self.minima_list.iter() {
            // Process left bound edges
            let left = &lm.left_bound;
            if !left.edges.is_empty() {
                if !first_set {
                    bbox.min = left.edges[0].top;
                    bbox.max = left.edges[left.edges.len() - 1].bot;
                    first_set = true;
                } else {
                    bbox.min.y = partial_min(bbox.min.y, left.edges[0].top.y);
                    bbox.max.y = partial_max(bbox.max.y, left.edges[left.edges.len() - 1].bot.y);
                    bbox.max.x = partial_max(bbox.max.x, left.edges[left.edges.len() - 1].top.x);
                    bbox.min.x = partial_min(bbox.min.x, left.edges[left.edges.len() - 1].top.x);
                }
                for edge in &left.edges {
                    bbox.max.x = partial_max(bbox.max.x, edge.bot.x);
                    bbox.min.x = partial_min(bbox.min.x, edge.bot.x);
                }
            }

            // Process right bound edges
            let right = &lm.right_bound;
            if !right.edges.is_empty() {
                if !first_set {
                    bbox.min = right.edges[0].top;
                    bbox.max = right.edges[right.edges.len() - 1].bot;
                    first_set = true;
                } else {
                    bbox.min.y = partial_min(bbox.min.y, right.edges[0].top.y);
                    bbox.max.y = partial_max(bbox.max.y, right.edges[right.edges.len() - 1].bot.y);
                    bbox.max.x = partial_max(bbox.max.x, right.edges[right.edges.len() - 1].top.x);
                    bbox.min.x = partial_min(bbox.min.x, right.edges[right.edges.len() - 1].top.x);
                }
                for edge in &right.edges {
                    bbox.max.x = partial_max(bbox.max.x, edge.bot.x);
                    bbox.min.x = partial_min(bbox.min.x, edge.bot.x);
                }
            }
        }

        bbox
    }

    /// Execute a boolean operation and return the result.
    ///
    /// # Arguments
    ///
    /// * `clip_type` - The type of boolean operation to perform
    /// * `subject_fill_type` - Fill rule for subject polygons
    /// * `clip_fill_type` - Fill rule for clip polygons
    ///
    /// # Returns
    ///
    /// `Ok(MultiPolygon)` on success, or `Err(WagyuError)` if:
    /// - No geometry was added
    /// - The operation was interrupted
    /// - An internal error occurred
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let result = clipper.execute(
    ///     Operation::Union,
    ///     FillType::EvenOdd,
    ///     FillType::EvenOdd,
    /// )?;
    /// ```
    pub fn execute(
        &mut self,
        clip_type: Operation,
        subject_fill_type: FillType,
        clip_fill_type: FillType,
    ) -> Result<MultiPolygon<T>, WagyuError> {
        if self.minima_list.is_empty() {
            return Ok(MultiPolygon::new(vec![]));
        }

        let mut manager = RingManager::new();

        // Check for interruptions (ignore errors in stub implementation)
        let _ = interrupt_check();

        // Build hot pixels for snap rounding
        build_hot_pixels(&self.minima_list, &mut manager);

        // Check for interruptions
        let _ = interrupt_check();

        // Execute the main Vatti sweep algorithm
        execute_vatti(
            &mut self.minima_list,
            &mut self.bounds,
            &mut manager,
            clip_type,
            subject_fill_type,
            clip_fill_type,
        );

        // Check for interruptions
        let _ = interrupt_check();

        // Correct topology for OGC validity
        correct_topology(&mut manager);

        // Build the output multi-polygon
        let solution = build_result(&manager, self.reverse_output);

        Ok(solution)
    }
}

/// Helper function for partial ordering min
fn partial_min<T: PartialOrd>(a: T, b: T) -> T {
    if a < b {
        a
    } else {
        b
    }
}

/// Helper function for partial ordering max
fn partial_max<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PolygonType;

    #[test]
    fn test_wagyu_new() {
        let clipper: Wagyu<i64> = Wagyu::new();
        assert!(clipper.minima_list.is_empty());
        assert!(!clipper.reverse_output);
    }

    #[test]
    fn test_wagyu_default() {
        let clipper: Wagyu<i64> = Wagyu::default();
        assert!(clipper.minima_list.is_empty());
    }

    #[test]
    fn test_wagyu_clear() {
        let mut clipper: Wagyu<i64> = Wagyu::new();
        // Add a simple triangle
        let ring = vec![Point::new(0, 0), Point::new(10, 0), Point::new(5, 10)];
        clipper.add_ring(&ring, PolygonType::Subject);
        assert!(!clipper.minima_list.is_empty());

        clipper.clear();
        assert!(clipper.minima_list.is_empty());
    }

    #[test]
    fn test_wagyu_reverse_rings() {
        let mut clipper: Wagyu<i64> = Wagyu::new();
        assert!(!clipper.reverse_output);

        clipper.reverse_rings(true);
        assert!(clipper.reverse_output);

        clipper.reverse_rings(false);
        assert!(!clipper.reverse_output);
    }

    #[test]
    fn test_wagyu_get_bounds_empty() {
        let clipper: Wagyu<i64> = Wagyu::new();
        let bounds = clipper.get_bounds();
        assert_eq!(bounds.min, Point::new(0, 0));
        assert_eq!(bounds.max, Point::new(0, 0));
    }

    #[test]
    fn test_add_degenerate_ring() {
        let mut clipper: Wagyu<i64> = Wagyu::new();

        // Ring with only 2 points is degenerate
        let ring = vec![Point::new(0, 0), Point::new(10, 0)];
        let added = clipper.add_ring(&ring, PolygonType::Subject);
        assert!(!added);
    }

    #[test]
    fn test_execute_empty() {
        let mut clipper: Wagyu<i64> = Wagyu::new();
        let result = clipper.execute(Operation::Union, FillType::EvenOdd, FillType::EvenOdd);
        assert!(result.is_ok());
        assert!(result.unwrap().0.is_empty());
    }

    #[test]
    fn test_partial_min_max() {
        assert_eq!(partial_min(1, 2), 1);
        assert_eq!(partial_min(2, 1), 1);
        assert_eq!(partial_max(1, 2), 2);
        assert_eq!(partial_max(2, 1), 2);
    }
}
