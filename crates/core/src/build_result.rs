//! Build Result - Construct final polygon output from rings.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/build_result.hpp
//!
//! This module provides functions to convert the algorithm's ring output
//! into proper geo_types polygon structures. The main entry point is
//! `build_result()` which takes a collection of rings and produces a
//! `MultiPolygon`.
//!
//! The C++ implementation has three functions:
//! - `push_ring_to_polygon`: Converts a Ring to LinearRing
//! - `build_result_polygons`: Recursively builds polygon hierarchy
//! - `build_result`: Entry point

use geo_types::{Coord, CoordNum, LineString, MultiPolygon, Polygon};

use crate::point::Point;
use crate::Ring;

/// A manager for rings during polygon construction.
///
/// This struct holds all rings and tracks which are top-level exterior rings
/// (those without a parent). The ring hierarchy is represented through
/// parent/child indices in the Ring structs themselves.
#[derive(Debug, Default)]
pub struct RingManager<T: CoordNum> {
    /// All rings indexed by their ring_index
    rings: Vec<Ring<T>>,
    /// Indices of top-level exterior rings (rings with no parent)
    top_level_rings: Vec<usize>,
    /// Hot pixels for snap rounding (grid points needing special handling)
    pub hot_pixels: Vec<Point<T>>,
    /// Current index into hot_pixels during sweep
    pub current_hp_idx: usize,
}

impl<T: CoordNum> RingManager<T> {
    /// Create a new empty ring manager.
    pub fn new() -> Self {
        RingManager {
            rings: Vec::new(),
            top_level_rings: Vec::new(),
            hot_pixels: Vec::new(),
            current_hp_idx: 0,
        }
    }

    /// Add a ring to the manager.
    ///
    /// Returns the index assigned to the ring.
    pub fn add_ring(&mut self, mut ring: Ring<T>) -> usize {
        let index = self.rings.len();
        ring.set_ring_index(index);

        // If this ring has no parent, it's a top-level exterior ring
        if ring.parent().is_none() && !ring.is_hole() {
            self.top_level_rings.push(index);
        }

        self.rings.push(ring);
        index
    }

    /// Get a reference to a ring by index.
    pub fn get(&self, index: usize) -> Option<&Ring<T>> {
        self.rings.get(index)
    }

    /// Get a mutable reference to a ring by index.
    pub fn get_mut(&mut self, index: usize) -> Option<&mut Ring<T>> {
        self.rings.get_mut(index)
    }

    /// Returns the indices of top-level exterior rings.
    pub fn top_level_rings(&self) -> &[usize] {
        &self.top_level_rings
    }

    /// Returns the number of rings.
    pub fn len(&self) -> usize {
        self.rings.len()
    }

    /// Returns true if there are no rings.
    pub fn is_empty(&self) -> bool {
        self.rings.is_empty()
    }

    /// Set the parent of a ring and update the parent's children list.
    ///
    /// This maintains the bidirectional relationship between parent and child.
    pub fn set_parent(&mut self, child_index: usize, parent_index: usize) {
        // Set the child's parent
        if let Some(child) = self.rings.get_mut(child_index) {
            child.set_parent(Some(parent_index));
        }

        // Add child to parent's children list
        if let Some(parent) = self.rings.get_mut(parent_index) {
            parent.add_child(child_index);
        }
    }

    /// Update the current hot pixel iterator to skip past hot pixels above scanline_y.
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - update_current_hp_itr
    ///
    /// Hot pixels are sorted by Y descending, so we advance past any with Y > scanline_y.
    ///
    /// # Arguments
    /// * `scanline_y` - The current scanline Y coordinate
    pub fn update_current_hp_itr(&mut self, scanline_y: T) {
        while self.current_hp_idx < self.hot_pixels.len() {
            let hp_y = self.hot_pixels[self.current_hp_idx].y;
            // Hot pixels are sorted by Y descending (larger Y first)
            // Skip while hot pixel Y > scanline_y
            if hp_y > scanline_y {
                self.current_hp_idx += 1;
            } else {
                break;
            }
        }
    }

    /// Returns an iterator over all ring indices.
    pub fn ring_indices(&self) -> impl Iterator<Item = usize> {
        0..self.rings.len()
    }

    /// Clear a ring's parent reference (used during tree rebuilding).
    pub fn clear_parent(&mut self, ring_index: usize) {
        if let Some(ring) = self.rings.get_mut(ring_index) {
            ring.set_parent(None);
        }
    }

    /// Clear a ring's children (used during tree rebuilding).
    pub fn clear_children(&mut self, ring_index: usize) {
        if let Some(ring) = self.rings.get_mut(ring_index) {
            ring.clear_children();
        }
    }

    /// Recalculate top-level rings after tree modifications.
    pub fn recalculate_top_level_rings(&mut self) {
        self.top_level_rings.clear();
        for i in 0..self.rings.len() {
            if self.rings[i].parent().is_none() && !self.rings[i].is_hole() {
                self.top_level_rings.push(i);
            }
        }
    }
}

/// Convert a Ring to a LineString (linear ring in geo_types terminology).
///
/// This function extracts points from the ring and creates a closed
/// LineString suitable for use in a geo_types Polygon.
///
/// # Arguments
/// * `ring` - The ring to convert
/// * `reverse` - If true, reverse the point order
///
/// # Returns
/// A LineString containing the ring's points (closed)
pub fn ring_to_linestring<T: CoordNum + Copy>(ring: &Ring<T>, reverse: bool) -> LineString<T> {
    let points = ring.points();

    if points.is_empty() {
        return LineString::new(Vec::new());
    }

    let mut coords: Vec<Coord<T>> = if reverse {
        points.iter().rev().copied().collect()
    } else {
        points.to_vec()
    };

    // Ensure the ring is closed (first point == last point)
    if let (Some(first), Some(last)) = (coords.first(), coords.last()) {
        if first != last {
            coords.push(*first);
        }
    }

    LineString::new(coords)
}

/// Build a single polygon from an exterior ring and its child holes.
///
/// This function creates a Polygon with the exterior ring and recursively
/// processes child rings (holes). Grandchildren of holes become new
/// exterior rings in the result.
///
/// # Arguments
/// * `manager` - The ring manager containing all rings
/// * `exterior_index` - Index of the exterior ring
/// * `reverse_output` - If true, reverse winding direction
///
/// # Returns
/// A tuple of (Polygon, Vec<usize>) where the vector contains indices of
/// grandchild rings that need to become new polygons.
fn build_polygon<T: CoordNum + Copy>(
    manager: &RingManager<T>,
    exterior_index: usize,
    reverse_output: bool,
) -> (Polygon<T>, Vec<usize>) {
    let mut grandchildren = Vec::new();

    let exterior_ring = match manager.get(exterior_index) {
        Some(r) => r,
        None => {
            return (
                Polygon::new(LineString::new(Vec::new()), Vec::new()),
                grandchildren,
            )
        }
    };

    // Convert exterior ring to LineString
    let exterior = ring_to_linestring(exterior_ring, reverse_output);

    // Process children (holes)
    let mut holes = Vec::new();
    for &hole_index in exterior_ring.children() {
        if let Some(hole_ring) = manager.get(hole_index) {
            // Add the hole with reversed winding (holes have opposite winding to exterior)
            holes.push(ring_to_linestring(hole_ring, !reverse_output));

            // Grandchildren of holes become new exterior rings
            for &grandchild_index in hole_ring.children() {
                grandchildren.push(grandchild_index);
            }
        }
    }

    (Polygon::new(exterior, holes), grandchildren)
}

/// Build the final result as a MultiPolygon from the ring manager.
///
/// This is the main entry point for converting algorithm output to
/// geo_types structures. It processes all top-level exterior rings
/// and recursively handles the ring hierarchy.
///
/// # Arguments
/// * `manager` - The ring manager containing all rings
/// * `reverse_output` - If true, reverse winding direction of output
///
/// # Returns
/// A MultiPolygon containing all the polygons
pub fn build_result<T: CoordNum + Copy>(
    manager: &RingManager<T>,
    reverse_output: bool,
) -> MultiPolygon<T> {
    let mut polygons = Vec::new();

    // Start with top-level exterior rings
    let mut pending_exteriors: Vec<usize> = manager.top_level_rings().to_vec();

    // Process all exterior rings (including grandchildren promoted to exterior)
    while let Some(exterior_index) = pending_exteriors.pop() {
        let (polygon, grandchildren) = build_polygon(manager, exterior_index, reverse_output);

        // Only add non-empty polygons
        if !polygon.exterior().0.is_empty() {
            polygons.push(polygon);
        }

        // Grandchildren become new exterior rings
        pending_exteriors.extend(grandchildren);
    }

    MultiPolygon::new(polygons)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::Coord;

    // ==================== Helper Functions ====================

    fn make_square_ring(size: f64) -> Ring<f64> {
        let mut ring = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: size, y: 0.0 });
        ring.push_point(Coord { x: size, y: size });
        ring.push_point(Coord { x: 0.0, y: size });
        ring
    }

    fn make_square_ring_at(x: f64, y: f64, size: f64) -> Ring<f64> {
        let mut ring = Ring::empty();
        ring.push_point(Coord { x, y });
        ring.push_point(Coord { x: x + size, y });
        ring.push_point(Coord {
            x: x + size,
            y: y + size,
        });
        ring.push_point(Coord { x, y: y + size });
        ring
    }

    // ==================== RingManager Tests ====================

    #[test]
    fn ring_manager_new_is_empty() {
        let manager: RingManager<f64> = RingManager::new();
        assert!(manager.is_empty());
        assert_eq!(manager.len(), 0);
        assert!(manager.top_level_rings().is_empty());
    }

    #[test]
    fn ring_manager_add_ring_assigns_index() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_square_ring(10.0);

        let index = manager.add_ring(ring);

        assert_eq!(index, 0);
        assert_eq!(manager.len(), 1);
        assert!(!manager.is_empty());
    }

    #[test]
    fn ring_manager_add_multiple_rings() {
        let mut manager: RingManager<f64> = RingManager::new();

        let idx0 = manager.add_ring(make_square_ring(10.0));
        let idx1 = manager.add_ring(make_square_ring(20.0));
        let idx2 = manager.add_ring(make_square_ring(30.0));

        assert_eq!(idx0, 0);
        assert_eq!(idx1, 1);
        assert_eq!(idx2, 2);
        assert_eq!(manager.len(), 3);
    }

    #[test]
    fn ring_manager_top_level_rings_tracks_exteriors_without_parent() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Add two exterior rings (no parent)
        manager.add_ring(make_square_ring(10.0));
        manager.add_ring(make_square_ring(20.0));

        // Add a hole (has parent, is_hole = true)
        let mut hole = make_square_ring(5.0);
        hole.set_hole(true);
        hole.set_parent(Some(0));
        manager.add_ring(hole);

        // Only the two exterior rings should be top-level
        assert_eq!(manager.top_level_rings().len(), 2);
        assert!(manager.top_level_rings().contains(&0));
        assert!(manager.top_level_rings().contains(&1));
    }

    #[test]
    fn ring_manager_get_returns_ring() {
        let mut manager: RingManager<f64> = RingManager::new();
        manager.add_ring(make_square_ring(10.0));

        let ring = manager.get(0);
        assert!(ring.is_some());
        assert_eq!(ring.unwrap().len(), 4);
    }

    #[test]
    fn ring_manager_get_invalid_index_returns_none() {
        let manager: RingManager<f64> = RingManager::new();
        assert!(manager.get(0).is_none());
        assert!(manager.get(100).is_none());
    }

    #[test]
    fn ring_manager_set_parent_establishes_relationship() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Add exterior
        let exterior_idx = manager.add_ring(make_square_ring(10.0));

        // Add hole without parent initially
        let mut hole = make_square_ring(5.0);
        hole.set_hole(true);
        let hole_idx = manager.add_ring(hole);

        // Establish parent relationship
        manager.set_parent(hole_idx, exterior_idx);

        // Verify relationship
        let hole = manager.get(hole_idx).unwrap();
        assert_eq!(hole.parent(), Some(exterior_idx));

        let exterior = manager.get(exterior_idx).unwrap();
        assert!(exterior.children().contains(&hole_idx));
    }

    // ==================== ring_to_linestring Tests ====================

    #[test]
    fn ring_to_linestring_empty_ring_returns_empty_linestring() {
        let ring: Ring<f64> = Ring::empty();
        let ls = ring_to_linestring(&ring, false);
        assert!(ls.0.is_empty());
    }

    #[test]
    fn ring_to_linestring_converts_points() {
        let ring = make_square_ring(10.0);
        let ls = ring_to_linestring(&ring, false);

        // Should have 5 points (4 original + 1 to close)
        assert_eq!(ls.0.len(), 5);

        // First point should match
        assert_eq!(ls.0[0], Coord { x: 0.0, y: 0.0 });

        // Should be closed (first == last)
        assert_eq!(ls.0[0], ls.0[4]);
    }

    #[test]
    fn ring_to_linestring_reverse_reverses_point_order() {
        let ring = make_square_ring(10.0);

        let ls_forward = ring_to_linestring(&ring, false);
        let ls_reverse = ring_to_linestring(&ring, true);

        // Forward: (0,0) -> (10,0) -> (10,10) -> (0,10) -> (0,0)
        assert_eq!(ls_forward.0[1], Coord { x: 10.0, y: 0.0 });

        // Reverse: (0,10) -> (10,10) -> (10,0) -> (0,0) -> (0,10)
        assert_eq!(ls_reverse.0[0], Coord { x: 0.0, y: 10.0 });
        assert_eq!(ls_reverse.0[1], Coord { x: 10.0, y: 10.0 });
    }

    #[test]
    fn ring_to_linestring_already_closed_ring_not_doubled() {
        let mut ring: Ring<f64> = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 10.0, y: 0.0 });
        ring.push_point(Coord { x: 10.0, y: 10.0 });
        ring.push_point(Coord { x: 0.0, y: 0.0 }); // Already closed

        let ls = ring_to_linestring(&ring, false);

        // Should still be 4 points (not 5)
        assert_eq!(ls.0.len(), 4);
    }

    // ==================== build_result Tests ====================

    #[test]
    fn build_result_empty_manager_returns_empty_multipolygon() {
        let manager: RingManager<f64> = RingManager::new();
        let result = build_result(&manager, false);

        assert!(result.0.is_empty());
    }

    #[test]
    fn build_result_single_exterior_ring() {
        let mut manager: RingManager<f64> = RingManager::new();
        manager.add_ring(make_square_ring(10.0));

        let result = build_result(&manager, false);

        assert_eq!(result.0.len(), 1);
        let poly = &result.0[0];
        assert_eq!(poly.exterior().0.len(), 5); // 4 points + close
        assert!(poly.interiors().is_empty());
    }

    #[test]
    fn build_result_multiple_exterior_rings() {
        let mut manager: RingManager<f64> = RingManager::new();
        manager.add_ring(make_square_ring_at(0.0, 0.0, 10.0));
        manager.add_ring(make_square_ring_at(20.0, 0.0, 10.0));

        let result = build_result(&manager, false);

        assert_eq!(result.0.len(), 2);
    }

    #[test]
    fn build_result_exterior_with_hole() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Add exterior (10x10 square)
        let exterior_idx = manager.add_ring(make_square_ring(10.0));

        // Add hole (2x2 square inside)
        let mut hole = make_square_ring_at(3.0, 3.0, 2.0);
        hole.set_hole(true);
        let hole_idx = manager.add_ring(hole);

        // Link hole to exterior
        manager.set_parent(hole_idx, exterior_idx);

        let result = build_result(&manager, false);

        assert_eq!(result.0.len(), 1);
        let poly = &result.0[0];
        assert_eq!(poly.interiors().len(), 1);
    }

    #[test]
    fn build_result_nested_rings_grandchildren_become_new_polygons() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Outer exterior (20x20)
        let outer_idx = manager.add_ring(make_square_ring(20.0));

        // Hole in outer (15x15 at 2,2)
        let mut hole = make_square_ring_at(2.0, 2.0, 15.0);
        hole.set_hole(true);
        let hole_idx = manager.add_ring(hole);
        manager.set_parent(hole_idx, outer_idx);

        // Island inside hole (5x5 at 5,5) - grandchild becomes new polygon
        // We set the parent to the hole BEFORE adding, so it won't be treated
        // as a top-level ring. Then set_parent establishes the bidirectional link.
        let mut island = make_square_ring_at(5.0, 5.0, 5.0);
        island.set_parent(Some(hole_idx)); // Pre-set parent so not added to top_level
        let island_idx = manager.add_ring(island);

        // Link to hole's children
        if let Some(hole_ring) = manager.get_mut(hole_idx) {
            hole_ring.add_child(island_idx);
        }

        let result = build_result(&manager, false);

        // Should have 2 polygons: outer with hole, and island (promoted from grandchild)
        assert_eq!(result.0.len(), 2);
    }

    #[test]
    fn build_result_reverse_output_reverses_winding() {
        let mut manager: RingManager<f64> = RingManager::new();
        manager.add_ring(make_square_ring(10.0));

        let result_forward = build_result(&manager, false);
        let result_reverse = build_result(&manager, true);

        // The second point should be different between forward and reverse
        let second_forward = result_forward.0[0].exterior().0[1];
        let second_reverse = result_reverse.0[0].exterior().0[1];

        // Forward: (0,0) -> (10,0)
        // Reverse: (0,10) -> ... (starts from last point going backwards)
        assert_ne!(second_forward, second_reverse);
    }

    // ==================== Integration Tests ====================

    #[test]
    fn integration_complex_polygon_with_multiple_holes() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Large exterior
        let exterior_idx = manager.add_ring(make_square_ring(100.0));

        // Multiple holes
        let mut hole1 = make_square_ring_at(10.0, 10.0, 20.0);
        hole1.set_hole(true);
        let hole1_idx = manager.add_ring(hole1);
        manager.set_parent(hole1_idx, exterior_idx);

        let mut hole2 = make_square_ring_at(50.0, 10.0, 20.0);
        hole2.set_hole(true);
        let hole2_idx = manager.add_ring(hole2);
        manager.set_parent(hole2_idx, exterior_idx);

        let mut hole3 = make_square_ring_at(10.0, 50.0, 20.0);
        hole3.set_hole(true);
        let hole3_idx = manager.add_ring(hole3);
        manager.set_parent(hole3_idx, exterior_idx);

        let result = build_result(&manager, false);

        assert_eq!(result.0.len(), 1);
        assert_eq!(result.0[0].interiors().len(), 3);
    }

    #[test]
    fn integration_separate_polygons_no_nesting() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Three separate squares
        manager.add_ring(make_square_ring_at(0.0, 0.0, 10.0));
        manager.add_ring(make_square_ring_at(20.0, 0.0, 10.0));
        manager.add_ring(make_square_ring_at(40.0, 0.0, 10.0));

        let result = build_result(&manager, false);

        assert_eq!(result.0.len(), 3);
        for poly in &result.0 {
            assert!(poly.interiors().is_empty());
        }
    }
}
