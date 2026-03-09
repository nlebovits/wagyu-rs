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
    /// Ring indices that were merged into other rings during Vatti sweep.
    /// These rings' points should be cleared before topology correction.
    merged_rings: Vec<usize>,
}

impl<T: CoordNum> RingManager<T> {
    /// Create a new empty ring manager.
    pub fn new() -> Self {
        RingManager {
            rings: Vec::new(),
            top_level_rings: Vec::new(),
            hot_pixels: Vec::new(),
            current_hp_idx: 0,
            merged_rings: Vec::new(),
        }
    }

    /// Mark a ring as merged (its points were copied to another ring).
    /// The ring's points will be cleared before topology correction.
    pub fn mark_as_merged(&mut self, ring_idx: usize) {
        if !self.merged_rings.contains(&ring_idx) {
            self.merged_rings.push(ring_idx);
        }
    }

    /// Clear points from all rings that were marked as merged.
    /// Call this at the start of topology correction.
    pub fn clear_merged_rings(&mut self) {
        for &ring_idx in &self.merged_rings.clone() {
            if let Some(ring) = self.rings.get_mut(ring_idx) {
                ring.points_mut().clear();
            }
        }
        self.merged_rings.clear();
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
        if crate::debug::debug_enabled() {
            eprintln!("[SET_PARENT] child={} parent={}", child_index, parent_index);
        }
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
    ///
    /// Filters out empty rings (those with fewer than 3 points) since they
    /// cannot form valid polygons.
    pub fn recalculate_top_level_rings(&mut self) {
        self.top_level_rings.clear();
        for i in 0..self.rings.len() {
            let ring = &self.rings[i];
            // Skip empty or degenerate rings (need at least 3 points for a polygon)
            if ring.points().len() < 3 {
                continue;
            }
            if ring.parent().is_none() && !ring.is_hole() {
                self.top_level_rings.push(i);
            }
        }
    }

    /// Transfer all children from ring2 to ring1 and remove ring2 from hierarchy.
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring.hpp - ring1_replaces_ring2 (lines 314-329)
    ///
    /// This function:
    /// 1. Transfers all children from ring2 to ring1 (or top_level_rings if ring1 is None)
    /// 2. Updates each child's parent pointer to ring1
    /// 3. Removes ring2 from its parent's children list
    /// 4. Clears ring2's points
    ///
    /// # Arguments
    /// * `ring1_idx` - Index of the ring to receive children (or None for top-level)
    /// * `ring2_idx` - Index of the ring being replaced/removed
    pub fn ring1_replaces_ring2(&mut self, ring1_idx: Option<usize>, ring2_idx: usize) {
        // Get ring2's children and parent before we start modifying
        let ring2_children: Vec<usize> = match self.rings.get(ring2_idx) {
            Some(r) => r.children().to_vec(),
            None => return,
        };
        let ring2_parent = self.rings.get(ring2_idx).and_then(|r| r.parent());

        // Transfer children from ring2 to ring1 (or top_level_rings)
        for child_idx in ring2_children {
            // Update child's parent to ring1
            if let Some(child) = self.rings.get_mut(child_idx) {
                child.set_parent(ring1_idx);
            }

            // Add child to ring1's children (or top_level_rings)
            match ring1_idx {
                Some(r1_idx) => {
                    if let Some(ring1) = self.rings.get_mut(r1_idx) {
                        ring1.add_child(child_idx);
                    }
                }
                None => {
                    // Add to top-level rings if not already there
                    if !self.top_level_rings.contains(&child_idx) {
                        self.top_level_rings.push(child_idx);
                    }
                }
            }
        }

        // Remove ring2 from its parent's children list
        match ring2_parent {
            Some(parent_idx) => {
                if let Some(parent) = self.rings.get_mut(parent_idx) {
                    parent.remove_child(ring2_idx);
                }
            }
            None => {
                // ring2 was a top-level ring, remove from top_level_rings
                self.top_level_rings.retain(|&idx| idx != ring2_idx);
            }
        }

        // Clear ring2's points and children
        if let Some(ring2) = self.rings.get_mut(ring2_idx) {
            ring2.points_mut().clear();
            ring2.clear_children();
        }
    }

    /// Check if a ring is a hole based on its area sign.
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring.hpp - ring_is_hole
    ///
    /// In the wagyu convention:
    /// - Negative area = clockwise = hole
    /// - Positive area = counter-clockwise = exterior
    pub fn ring_is_hole(&self, ring_idx: usize) -> bool {
        match self.rings.get(ring_idx) {
            Some(ring) => {
                // Calculate area using ring_util's ring_area function
                crate::ring_util::ring_area(ring.points()) < 0.0
            }
            None => false,
        }
    }

    // =========================================================================
    // Methods for correct_self_intersections
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
    // =========================================================================

    /// Create a new empty ring and return its index.
    ///
    /// The ring has no parent, no children, and no points.
    /// Caller is responsible for setting up hierarchy with assign_new_ring_parents.
    pub fn create_new_ring(&mut self) -> usize {
        let index = self.rings.len();
        let mut ring = Ring::empty();
        ring.set_ring_index(index);
        self.rings.push(ring);
        index
    }

    /// Assign a ring as a child of a parent (or as top-level if parent is None).
    ///
    /// This function is safe to call on rings that already have a parent - it will
    /// remove the ring from the old parent's children list first (fix for issue #57).
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - assign_as_child
    ///
    /// DIVERGENCE FROM WAGYU: C++ assign_as_child assumes the ring is brand new with no
    /// existing parent. Rust version is defensive and removes from old parent if present.
    /// This prevents stale parent-child references if called on an already-parented ring.
    pub fn assign_as_child(&mut self, child_idx: usize, new_parent_idx: Option<usize>) {
        // Step 1: Remove from old parent first (if any) - Fix for issue #57
        // This makes assign_as_child safe for re-assignment, preventing duplicate entries
        let old_parent = self.rings.get(child_idx).and_then(|r| r.parent());
        match old_parent {
            Some(old_p) if Some(old_p) != new_parent_idx => {
                // Remove from old parent's children list
                if let Some(old_parent_ring) = self.rings.get_mut(old_p) {
                    old_parent_ring.remove_child(child_idx);
                }
            }
            None => {
                // Remove from top_level_rings if present
                self.top_level_rings.retain(|&idx| idx != child_idx);
            }
            _ => {
                // old_parent == new_parent_idx, no need to remove
            }
        }

        // Step 2: Set hole status based on parent
        let is_hole = match new_parent_idx {
            Some(p) => !self.ring_is_hole(p),
            None => false,
        };

        // Step 3: Update ring's parent pointer and hole status
        if let Some(ring) = self.rings.get_mut(child_idx) {
            ring.set_hole(is_hole);
            ring.set_parent(new_parent_idx);
        }

        // Step 4: Add to new parent's children list (or top_level_rings)
        match new_parent_idx {
            Some(p) => {
                if let Some(parent) = self.rings.get_mut(p) {
                    // Only add if not already present (idempotent)
                    if !parent.children().contains(&child_idx) {
                        parent.add_child(child_idx);
                    }
                }
            }
            None => {
                if !is_hole && !self.top_level_rings.contains(&child_idx) {
                    self.top_level_rings.push(child_idx);
                }
            }
        }
    }

    /// Reassign a ring from its current parent to a new parent.
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - reassign_as_child
    pub fn reassign_as_child(&mut self, child_idx: usize, new_parent_idx: usize) {
        // Remove from old parent
        let old_parent = self.rings.get(child_idx).and_then(|r| r.parent());
        match old_parent {
            Some(old_p) => {
                if let Some(old_parent_ring) = self.rings.get_mut(old_p) {
                    old_parent_ring.remove_child(child_idx);
                }
            }
            None => {
                self.top_level_rings.retain(|&idx| idx != child_idx);
            }
        }
        // Assign to new parent
        self.assign_as_child(child_idx, Some(new_parent_idx));
    }

    /// Assign a ring as a sibling of another ring (same parent).
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - assign_as_sibling
    pub fn assign_as_sibling(&mut self, new_ring_idx: usize, sibling_idx: usize) {
        let sibling_parent = self.rings.get(sibling_idx).and_then(|r| r.parent());
        self.assign_as_child(new_ring_idx, sibling_parent);
    }

    /// Get the signed area of a ring as f64.
    pub fn ring_area_signed(&self, ring_idx: usize) -> f64 {
        match self.rings.get(ring_idx) {
            Some(ring) => crate::ring_util::ring_area(ring.points()),
            None => 0.0,
        }
    }

    /// Get the indices of all direct children of a ring.
    pub fn children(&self, ring_idx: usize) -> Vec<usize> {
        match self.rings.get(ring_idx) {
            Some(ring) => ring.children().to_vec(),
            None => Vec::new(),
        }
    }

    /// Get the parent index of a ring.
    pub fn parent(&self, ring_idx: usize) -> Option<usize> {
        self.rings.get(ring_idx)?.parent()
    }

    /// Calculate the depth of a ring in the parent chain.
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring.hpp - ring_depth (lines 405-415)
    ///
    /// Depth is the number of ancestors in the parent chain:
    /// - Depth 0: No parent (top-level ring)
    /// - Depth 1: One parent (child of top-level)
    /// - Depth 2: Two ancestors (grandchild of top-level)
    /// - etc.
    ///
    /// This is used to determine hole status structurally:
    /// - Even depth (0, 2, 4, ...) = exterior
    /// - Odd depth (1, 3, 5, ...) = hole
    pub fn ring_depth(&self, ring_idx: usize) -> usize {
        let mut depth = 0;
        let mut current = self.parent(ring_idx);
        while let Some(parent_idx) = current {
            depth += 1;
            current = self.parent(parent_idx);
        }
        depth
    }

    /// Determine if a ring is a hole based on its depth in the parent chain.
    ///
    /// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring.hpp - ring_is_hole (lines 418-423)
    ///
    /// C++ comment: "This is different than the 'normal' way of determining if
    /// a ring is a hole or not because it uses the depth of the ring to
    /// determine if it is a hole or not. This is only done initially when
    /// rings are output from Vatti."
    ///
    /// Returns true if depth is odd (ring is a hole structurally).
    pub fn depth_based_is_hole(&self, ring_idx: usize) -> bool {
        self.ring_depth(ring_idx) & 1 == 1
    }

    /// Check if a ring has been processed by correct_self_intersections.
    pub fn is_corrected(&self, ring_idx: usize) -> bool {
        self.rings
            .get(ring_idx)
            .map(|r| r.corrected)
            .unwrap_or(false)
    }

    /// Mark a ring as corrected (or uncorrected).
    pub fn set_corrected(&mut self, ring_idx: usize, corrected: bool) {
        if let Some(ring) = self.rings.get_mut(ring_idx) {
            ring.corrected = corrected;
        }
    }

    /// Return ring indices sorted by ascending absolute area (smallest first).
    ///
    /// Inner rings have smaller area than outer rings, so this processes
    /// inner rings before outer rings.
    pub fn sorted_ring_indices_smallest_to_largest(&self) -> Vec<usize> {
        let mut pairs: Vec<(usize, f64)> = self
            .rings
            .iter()
            .enumerate()
            .filter(|(_, r)| r.points().len() >= 3)
            .map(|(i, r)| (i, crate::ring_util::ring_area(r.points()).abs()))
            .collect();
        pairs.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        pairs.into_iter().map(|(i, _)| i).collect()
    }

    /// Replace the points of a ring with new_points.
    pub fn set_ring_points(&mut self, ring_idx: usize, new_points: Vec<geo_types::Coord<T>>) {
        if let Some(ring) = self.rings.get_mut(ring_idx) {
            *ring.points_mut() = new_points;
        }
    }

    /// Clone and return the points of a ring.
    pub fn ring_points_cloned(&self, ring_idx: usize) -> Vec<geo_types::Coord<T>> {
        match self.rings.get(ring_idx) {
            Some(ring) => ring.points().to_vec(),
            None => Vec::new(),
        }
    }

    /// Check if a ring has any points.
    pub fn ring_has_points(&self, ring_idx: usize) -> bool {
        self.rings
            .get(ring_idx)
            .map(|r| !r.points().is_empty())
            .unwrap_or(false)
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

    // Fix for issue #64: Skip degenerate exterior rings (< 3 points)
    // This is defensive - correct_tree should have already cleared these,
    // but we check here to prevent any edge cases from leaking through.
    if exterior_ring.points().len() < 3 {
        // Process children to collect grandchildren even if exterior is degenerate
        for &child_index in exterior_ring.children() {
            if let Some(child_ring) = manager.get(child_index) {
                for &grandchild_index in child_ring.children() {
                    grandchildren.push(grandchild_index);
                }
            }
        }
        return (
            Polygon::new(LineString::new(Vec::new()), Vec::new()),
            grandchildren,
        );
    }

    // Convert exterior ring to LineString
    let exterior = ring_to_linestring(exterior_ring, reverse_output);

    // Process children (holes)
    let mut holes = Vec::new();
    for &hole_index in exterior_ring.children() {
        if let Some(hole_ring) = manager.get(hole_index) {
            // Skip empty or degenerate rings (need at least 3 points for a valid hole)
            if hole_ring.points().len() < 3 {
                // Still process grandchildren even if this hole is empty
                for &grandchild_index in hole_ring.children() {
                    grandchildren.push(grandchild_index);
                }
                continue;
            }

            // PORT FROM: C++ build_result.hpp - holes use same reverse_output flag as exterior
            // The ring already has correct CW winding from correct_orientations
            holes.push(ring_to_linestring(hole_ring, reverse_output));

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
    // Log ring close for each completed ring
    for i in 0..manager.len() {
        if let Some(ring) = manager.get(i) {
            crate::debug::log_ring_close(i, ring.points().len());
        }
    }

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

    // ==================== ring1_replaces_ring2 Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring.hpp tests

    #[test]
    fn ring1_replaces_ring2_transfers_children_to_ring1() {
        // Setup: ring0 (parent) -> ring1 (child of ring0)
        //        ring2 has ring3 as a child
        // After ring1_replaces_ring2(ring1, ring2):
        //   ring3 should become child of ring1
        let mut manager: RingManager<f64> = RingManager::new();

        // ring0: exterior
        let idx0 = manager.add_ring(make_square_ring(100.0));

        // ring1: child of ring0
        let mut ring1 = make_square_ring(50.0);
        ring1.set_parent(Some(idx0));
        let idx1 = manager.add_ring(ring1);
        manager.get_mut(idx0).unwrap().add_child(idx1);

        // ring2: separate ring with a child
        let idx2 = manager.add_ring(make_square_ring(40.0));

        // ring3: child of ring2
        let mut ring3 = make_square_ring(20.0);
        ring3.set_parent(Some(idx2));
        let idx3 = manager.add_ring(ring3);
        manager.get_mut(idx2).unwrap().add_child(idx3);

        // Verify setup
        assert!(manager.get(idx2).unwrap().children().contains(&idx3));
        assert_eq!(manager.get(idx3).unwrap().parent(), Some(idx2));

        // Replace ring2 with ring1
        manager.ring1_replaces_ring2(Some(idx1), idx2);

        // ring3 should now be child of ring1
        assert!(manager.get(idx1).unwrap().children().contains(&idx3));
        assert_eq!(manager.get(idx3).unwrap().parent(), Some(idx1));

        // ring2 should have no children and empty points
        assert!(manager.get(idx2).unwrap().children().is_empty());
        assert!(manager.get(idx2).unwrap().points().is_empty());
    }

    #[test]
    fn ring1_replaces_ring2_with_none_moves_children_to_top_level() {
        // If ring1 is None, children should become top-level rings
        let mut manager: RingManager<f64> = RingManager::new();

        // ring0: has ring1 as a child
        let idx0 = manager.add_ring(make_square_ring(100.0));

        let mut ring1 = make_square_ring(50.0);
        ring1.set_parent(Some(idx0));
        let idx1 = manager.add_ring(ring1);
        manager.get_mut(idx0).unwrap().add_child(idx1);

        // Verify ring1 is not top-level initially
        assert!(!manager.top_level_rings().contains(&idx1));

        // Replace ring0 with None
        manager.ring1_replaces_ring2(None, idx0);

        // ring1 should now be top-level
        assert!(manager.top_level_rings().contains(&idx1));
        assert_eq!(manager.get(idx1).unwrap().parent(), None);

        // ring0 should be cleared
        assert!(manager.get(idx0).unwrap().children().is_empty());
        assert!(manager.get(idx0).unwrap().points().is_empty());
    }

    #[test]
    fn ring1_replaces_ring2_removes_from_parent_children() {
        // ring2 should be removed from its parent's children list
        let mut manager: RingManager<f64> = RingManager::new();

        // ring0: parent with ring1 and ring2 as children
        let idx0 = manager.add_ring(make_square_ring(100.0));

        let mut ring1 = make_square_ring(50.0);
        ring1.set_parent(Some(idx0));
        let idx1 = manager.add_ring(ring1);
        manager.get_mut(idx0).unwrap().add_child(idx1);

        let mut ring2 = make_square_ring(40.0);
        ring2.set_parent(Some(idx0));
        let idx2 = manager.add_ring(ring2);
        manager.get_mut(idx0).unwrap().add_child(idx2);

        // Verify setup: ring0 has both children
        assert!(manager.get(idx0).unwrap().children().contains(&idx1));
        assert!(manager.get(idx0).unwrap().children().contains(&idx2));

        // Replace ring2 with ring1 (merge)
        manager.ring1_replaces_ring2(Some(idx1), idx2);

        // ring0 should no longer have ring2 as a child
        assert!(manager.get(idx0).unwrap().children().contains(&idx1));
        assert!(!manager.get(idx0).unwrap().children().contains(&idx2));
    }

    #[test]
    fn ring_is_hole_detects_clockwise_as_hole() {
        // Clockwise winding = negative area = hole
        let mut manager: RingManager<f64> = RingManager::new();

        // CCW ring (positive area) - exterior
        let ccw_ring = make_square_ring(10.0); // Our helper makes CCW
        let idx_ccw = manager.add_ring(ccw_ring);

        // CW ring (negative area) - hole
        // Reverse the points to make it CW
        let mut cw_ring = Ring::empty();
        cw_ring.push_point(Coord { x: 0.0, y: 0.0 });
        cw_ring.push_point(Coord { x: 0.0, y: 10.0 });
        cw_ring.push_point(Coord { x: 10.0, y: 10.0 });
        cw_ring.push_point(Coord { x: 10.0, y: 0.0 });
        let idx_cw = manager.add_ring(cw_ring);

        // CCW should not be detected as hole
        assert!(!manager.ring_is_hole(idx_ccw));

        // CW should be detected as hole
        assert!(manager.ring_is_hole(idx_cw));
    }

    // ==================== assign_as_child Tests (Issue #57) ====================

    /// TDD RED test for issue #57:
    /// When assign_as_child is called on a ring that already has a parent,
    /// it must remove the ring from the old parent's children list.
    /// Otherwise, the ring appears in multiple parents' children lists.
    #[test]
    fn assign_as_child_removes_from_old_parent() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Create parent1, parent2, and a child ring
        let parent1 = manager.add_ring(make_square_ring(100.0));
        let parent2 = manager.add_ring(make_square_ring(100.0));
        let child = manager.add_ring(make_square_ring(10.0));

        // Assign child to parent1 first
        manager.assign_as_child(child, Some(parent1));

        // Verify setup: child is in parent1's children list
        assert!(
            manager.children(parent1).contains(&child),
            "Setup: child should be in parent1's children"
        );
        assert_eq!(
            manager.parent(child),
            Some(parent1),
            "Setup: child's parent should be parent1"
        );

        // Now reassign child to parent2 using assign_as_child
        // (This is the bug: the current code doesn't remove from parent1)
        manager.assign_as_child(child, Some(parent2));

        // Child should be in parent2's children
        assert!(
            manager.children(parent2).contains(&child),
            "child should be in parent2's children after reassignment"
        );
        assert_eq!(
            manager.parent(child),
            Some(parent2),
            "child's parent should be parent2"
        );

        // KEY ASSERTION (Issue #57):
        // Child must NOT be in parent1's children list anymore
        assert!(
            !manager.children(parent1).contains(&child),
            "BUG #57: child is still in parent1's children list after being reassigned to parent2. \
             assign_as_child must remove from old parent before adding to new parent."
        );
    }

    /// Integration test: verify ring hierarchy consistency after multiple reassignments.
    /// A ring should appear in exactly one parent's children list at all times.
    #[test]
    fn assign_as_child_maintains_hierarchy_invariant() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Create a tree: root -> [p1, p2, p3] -> child moves between them
        let root = manager.add_ring(make_square_ring(200.0));
        let p1 = manager.add_ring(make_square_ring(50.0));
        let p2 = manager.add_ring(make_square_ring(50.0));
        let p3 = manager.add_ring(make_square_ring(50.0));
        let child = manager.add_ring(make_square_ring(10.0));

        // Set up tree structure
        manager.assign_as_child(p1, Some(root));
        manager.assign_as_child(p2, Some(root));
        manager.assign_as_child(p3, Some(root));

        // Helper to count how many parents list 'child' in their children
        let count_parents = |mgr: &RingManager<f64>, c: usize| -> usize {
            [root, p1, p2, p3]
                .iter()
                .filter(|&&p| mgr.children(p).contains(&c))
                .count()
        };

        // Initially child is top-level (in top_level_rings)
        assert!(manager.top_level_rings().contains(&child));
        assert_eq!(count_parents(&manager, child), 0);

        // Move child through multiple parents
        manager.assign_as_child(child, Some(p1));
        assert_eq!(
            count_parents(&manager, child),
            1,
            "child should be in exactly one parent's children after assign to p1"
        );
        assert!(!manager.top_level_rings().contains(&child));

        manager.assign_as_child(child, Some(p2));
        assert_eq!(
            count_parents(&manager, child),
            1,
            "child should be in exactly one parent's children after assign to p2"
        );

        manager.assign_as_child(child, Some(p3));
        assert_eq!(
            count_parents(&manager, child),
            1,
            "child should be in exactly one parent's children after assign to p3"
        );

        // Move back to p1
        manager.assign_as_child(child, Some(p1));
        assert_eq!(
            count_parents(&manager, child),
            1,
            "child should be in exactly one parent's children after returning to p1"
        );
        assert!(manager.children(p1).contains(&child));
        assert!(!manager.children(p2).contains(&child));
        assert!(!manager.children(p3).contains(&child));

        // Move back to top-level
        manager.assign_as_child(child, None);
        assert_eq!(
            count_parents(&manager, child),
            0,
            "child should not be in any parent's children after assign to None"
        );
        assert!(manager.top_level_rings().contains(&child));
    }

    /// Test that assign_as_child removes from top_level_rings when assigning to a parent.
    #[test]
    fn assign_as_child_removes_from_top_level_rings() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Create parent and a top-level ring
        let parent = manager.add_ring(make_square_ring(100.0));
        let ring = manager.add_ring(make_square_ring(10.0));

        // Initially, ring should be in top_level_rings (added by add_ring)
        assert!(
            manager.top_level_rings().contains(&ring),
            "Setup: ring should be in top_level_rings"
        );

        // Assign to parent
        manager.assign_as_child(ring, Some(parent));

        // Ring should be removed from top_level_rings
        assert!(
            !manager.top_level_rings().contains(&ring),
            "BUG #57: ring is still in top_level_rings after being assigned to a parent"
        );
    }

    // ==================== Issue #64: Degenerate ring filtering tests ====================

    /// Test that build_polygon skips degenerate exterior rings (< 3 points).
    /// Fix for issue #64: Defensive filtering in build_polygon.
    #[test]
    fn build_polygon_skips_degenerate_exterior() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Create a degenerate ring with only 2 points
        let mut degenerate: Ring<f64> = Ring::empty();
        degenerate.push_point(Coord { x: 0.0, y: 0.0 });
        degenerate.push_point(Coord { x: 10.0, y: 10.0 });
        let degenerate_idx = manager.add_ring(degenerate);

        // Build polygon from degenerate exterior
        let (polygon, _grandchildren) = build_polygon(&manager, degenerate_idx, false);

        // Should return empty polygon
        assert!(
            polygon.exterior().0.is_empty(),
            "Degenerate exterior (< 3 points) should produce empty polygon"
        );
    }

    /// Test that build_result filters out degenerate exterior rings.
    #[test]
    fn build_result_filters_degenerate_rings() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Add one valid ring
        manager.add_ring(make_square_ring(10.0));

        // Add one degenerate ring (only 2 points)
        let mut degenerate: Ring<f64> = Ring::empty();
        degenerate.push_point(Coord { x: 50.0, y: 50.0 });
        degenerate.push_point(Coord { x: 60.0, y: 60.0 });
        manager.add_ring(degenerate);

        let result = build_result(&manager, false);

        // Should only have 1 polygon (the valid one)
        assert_eq!(
            result.0.len(),
            1,
            "Degenerate rings should be filtered from build_result output"
        );
    }
}
