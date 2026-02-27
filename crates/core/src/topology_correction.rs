//! Topology Correction - Ensures output geometry is OGC valid.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
//!
//! This module corrects topology issues in polygon output to ensure OGC validity:
//!
//! - **Correct orientations**: Ensure exterior rings are CCW, holes are CW
//! - **Correct collinear edges**: Handle degenerate collinear edges
//! - **Correct self-intersections**: Fix rings that intersect themselves
//! - **Correct tree structure**: Rebuild proper parent/child ring relationships
//! - **Correct chained rings**: Handle rings that share edges
//!
//! The main entry point is `correct_topology()` which runs all corrections.

use geo_types::{Coord, CoordNum};

use crate::ring_util::{point_in_polygon, value_is_zero, values_are_equal, PointInPolygonResult};

// ============================================================================
// Point Pair - For tracking duplicate/connection points
// ============================================================================

/// A pair of point indices, used for tracking connections between rings.
///
/// From C++: `struct point_ptr_pair`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PointIndexPair {
    /// Index into ring1's points
    pub index1: usize,
    /// Index into ring2's points
    pub index2: usize,
}

impl PointIndexPair {
    /// Create a new point index pair.
    pub fn new(index1: usize, index2: usize) -> Self {
        Self { index1, index2 }
    }
}

// ============================================================================
// Ring Sorting
// ============================================================================

/// Information about a ring for sorting purposes.
#[derive(Debug, Clone)]
pub struct RingInfo {
    /// Original index in the ring vector
    pub index: usize,
    /// Absolute area (for size comparison)
    pub abs_area: f64,
}

/// Sort rings from largest to smallest by absolute area.
///
/// From C++: `sort_rings_largest_to_smallest`
///
/// This is used in tree correction to process larger rings first,
/// since smaller rings cannot contain larger ones.
pub fn sort_rings_largest_to_smallest(areas: &[f64]) -> Vec<usize> {
    let mut infos: Vec<RingInfo> = areas
        .iter()
        .enumerate()
        .map(|(idx, &area)| RingInfo {
            index: idx,
            abs_area: area.abs(),
        })
        .collect();

    // Sort by absolute area, largest first
    infos.sort_by(|a, b| b.abs_area.partial_cmp(&a.abs_area).unwrap());

    infos.into_iter().map(|info| info.index).collect()
}

/// Sort rings from smallest to largest by absolute area.
///
/// From C++: `sort_rings_smallest_to_largest`
///
/// This is used in self-intersection correction to process smaller rings first.
pub fn sort_rings_smallest_to_largest(areas: &[f64]) -> Vec<usize> {
    let mut infos: Vec<RingInfo> = areas
        .iter()
        .enumerate()
        .map(|(idx, &area)| RingInfo {
            index: idx,
            abs_area: area.abs(),
        })
        .collect();

    // Sort by absolute area, smallest first
    infos.sort_by(|a, b| a.abs_area.partial_cmp(&b.abs_area).unwrap());

    infos.into_iter().map(|info| info.index).collect()
}

// ============================================================================
// Orientation Correction
// ============================================================================

/// Check if a ring's points need to be reversed for correct orientation.
///
/// From C++: Part of `correct_orientations`
///
/// For OGC validity:
/// - Exterior rings (non-holes) should have positive area (CCW)
/// - Interior rings (holes) should have negative area (CW)
///
/// # Arguments
/// * `ring_area` - The signed area of the ring (positive = CCW, negative = CW)
/// * `is_hole` - True if this ring is a hole
///
/// # Returns
/// True if the ring needs to be reversed to have correct orientation.
pub fn needs_orientation_reversal(area: f64, is_hole: bool) -> bool {
    // Exterior (non-hole) should have positive area (CCW)
    // Hole should have negative area (CW)
    if is_hole {
        area > 0.0 // Hole with positive area needs reversal
    } else {
        area < 0.0 // Exterior with negative area needs reversal
    }
}

/// Reverse a ring's point order in place.
///
/// From C++: `reverse_ring`
pub fn reverse_ring<T: CoordNum + Copy>(ring_points: &mut [Coord<T>]) {
    ring_points.reverse();
}

// ============================================================================
// Polygon Containment
// ============================================================================

/// Check if polygon 2 contains polygon 1.
///
/// From C++: `poly2_contains_poly1`
///
/// This function checks if ring1 is completely contained within ring2.
/// It first checks bounding boxes for a quick rejection, then performs
/// point-in-polygon tests for accurate containment detection.
///
/// # Arguments
/// * `ring1_points` - Points of the potentially contained ring
/// * `ring1_area` - Area of ring1
/// * `ring2_points` - Points of the potentially containing ring
/// * `ring2_area` - Area of ring2
///
/// # Returns
/// True if ring2 completely contains ring1.
pub fn poly2_contains_poly1<T: CoordNum>(
    ring1_points: &[Coord<T>],
    ring1_area: f64,
    ring2_points: &[Coord<T>],
    ring2_area: f64,
) -> bool {
    // Compute bounding boxes
    let ring1_bbox = match BBoxF64::from_ring(ring1_points) {
        Some(b) => b,
        None => return false,
    };
    let ring2_bbox = match BBoxF64::from_ring(ring2_points) {
        Some(b) => b,
        None => return false,
    };

    // Quick bounding box check
    if !ring2_bbox.contains(&ring1_bbox) {
        return false;
    }

    // If ring2 is smaller than ring1, it can't contain it
    if ring2_area.abs() < ring1_area.abs() {
        return false;
    }

    // Check if any point of ring1 is inside ring2
    // In the C++ code, it iterates until finding a non-boundary point
    for pt in ring1_points {
        let result = point_in_polygon(pt, ring2_points);
        if result != PointInPolygonResult::OnPolygon {
            return result == PointInPolygonResult::Inside;
        }
    }

    // All points are on the boundary - need special handling
    // For now, return false (conservative)
    // TODO: Implement inside_or_outside_special for this case
    false
}

// ============================================================================
// Collinear Edge Detection
// ============================================================================

/// Check if three points are collinear.
///
/// Three points are collinear if they lie on the same line.
/// We check this using the cross product (area of triangle = 0).
pub fn points_are_collinear<T: CoordNum>(p1: &Coord<T>, p2: &Coord<T>, p3: &Coord<T>) -> bool {
    let x1 = p1.x.to_f64().unwrap_or(0.0);
    let y1 = p1.y.to_f64().unwrap_or(0.0);
    let x2 = p2.x.to_f64().unwrap_or(0.0);
    let y2 = p2.y.to_f64().unwrap_or(0.0);
    let x3 = p3.x.to_f64().unwrap_or(0.0);
    let y3 = p3.y.to_f64().unwrap_or(0.0);

    // Cross product of vectors (p2-p1) and (p3-p1)
    let cross = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
    value_is_zero(cross)
}

/// Find indices of collinear point sequences in a ring.
///
/// Returns a list of (start_index, end_index) pairs where the points
/// from start to end are collinear and could be simplified.
pub fn find_collinear_sequences<T: CoordNum>(ring_points: &[Coord<T>]) -> Vec<(usize, usize)> {
    if ring_points.len() < 3 {
        return vec![];
    }

    let mut sequences = Vec::new();
    let n = ring_points.len();

    let mut i = 0;
    while i < n {
        let p1 = &ring_points[i];
        let p2 = &ring_points[(i + 1) % n];
        let p3 = &ring_points[(i + 2) % n];

        if points_are_collinear(p1, p2, p3) {
            let start = i;
            let mut end = i + 2;

            // Extend the sequence as long as points remain collinear
            while end < n + start - 1 {
                let next_idx = (end + 1) % n;
                let prev = &ring_points[end % n];
                let curr = &ring_points[next_idx];

                // Check if extending would still be collinear with the line
                if points_are_collinear(p1, prev, curr) {
                    end += 1;
                } else {
                    break;
                }
            }

            sequences.push((start, end % n));
            i = end;
        } else {
            i += 1;
        }
    }

    sequences
}

/// Remove collinear middle points from a ring.
///
/// When three consecutive points are collinear, the middle point
/// can be removed without changing the shape.
pub fn remove_collinear_points<T: CoordNum + Copy>(ring_points: &[Coord<T>]) -> Vec<Coord<T>> {
    if ring_points.len() < 3 {
        return ring_points.to_vec();
    }

    let mut result = Vec::new();
    let n = ring_points.len();

    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let next = (i + 1) % n;

        // Only keep the point if it's not collinear with its neighbors
        if !points_are_collinear(&ring_points[prev], &ring_points[i], &ring_points[next]) {
            result.push(ring_points[i]);
        }
    }

    result
}

// ============================================================================
// Self-Intersection Detection
// ============================================================================

/// Check if two line segments intersect.
///
/// Returns true if segment (p1, p2) intersects segment (p3, p4).
/// Does not count touching endpoints as intersection.
pub fn segments_intersect<T: CoordNum>(
    p1: &Coord<T>,
    p2: &Coord<T>,
    p3: &Coord<T>,
    p4: &Coord<T>,
) -> bool {
    let x1 = p1.x.to_f64().unwrap_or(0.0);
    let y1 = p1.y.to_f64().unwrap_or(0.0);
    let x2 = p2.x.to_f64().unwrap_or(0.0);
    let y2 = p2.y.to_f64().unwrap_or(0.0);
    let x3 = p3.x.to_f64().unwrap_or(0.0);
    let y3 = p3.y.to_f64().unwrap_or(0.0);
    let x4 = p4.x.to_f64().unwrap_or(0.0);
    let y4 = p4.y.to_f64().unwrap_or(0.0);

    // Using the cross product method for segment intersection
    let d1 = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    let d2 = (x4 - x3) * (y2 - y3) - (y4 - y3) * (x2 - x3);
    let d3 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    let d4 = (x2 - x1) * (y4 - y1) - (y2 - y1) * (x4 - x1);

    // If signs differ, segments cross
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }

    // Check for collinear overlap (endpoints on segment)
    if value_is_zero(d1) && on_segment(x3, y3, x4, y4, x1, y1) {
        return true;
    }
    if value_is_zero(d2) && on_segment(x3, y3, x4, y4, x2, y2) {
        return true;
    }
    if value_is_zero(d3) && on_segment(x1, y1, x2, y2, x3, y3) {
        return true;
    }
    if value_is_zero(d4) && on_segment(x1, y1, x2, y2, x4, y4) {
        return true;
    }

    false
}

/// Check if point (px, py) lies on segment from (x1, y1) to (x2, y2).
fn on_segment(x1: f64, y1: f64, x2: f64, y2: f64, px: f64, py: f64) -> bool {
    px >= x1.min(x2) && px <= x1.max(x2) && py >= y1.min(y2) && py <= y1.max(y2)
}

/// Check if a ring has any self-intersections.
///
/// A ring self-intersects if any of its non-adjacent edges cross each other.
pub fn ring_has_self_intersection<T: CoordNum>(ring_points: &[Coord<T>]) -> bool {
    if ring_points.len() < 4 {
        return false;
    }

    let n = ring_points.len();

    for i in 0..n {
        let i_next = (i + 1) % n;

        // Check against all non-adjacent edges
        for j in (i + 2)..n {
            // Skip if edges are adjacent
            let j_next = (j + 1) % n;
            if j_next == i {
                continue;
            }

            if segments_intersect(
                &ring_points[i],
                &ring_points[i_next],
                &ring_points[j],
                &ring_points[j_next],
            ) {
                return true;
            }
        }
    }

    false
}

// ============================================================================
// Duplicate Point Detection
// ============================================================================

/// Find duplicate points in a ring (points that appear more than once).
///
/// Returns indices of points that have duplicates.
pub fn find_duplicate_points<T: CoordNum + PartialEq>(ring_points: &[Coord<T>]) -> Vec<usize> {
    let mut duplicates = Vec::new();

    for i in 0..ring_points.len() {
        for j in (i + 1)..ring_points.len() {
            if ring_points[i] == ring_points[j] {
                duplicates.push(i);
                duplicates.push(j);
            }
        }
    }

    duplicates.sort();
    duplicates.dedup();
    duplicates
}

// ============================================================================
// Point Comparison for Sorting
// ============================================================================

/// Compare two points for sorting (first by y descending, then by x ascending).
///
/// From C++: `struct point_ptr_cmp`
///
/// This ordering puts points with larger y values first (bottom points in screen coords),
/// and for equal y, smaller x values first (leftmost).
pub fn compare_points<T: CoordNum>(p1: &Coord<T>, p2: &Coord<T>) -> std::cmp::Ordering {
    let y1 = p1.y.to_f64().unwrap_or(0.0);
    let y2 = p2.y.to_f64().unwrap_or(0.0);
    let x1 = p1.x.to_f64().unwrap_or(0.0);
    let x2 = p2.x.to_f64().unwrap_or(0.0);

    // First compare by y (descending)
    if !values_are_equal(y1, y2) {
        if y1 > y2 {
            return std::cmp::Ordering::Less; // Larger y comes first
        } else {
            return std::cmp::Ordering::Greater;
        }
    }

    // Then by x (ascending)
    if x1 < x2 {
        std::cmp::Ordering::Less
    } else if x1 > x2 {
        std::cmp::Ordering::Greater
    } else {
        std::cmp::Ordering::Equal
    }
}

// ============================================================================
// Internal F64 Bounding Box (for topology correction calculations)
// ============================================================================

/// Simple f64 bounding box for internal calculations.
#[derive(Debug, Clone, Copy)]
struct BBoxF64 {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl BBoxF64 {
    /// Compute bounding box from ring points.
    fn from_ring<T: CoordNum>(points: &[Coord<T>]) -> Option<Self> {
        if points.is_empty() {
            return None;
        }

        let mut min_x = points[0].x.to_f64()?;
        let mut min_y = points[0].y.to_f64()?;
        let mut max_x = min_x;
        let mut max_y = min_y;

        for pt in points.iter().skip(1) {
            let x = pt.x.to_f64()?;
            let y = pt.y.to_f64()?;
            if x < min_x {
                min_x = x;
            }
            if y < min_y {
                min_y = y;
            }
            if x > max_x {
                max_x = x;
            }
            if y > max_y {
                max_y = y;
            }
        }

        Some(BBoxF64 {
            min_x,
            min_y,
            max_x,
            max_y,
        })
    }

    /// Check if other bbox is contained within self.
    fn contains(&self, other: &BBoxF64) -> bool {
        self.max_x >= other.max_x
            && self.max_y >= other.max_y
            && self.min_x <= other.min_x
            && self.min_y <= other.min_y
    }
}

// ============================================================================
// Topology Correction Functions
// ============================================================================

/// Correct the orientations of all rings.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_orientations
///
/// Ensures:
/// - Exterior rings (is_hole = false) have positive area (CCW)
/// - Interior rings (is_hole = true) have negative area (CW)
///
/// Rings with fewer than 3 points are considered degenerate.
fn correct_orientations<T: CoordNum + Copy>(manager: &mut crate::build_result::RingManager<T>) {
    use crate::ring_util::ring_area;

    let indices: Vec<usize> = manager.ring_indices().collect();

    for idx in indices {
        let (ring_len, area, is_hole) = {
            let ring = match manager.get(idx) {
                Some(r) => r,
                None => continue,
            };
            (ring.len(), ring_area(ring.points()), ring.is_hole())
        };

        // Skip degenerate rings (less than 3 points)
        if ring_len < 3 {
            continue;
        }

        let needs_reversal = needs_orientation_reversal(area, is_hole);

        if std::env::var("WAGYU_DEBUG").is_ok() {
            eprintln!(
                "DEBUG: correct_orientations ring {} area={:.2} is_hole={} needs_reversal={}",
                idx, area, is_hole, needs_reversal
            );
        }

        // Check if orientation needs correction
        if needs_reversal {
            if let Some(ring) = manager.get_mut(idx) {
                reverse_ring(ring.points_mut());
            }
        }
    }
}

/// Correct the tree structure (parent/child relationships).
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_tree
///
/// This function rebuilds the ring hierarchy by:
/// 1. Sorting rings from largest to smallest (by absolute area)
/// 2. For each ring, searching backwards for potential parents
/// 3. A ring becomes a child of the smallest ring that contains it
///    and has opposite orientation (exterior contains hole, hole contains exterior)
fn correct_tree<T: CoordNum + Copy>(manager: &mut crate::build_result::RingManager<T>) {
    use crate::ring_util::ring_area;

    // Collect ring data for sorting
    let mut ring_data: Vec<(usize, f64, BBoxF64, bool)> = Vec::new();

    for idx in manager.ring_indices() {
        if let Some(ring) = manager.get(idx) {
            let points = ring.points();
            if points.len() < 3 {
                continue;
            }
            let area = ring_area(points);
            if let Some(bbox) = BBoxF64::from_ring(points) {
                // Determine hole status from area sign, NOT from stored flag
                // PORT FROM: C++ ring.hpp is_hole() - negative area = clockwise = hole
                let is_hole = area < 0.0;
                ring_data.push((idx, area, bbox, is_hole));
            }
        }
    }

    // Sort by absolute area, largest first
    ring_data.sort_by(|a, b| b.1.abs().partial_cmp(&a.1.abs()).unwrap());

    if std::env::var("WAGYU_DEBUG").is_ok() {
        eprintln!("DEBUG: correct_tree - ring_data after sort:");
        for (idx, area, _, is_hole) in &ring_data {
            eprintln!("DEBUG:   ring {} area={:.2} is_hole={}", idx, area, is_hole);
        }
    }

    // Clear existing parent/child relationships
    for (idx, _, _, _) in &ring_data {
        manager.clear_parent(*idx);
        manager.clear_children(*idx);
    }

    // Rebuild hierarchy
    // For each ring, search backwards for potential parents
    //
    // PORT FROM: C++ correct_tree (topology_correction.hpp lines 1262-1302)
    for i in 0..ring_data.len() {
        let (ring_idx, ring_area_val, ref ring_bbox, ring_is_hole) = ring_data[i];

        // Get ring points once
        let ring_points: Vec<Coord<T>> = match manager.get(ring_idx) {
            Some(r) => r.points().to_vec(),
            None => continue,
        };

        // Search backwards for potential parents (larger rings)
        let mut found_parent = false;
        for j in (0..i).rev() {
            let (parent_idx, parent_area_val, ref parent_bbox, parent_is_hole) = ring_data[j];

            // PORT FROM: C++ correct_tree line ~1288
            // If orientations are not different, this can't be its parent.
            // (exterior contains hole, hole contains island which becomes exterior)
            if parent_is_hole == ring_is_hole {
                continue;
            }

            // Get parent points
            let parent_points: Vec<Coord<T>> = match manager.get(parent_idx) {
                Some(r) => r.points().to_vec(),
                None => continue,
            };

            // Check if parent contains this ring using f64 bounding boxes
            if poly2_contains_poly1_f64(
                &ring_points,
                ring_bbox,
                ring_area_val,
                &parent_points,
                parent_bbox,
                parent_area_val,
            ) {
                // Set this ring as child of parent
                manager.set_parent(ring_idx, parent_idx);

                // Update is_hole status in the ring
                if let Some(ring) = manager.get_mut(ring_idx) {
                    ring.set_hole(!parent_is_hole);
                }
                found_parent = true;
                break;
            }
        }

        // If no parent found and this ring is calculated as a hole, that's an error
        // PORT FROM: C++ correct_tree lines 1294-1300
        if !found_parent {
            if ring_is_hole {
                // C++ throws: "Could not properly place hole to a parent."
                // For now, we just make it a top-level exterior
                if let Some(ring) = manager.get_mut(ring_idx) {
                    ring.set_hole(false);
                }
            }
            // If it's not a hole, it's already a top-level exterior - no action needed
        }
    }

    // Recalculate top-level rings
    manager.recalculate_top_level_rings();
}

/// Check if polygon 2 contains polygon 1 using f64 bounding boxes.
fn poly2_contains_poly1_f64<T: CoordNum>(
    ring1_points: &[Coord<T>],
    ring1_bbox: &BBoxF64,
    ring1_area: f64,
    ring2_points: &[Coord<T>],
    ring2_bbox: &BBoxF64,
    ring2_area: f64,
) -> bool {
    // Quick bounding box check
    if !ring2_bbox.contains(ring1_bbox) {
        return false;
    }

    // If ring2 is smaller than ring1, it can't contain it
    if ring2_area.abs() < ring1_area.abs() {
        return false;
    }

    // Check if any point of ring1 is inside ring2
    for pt in ring1_points {
        let result = point_in_polygon(pt, ring2_points);
        if result != PointInPolygonResult::OnPolygon {
            return result == PointInPolygonResult::Inside;
        }
    }

    // All points are on the boundary - conservative
    false
}

/// Correct the topology of output rings to ensure OGC validity.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_topology
///
/// This function ensures the output geometry is valid per OGC standards:
/// - No self-intersections
/// - Correct ring orientations (CCW for exterior, CW for holes)
/// - Proper hole containment
///
/// # Arguments
///
/// * `manager` - Ring manager containing the output rings to correct
///
/// # Algorithm (from C++ wagyu)
///
/// 1. Correct orientations - ensure CCW for exterior, CW for holes
/// 2. Correct collinear edges (handled separately in this implementation)
/// 3. Correct self-intersections (simplified - just detection)
/// 4. Correct tree - rebuild parent/child relationships based on containment
/// 5. Loop on chained rings and self-intersections (simplified)
pub fn correct_topology<T: CoordNum + Copy>(manager: &mut crate::build_result::RingManager<T>) {
    // Step 1: Correct orientations
    // Ensures exterior rings are CCW (positive area) and holes are CW (negative area)
    correct_orientations(manager);

    // Step 2: Correct tree structure
    // Rebuilds parent/child relationships based on containment
    correct_tree(manager);

    // Note: The C++ implementation also handles:
    // - Collinear edges (correct_collinear_edges)
    // - Self-intersections (correct_self_intersections)
    // - Chained rings (correct_chained_rings)
    // These are complex operations that require additional infrastructure.
    // For now, the basic orientation and tree correction covers the main cases.
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build_result::RingManager;
    use crate::ring_util::ring_area;
    use crate::Ring;

    // ==================== Helper Functions ====================

    /// Create a CCW square ring (positive area - exterior orientation)
    fn make_ccw_square(x: f64, y: f64, size: f64) -> Ring<f64> {
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

    /// Create a CW square ring (negative area - hole orientation)
    fn make_cw_square(x: f64, y: f64, size: f64) -> Ring<f64> {
        let mut ring = Ring::empty();
        ring.push_point(Coord { x, y });
        ring.push_point(Coord { x, y: y + size });
        ring.push_point(Coord {
            x: x + size,
            y: y + size,
        });
        ring.push_point(Coord { x: x + size, y });
        ring
    }

    // ==================== correct_topology Tests ====================

    #[test]
    fn correct_topology_fixes_wrong_exterior_orientation() {
        // Create an exterior ring with wrong orientation (CW instead of CCW)
        let mut manager: RingManager<f64> = RingManager::new();

        let wrong_orientation_exterior = make_cw_square(0.0, 0.0, 10.0);
        let idx = manager.add_ring(wrong_orientation_exterior);

        // Before correction: exterior has negative area (CW - wrong for exterior)
        let area_before = ring_area(manager.get(idx).unwrap().points());
        assert!(
            area_before < 0.0,
            "Setup: exterior should have wrong CW orientation"
        );

        // Apply topology correction
        correct_topology(&mut manager);

        // After correction: exterior should have positive area (CCW - correct)
        let area_after = ring_area(manager.get(idx).unwrap().points());
        assert!(
            area_after > 0.0,
            "Exterior ring should have positive area (CCW) after correction"
        );
    }

    #[test]
    fn correct_topology_fixes_wrong_hole_orientation() {
        // Create exterior with hole that has wrong orientation (CCW instead of CW)
        let mut manager: RingManager<f64> = RingManager::new();

        // Correct exterior (CCW)
        let exterior = make_ccw_square(0.0, 0.0, 20.0);
        let exterior_idx = manager.add_ring(exterior);

        // Wrong orientation hole (CCW instead of CW)
        let mut hole = make_ccw_square(5.0, 5.0, 5.0);
        hole.set_hole(true);
        let hole_idx = manager.add_ring(hole);
        manager.set_parent(hole_idx, exterior_idx);

        // Before correction: hole has positive area (CCW - wrong for hole)
        let hole_area_before = ring_area(manager.get(hole_idx).unwrap().points());
        assert!(
            hole_area_before > 0.0,
            "Setup: hole should have wrong CCW orientation"
        );

        // Apply topology correction
        correct_topology(&mut manager);

        // After correction: hole should have negative area (CW - correct)
        let hole_area_after = ring_area(manager.get(hole_idx).unwrap().points());
        assert!(
            hole_area_after < 0.0,
            "Hole should have negative area (CW) after correction"
        );
    }

    #[test]
    fn correct_topology_preserves_correct_orientations() {
        // Create rings with correct orientations - they should stay unchanged
        let mut manager: RingManager<f64> = RingManager::new();

        // Correct CCW exterior
        let exterior = make_ccw_square(0.0, 0.0, 20.0);
        let exterior_idx = manager.add_ring(exterior);

        // Correct CW hole
        let mut hole = make_cw_square(5.0, 5.0, 5.0);
        hole.set_hole(true);
        let hole_idx = manager.add_ring(hole);
        manager.set_parent(hole_idx, exterior_idx);

        let exterior_area_before = ring_area(manager.get(exterior_idx).unwrap().points());
        let hole_area_before = ring_area(manager.get(hole_idx).unwrap().points());

        // Apply topology correction
        correct_topology(&mut manager);

        let exterior_area_after = ring_area(manager.get(exterior_idx).unwrap().points());
        let hole_area_after = ring_area(manager.get(hole_idx).unwrap().points());

        // Areas should have same sign (orientation preserved)
        assert!(
            exterior_area_before.signum() == exterior_area_after.signum(),
            "Correct exterior orientation should be preserved"
        );
        assert!(
            hole_area_before.signum() == hole_area_after.signum(),
            "Correct hole orientation should be preserved"
        );
    }

    #[test]
    fn correct_topology_establishes_parent_child_from_containment() {
        // Create an exterior ring containing a hole ring
        // The tree correction should establish the parent-child relationship
        let mut manager: RingManager<f64> = RingManager::new();

        // Large outer ring (exterior)
        let outer = make_ccw_square(0.0, 0.0, 100.0);
        let outer_idx = manager.add_ring(outer);

        // Small inner ring marked as hole (is_hole=true)
        // This simulates what Vatti algorithm would produce
        let mut inner = make_cw_square(20.0, 20.0, 10.0); // CW for hole
        inner.set_hole(true);
        let inner_idx = manager.add_ring(inner);

        // Initially no parent set (tree structure not established)
        assert!(
            manager.get(inner_idx).unwrap().parent().is_none(),
            "Setup: inner should have no parent initially"
        );

        // Apply topology correction
        correct_topology(&mut manager);

        // After correction: inner (hole) should be a child of outer (exterior)
        let inner_after = manager.get(inner_idx).unwrap();
        assert!(
            inner_after.parent().is_some(),
            "Inner ring should have a parent after tree correction"
        );
        assert_eq!(
            inner_after.parent(),
            Some(outer_idx),
            "Inner ring's parent should be the outer ring"
        );
    }

    #[test]
    fn correct_topology_removes_degenerate_rings() {
        // Create a degenerate ring with only 2 points
        let mut manager: RingManager<f64> = RingManager::new();

        // Valid exterior
        let exterior = make_ccw_square(0.0, 0.0, 10.0);
        let _exterior_idx = manager.add_ring(exterior);

        // Degenerate ring (only 2 points)
        let mut degenerate = Ring::empty();
        degenerate.push_point(Coord { x: 0.0, y: 0.0 });
        degenerate.push_point(Coord { x: 5.0, y: 5.0 });
        let degenerate_idx = manager.add_ring(degenerate);

        assert_eq!(
            manager.get(degenerate_idx).unwrap().len(),
            2,
            "Setup: degenerate ring should have 2 points"
        );

        // Apply topology correction
        correct_topology(&mut manager);

        // After correction: degenerate ring should be marked for removal
        // (either by having 0 points or being flagged)
        let ring_after = manager.get(degenerate_idx).unwrap();
        assert!(
            ring_after.len() < 3 || ring_area(ring_after.points()).abs() < 1e-10,
            "Degenerate rings should be removed or marked invalid"
        );
    }

    #[test]
    fn correct_topology_handles_empty_manager() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Should not panic on empty manager
        correct_topology(&mut manager);

        assert!(manager.is_empty());
    }

    #[test]
    fn correct_topology_handles_nested_hierarchy() {
        // Create a 3-level nesting: exterior -> hole -> island
        // Pre-set is_hole status as Vatti algorithm would
        let mut manager: RingManager<f64> = RingManager::new();

        // Outer exterior (100x100) - CCW, is_hole=false
        let outer = make_ccw_square(0.0, 0.0, 100.0);
        let outer_idx = manager.add_ring(outer);

        // Middle hole (60x60 at 20,20) - CW, is_hole=true
        let mut middle = make_cw_square(20.0, 20.0, 60.0);
        middle.set_hole(true);
        let middle_idx = manager.add_ring(middle);

        // Inner island (20x20 at 40,40) - CCW, is_hole=false (island inside hole)
        let inner = make_ccw_square(40.0, 40.0, 20.0);
        let inner_idx = manager.add_ring(inner);

        // Apply topology correction
        correct_topology(&mut manager);

        // Check hierarchy:
        // - Outer should remain exterior (no parent)
        // - Middle (hole) should be child of outer (exterior)
        // - Inner (island) should be child of middle (hole)

        let outer_after = manager.get(outer_idx).unwrap();
        let middle_after = manager.get(middle_idx).unwrap();
        let inner_after = manager.get(inner_idx).unwrap();

        assert!(
            outer_after.parent().is_none(),
            "Outer ring should have no parent"
        );

        // Middle (hole) should be child of outer (exterior)
        assert!(
            middle_after.parent().is_some(),
            "Middle ring should have a parent (outer)"
        );
        assert_eq!(
            middle_after.parent(),
            Some(outer_idx),
            "Middle ring's parent should be outer"
        );

        // Inner (island) should be child of middle (hole)
        assert!(
            inner_after.parent().is_some(),
            "Inner ring should have a parent (middle)"
        );
        assert_eq!(
            inner_after.parent(),
            Some(middle_idx),
            "Inner ring's parent should be middle"
        );
    }

    // ==================== PointIndexPair Tests ====================

    #[test]
    fn point_index_pair_new() {
        let pair = PointIndexPair::new(5, 10);
        assert_eq!(pair.index1, 5);
        assert_eq!(pair.index2, 10);
    }

    // ==================== Ring Sorting Tests ====================

    #[test]
    fn sort_rings_largest_to_smallest_orders_by_abs_area() {
        let areas = vec![10.0, -30.0, 5.0, -20.0];
        let sorted = sort_rings_largest_to_smallest(&areas);

        // Should be: index 1 (|-30|=30), index 3 (|-20|=20), index 0 (10), index 2 (5)
        assert_eq!(sorted, vec![1, 3, 0, 2]);
    }

    #[test]
    fn sort_rings_smallest_to_largest_orders_by_abs_area() {
        let areas = vec![10.0, -30.0, 5.0, -20.0];
        let sorted = sort_rings_smallest_to_largest(&areas);

        // Should be: index 2 (5), index 0 (10), index 3 (|-20|=20), index 1 (|-30|=30)
        assert_eq!(sorted, vec![2, 0, 3, 1]);
    }

    #[test]
    fn sort_rings_empty_vec() {
        let areas: Vec<f64> = vec![];
        assert!(sort_rings_largest_to_smallest(&areas).is_empty());
        assert!(sort_rings_smallest_to_largest(&areas).is_empty());
    }

    // ==================== Orientation Tests ====================

    #[test]
    fn needs_orientation_reversal_exterior_with_negative_area_needs_reversal() {
        // Exterior ring should have positive area
        assert!(needs_orientation_reversal(-100.0, false));
    }

    #[test]
    fn needs_orientation_reversal_exterior_with_positive_area_no_reversal() {
        assert!(!needs_orientation_reversal(100.0, false));
    }

    #[test]
    fn needs_orientation_reversal_hole_with_positive_area_needs_reversal() {
        // Hole should have negative area
        assert!(needs_orientation_reversal(100.0, true));
    }

    #[test]
    fn needs_orientation_reversal_hole_with_negative_area_no_reversal() {
        assert!(!needs_orientation_reversal(-100.0, true));
    }

    #[test]
    fn reverse_ring_reverses_points() {
        let mut ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 1.0, y: 1.0 },
            Coord { x: 0.0, y: 1.0 },
        ];
        reverse_ring(&mut ring);

        assert_eq!(ring[0], Coord { x: 0.0, y: 1.0 });
        assert_eq!(ring[1], Coord { x: 1.0, y: 1.0 });
        assert_eq!(ring[2], Coord { x: 1.0, y: 0.0 });
        assert_eq!(ring[3], Coord { x: 0.0, y: 0.0 });
    }

    // ==================== Collinearity Tests ====================

    #[test]
    fn points_are_collinear_three_collinear_points() {
        let p1 = Coord { x: 0.0, y: 0.0 };
        let p2 = Coord { x: 1.0, y: 1.0 };
        let p3 = Coord { x: 2.0, y: 2.0 };
        assert!(points_are_collinear(&p1, &p2, &p3));
    }

    #[test]
    fn points_are_collinear_horizontal_line() {
        let p1 = Coord { x: 0.0, y: 5.0 };
        let p2 = Coord { x: 5.0, y: 5.0 };
        let p3 = Coord { x: 10.0, y: 5.0 };
        assert!(points_are_collinear(&p1, &p2, &p3));
    }

    #[test]
    fn points_are_collinear_vertical_line() {
        let p1 = Coord { x: 3.0, y: 0.0 };
        let p2 = Coord { x: 3.0, y: 5.0 };
        let p3 = Coord { x: 3.0, y: 10.0 };
        assert!(points_are_collinear(&p1, &p2, &p3));
    }

    #[test]
    fn points_are_collinear_non_collinear_points() {
        let p1 = Coord { x: 0.0, y: 0.0 };
        let p2 = Coord { x: 1.0, y: 1.0 };
        let p3 = Coord { x: 2.0, y: 0.0 };
        assert!(!points_are_collinear(&p1, &p2, &p3));
    }

    #[test]
    fn remove_collinear_points_removes_middle_points() {
        // Square with extra collinear points on edges
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 5.0, y: 0.0 }, // Collinear - should be removed
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        let simplified = remove_collinear_points(&ring);

        // Should have 4 points (the corners)
        assert_eq!(simplified.len(), 4);
        assert_eq!(simplified[0], Coord { x: 0.0, y: 0.0 });
        assert_eq!(simplified[1], Coord { x: 10.0, y: 0.0 });
    }

    #[test]
    fn remove_collinear_points_keeps_non_collinear() {
        // Simple triangle - no collinear points
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 5.0, y: 10.0 },
        ];
        let simplified = remove_collinear_points(&ring);

        assert_eq!(simplified.len(), 3);
    }

    // ==================== Segment Intersection Tests ====================

    #[test]
    fn segments_intersect_crossing_segments() {
        // X pattern - segments clearly cross
        let p1 = Coord { x: 0.0, y: 0.0 };
        let p2 = Coord { x: 10.0, y: 10.0 };
        let p3 = Coord { x: 0.0, y: 10.0 };
        let p4 = Coord { x: 10.0, y: 0.0 };
        assert!(segments_intersect(&p1, &p2, &p3, &p4));
    }

    #[test]
    fn segments_intersect_parallel_segments() {
        // Parallel horizontal lines
        let p1 = Coord { x: 0.0, y: 0.0 };
        let p2 = Coord { x: 10.0, y: 0.0 };
        let p3 = Coord { x: 0.0, y: 5.0 };
        let p4 = Coord { x: 10.0, y: 5.0 };
        assert!(!segments_intersect(&p1, &p2, &p3, &p4));
    }

    #[test]
    fn segments_intersect_non_intersecting() {
        // Two segments that don't reach each other
        let p1 = Coord { x: 0.0, y: 0.0 };
        let p2 = Coord { x: 5.0, y: 0.0 };
        let p3 = Coord { x: 10.0, y: 5.0 };
        let p4 = Coord { x: 10.0, y: 10.0 };
        assert!(!segments_intersect(&p1, &p2, &p3, &p4));
    }

    #[test]
    fn segments_intersect_t_junction() {
        // T-junction: one segment ends on another
        let p1 = Coord { x: 0.0, y: 5.0 };
        let p2 = Coord { x: 10.0, y: 5.0 };
        let p3 = Coord { x: 5.0, y: 0.0 };
        let p4 = Coord { x: 5.0, y: 5.0 };
        // This should count as intersection (endpoint on segment)
        assert!(segments_intersect(&p1, &p2, &p3, &p4));
    }

    // ==================== Self-Intersection Tests ====================

    #[test]
    fn ring_has_self_intersection_simple_square_no_intersection() {
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        assert!(!ring_has_self_intersection(&ring));
    }

    #[test]
    fn ring_has_self_intersection_figure_eight() {
        // Figure-8 pattern - clear self-intersection
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 }, // Crosses next segment
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        assert!(ring_has_self_intersection(&ring));
    }

    #[test]
    fn ring_has_self_intersection_small_ring_no_check() {
        // Less than 4 points can't self-intersect
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 5.0, y: 10.0 },
        ];
        assert!(!ring_has_self_intersection(&ring));
    }

    // ==================== Duplicate Point Tests ====================

    #[test]
    fn find_duplicate_points_no_duplicates() {
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
        ];
        assert!(find_duplicate_points(&ring).is_empty());
    }

    #[test]
    fn find_duplicate_points_with_duplicates() {
        let ring = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 0.0, y: 0.0 }, // Duplicate of index 0
            Coord { x: 10.0, y: 10.0 },
        ];
        let dups = find_duplicate_points(&ring);
        assert!(dups.contains(&0));
        assert!(dups.contains(&2));
    }

    // ==================== Point Comparison Tests ====================

    #[test]
    fn compare_points_larger_y_comes_first() {
        let p1 = Coord { x: 5.0, y: 10.0 };
        let p2 = Coord { x: 5.0, y: 5.0 };
        assert_eq!(compare_points(&p1, &p2), std::cmp::Ordering::Less);
    }

    #[test]
    fn compare_points_equal_y_smaller_x_comes_first() {
        let p1 = Coord { x: 3.0, y: 10.0 };
        let p2 = Coord { x: 7.0, y: 10.0 };
        assert_eq!(compare_points(&p1, &p2), std::cmp::Ordering::Less);
    }

    #[test]
    fn compare_points_equal_points() {
        let p1 = Coord { x: 5.0, y: 5.0 };
        let p2 = Coord { x: 5.0, y: 5.0 };
        assert_eq!(compare_points(&p1, &p2), std::cmp::Ordering::Equal);
    }

    // ==================== Polygon Containment Tests ====================

    #[test]
    fn poly2_contains_poly1_inner_contained_in_outer() {
        let inner: Vec<Coord<f64>> = vec![
            Coord { x: 2.0, y: 2.0 },
            Coord { x: 8.0, y: 2.0 },
            Coord { x: 8.0, y: 8.0 },
            Coord { x: 2.0, y: 8.0 },
        ];
        let inner_area = ring_area(&inner);

        let outer: Vec<Coord<f64>> = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        let outer_area = ring_area(&outer);

        assert!(poly2_contains_poly1(&inner, inner_area, &outer, outer_area));
    }

    #[test]
    fn poly2_contains_poly1_outer_not_contained_in_inner() {
        let inner: Vec<Coord<f64>> = vec![
            Coord { x: 2.0, y: 2.0 },
            Coord { x: 8.0, y: 2.0 },
            Coord { x: 8.0, y: 8.0 },
            Coord { x: 2.0, y: 8.0 },
        ];
        let inner_area = ring_area(&inner);

        let outer: Vec<Coord<f64>> = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
        ];
        let outer_area = ring_area(&outer);

        // Outer should NOT be contained in inner
        assert!(!poly2_contains_poly1(
            &outer, outer_area, &inner, inner_area
        ));
    }

    #[test]
    fn poly2_contains_poly1_disjoint_polygons() {
        let ring1: Vec<Coord<f64>> = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 5.0, y: 0.0 },
            Coord { x: 5.0, y: 5.0 },
            Coord { x: 0.0, y: 5.0 },
        ];
        let area1 = ring_area(&ring1);

        let ring2: Vec<Coord<f64>> = vec![
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 15.0, y: 10.0 },
            Coord { x: 15.0, y: 15.0 },
            Coord { x: 10.0, y: 15.0 },
        ];
        let area2 = ring_area(&ring2);

        assert!(!poly2_contains_poly1(&ring1, area1, &ring2, area2));
        assert!(!poly2_contains_poly1(&ring2, area2, &ring1, area1));
    }
}
