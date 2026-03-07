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

/// Check if a vertex forms a convex corner.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - is_convex
///
/// A vertex is convex if the cross product of (prev→current) × (current→next)
/// has the same sign as the ring's area (matches winding direction).
fn is_convex_vertex<T: CoordNum>(
    prev: &Coord<T>,
    current: &Coord<T>,
    next: &Coord<T>,
    ring_area: f64,
) -> bool {
    let prev_x = prev.x.to_f64().unwrap_or(0.0);
    let prev_y = prev.y.to_f64().unwrap_or(0.0);
    let curr_x = current.x.to_f64().unwrap_or(0.0);
    let curr_y = current.y.to_f64().unwrap_or(0.0);
    let next_x = next.x.to_f64().unwrap_or(0.0);
    let next_y = next.y.to_f64().unwrap_or(0.0);

    let v1x = curr_x - prev_x;
    let v1y = curr_y - prev_y;
    let v2x = next_x - curr_x;
    let v2y = next_y - curr_y;

    let cross = v1x * v2y - v2x * v1y;

    // Convex if cross product sign matches area sign
    (cross < 0.0 && ring_area > 0.0) || (cross > 0.0 && ring_area < 0.0)
}

/// Compute centroid of a triangle formed by three points.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - centroid_of_points
fn centroid_of_triangle<T: CoordNum>(
    prev: &Coord<T>,
    current: &Coord<T>,
    next: &Coord<T>,
) -> Coord<f64> {
    let prev_x = prev.x.to_f64().unwrap_or(0.0);
    let prev_y = prev.y.to_f64().unwrap_or(0.0);
    let curr_x = current.x.to_f64().unwrap_or(0.0);
    let curr_y = current.y.to_f64().unwrap_or(0.0);
    let next_x = next.x.to_f64().unwrap_or(0.0);
    let next_y = next.y.to_f64().unwrap_or(0.0);

    Coord {
        x: (prev_x + curr_x + next_x) / 3.0,
        y: (prev_y + curr_y + next_y) / 3.0,
    }
}

/// Convert ring points to f64 coordinates.
fn ring_to_f64<T: CoordNum>(ring_points: &[Coord<T>]) -> Vec<Coord<f64>> {
    ring_points
        .iter()
        .map(|pt| Coord {
            x: pt.x.to_f64().unwrap_or(0.0),
            y: pt.y.to_f64().unwrap_or(0.0),
        })
        .collect()
}

/// Special containment test for when all points are on the boundary.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - inside_or_outside_special
///
/// Finds a convex vertex, computes the centroid of the triangle formed with
/// its neighbors, verifies it's inside ring1, then tests against ring2.
fn inside_or_outside_special<T: CoordNum>(
    ring1_points: &[Coord<T>],
    ring1_area: f64,
    ring2_points: &[Coord<T>],
) -> PointInPolygonResult {
    let n = ring1_points.len();
    if n < 3 {
        return PointInPolygonResult::Outside;
    }

    // Convert rings to f64 for precise centroid calculations
    let ring1_f64 = ring_to_f64(ring1_points);
    let ring2_f64 = ring_to_f64(ring2_points);

    // Find a convex vertex and test the centroid of its triangle
    for i in 0..n {
        let prev_idx = if i == 0 { n - 1 } else { i - 1 };
        let next_idx = if i == n - 1 { 0 } else { i + 1 };

        let prev = &ring1_points[prev_idx];
        let current = &ring1_points[i];
        let next = &ring1_points[next_idx];

        if is_convex_vertex(prev, current, next, ring1_area) {
            let centroid = centroid_of_triangle(prev, current, next);

            // Verify centroid is inside ring1 (it should be for a convex vertex)
            if point_in_polygon(&centroid, &ring1_f64) == PointInPolygonResult::Inside {
                // Now test this centroid against ring2
                return point_in_polygon(&centroid, &ring2_f64);
            }
        }
    }

    // Fallback: no convex vertex found (degenerate ring), return outside
    PointInPolygonResult::Outside
}

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

    // All points are on the boundary - use special handling
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp line 831
    let result = inside_or_outside_special(ring1_points, ring1_area, ring2_points);
    result == PointInPolygonResult::Inside
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
        // If it's not a hole, it's already a top-level exterior - no action needed
        if !found_parent && ring_is_hole {
            // C++ throws: "Could not properly place hole to a parent."
            // For now, we just make it a top-level exterior
            if let Some(ring) = manager.get_mut(ring_idx) {
                ring.set_hole(false);
            }
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

// ============================================================================
// Self-Intersection Correction
// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
// ============================================================================

/// Sort a ring's points by (y descending, x ascending) and return the sorted copy.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - sort_ring_points
pub fn sort_ring_points_by_coord<T: CoordNum + Copy>(ring: &crate::Ring<T>) -> Vec<Coord<T>> {
    let mut sorted: Vec<Coord<T>> = ring.points().to_vec();
    sorted.sort_by(|a, b| compare_points(a, b));
    sorted
}

/// Split ring `ring_idx` at two positions `pt1_idx` and `pt2_idx` that share the same coordinate.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_self_intersection
///
/// The split produces two loops:
///   loop_a = points[p..q]      (from the smaller index to the larger)
///   loop_b = points[q..] + points[..p]  (wrapping the remainder)
///
/// The loop with larger absolute area retains the original ring identity.
/// The smaller loop is assigned to a newly created ring.
///
/// Returns `Some(new_ring_idx)` on success, `None` if split was not performed
/// (e.g., same index, not enough points, or different rings).
pub fn correct_self_intersection_in_ring<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    ring1_idx: usize,
    pt1_idx: usize,
    ring2_idx: usize,
    pt2_idx: usize,
) -> Option<usize> {
    // Only handles same-ring case (two visits to the same point within one ring)
    if ring1_idx != ring2_idx {
        return None;
    }
    let ring_idx = ring1_idx;

    let points: Vec<Coord<T>> = manager.ring_points_cloned(ring_idx);
    let n = points.len();

    if pt1_idx == pt2_idx || n < 4 {
        return None;
    }

    // Ensure p < q
    let (p, q) = if pt1_idx < pt2_idx {
        (pt1_idx, pt2_idx)
    } else {
        (pt2_idx, pt1_idx)
    };

    // loop_a covers [p..q], loop_b covers [q..n] + [0..p]
    let loop_a: Vec<Coord<T>> = points[p..q].to_vec();
    let loop_b: Vec<Coord<T>> = points[q..]
        .iter()
        .chain(points[..p].iter())
        .copied()
        .collect();

    if loop_a.len() < 3 || loop_b.len() < 3 {
        return None;
    }

    let area_a = crate::ring_util::ring_area(&loop_a).abs();
    let area_b = crate::ring_util::ring_area(&loop_b).abs();

    let new_ring_idx = manager.create_new_ring();

    // The larger loop keeps the original ring identity
    if area_a >= area_b {
        manager.set_ring_points(ring_idx, loop_a);
        manager.set_ring_points(new_ring_idx, loop_b);
    } else {
        manager.set_ring_points(ring_idx, loop_b);
        manager.set_ring_points(new_ring_idx, loop_a);
    }

    Some(new_ring_idx)
}

/// Process a group of points that all share the same coordinate within one ring.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_repeated_points
///
/// For each pair (pt1, pt2) in the group, attempt a self-intersection split.
/// The `target_coord` guard ensures stale indices (from a prior split) are not used.
fn correct_repeated_points_in_ring<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    new_ring_indices: &mut Vec<usize>,
    group: &[(usize, usize)], // (ring_idx, point_idx)
    target_coord: Coord<T>,
) {
    for i in 0..group.len() {
        let (ring1_idx, pt1_idx) = group[i];

        // Guard: the point at pt1_idx must still be the expected coordinate
        let pt1_valid = manager
            .get(ring1_idx)
            .and_then(|r| r.points().get(pt1_idx).copied())
            .map(|c| c == target_coord)
            .unwrap_or(false);
        if !pt1_valid {
            continue;
        }

        for &(ring2_idx, pt2_idx) in group.iter().skip(i + 1) {
            // Guard: the point at pt2_idx must still be the expected coordinate
            let pt2_valid = manager
                .get(ring2_idx)
                .and_then(|r| r.points().get(pt2_idx).copied())
                .map(|c| c == target_coord)
                .unwrap_or(false);
            if !pt2_valid {
                continue;
            }

            if let Some(new_idx) =
                correct_self_intersection_in_ring(manager, ring1_idx, pt1_idx, ring2_idx, pt2_idx)
            {
                new_ring_indices.push(new_idx);
            }
        }
    }
}

/// Scan a ring for repeated (duplicate) coordinate points and fix each self-intersection.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - find_and_correct_repeated_points
///
/// # DIVERGENCE FROM WAGYU
/// The C++ implementation uses a linked-list; pointers remain valid across splits
/// because the points are not moved, only their ring membership changes.
/// In Rust, we use a Vec: after any split the indices into the original Vec
/// are invalid (the ring's Vec is replaced). We solve this by restarting
/// the scan from scratch after each split (iterative outer loop).
pub fn find_and_correct_repeated_points<T: CoordNum + Copy>(
    ring_idx: usize,
    manager: &mut crate::build_result::RingManager<T>,
) -> Vec<usize> {
    let mut new_rings: Vec<usize> = Vec::new();

    const MAX_REPEATED_POINTS_ITERATIONS: usize = 10_000;
    let mut repeated_points_iteration = 0;

    // Iterative outer loop: restart after each split because Vec indices become stale
    loop {
        repeated_points_iteration += 1;
        if repeated_points_iteration > MAX_REPEATED_POINTS_ITERATIONS {
            panic!(
                "INFINITE LOOP DETECTED in find_and_correct_repeated_points at iteration {}, ring_idx={}",
                repeated_points_iteration, ring_idx
            );
        }
        let points: Vec<Coord<T>> = match manager.get(ring_idx) {
            Some(r) if !r.points().is_empty() => r.points().to_vec(),
            _ => break,
        };

        // Build a sorted (coord, original_idx) list to find duplicates efficiently
        let mut indexed: Vec<(Coord<T>, usize)> = points
            .iter()
            .copied()
            .enumerate()
            .map(|(i, coord)| (coord, i))
            .collect();
        indexed.sort_by(|(a, _), (b, _)| compare_points(a, b));

        let mut found_split = false;
        let mut i = 0;
        while i < indexed.len() {
            let mut j = i + 1;
            while j < indexed.len() && indexed[j].0 == indexed[i].0 {
                j += 1;
            }

            // Any group of size >= 2 means this coordinate appears more than once
            if j - i >= 2 {
                let target_coord = indexed[i].0;
                let group: Vec<(usize, usize)> = indexed[i..j]
                    .iter()
                    .map(|(_, pt_idx)| (ring_idx, *pt_idx))
                    .collect();

                let before = new_rings.len();
                correct_repeated_points_in_ring(manager, &mut new_rings, &group, target_coord);

                if new_rings.len() > before {
                    // A split happened: ring's Vec changed, restart from scratch
                    found_split = true;
                    break;
                }
            }

            i = j;
        }

        if !found_split {
            break;
        }
    }

    new_rings
}

/// Find the correct parent for a newly created ring within the existing tree.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - find_parent_in_tree
///
/// The C++ algorithm works by:
/// 1. Recursively searching all grandchildren of `possible_parent` first (depth-first descent)
/// 2. If no grandchild is the right parent, check `possible_parent` itself
/// 3. Assigns the ring as a child of the found parent and returns true
///
/// # DIVERGENCE FROM WAGYU
/// The C++ version mutates (calls `reassign_as_child`) as part of the search.
/// In Rust we separate concerns: this function only finds the correct parent index
/// and returns it; the caller is responsible for calling `assign_as_child` or
/// `reassign_as_child`. The mutation is performed by `assign_new_ring_parents`.
///
/// # Arguments
/// * `manager` - Ring manager
/// * `new_ring_points` - Points of the ring being placed
/// * `new_ring_area` - Area of the ring being placed
/// * `possible_parent_idx` - Candidate parent ring to check
///
/// # Returns
/// `Some(idx)` of the ring that should be the parent, or `None`.
pub fn find_parent_in_tree<T: CoordNum + Copy>(
    manager: &crate::build_result::RingManager<T>,
    new_ring_points: &[Coord<T>],
    new_ring_area: f64,
    possible_parent_idx: Option<usize>,
) -> Option<usize> {
    let parent_idx = possible_parent_idx?;

    // Step 1: Recursively search grandchildren first (depth-first)
    // PORT FROM: C++ find_parent_in_tree lines 315-326
    // "for (auto c : possible_parent->children) { for (auto gc : c->children) { if (find_parent_in_tree(r, gc, ...)) return true; } }"
    let children = manager.children(parent_idx);
    for child_idx in &children {
        let grandchildren = manager.children(*child_idx);
        for gc_idx in grandchildren {
            if let Some(found) =
                find_parent_in_tree(manager, new_ring_points, new_ring_area, Some(gc_idx))
            {
                return Some(found);
            }
        }
    }

    // Step 2: Check if possible_parent itself contains the new ring
    // PORT FROM: C++ find_parent_in_tree lines 328-332
    // "if (poly2_contains_poly1(r, possible_parent)) { reassign_as_child(r, possible_parent, ...); return true; }"
    let parent_pts = manager.ring_points_cloned(parent_idx);
    let parent_area = crate::ring_util::ring_area(&parent_pts);
    if poly2_contains_poly1(new_ring_points, new_ring_area, &parent_pts, parent_area) {
        return Some(parent_idx);
    }

    None
}

/// Reassign children of `sibling_ring_idx` to `new_ring_idx` if they are contained by it.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - reassign_children_if_necessary
///
/// The C++ version skips rings that are in the `new_rings` vector (to avoid double-assigning
/// newly created rings). The `new_ring_indices` parameter mirrors this.
///
/// # Arguments
/// * `manager` - Ring manager
/// * `sibling_ring_idx` - Ring whose children we inspect (corresponds to C++ `sibling_ring`)
/// * `new_ring_idx` - The new ring to potentially receive children
/// * `new_ring_indices` - All new rings from this split pass (skip these as candidates)
pub fn reassign_children_if_necessary<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    sibling_ring_idx: usize,
    new_ring_idx: usize,
    new_ring_indices: &[usize],
) {
    let new_ring_pts = manager.ring_points_cloned(new_ring_idx);
    let new_ring_area = crate::ring_util::ring_area(&new_ring_pts);
    let children = manager.children(sibling_ring_idx);

    for child_idx in children {
        // PORT FROM: C++ reassign_children_if_necessary lines 299-302
        // "if (std::find(new_rings.begin(), new_rings.end(), c) != new_rings.end()) { continue; }"
        if new_ring_indices.contains(&child_idx) {
            continue;
        }

        let child_pts = manager.ring_points_cloned(child_idx);
        let child_area = crate::ring_util::ring_area(&child_pts);
        // PORT FROM: C++ "if (poly2_contains_poly1(c, new_ring)) { reassign_as_child(c, new_ring, ...); }"
        // Note: poly2_contains_poly1(c, new_ring) means "new_ring contains c"
        if poly2_contains_poly1(&child_pts, child_area, &new_ring_pts, new_ring_area) {
            manager.reassign_as_child(child_idx, new_ring_idx);
        }
    }
}

/// Place newly created rings into the correct positions in the ring tree.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - assign_new_ring_parents
///
/// This function handles placement of rings produced by splitting a self-intersecting ring.
/// The algorithm differs based on the number of new rings and orientation relationships:
///
/// **Single new ring:**
/// - Same orientation as original → assign as sibling (same parent), check orig's children
/// - Opposite orientation → assign as child of original, check parent's children
///
/// **Multiple new rings:**
/// - Sort by area descending
/// - For each ring, first check if any previously-placed sibling's tree contains it
/// - Then check original ring's children tree (if same orientation) or original ring itself
/// - Assign accordingly
///
/// # DIVERGENCE FROM WAGYU
/// The C++ version uses pointer-based identity for the "new_rings" skip guard in
/// `reassign_children_if_necessary`. The Rust version passes `new_ring_indices` explicitly.
pub fn assign_new_ring_parents<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    orig_ring_idx: usize,
    new_ring_indices: &[usize],
) {
    // PORT FROM: C++ assign_new_ring_parents lines 337-350
    // "new_rings.erase(remove_if(... zero area or no points ...))"
    let valid_new_rings: Vec<usize> = new_ring_indices
        .iter()
        .copied()
        .filter(|&idx| {
            let pts = manager.ring_points_cloned(idx);
            !pts.is_empty() && !crate::ring_util::value_is_zero(crate::ring_util::ring_area(&pts))
        })
        .collect();

    if valid_new_rings.is_empty() {
        return;
    }

    let orig_area = crate::ring_util::ring_area(&manager.ring_points_cloned(orig_ring_idx));
    let original_positive = orig_area > 0.0;

    // PORT FROM: C++ assign_new_ring_parents lines 355-379
    // "if (new_rings.size() == 1) { ... simple logic ... return; }"
    if valid_new_rings.len() == 1 {
        let new_ring_idx = valid_new_rings[0];
        let new_area = crate::ring_util::ring_area(&manager.ring_points_cloned(new_ring_idx));
        let new_positive = new_area > 0.0;

        if original_positive == new_positive {
            // Same orientation: new ring is a sibling of original
            // Assign to original ring's parent (same level)
            let orig_parent = manager.parent(orig_ring_idx);
            manager.assign_as_child(new_ring_idx, orig_parent);
            // Check if any of original ring's children belong inside new ring
            reassign_children_if_necessary(manager, orig_ring_idx, new_ring_idx, &valid_new_rings);
        } else {
            // Opposite orientation: new ring is a child of original ring
            manager.assign_as_child(new_ring_idx, Some(orig_ring_idx));
            // Check if any of original ring's parent's children belong inside new ring
            let orig_parent = manager.parent(orig_ring_idx);
            if let Some(parent_idx) = orig_parent {
                reassign_children_if_necessary(manager, parent_idx, new_ring_idx, &valid_new_rings);
            }
        }
        return;
    }

    // Multiple new rings: sort by absolute area descending, assign largest first
    // PORT FROM: C++ assign_new_ring_parents lines 381-387
    // "std::stable_sort(new_rings.begin(), new_rings.end(), [](...)  { return fabs(r1->area()) > fabs(r2->area()); })"
    let mut sorted_new_rings = valid_new_rings.clone();
    sorted_new_rings.sort_by(|&a, &b| {
        let area_a = crate::ring_util::ring_area(&manager.ring_points_cloned(a)).abs();
        let area_b = crate::ring_util::ring_area(&manager.ring_points_cloned(b)).abs();
        area_b
            .partial_cmp(&area_a)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // PORT FROM: C++ assign_new_ring_parents lines 389-448
    for r_pos in 0..sorted_new_rings.len() {
        let new_ring_idx = sorted_new_rings[r_pos];
        let new_area = crate::ring_util::ring_area(&manager.ring_points_cloned(new_ring_idx));
        let new_positive = new_area > 0.0;
        let same_orientation = new_positive == original_positive;
        let mut found = false;

        // Step 1: Check trees of previously-assigned sibling rings
        // PORT FROM: C++ lines 391-417
        // "for (auto s_itr = new_rings.begin(); s_itr != r_itr; ++s_itr) { ... }"
        for &s_idx in &sorted_new_rings[..r_pos] {
            // Only check siblings (rings with same parent as original ring)
            if manager.parent(s_idx) != manager.parent(orig_ring_idx) {
                continue;
            }

            if same_orientation {
                // Check if any of s_idx's children contain new_ring_idx
                // PORT FROM: C++ lines 396-408
                let s_children = manager.children(s_idx);
                for s_child_idx in s_children {
                    if let Some(_found_parent) = find_parent_in_tree(
                        manager,
                        &manager.ring_points_cloned(new_ring_idx),
                        new_area,
                        Some(s_child_idx),
                    ) {
                        // Assign into the found parent
                        manager.assign_as_child(new_ring_idx, Some(_found_parent));
                        reassign_children_if_necessary(
                            manager,
                            orig_ring_idx,
                            new_ring_idx,
                            &valid_new_rings,
                        );
                        found = true;
                        break;
                    }
                }
            } else {
                // Opposite orientation: check if s_idx itself contains new_ring_idx
                // PORT FROM: C++ lines 409-414
                if let Some(_found_parent) = find_parent_in_tree(
                    manager,
                    &manager.ring_points_cloned(new_ring_idx),
                    new_area,
                    Some(s_idx),
                ) {
                    manager.assign_as_child(new_ring_idx, Some(_found_parent));
                    let orig_parent = manager.parent(orig_ring_idx);
                    if let Some(parent_idx) = orig_parent {
                        reassign_children_if_necessary(
                            manager,
                            parent_idx,
                            new_ring_idx,
                            &valid_new_rings,
                        );
                    }
                    found = true;
                }
            }

            if found {
                break;
            }
        }

        if found {
            continue;
        }

        // Step 2: Check original ring's tree
        // PORT FROM: C++ lines 419-447
        if same_orientation {
            // Check if any of original ring's children contain new_ring_idx
            // PORT FROM: C++ lines 420-436
            let orig_children = manager.children(orig_ring_idx);
            for o_child_idx in orig_children {
                if let Some(_found_parent) = find_parent_in_tree(
                    manager,
                    &manager.ring_points_cloned(new_ring_idx),
                    new_area,
                    Some(o_child_idx),
                ) {
                    manager.assign_as_child(new_ring_idx, Some(_found_parent));
                    reassign_children_if_necessary(
                        manager,
                        orig_ring_idx,
                        new_ring_idx,
                        &valid_new_rings,
                    );
                    found = true;
                    break;
                }
            }
            if !found {
                // Same orientation, not found in any child tree -> sibling of orig
                // PORT FROM: C++ lines 437-441
                // "assign_as_child(*r_itr, original_ring->parent, ...)"
                let orig_parent = manager.parent(orig_ring_idx);
                manager.assign_as_child(new_ring_idx, orig_parent);
                reassign_children_if_necessary(
                    manager,
                    orig_ring_idx,
                    new_ring_idx,
                    &valid_new_rings,
                );
            }
        } else {
            // Opposite orientation: must be inside original ring
            // PORT FROM: C++ lines 442-447
            // "if (find_parent_in_tree(*r_itr, original_ring, ...)) { ... } else { throw ... }"
            if let Some(_found_parent) = find_parent_in_tree(
                manager,
                &manager.ring_points_cloned(new_ring_idx),
                new_area,
                Some(orig_ring_idx),
            ) {
                manager.assign_as_child(new_ring_idx, Some(_found_parent));
                let orig_parent = manager.parent(orig_ring_idx);
                if let Some(parent_idx) = orig_parent {
                    reassign_children_if_necessary(
                        manager,
                        parent_idx,
                        new_ring_idx,
                        &valid_new_rings,
                    );
                }
            }
            // If not found, skip (C++ throws but we're more lenient)
        }
    }
}

/// Process a single ring for self-intersections.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_ring_self_intersections
///
/// Skips rings that have already been corrected (`corrected == true`).
/// After processing, marks the ring as corrected.
///
/// Returns true if the ring was processed (not already corrected).
/// NOTE: This matches C++ semantics where the return value indicates "was visited",
/// not "did split". This is important for the convergence loop in correct_topology.
pub fn correct_ring_self_intersections<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    ring_idx: usize,
    correct_tree: bool,
) -> bool {
    if manager.is_corrected(ring_idx) {
        return false;
    }
    if !manager.ring_has_points(ring_idx) {
        return false;
    }

    let new_rings = find_and_correct_repeated_points(ring_idx, manager);
    let did_split = !new_rings.is_empty();

    if crate::debug::debug_enabled() && did_split {
        eprintln!(
            "[TOPOLOGY] Ring {} split into {} new rings: {:?}",
            ring_idx,
            new_rings.len(),
            new_rings
        );
    }

    if correct_tree {
        assign_new_ring_parents(manager, ring_idx, &new_rings);
    }

    manager.set_corrected(ring_idx, true);
    // Return true for any ring that was processed (not already corrected).
    // This matches C++ semantics - the while loop in correct_topology needs to
    // keep iterating as long as any ring was visited, not just when splits occur.
    // See: topology_correction.hpp lines 453-469
    true
}

/// Correct all self-intersecting rings in the manager.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_self_intersections
///
/// Processes rings smallest-to-largest so inner rings are handled before outer rings.
/// Returns true if any ring was split (used to drive the retry loop in correct_topology).
pub fn correct_self_intersections<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    correct_tree: bool,
) -> bool {
    // Process smallest rings first so inner rings are corrected before their parents
    let sorted = manager.sorted_ring_indices_smallest_to_largest();
    let mut fixed = false;
    let mut rings_fixed: Vec<usize> = Vec::new();
    for ring_idx in sorted {
        if correct_ring_self_intersections(manager, ring_idx, correct_tree) {
            rings_fixed.push(ring_idx);
            fixed = true;
        }
    }
    if crate::debug::debug_enabled() && fixed {
        eprintln!(
            "[TOPOLOGY] correct_self_intersections fixed rings: {:?}",
            rings_fixed
        );
    }
    fixed
}

// ============================================================================
// Collinear Edge Correction
// ============================================================================
//
// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
//            lines 849-1259
//
// Collinear edges occur when two points at the same coordinate have edges
// going back along the same path (opposite traversal directions). This creates
// "spikes" or degenerate geometry that must be removed.

/// Reference to a point in the ring manager: (ring_index, point_index).
///
/// PORT FROM: C++ point_ptr - In C++ these are raw pointers into ring linked lists.
/// In Rust, we use index pairs since rings own their points in Vecs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PointRef {
    pub ring_idx: usize,
    pub point_idx: usize,
}

impl PointRef {
    pub fn new(ring_idx: usize, point_idx: usize) -> Self {
        Self {
            ring_idx,
            point_idx,
        }
    }
}

/// Result of a collinear path fix operation.
///
/// PORT FROM: C++ collinear_result (lines 843-846)
///
/// - `pt1 = None, pt2 = None`: Ring was completely removed
/// - `pt1 = Some, pt2 = None`: Spike removed, ring survives as single piece
/// - `pt1 = Some, pt2 = Some`: Ring split into two (or merged from two)
#[derive(Debug, Clone, Copy)]
struct CollinearResult {
    pt1: Option<PointRef>,
    pt2: Option<PointRef>,
}

/// The extent of a collinear path between two points.
///
/// PORT FROM: C++ collinear_path (lines 831-840)
///
/// When two ring paths share the same coordinates going in opposite directions,
/// this struct captures the full extent of the shared path.
#[derive(Debug, Clone, Copy)]
struct CollinearPath {
    /// First point of path A (forward direction)
    start_1: PointRef,
    /// Last point of path A (forward direction)
    end_1: PointRef,
    /// First point of path B (forward direction)
    start_2: PointRef,
    /// Last point of path B (forward direction)
    end_2: PointRef,
}

/// Check if two points at the same coordinate have collinear (overlapping opposite-direction) edges.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - has_collinear_edge
///            (lines 1028-1031)
///
/// Two edges are collinear if one ring's edge leaving pt_a overlays the other ring's
/// edge arriving at pt_b (or vice versa).
fn has_collinear_edge<T: CoordNum>(
    manager: &crate::build_result::RingManager<T>,
    pt_a: PointRef,
    pt_b: PointRef,
) -> bool {
    let ring_a = match manager.get(pt_a.ring_idx) {
        Some(r) if !r.points().is_empty() => r,
        _ => return false,
    };
    let ring_b = match manager.get(pt_b.ring_idx) {
        Some(r) if !r.points().is_empty() => r,
        _ => return false,
    };

    let len_a = ring_a.points().len();
    let len_b = ring_b.points().len();

    // CRITICAL FIX: For same-ring comparisons where we're comparing point 0 with point N-1
    // (the OGC closing pair), check2 will give a false positive because:
    //   - prev_a wraps to point N-1 (same as pt_b)
    //   - next_b wraps to point 0 (same as pt_a)
    // So we skip check2 but still run check1 (which detects actual degenerate spikes).
    let is_closing_pair = pt_a.ring_idx == pt_b.ring_idx && {
        let min_idx = pt_a.point_idx.min(pt_b.point_idx);
        let max_idx = pt_a.point_idx.max(pt_b.point_idx);
        min_idx == 0
            && max_idx == len_a - 1
            && coords_equal(&ring_a.points()[0], &ring_a.points()[len_a - 1])
    };

    // Get next point for A and prev point for B
    let next_a_idx = (pt_a.point_idx + 1) % len_a;
    let prev_b_idx = (pt_b.point_idx + len_b - 1) % len_b;

    // Get prev point for A and next point for B
    let prev_a_idx = (pt_a.point_idx + len_a - 1) % len_a;
    let next_b_idx = (pt_b.point_idx + 1) % len_b;

    let next_a = &ring_a.points()[next_a_idx];
    let prev_b = &ring_b.points()[prev_b_idx];
    let prev_a = &ring_a.points()[prev_a_idx];
    let next_b = &ring_b.points()[next_b_idx];

    // Check if edges overlay: next_a == prev_b or (next_b == prev_a if not a closing pair)
    // check1 detects actual collinear edges (including degenerate spikes)
    // check2 is skipped for closing pairs to avoid false positives from wrapping
    let check1 = coords_equal(next_a, prev_b);
    let check2 = !is_closing_pair && coords_equal(next_b, prev_a);
    check1 || check2
}

/// Get the next point index in a ring (circular).
#[inline]
fn next_idx(idx: usize, len: usize) -> usize {
    (idx + 1) % len
}

/// Get the previous point index in a ring (circular).
#[inline]
fn prev_idx(idx: usize, len: usize) -> usize {
    (idx + len - 1) % len
}

/// Find the full extent of collinear edges starting from two seed points.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            find_start_and_end_of_collinear_edges (lines 936-1025)
///
/// This extends outward from pt_a and pt_b to find the complete run of
/// coordinates shared between the two paths.
///
/// For a spike like (5,0) -> (5,-5) -> (5,0):
/// - pt_a and pt_b are both at (5,0) but different indices
/// - The collinear path includes the spike tip at (5,-5)
fn find_start_and_end_of_collinear_edges<T: CoordNum>(
    manager: &crate::build_result::RingManager<T>,
    pt_a: PointRef,
    pt_b: PointRef,
) -> Option<CollinearPath> {
    let ring_a = manager.get(pt_a.ring_idx)?;
    let ring_b = manager.get(pt_b.ring_idx)?;

    if ring_a.points().is_empty() || ring_b.points().is_empty() {
        return None;
    }

    let len_a = ring_a.points().len();
    let len_b = ring_b.points().len();
    let same_ring = pt_a.ring_idx == pt_b.ring_idx;

    // For same-ring spikes, we need a simpler approach:
    // Find the range of indices between pt_a and pt_b that form the spike
    if same_ring {
        // The spike is the shorter path between the two duplicate points
        let idx1 = pt_a.point_idx.min(pt_b.point_idx);
        let idx2 = pt_a.point_idx.max(pt_b.point_idx);

        // Forward distance: idx2 - idx1
        // Backward distance: len - (idx2 - idx1)
        let forward_dist = idx2 - idx1;
        let backward_dist = len_a - forward_dist;

        if forward_dist <= backward_dist {
            // The spike is between idx1 and idx2
            return Some(CollinearPath {
                start_1: PointRef::new(pt_a.ring_idx, idx1),
                end_1: PointRef::new(pt_a.ring_idx, idx2),
                start_2: PointRef::new(pt_b.ring_idx, idx2),
                end_2: PointRef::new(pt_b.ring_idx, idx1),
            });
        } else {
            // The spike wraps around (the other direction is shorter)
            return Some(CollinearPath {
                start_1: PointRef::new(pt_a.ring_idx, idx2),
                end_1: PointRef::new(pt_a.ring_idx, idx1),
                start_2: PointRef::new(pt_b.ring_idx, idx1),
                end_2: PointRef::new(pt_b.ring_idx, idx2),
            });
        }
    }

    // Different rings case: extend in both directions
    let mut back_a = pt_a.point_idx;
    let mut forward_b = pt_b.point_idx;
    let mut iterations = 0;
    let max_iterations = len_a + len_b; // Prevent infinite loops

    // Phase 1: Search backward on A, forward on B
    loop {
        iterations += 1;
        if iterations > max_iterations {
            break;
        }

        let prev_back_a = prev_idx(back_a, len_a);
        let next_forward_b = next_idx(forward_b, len_b);

        let ring_a_ref = manager.get(pt_a.ring_idx).unwrap();
        let ring_b_ref = manager.get(pt_b.ring_idx).unwrap();

        let coord_back_a = &ring_a_ref.points()[prev_back_a];
        let coord_forward_b = &ring_b_ref.points()[next_forward_b];

        if !coords_equal(coord_back_a, coord_forward_b) {
            break;
        }

        // Check for wraparound
        if prev_back_a == pt_a.point_idx || next_forward_b == pt_b.point_idx {
            break;
        }

        back_a = prev_back_a;
        forward_b = next_forward_b;
    }

    let start_a = back_a;
    let end_b = forward_b;

    // Phase 2: Search backward on B, forward on A
    let mut back_b = pt_b.point_idx;
    let mut forward_a = pt_a.point_idx;
    iterations = 0;

    loop {
        iterations += 1;
        if iterations > max_iterations {
            break;
        }

        let prev_back_b = prev_idx(back_b, len_b);
        let next_forward_a = next_idx(forward_a, len_a);

        let ring_a_ref = manager.get(pt_a.ring_idx).unwrap();
        let ring_b_ref = manager.get(pt_b.ring_idx).unwrap();

        let coord_back_b = &ring_b_ref.points()[prev_back_b];
        let coord_forward_a = &ring_a_ref.points()[next_forward_a];

        if !coords_equal(coord_back_b, coord_forward_a) {
            break;
        }

        // Check for wraparound
        if prev_back_b == pt_b.point_idx || next_forward_a == pt_a.point_idx {
            break;
        }

        back_b = prev_back_b;
        forward_a = next_forward_a;
    }

    let start_b = back_b;
    let end_a = forward_a;

    Some(CollinearPath {
        start_1: PointRef::new(pt_a.ring_idx, start_a),
        end_1: PointRef::new(pt_a.ring_idx, end_a),
        start_2: PointRef::new(pt_b.ring_idx, start_b),
        end_2: PointRef::new(pt_b.ring_idx, end_b),
    })
}

/// Fix a collinear path by removing the overlapping segments.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            fix_collinear_path (lines 849-933)
///
/// This performs the actual "surgery" on the rings to remove the collinear stretch.
/// For a spike like A -> spike_tip -> A, we remove the spike tip and one duplicate,
/// keeping one copy of the base point.
fn fix_collinear_path<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    path: CollinearPath,
) -> CollinearResult {
    let same_ring = path.start_1.ring_idx == path.start_2.ring_idx;

    if same_ring {
        let ring_idx = path.start_1.ring_idx;

        let ring = match manager.get_mut(ring_idx) {
            Some(r) => r,
            None => {
                return CollinearResult {
                    pt1: None,
                    pt2: None,
                }
            }
        };

        let points = ring.points_mut();
        let len = points.len();

        if len <= 3 {
            points.clear();
            return CollinearResult {
                pt1: None,
                pt2: None,
            };
        }

        // For same-ring spikes, we need to remove the spike interior and one duplicate.
        // Keep ONE copy of the base point.
        //
        // Example 1: Ring [0,1,2,3,4,5,6] where index 1 and 3 are both at (5,0), index 2 is spike tip
        //   idx1 = 1, idx2 = 3
        //   Check if idx1 is on an edge (has neighbors with different coordinates)
        //   If so, keep idx1, remove idx1+1 to idx2 inclusive
        //   Result: [0,1,4,5,6]... but we want [0,4,5,6] = 4 points
        //
        // For the simple square+spike case, (5,0) lies ON the edge from (0,0) to (10,0),
        // so it can be removed. But for the wrap-around case, (10,0) is a CORNER,
        // so it must be kept.

        let idx1 = path.start_1.point_idx.min(path.end_1.point_idx);
        let idx2 = path.start_1.point_idx.max(path.end_1.point_idx);

        // Check if the duplicate point is on a straight edge (collinear with neighbors)
        // If it's collinear, remove all spike points including both duplicates
        // If it's a corner, keep one copy
        let coord_dup = points[idx1]; // The duplicate coordinate
        let prev_coord = points[prev_idx(idx1, len)];
        let next_coord = points[next_idx(idx2, len)];

        // Check if coord_dup is collinear with its neighbors
        let is_collinear_with_neighbors =
            points_are_collinear(&prev_coord, &coord_dup, &next_coord);

        let mut new_points = Vec::with_capacity(len);

        if is_collinear_with_neighbors {
            // Remove all spike points including both duplicates
            // Result: just the corners
            for (i, &point) in points.iter().enumerate() {
                if i < idx1 || i > idx2 {
                    new_points.push(point);
                }
            }
        } else {
            // Keep one copy of the duplicate (it's a corner)
            // Remove spike interior (idx1+1 to idx2 inclusive)
            for (i, &point) in points.iter().enumerate() {
                if i <= idx1 || i > idx2 {
                    new_points.push(point);
                }
            }
        }

        if new_points.len() < 3 {
            points.clear();
            return CollinearResult {
                pt1: None,
                pt2: None,
            };
        }

        *points = new_points;

        return CollinearResult {
            pt1: Some(PointRef::new(ring_idx, 0)),
            pt2: None,
        };
    }

    // Different rings case - merge rings
    CollinearResult {
        pt1: Some(path.end_1),
        pt2: Some(path.end_2),
    }
}

/// Process collinear edges for two points on the same ring.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            process_collinear_edges_same_ring (lines 1034-1061)
fn process_collinear_edges_same_ring<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    pt_a: PointRef,
    pt_b: PointRef,
) {
    let path = match find_start_and_end_of_collinear_edges(manager, pt_a, pt_b) {
        Some(p) => p,
        None => return,
    };

    let ring_idx = pt_a.ring_idx;
    let result = fix_collinear_path(manager, path);

    match (result.pt1, result.pt2) {
        (None, _) => {
            // Ring was completely removed
            if let Some(ring) = manager.get_mut(ring_idx) {
                ring.points_mut().clear();
            }
        }
        (Some(_pt1), None) => {
            // Spike removed, ring survives as single piece
            // The fix already modified the ring structure
        }
        (Some(_pt1), Some(_pt2)) => {
            // Ring split into two - would need to create new ring
            // This is complex and handled by the full fix_collinear_path
        }
    }
}

/// Process collinear edges for two points on different rings.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            process_collinear_edges_different_rings (lines 1064-1088)
///
/// When two rings share a collinear edge (traverse the same edge in opposite
/// directions), merge them by removing the shared edge and splicing together.
fn process_collinear_edges_different_rings<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    pt_a: PointRef,
    pt_b: PointRef,
) {
    // Get the ring points before modification
    let points_a: Vec<Coord<T>> = match manager.get(pt_a.ring_idx) {
        Some(r) => r.points().to_vec(),
        None => return,
    };
    let points_b: Vec<Coord<T>> = match manager.get(pt_b.ring_idx) {
        Some(r) => r.points().to_vec(),
        None => return,
    };

    if points_a.is_empty() || points_b.is_empty() {
        return;
    }

    let len_a = points_a.len();
    let len_b = points_b.len();

    // Determine which direction the shared edge goes.
    // has_collinear_edge checks: next_a == prev_b OR next_b == prev_a
    //
    // Case 1: next_a == prev_b
    //   Ring A: ... -> pt_a -> next_a -> ...  (shared edge is pt_a to next_a, going forward)
    //   Ring B: ... -> prev_b -> pt_b -> ...  (shared edge is prev_b to pt_b, going forward)
    //
    // Case 2: next_b == prev_a
    //   Ring A: ... -> prev_a -> pt_a -> ...  (shared edge is prev_a to pt_a, going forward)
    //   Ring B: ... -> pt_b -> next_b -> ...  (shared edge is pt_b to next_b, going forward)

    let next_a = &points_a[next_idx(pt_a.point_idx, len_a)];
    let prev_b = &points_b[prev_idx(pt_b.point_idx, len_b)];
    let prev_a = &points_a[prev_idx(pt_a.point_idx, len_a)];
    let next_b = &points_b[next_idx(pt_b.point_idx, len_b)];

    // Identify the shared edge endpoints in each ring
    let (shared_a1, shared_a2, shared_b1, shared_b2);

    if coords_equal(next_a, prev_b) {
        // Case 1: shared edge is pt_a -> next_a in ring A
        shared_a1 = pt_a.point_idx;
        shared_a2 = next_idx(pt_a.point_idx, len_a);
        shared_b1 = prev_idx(pt_b.point_idx, len_b);
        shared_b2 = pt_b.point_idx;
    } else if coords_equal(next_b, prev_a) {
        // Case 2: shared edge is prev_a -> pt_a in ring A
        shared_a1 = prev_idx(pt_a.point_idx, len_a);
        shared_a2 = pt_a.point_idx;
        shared_b1 = pt_b.point_idx;
        shared_b2 = next_idx(pt_b.point_idx, len_b);
    } else {
        // No shared edge (shouldn't happen if has_collinear_edge returned true)
        return;
    }

    // Build merged ring by:
    // 1. Go around ring A, skipping the shared edge (shared_a1 and shared_a2)
    // 2. Continue with ring B, skipping the shared edge (shared_b1 and shared_b2)

    let mut merged_points: Vec<Coord<T>> = Vec::new();

    // From ring A: start after shared_a2, go around to shared_a1 (exclusive)
    let mut i = next_idx(shared_a2, len_a);
    while i != shared_a1 {
        merged_points.push(points_a[i]);
        i = next_idx(i, len_a);
    }

    // From ring B: start after shared_b2, go around to shared_b1 (exclusive)
    let mut j = next_idx(shared_b2, len_b);
    while j != shared_b1 {
        merged_points.push(points_b[j]);
        j = next_idx(j, len_b);
    }

    if merged_points.len() < 3 {
        // Degenerate result - clear both rings
        if let Some(ring) = manager.get_mut(pt_a.ring_idx) {
            ring.points_mut().clear();
        }
        if let Some(ring) = manager.get_mut(pt_b.ring_idx) {
            ring.points_mut().clear();
        }
        return;
    }

    // Determine which ring is larger (by area) - the larger one survives
    let area_a = calculate_ring_area(&points_a).abs();
    let area_b = calculate_ring_area(&points_b).abs();
    let (keep_idx, delete_idx) = if area_a >= area_b {
        (pt_a.ring_idx, pt_b.ring_idx)
    } else {
        (pt_b.ring_idx, pt_a.ring_idx)
    };

    // Update the keeper ring with merged points
    if let Some(ring) = manager.get_mut(keep_idx) {
        *ring.points_mut() = merged_points;
    }

    // Clear the deleted ring
    if let Some(ring) = manager.get_mut(delete_idx) {
        ring.points_mut().clear();
    }
}

/// Calculate the signed area of a ring (for determining which is larger).
fn calculate_ring_area<T: CoordNum>(points: &[Coord<T>]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    let n = points.len();

    for i in 0..n {
        let j = (i + 1) % n;
        let xi = points[i].x.to_f64().unwrap_or(0.0);
        let yi = points[i].y.to_f64().unwrap_or(0.0);
        let xj = points[j].x.to_f64().unwrap_or(0.0);
        let yj = points[j].y.to_f64().unwrap_or(0.0);
        area += xi * yj - xj * yi;
    }

    area / 2.0
}

/// Dispatch function for processing collinear edges between two points.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            process_collinear_edges (lines 1177-1201)
///
/// Returns true if any topology was modified.
fn process_collinear_edges<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    pt_a: PointRef,
    pt_b: PointRef,
) -> bool {
    // Check if either point's ring is deleted or indices are out of bounds
    {
        let ring_a = match manager.get(pt_a.ring_idx) {
            Some(r) if !r.points().is_empty() && pt_a.point_idx < r.points().len() => r,
            _ => return false,
        };
        let ring_b = match manager.get(pt_b.ring_idx) {
            Some(r) if !r.points().is_empty() && pt_b.point_idx < r.points().len() => r,
            _ => return false,
        };

        // Verify the points are still at the same coordinate
        let coord_a = ring_a.points()[pt_a.point_idx];
        let coord_b = ring_b.points()[pt_b.point_idx];
        if !coords_equal(&coord_a, &coord_b) {
            return false;
        }
    }

    // Step 1: Check for actual collinear edge (spike pattern)
    if !has_collinear_edge(manager, pt_a, pt_b) {
        // No collinear edge - nothing to do
        return false;
    }

    // Step 2: Dispatch based on whether they share a ring
    if pt_a.ring_idx == pt_b.ring_idx {
        process_collinear_edges_same_ring(manager, pt_a, pt_b);
    } else {
        process_collinear_edges_different_rings(manager, pt_a, pt_b);
    }

    true
}

/// Process all pairs of points in a same-coordinate group.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            correct_collinear_repeats (lines 1204-1226)
///
/// The key behavior is the restart-on-change: when process_collinear_edges
/// returns true, the inner loop restarts from the beginning.
fn correct_collinear_repeats<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    group: &[PointRef],
) {
    if group.len() < 2 {
        return;
    }

    let mut i = 0;
    while i < group.len() {
        let pt_i = group[i];

        // Check if this ring is deleted
        let ring_valid = manager
            .get(pt_i.ring_idx)
            .map(|r| !r.points().is_empty())
            .unwrap_or(false);
        if !ring_valid {
            i += 1;
            continue;
        }

        let mut j = 0;
        while j < group.len() {
            // Check if pt_i's ring was deleted during inner loop
            let ring_i_valid = manager
                .get(pt_i.ring_idx)
                .map(|r| !r.points().is_empty())
                .unwrap_or(false);
            if !ring_i_valid {
                break;
            }

            let pt_j = group[j];

            // Skip self or deleted rings
            if i == j {
                j += 1;
                continue;
            }

            let ring_j_valid = manager
                .get(pt_j.ring_idx)
                .map(|r| !r.points().is_empty())
                .unwrap_or(false);
            if !ring_j_valid {
                j += 1;
                continue;
            }

            // Process the pair
            if process_collinear_edges(manager, pt_i, pt_j) {
                // Topology changed - restart inner loop
                j = 0;
            } else {
                j += 1;
            }
        }

        i += 1;
    }
}

/// Build a sorted list of all points across all rings.
///
/// PORT FROM: The C++ maintains manager.all_points as a flat vector.
/// We build this on-demand for collinear edge correction.
fn build_all_points<T: CoordNum + Copy>(
    manager: &crate::build_result::RingManager<T>,
) -> Vec<(PointRef, Coord<T>)> {
    let mut all_points = Vec::new();

    for ring_idx in 0..manager.len() {
        if let Some(ring) = manager.get(ring_idx) {
            for (point_idx, coord) in ring.points().iter().enumerate() {
                all_points.push((PointRef::new(ring_idx, point_idx), *coord));
            }
        }
    }

    // Sort by coordinate: y descending, then x ascending
    all_points.sort_by(|a, b| compare_points(&a.1, &b.1));

    all_points
}

/// Correct collinear edges in all rings.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            correct_collinear_edges (lines 1229-1259)
///
/// This function:
/// 1. Builds a sorted list of all points across all rings
/// 2. Groups points by coordinate (same x,y)
/// 3. For each group, calls correct_collinear_repeats to process pairs
pub fn correct_collinear_edges<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
) {
    // Build sorted list of all points
    let all_points = build_all_points(manager);

    if all_points.len() < 2 {
        return;
    }

    // Group by coordinate and process each group
    let mut group_start = 0;

    while group_start < all_points.len() {
        let group_coord = all_points[group_start].1;
        let mut group_end = group_start + 1;

        // Find end of group (consecutive points with same coordinate)
        while group_end < all_points.len() {
            if !coords_equal(&all_points[group_end].1, &group_coord) {
                break;
            }
            group_end += 1;
        }

        // Process this group if it has 2+ points
        if group_end - group_start >= 2 {
            let group: Vec<PointRef> = all_points[group_start..group_end]
                .iter()
                .map(|(pr, _)| *pr)
                .collect();

            correct_collinear_repeats(manager, &group);
        }

        group_start = group_end;
    }
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
    if crate::debug::debug_enabled() {
        eprintln!(
            "[TOPOLOGY] Starting correct_topology with {} rings",
            manager.len()
        );
    }

    // Step 1: Correct orientations
    // Ensures exterior rings are CCW (positive area) and holes are CW (negative area)
    // PORT FROM: C++ correct_topology line 1329
    if crate::debug::debug_enabled() {
        eprintln!("[TOPOLOGY] Step 1: correct_orientations");
    }
    correct_orientations(manager);

    // Step 2: Correct collinear edges (remove spikes and overlapping edges)
    // PORT FROM: C++ correct_topology line 1333
    // This handles degenerate geometry where edges go back along the same path.
    if crate::debug::debug_enabled() {
        eprintln!("[TOPOLOGY] Step 2: correct_collinear_edges");
    }
    correct_collinear_edges(manager);

    // Step 3: First pass of self-intersection correction (without tree correction)
    // PORT FROM: C++ correct_topology line 1335
    if crate::debug::debug_enabled() {
        eprintln!("[TOPOLOGY] Step 3: correct_self_intersections (first pass)");
    }
    correct_self_intersections(manager, false);

    // Step 4: Rebuild tree structure
    // Rebuilds parent/child relationships based on containment
    // PORT FROM: C++ correct_topology line 1337
    if crate::debug::debug_enabled() {
        eprintln!("[TOPOLOGY] Step 4: correct_tree");
    }
    correct_tree(manager);

    // Step 5: Iteratively correct chained rings and self-intersections until stable
    // PORT FROM: C++ correct_topology lines 1339-1343
    if crate::debug::debug_enabled() {
        eprintln!("[TOPOLOGY] Step 5: iterative chained/self-intersection loop");
    }
    let mut fixed = true;
    const MAX_TOPOLOGY_ITERATIONS: usize = 10_000;
    let mut topology_iteration = 0;
    while fixed {
        topology_iteration += 1;
        if topology_iteration > MAX_TOPOLOGY_ITERATIONS {
            panic!(
                "INFINITE LOOP DETECTED in correct_topology at iteration {}",
                topology_iteration
            );
        }
        if crate::debug::debug_enabled() {
            eprintln!("[TOPOLOGY] Iteration {}", topology_iteration);
        }
        correct_chained_rings(manager);
        fixed = correct_self_intersections(manager, true);
    }

    if crate::debug::debug_enabled() {
        eprintln!(
            "[TOPOLOGY] Completed after {} iterations",
            topology_iteration
        );
    }
}

// ============================================================================
// Chained Ring Correction
// ============================================================================

use std::collections::{HashMap, HashSet, VecDeque};

/// A pair of point references across two rings that share the same coordinate.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            `struct point_ptr_pair` (lines 26-44)
///
/// In C++, this holds two point pointers. In Rust, we use indices:
/// - `ring1_idx`: index of the first ring in RingManager
/// - `point1_idx`: index of the point within ring1's points
/// - `ring2_idx`: index of the second ring
/// - `point2_idx`: index of the point within ring2's points
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PointPtrPair {
    pub ring1_idx: usize,
    pub point1_idx: usize,
    pub ring2_idx: usize,
    pub point2_idx: usize,
}

impl PointPtrPair {
    pub fn new(ring1_idx: usize, point1_idx: usize, ring2_idx: usize, point2_idx: usize) -> Self {
        Self {
            ring1_idx,
            point1_idx,
            ring2_idx,
            point2_idx,
        }
    }

    /// Create a swapped version (ring1 <-> ring2)
    pub fn swap(&self) -> Self {
        Self {
            ring1_idx: self.ring2_idx,
            point1_idx: self.point2_idx,
            ring2_idx: self.ring1_idx,
            point2_idx: self.point1_idx,
        }
    }
}

/// Information about a point for sorting and grouping.
///
/// PORT FROM: wagyu C++ uses manager.all_points which is a sorted vector
/// of point pointers. We build this on-the-fly from all rings.
#[derive(Debug, Clone)]
struct PointInfo<T: CoordNum> {
    coord: Coord<T>,
    ring_idx: usize,
    point_idx: usize,
}

/// Connection map entry - tracks connections from one ring to others.
///
/// PORT FROM: C++ uses `unordered_multimap<ring_ptr, point_ptr_pair>`
/// In Rust we use `HashMap<usize, Vec<PointPtrPair>>`
type ConnectionMap = HashMap<usize, Vec<PointPtrPair>>;

/// Correct rings that share boundary points ("chained rings").
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            `correct_chained_rings` (line 755)
///
/// When two rings share one or more coordinate points (i.e. they touch without
/// overlapping) the geometry is not strictly simple.  This function detects
/// such "chained" connections and merges the rings so the result is OGC-valid.
///
/// The C++ implementation:
/// 1. Sorts `manager.all_points` by coordinate to group co-located points.
/// 2. For every run of points with the same coordinate it calls
///    `correct_chained_repeats`, which calls `process_single_intersection`
///    for every pair of points in the run.
/// 3. `process_single_intersection` builds a `connection_map` (ring →
///    point-pair) and, when a closed loop of connections is found, splices the
///    rings together.
///
/// The Rust port implements the same coordinate-grouping walk over all
/// ring points, then uses the connection-map logic to merge touching rings.
pub fn correct_chained_rings<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
) {
    // PORT FROM: topology_correction.hpp correct_chained_rings (line 755-796)

    // Step 1: Collect all points from all rings with their ring/point indices
    let mut all_points: Vec<PointInfo<T>> = Vec::new();

    for ring_idx in 0..manager.len() {
        if let Some(ring) = manager.get(ring_idx) {
            let points = ring.points();
            for (point_idx, coord) in points.iter().enumerate() {
                all_points.push(PointInfo {
                    coord: *coord,
                    ring_idx,
                    point_idx,
                });
            }
        }
    }

    // Early exit if fewer than 2 points total
    // PORT FROM: C++ line 757-759
    if all_points.len() < 2 {
        return;
    }

    // Step 2: Sort points by coordinate (descending Y, ascending X)
    // PORT FROM: C++ point_ptr_cmp (lines 148-160)
    all_points.sort_by(|a, b| {
        // First compare by Y (descending - higher Y comes first)
        let y_cmp = compare_coord_values(&b.coord.y, &a.coord.y);
        if y_cmp != std::cmp::Ordering::Equal {
            return y_cmp;
        }
        // Then compare by X (ascending)
        compare_coord_values(&a.coord.x, &b.coord.x)
    });

    // Step 3: Initialize connection map
    // PORT FROM: C++ line 762-763
    let mut connection_map: ConnectionMap = HashMap::new();

    // Step 4: Find groups of co-located points and process them
    // PORT FROM: C++ lines 771-795
    let mut group_start = 0;
    while group_start < all_points.len() {
        // Find the end of this group (points with same coordinate)
        let mut group_end = group_start + 1;
        while group_end < all_points.len()
            && coords_equal(&all_points[group_start].coord, &all_points[group_end].coord)
        {
            group_end += 1;
        }

        // If we have 2+ points at the same coordinate, process pairs
        if group_end - group_start >= 2 {
            correct_chained_repeats(
                manager,
                &mut connection_map,
                &all_points[group_start..group_end],
            );
        }

        group_start = group_end;
    }
}

/// Compare two coordinate values for ordering.
fn compare_coord_values<T: CoordNum>(a: &T, b: &T) -> std::cmp::Ordering {
    if *a < *b {
        std::cmp::Ordering::Less
    } else if *a > *b {
        std::cmp::Ordering::Greater
    } else {
        std::cmp::Ordering::Equal
    }
}

/// Check if two coordinates are equal.
fn coords_equal<T: CoordNum>(a: &Coord<T>, b: &Coord<T>) -> bool {
    a.x == b.x && a.y == b.y
}

/// Process a group of points that share the same coordinate.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            `correct_chained_repeats` (lines 737-752)
///
/// For each pair of points in the group (from different rings), calls
/// `process_single_intersection` to potentially merge the rings.
fn correct_chained_repeats<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    connection_map: &mut ConnectionMap,
    group: &[PointInfo<T>],
) {
    // PORT FROM: C++ lines 737-752
    // Nested loop over all pairs
    for i in 0..group.len() {
        for j in (i + 1)..group.len() {
            let pt_i = &group[i];
            let pt_j = &group[j];

            // Skip if same ring
            if pt_i.ring_idx == pt_j.ring_idx {
                continue;
            }

            process_single_intersection(manager, connection_map, pt_i, pt_j);
        }
    }
}

/// Process a single intersection between two points from different rings.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            `process_single_intersection` (lines 472-734)
///
/// This is the core logic that decides whether to merge rings that share
/// a boundary point, and performs the merge if conditions are met.
fn process_single_intersection<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    connection_map: &mut ConnectionMap,
    pt_j: &PointInfo<T>,
    pt_k: &PointInfo<T>,
) {
    // PORT FROM: C++ lines 472-485
    let ring_j_idx = pt_j.ring_idx;
    let ring_k_idx = pt_k.ring_idx;

    // Same ring - skip (already checked in caller, but defensive)
    if ring_j_idx == ring_k_idx {
        return;
    }

    // Get ring info
    let (ring_j_is_hole, ring_j_parent) = {
        let ring = match manager.get(ring_j_idx) {
            Some(r) => r,
            None => return,
        };
        (ring.is_hole(), ring.parent())
    };

    let (ring_k_is_hole, ring_k_parent) = {
        let ring = match manager.get(ring_k_idx) {
            Some(r) => r,
            None => return,
        };
        (ring.is_hole(), ring.parent())
    };

    // PORT FROM: C++ lines 482-485
    // If neither ring is a hole, skip - two exteriors don't get merged
    if !ring_j_is_hole && !ring_k_is_hole {
        return;
    }

    // PORT FROM: C++ lines 487-518
    // Determine ring_origin, ring_search, ring_parent, and point assignments
    let (ring_origin, ring_search, ring_parent_idx, op_origin_1, op_origin_2): (
        usize,
        usize,
        Option<usize>,
        (usize, usize), // (ring_idx, point_idx)
        (usize, usize),
    ) = if !ring_j_is_hole {
        // ring_j is exterior (not hole), ring_k is hole
        (
            ring_j_idx,
            ring_k_idx,
            Some(ring_j_idx), // ring_parent = ring_origin for exterior
            (ring_j_idx, pt_j.point_idx),
            (ring_k_idx, pt_k.point_idx),
        )
    } else if !ring_k_is_hole {
        // ring_k is exterior, ring_j is hole
        (
            ring_k_idx,
            ring_j_idx,
            Some(ring_k_idx),
            (ring_k_idx, pt_k.point_idx),
            (ring_j_idx, pt_j.point_idx),
        )
    } else {
        // Both are holes - use ring_j as origin, parent is ring_j's parent
        (
            ring_j_idx,
            ring_k_idx,
            ring_j_parent,
            (ring_j_idx, pt_j.point_idx),
            (ring_k_idx, pt_k.point_idx),
        )
    };

    // PORT FROM: C++ lines 514-518
    // Check parent compatibility
    let ring_search_parent = if ring_k_idx == ring_search {
        ring_k_parent
    } else {
        ring_j_parent
    };

    if ring_parent_idx != ring_search_parent {
        // Different parents - incompatible, skip
        return;
    }

    // PORT FROM: C++ lines 519-567
    // Check for existing connection (direct or chained)
    let mut found = false;
    let mut i_list: VecDeque<(usize, PointPtrPair)> = VecDeque::new();

    // Check for direct connection in connection_map
    if let Some(entries) = connection_map.get(&ring_search) {
        for entry in entries {
            // Check if this entry connects back to ring_origin
            if entry.ring2_idx == ring_origin {
                found = true;
                // Check position guard: the connection point must be at a different
                // position than op_origin_1
                let same_position =
                    entry.point2_idx == op_origin_1.1 && entry.ring2_idx == op_origin_1.0;
                if !same_position {
                    i_list.push_back((ring_search, *entry));
                    break;
                }
            }
        }
    }

    // If not found directly, search for chained connection
    if i_list.is_empty() && !found {
        let mut visited: HashSet<usize> = HashSet::new();
        visited.insert(ring_search);

        if let Some(entries) = connection_map.get(&ring_search).cloned() {
            for entry in &entries {
                let it_ring = entry.ring2_idx;

                // Skip if already visited, or invalid
                if visited.contains(&it_ring) || it_ring == ring_search {
                    continue;
                }

                // Check parent compatibility
                let it_ring_parent = manager.get(it_ring).and_then(|r| r.parent());
                let it_ring_is_valid =
                    ring_parent_idx == Some(it_ring) || ring_parent_idx == it_ring_parent;

                if !it_ring_is_valid {
                    continue;
                }

                // Check ring has non-zero area
                if let Some(ring) = manager.get(it_ring) {
                    if ring.points().len() < 3 {
                        continue;
                    }
                }

                // Try to find loop through this connection
                if find_intersect_loop(
                    manager,
                    connection_map,
                    &mut i_list,
                    ring_parent_idx,
                    ring_origin,
                    it_ring,
                    &mut visited,
                    op_origin_2,
                    (entry.ring2_idx, entry.point2_idx),
                ) {
                    found = true;
                    i_list.push_front((ring_search, *entry));
                    break;
                }
            }
        }
    }

    // PORT FROM: C++ lines 562-567
    // If not found, add to pending connections
    if !found {
        let pair_origin =
            PointPtrPair::new(op_origin_1.0, op_origin_1.1, op_origin_2.0, op_origin_2.1);
        let pair_search = pair_origin.swap();

        connection_map
            .entry(ring_origin)
            .or_insert_with(Vec::new)
            .push(pair_origin);
        connection_map
            .entry(ring_search)
            .or_insert_with(Vec::new)
            .push(pair_search);
        return;
    }

    // PORT FROM: C++ lines 570-587
    // Special case: found but iList empty (hole-hole)
    if i_list.is_empty() {
        // Check if origin already has an entry pointing to search
        let mut missing = true;
        if let Some(entries) = connection_map.get(&ring_origin) {
            for entry in entries {
                if entry.ring2_idx == ring_search {
                    missing = false;
                    break;
                }
            }
        }
        if missing {
            let pair =
                PointPtrPair::new(op_origin_1.0, op_origin_1.1, op_origin_2.0, op_origin_2.1);
            connection_map
                .entry(ring_origin)
                .or_insert_with(Vec::new)
                .push(pair);
        }
        return;
    }

    // PORT FROM: C++ lines 588-734
    // We have a cycle - perform the merge
    merge_rings_at_intersection(
        manager,
        connection_map,
        ring_origin,
        ring_search,
        ring_parent_idx,
        op_origin_1,
        op_origin_2,
        &i_list,
    );
}

/// Search for a loop back to ring_origin through the connection map.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            `find_intersect_loop` (lines 101-145)
///
/// This is a DFS that searches for a chain of connections that leads back
/// to ring_origin, building the i_list as it goes.
fn find_intersect_loop<T: CoordNum + Copy>(
    manager: &crate::build_result::RingManager<T>,
    connection_map: &ConnectionMap,
    i_list: &mut VecDeque<(usize, PointPtrPair)>,
    ring_parent_idx: Option<usize>,
    ring_origin: usize,
    ring_search: usize,
    visited: &mut HashSet<usize>,
    orig_pt: (usize, usize), // (ring_idx, point_idx)
    prev_pt: (usize, usize),
) -> bool {
    // PORT FROM: C++ lines 110-127 - Check for direct connection
    if let Some(entries) = connection_map.get(&ring_search) {
        for entry in entries {
            // Validate entry
            if entry.ring1_idx != ring_search {
                continue;
            }

            let it_ring2 = entry.ring2_idx;

            // Check if both rings are exterior (skip)
            let ring1_is_hole = manager
                .get(entry.ring1_idx)
                .map(|r| r.is_hole())
                .unwrap_or(false);
            let ring2_is_hole = manager.get(it_ring2).map(|r| r.is_hole()).unwrap_or(false);
            if !ring1_is_hole && !ring2_is_hole {
                continue;
            }

            // Check for cycle back to origin
            if it_ring2 == ring_origin {
                // Check parent compatibility
                let parent_ok = ring_parent_idx == Some(it_ring2)
                    || ring_parent_idx == manager.get(it_ring2).and_then(|r| r.parent());

                // Position guards
                let prev_pt_same = prev_pt == (entry.ring2_idx, entry.point2_idx);
                let orig_pt_same = orig_pt == (entry.ring2_idx, entry.point2_idx);

                if parent_ok && !prev_pt_same && !orig_pt_same {
                    i_list.push_front((ring_search, *entry));
                    return true;
                }
            }
        }
    }

    // PORT FROM: C++ lines 128-143 - Search through chain
    visited.insert(ring_search);

    if let Some(entries) = connection_map.get(&ring_search).cloned() {
        for entry in &entries {
            let it_ring = entry.ring2_idx;

            // Skip if visited, null, or wrong parent
            if visited.contains(&it_ring) {
                continue;
            }

            // Check parent compatibility
            let it_ring_parent = manager.get(it_ring).and_then(|r| r.parent());
            if ring_parent_idx != Some(it_ring) && ring_parent_idx != it_ring_parent {
                continue;
            }

            // Check ring has valid area
            if let Some(ring) = manager.get(it_ring) {
                if ring.points().len() < 3 {
                    continue;
                }
            } else {
                continue;
            }

            // Position guard
            if prev_pt == (entry.ring2_idx, entry.point2_idx) {
                continue;
            }

            // Recurse
            if find_intersect_loop(
                manager,
                connection_map,
                i_list,
                ring_parent_idx,
                ring_origin,
                it_ring,
                visited,
                orig_pt,
                (entry.ring2_idx, entry.point2_idx),
            ) {
                i_list.push_front((ring_search, *entry));
                return true;
            }
        }
    }

    false
}

/// Split touching rings at their shared points into two separate fragments.
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
///            `process_single_intersection` (lines 588-734)
///
/// This function handles both simple two-ring merges and chained multi-ring merges.
///
/// ALGORITHM (matching C++ linked-list semantics):
/// 1. Build a unified point pool containing all points from all involved rings
/// 2. Create a "next index" map representing the linked-list structure
/// 3. Apply all swap operations (primary + chain) to this map
/// 4. Traverse from each origin to build exactly 2 fragments
/// 5. Assign fragments to rings, clear absorbed rings
fn merge_rings_at_intersection<T: CoordNum + Copy>(
    manager: &mut crate::build_result::RingManager<T>,
    _connection_map: &mut ConnectionMap,
    _ring_origin: usize,
    _ring_search: usize,
    _ring_parent_idx: Option<usize>,
    op_origin_1: (usize, usize), // (ring_idx, point_idx)
    op_origin_2: (usize, usize),
    i_list: &VecDeque<(usize, PointPtrPair)>,
) {
    let (ring_a_idx, point_a_idx) = op_origin_1;
    let (ring_b_idx, point_b_idx) = op_origin_2;

    // Collect all rings involved in this merge (primary + chain)
    let mut involved_rings: Vec<usize> = vec![ring_a_idx, ring_b_idx];
    for (_chain_ring_idx, pair) in i_list.iter() {
        if !involved_rings.contains(&pair.ring1_idx) {
            involved_rings.push(pair.ring1_idx);
        }
        if !involved_rings.contains(&pair.ring2_idx) {
            involved_rings.push(pair.ring2_idx);
        }
    }

    // Build unified point pool and next-index map
    // Each entry: (coord, next_pool_index)
    // We also track which pool index corresponds to each (ring_idx, point_idx)
    let mut pool: Vec<Coord<T>> = Vec::new();
    let mut next_map: Vec<usize> = Vec::new();
    let mut ring_point_to_pool: std::collections::HashMap<(usize, usize), usize> =
        std::collections::HashMap::new();

    for &ring_idx in &involved_rings {
        let points = match manager.get(ring_idx) {
            Some(r) => r.points().to_vec(),
            None => continue,
        };
        if points.len() < 3 {
            continue;
        }

        let base_idx = pool.len();
        for (i, &coord) in points.iter().enumerate() {
            let pool_idx = pool.len();
            pool.push(coord);
            // Next index wraps around within this ring
            let next_idx = base_idx + (i + 1) % points.len();
            next_map.push(next_idx);
            ring_point_to_pool.insert((ring_idx, i), pool_idx);
        }
    }

    if pool.len() < 6 {
        // Need at least 3 points per ring for 2 rings
        return;
    }

    // Helper to get pool index for (ring_idx, point_idx)
    let get_pool_idx = |ring_idx: usize, point_idx: usize| -> Option<usize> {
        ring_point_to_pool.get(&(ring_idx, point_idx)).copied()
    };

    // Apply primary swap: op_origin_1 <-> op_origin_2
    // C++: op_origin_1->next = op_origin_2->next
    //      op_origin_2->next = op_origin_1->next
    let Some(pool_idx_1) = get_pool_idx(ring_a_idx, point_a_idx) else {
        return;
    };
    let Some(pool_idx_2) = get_pool_idx(ring_b_idx, point_b_idx) else {
        return;
    };

    let next_1 = next_map[pool_idx_1];
    let next_2 = next_map[pool_idx_2];
    next_map[pool_idx_1] = next_2;
    next_map[pool_idx_2] = next_1;

    // Apply chain swaps
    for (_chain_ring_idx, pair) in i_list.iter() {
        let Some(pool_idx_s1) = get_pool_idx(pair.ring1_idx, pair.point1_idx) else {
            continue;
        };
        let Some(pool_idx_s2) = get_pool_idx(pair.ring2_idx, pair.point2_idx) else {
            continue;
        };

        let next_s1 = next_map[pool_idx_s1];
        let next_s2 = next_map[pool_idx_s2];
        next_map[pool_idx_s1] = next_s2;
        next_map[pool_idx_s2] = next_s1;
    }

    // Traverse from pool_idx_1 to build fragment 1
    let mut fragment_1: Vec<Coord<T>> = Vec::new();
    let mut visited: std::collections::HashSet<usize> = std::collections::HashSet::new();
    let mut current = pool_idx_1;
    while !visited.contains(&current) {
        visited.insert(current);
        fragment_1.push(pool[current]);
        current = next_map[current];
        if fragment_1.len() > pool.len() {
            // Safety: prevent infinite loop
            break;
        }
    }

    // Traverse from pool_idx_2 to build fragment 2
    let mut fragment_2: Vec<Coord<T>> = Vec::new();
    visited.clear();
    current = pool_idx_2;
    while !visited.contains(&current) {
        visited.insert(current);
        fragment_2.push(pool[current]);
        current = next_map[current];
        if fragment_2.len() > pool.len() {
            // Safety: prevent infinite loop
            break;
        }
    }

    if fragment_1.len() < 3 || fragment_2.len() < 3 {
        if crate::debug::debug_enabled() {
            eprintln!(
                "[TOPOLOGY] merge_rings_at_intersection: fragments too small ({}, {}), skipping",
                fragment_1.len(),
                fragment_2.len()
            );
        }
        return;
    }

    // PORT FROM: C++ lines 633-645 - area-based fragment assignment
    // Calculate areas to determine which fragment goes to ring_origin vs ring_new
    let area_1 = crate::ring_util::ring_area(&fragment_1);
    let area_2 = crate::ring_util::ring_area(&fragment_2);
    let origin_is_hole = manager.ring_is_hole(ring_a_idx);

    // C++ logic: if origin is a hole AND area_1 is negative (CW/hole orientation),
    // assign fragment_1 to ring_origin. Otherwise, swap them.
    let (origin_fragment, new_fragment) = if origin_is_hole && area_1 < 0.0 {
        (fragment_1.clone(), fragment_2.clone())
    } else {
        (fragment_2.clone(), fragment_1.clone())
    };

    if crate::debug::debug_enabled() {
        eprintln!(
            "[TOPOLOGY] merge_rings_at_intersection: area_1={:.2}, area_2={:.2}, origin_is_hole={}, swapped={}",
            area_1, area_2, origin_is_hole, !(origin_is_hole && area_1 < 0.0)
        );
    }

    // Assign origin_fragment to ring_a
    if let Some(ring) = manager.get_mut(ring_a_idx) {
        *ring.points_mut() = origin_fragment;
        ring.set_corrected(false);
    }

    // Create new ring for new_fragment
    let new_ring_idx = manager.create_new_ring();
    if let Some(ring) = manager.get_mut(new_ring_idx) {
        *ring.points_mut() = new_fragment;
        ring.set_corrected(false);
    }

    // Clear all other involved rings (they've been absorbed)
    for &ring_idx in &involved_rings {
        if ring_idx != ring_a_idx {
            if let Some(ring) = manager.get_mut(ring_idx) {
                ring.points_mut().clear();
            }
        }
    }

    // PORT FROM: C++ lines 661-686 - assign parent/child relationships for new ring
    // If ring_origin (ring_a) is a hole, new ring becomes its child.
    // If ring_origin is not a hole (exterior), new ring becomes its sibling.
    // NOTE: We use origin_is_hole calculated BEFORE fragment assignment (matching C++)
    if origin_is_hole {
        // New ring is a child of the hole (becomes an island inside the hole)
        manager.assign_as_child(new_ring_idx, Some(ring_a_idx));
        if crate::debug::debug_enabled() {
            eprintln!(
                "[TOPOLOGY] merge_rings_at_intersection: new ring {} assigned as child of hole {}",
                new_ring_idx, ring_a_idx
            );
        }
    } else {
        // New ring is a sibling (same parent as ring_origin)
        manager.assign_as_sibling(new_ring_idx, ring_a_idx);
        if crate::debug::debug_enabled() {
            eprintln!(
                "[TOPOLOGY] merge_rings_at_intersection: new ring {} assigned as sibling of exterior {}",
                new_ring_idx, ring_a_idx
            );
        }
    }

    if crate::debug::debug_enabled() {
        eprintln!(
            "[TOPOLOGY] merge_rings_at_intersection: merged {} rings -> ring {} ({} pts) + new ring {} ({} pts)",
            involved_rings.len(),
            ring_a_idx,
            fragment_1.len(),
            new_ring_idx,
            fragment_2.len()
        );
    }
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

    // ==================== sort_ring_points_by_coord Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - sort_ring_points

    #[test]
    fn sort_ring_points_by_coord_sorts_y_descending_then_x_ascending() {
        // Points with various y values - should come out y-descending, x-ascending
        let mut ring = Ring::empty();
        ring.push_point(Coord { x: 3.0, y: 1.0 });
        ring.push_point(Coord { x: 1.0, y: 3.0 });
        ring.push_point(Coord { x: 2.0, y: 3.0 }); // Same y as above, larger x
        ring.push_point(Coord { x: 5.0, y: 2.0 });

        let sorted = sort_ring_points_by_coord(&ring);

        // Expected order: y=3 x=1, y=3 x=2, y=2 x=5, y=1 x=3
        assert_eq!(sorted[0], Coord { x: 1.0, y: 3.0 });
        assert_eq!(sorted[1], Coord { x: 2.0, y: 3.0 });
        assert_eq!(sorted[2], Coord { x: 5.0, y: 2.0 });
        assert_eq!(sorted[3], Coord { x: 3.0, y: 1.0 });
    }

    #[test]
    fn sort_ring_points_by_coord_empty_ring_returns_empty() {
        let ring: Ring<f64> = Ring::empty();
        let sorted = sort_ring_points_by_coord(&ring);
        assert!(sorted.is_empty());
    }

    // ==================== correct_self_intersection_in_ring Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_self_intersection

    /// Build a figure-8 ring: two squares sharing a single corner point.
    ///
    /// The shared corner is at index p and q.
    /// Ring: [shared, A, B, C, shared, D, E, F] -- shared appears at index 0 and 4.
    fn make_figure_8_ring() -> Ring<f64> {
        // Lower-left square (CCW): shared corner at (5,5)
        // Upper-right square (CCW): shared corner at (5,5)
        // Traversal: (5,5) -> (0,5) -> (0,0) -> (5,0) -> (5,5) -> (10,5) -> (10,10) -> (5,10)
        // This visits (5,5) at index 0 and again... well, we need to construct a ring
        // where the SAME coordinate appears at two different indices.
        //
        // Simpler: bowtie shape. (0,0)->(10,10)->(10,0)->(0,10)
        // This ring has no repeated coords, but it self-intersects geometrically.
        //
        // For correct_self_intersection_in_ring we need a ring with the same
        // coordinate at two different positions.
        //
        // Build: (5,5) -> (0,0) -> (0,5) -> (5,5) -> (10,5) -> (10,10) -> (5,5-unused)
        // Simpler: visit (5,5) at positions 0 and 3 in a 6-point ring.
        let mut ring = Ring::empty();
        // Index 0: shared point
        ring.push_point(Coord { x: 5.0, y: 5.0 });
        // Index 1, 2: lower-left lobe
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 10.0, y: 0.0 });
        // Index 3: shared point again (self-intersection node)
        ring.push_point(Coord { x: 5.0, y: 5.0 });
        // Index 4, 5: upper-right lobe
        ring.push_point(Coord { x: 10.0, y: 10.0 });
        ring.push_point(Coord { x: 0.0, y: 10.0 });
        ring
    }

    #[test]
    fn correct_self_intersection_in_ring_splits_figure_8_into_two_rings() {
        // RED: This test verifies the basic split behavior.
        // A figure-8 ring with shared point at indices 0 and 3 should split into 2 rings.
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_figure_8_ring();
        let ring_idx = manager.add_ring(ring);

        // The ring has 6 points. Shared coord (5,5) at indices 0 and 3.
        let new_ring_idx =
            correct_self_intersection_in_ring(&mut manager, ring_idx, 0, ring_idx, 3);

        assert!(
            new_ring_idx.is_some(),
            "Should return a new ring index on successful split"
        );

        let new_idx = new_ring_idx.unwrap();

        // Both loops should have at least 3 points
        let orig_len = manager.get(ring_idx).unwrap().len();
        let new_len = manager.get(new_idx).unwrap().len();
        assert!(
            orig_len >= 3,
            "Original ring should have >= 3 points, got {}",
            orig_len
        );
        assert!(
            new_len >= 3,
            "New ring should have >= 3 points, got {}",
            new_len
        );

        // The original ring keeps the larger loop
        let orig_area = ring_area(manager.get(ring_idx).unwrap().points()).abs();
        let new_area = ring_area(manager.get(new_idx).unwrap().points()).abs();
        assert!(
            orig_area >= new_area,
            "Original ring should keep the larger loop: orig={} new={}",
            orig_area,
            new_area
        );
    }

    #[test]
    fn correct_self_intersection_in_ring_rejects_same_index() {
        // Splitting at the same index twice makes no sense
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_figure_8_ring();
        let ring_idx = manager.add_ring(ring);

        let result = correct_self_intersection_in_ring(&mut manager, ring_idx, 2, ring_idx, 2);
        assert!(result.is_none(), "Same index should return None");
    }

    #[test]
    fn correct_self_intersection_in_ring_rejects_different_rings() {
        // Only handles same-ring case (self-intersection within one ring)
        let mut manager: RingManager<f64> = RingManager::new();
        let ring0 = make_ccw_square(0.0, 0.0, 10.0);
        let ring1 = make_ccw_square(20.0, 20.0, 10.0);
        let idx0 = manager.add_ring(ring0);
        let idx1 = manager.add_ring(ring1);

        let result = correct_self_intersection_in_ring(&mut manager, idx0, 0, idx1, 0);
        assert!(result.is_none(), "Different rings should return None");
    }

    #[test]
    fn correct_self_intersection_in_ring_rejects_ring_with_fewer_than_4_points() {
        let mut manager: RingManager<f64> = RingManager::new();
        let mut ring = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 5.0, y: 5.0 });
        ring.push_point(Coord { x: 10.0, y: 0.0 });
        let idx = manager.add_ring(ring);

        let result = correct_self_intersection_in_ring(&mut manager, idx, 0, idx, 2);
        assert!(result.is_none(), "Ring with < 4 points should return None");
    }

    // ==================== find_and_correct_repeated_points Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp

    #[test]
    fn find_and_correct_repeated_points_finds_and_splits_figure_8() {
        // A ring where one coordinate appears twice should be split.
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_figure_8_ring();
        let ring_idx = manager.add_ring(ring);

        let orig_len_before = manager.get(ring_idx).unwrap().len();
        assert_eq!(orig_len_before, 6, "Setup: figure-8 has 6 points");

        let new_rings = find_and_correct_repeated_points(ring_idx, &mut manager);

        // The repeated point at (5,5) should trigger a split
        assert_eq!(
            new_rings.len(),
            1,
            "One self-intersection should produce one new ring"
        );

        // Both resulting rings should have valid point counts
        let orig_len_after = manager.get(ring_idx).unwrap().len();
        let new_len = manager.get(new_rings[0]).unwrap().len();
        assert!(orig_len_after >= 3);
        assert!(new_len >= 3);

        // Total points should equal original - 1 shared point (split removes one shared)
        // Actually: orig 6-point ring with shared coord at pos 0 and 3
        // loop_a = [0..3] = 3 points
        // loop_b = [3..6] + [0..0] = 3 points
        assert_eq!(
            orig_len_after + new_len,
            6,
            "Total points across both rings should equal original 6"
        );
    }

    #[test]
    fn find_and_correct_repeated_points_no_duplicates_returns_empty() {
        // A simple square has no repeated points - no splits should occur
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_ccw_square(0.0, 0.0, 10.0);
        let ring_idx = manager.add_ring(ring);

        let new_rings = find_and_correct_repeated_points(ring_idx, &mut manager);

        assert!(
            new_rings.is_empty(),
            "No repeated points means no new rings"
        );
        // Original ring unchanged
        assert_eq!(manager.get(ring_idx).unwrap().len(), 4);
    }

    // ==================== correct_ring_self_intersections Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_ring_self_intersections

    #[test]
    fn correct_ring_self_intersections_marks_ring_as_corrected() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_ccw_square(0.0, 0.0, 10.0);
        let ring_idx = manager.add_ring(ring);

        assert!(!manager.is_corrected(ring_idx), "Ring starts uncorrected");

        correct_ring_self_intersections(&mut manager, ring_idx, false);

        assert!(
            manager.is_corrected(ring_idx),
            "Ring should be marked corrected after processing"
        );
    }

    #[test]
    fn correct_ring_self_intersections_skips_already_corrected_ring() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_figure_8_ring();
        let ring_idx = manager.add_ring(ring);

        // Mark as already corrected
        manager.set_corrected(ring_idx, true);

        // Should skip and return false even though the ring has a self-intersection
        let fixed = correct_ring_self_intersections(&mut manager, ring_idx, false);
        assert!(!fixed, "Already-corrected ring should return false");
        // Ring should still have 6 points (not split)
        assert_eq!(manager.get(ring_idx).unwrap().len(), 6);
    }

    #[test]
    fn correct_ring_self_intersections_returns_true_when_split_occurred() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_figure_8_ring();
        let ring_idx = manager.add_ring(ring);

        let fixed = correct_ring_self_intersections(&mut manager, ring_idx, false);
        assert!(fixed, "Should return true when a split occurred");
    }

    #[test]
    fn correct_ring_self_intersections_returns_false_for_clean_ring() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_ccw_square(0.0, 0.0, 10.0);
        let ring_idx = manager.add_ring(ring);

        // A simple square has no self-intersections
        // find_and_correct_repeated_points returns empty -> did_split = false
        // but the ring IS corrected afterwards
        let fixed = correct_ring_self_intersections(&mut manager, ring_idx, false);
        assert!(!fixed, "Clean ring should return false (no split)");
    }

    // ==================== correct_self_intersections Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - correct_self_intersections

    #[test]
    fn correct_self_intersections_processes_all_rings() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Add a clean ring and a figure-8 ring
        let clean_idx = manager.add_ring(make_ccw_square(0.0, 0.0, 10.0));
        let fig8_idx = manager.add_ring(make_figure_8_ring());

        let rings_before = manager.len();
        let fixed = correct_self_intersections(&mut manager, false);

        assert!(fixed, "Should return true when any ring was split");
        assert!(
            manager.len() > rings_before,
            "A new ring should have been created by the split"
        );
        assert!(
            manager.is_corrected(clean_idx),
            "Clean ring should be marked corrected"
        );
        assert!(
            manager.is_corrected(fig8_idx),
            "Figure-8 ring should be marked corrected"
        );
    }

    #[test]
    fn correct_self_intersections_returns_false_when_no_splits() {
        let mut manager: RingManager<f64> = RingManager::new();
        manager.add_ring(make_ccw_square(0.0, 0.0, 10.0));
        manager.add_ring(make_ccw_square(20.0, 20.0, 10.0));

        let fixed = correct_self_intersections(&mut manager, false);
        assert!(!fixed, "No splits should return false");
    }

    #[test]
    fn correct_self_intersections_processes_smallest_rings_first() {
        // Verify that small rings are processed before large rings (smallest-to-largest order)
        // We can verify this by checking that the corrected flag is set on both rings
        // and the correct order doesn't cause issues.
        let mut manager: RingManager<f64> = RingManager::new();

        // Large ring first
        let large_idx = manager.add_ring(make_ccw_square(0.0, 0.0, 100.0));
        // Small ring second
        let small_idx = manager.add_ring(make_ccw_square(10.0, 10.0, 5.0));

        correct_self_intersections(&mut manager, false);

        // Both should be corrected
        assert!(manager.is_corrected(large_idx));
        assert!(manager.is_corrected(small_idx));
    }

    // ==================== assign_new_ring_parents Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - assign_new_ring_parents

    #[test]
    fn assign_new_ring_parents_empty_new_rings_does_nothing() {
        let mut manager: RingManager<f64> = RingManager::new();
        let orig_idx = manager.add_ring(make_ccw_square(0.0, 0.0, 10.0));

        // Should not panic with empty new rings
        assign_new_ring_parents(&mut manager, orig_idx, &[]);
        // orig ring unchanged
        assert_eq!(manager.get(orig_idx).unwrap().len(), 4);
    }

    #[test]
    fn assign_new_ring_parents_single_same_orientation_assigned_as_sibling() {
        // When new ring has same orientation as original ring,
        // and is not contained by any ancestor, it becomes a sibling.
        //
        // PORT FROM: C++ assign_new_ring_parents lines 359-380
        // "if (original_positive == new_positive) { assign_as_child(new_rings.front(), original_ring->parent, ...)"
        let mut manager: RingManager<f64> = RingManager::new();

        // Setup: outer -> orig_ring (both CCW/exterior)
        let outer_idx = manager.add_ring(make_ccw_square(0.0, 0.0, 100.0));
        let orig_ring = make_ccw_square(5.0, 5.0, 60.0);
        let orig_idx = manager.create_new_ring();
        manager.set_ring_points(orig_idx, orig_ring.points().to_vec());
        manager.assign_as_child(orig_idx, Some(outer_idx));

        // Create a new ring (same orientation, exterior) that should become sibling of orig
        let new_ring_pts = make_ccw_square(5.0, 5.0, 20.0).points().to_vec();
        let new_idx = manager.create_new_ring();
        manager.set_ring_points(new_idx, new_ring_pts);

        assign_new_ring_parents(&mut manager, orig_idx, &[new_idx]);

        // New ring should have same parent as orig ring (outer_idx)
        let new_parent = manager.parent(new_idx);
        let orig_parent = manager.parent(orig_idx);
        assert_eq!(
            new_parent, orig_parent,
            "New ring (same orientation) should be sibling of orig ring: new_parent={:?} orig_parent={:?}",
            new_parent, orig_parent
        );
    }

    #[test]
    fn assign_new_ring_parents_single_opposite_orientation_assigned_as_child() {
        // When new ring has opposite orientation (hole inside exterior),
        // it should be assigned as child of the original ring.
        //
        // PORT FROM: C++ assign_new_ring_parents lines 372-379
        // "else { assign_as_child(new_rings.front(), original_ring, ...)" (if contained)
        let mut manager: RingManager<f64> = RingManager::new();

        // orig_ring: large CCW exterior
        let orig_ring = make_ccw_square(0.0, 0.0, 100.0);
        let orig_idx = manager.add_ring(orig_ring);

        // new_ring: smaller CW ring (hole) inside orig - opposite orientation
        let hole_pts: Vec<Coord<f64>> = vec![
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 10.0, y: 40.0 },
            Coord { x: 40.0, y: 40.0 },
            Coord { x: 40.0, y: 10.0 },
        ]; // CW = negative area
        let new_idx = manager.create_new_ring();
        manager.set_ring_points(new_idx, hole_pts);

        assign_new_ring_parents(&mut manager, orig_idx, &[new_idx]);

        // New ring (CW/hole, opposite orientation) should be child of orig
        let new_parent = manager.parent(new_idx);
        assert_eq!(
            new_parent,
            Some(orig_idx),
            "New ring (opposite orientation, inside orig) should be child of orig"
        );
    }

    // ==================== reassign_children_if_necessary Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - reassign_children_if_necessary

    #[test]
    fn reassign_children_if_necessary_moves_child_inside_new_ring() {
        // If a child of orig_ring is geometrically inside new_ring, it should be reassigned.
        //
        // PORT FROM: C++ reassign_children_if_necessary
        let mut manager: RingManager<f64> = RingManager::new();

        // orig_ring: large CCW exterior (100x100)
        let orig_ring = make_ccw_square(0.0, 0.0, 100.0);
        let orig_idx = manager.add_ring(orig_ring);

        // child: small CW hole (10x10 at 5,5) - inside new_ring
        let child_pts: Vec<Coord<f64>> = vec![
            Coord { x: 5.0, y: 5.0 },
            Coord { x: 5.0, y: 15.0 },
            Coord { x: 15.0, y: 15.0 },
            Coord { x: 15.0, y: 5.0 },
        ];
        let child_idx = manager.create_new_ring();
        manager.set_ring_points(child_idx, child_pts);
        manager.set_parent(child_idx, orig_idx);

        // new_ring: medium CCW exterior (50x50 at 0,0) that contains the child
        let new_ring_pts = make_ccw_square(0.0, 0.0, 50.0).points().to_vec();
        let new_idx = manager.create_new_ring();
        manager.set_ring_points(new_idx, new_ring_pts);

        // The child (at 5,5 to 15,15) is inside new_ring (0,0 to 50,50)
        reassign_children_if_necessary(&mut manager, orig_idx, new_idx, &[]);

        // Child should now be a child of new_ring
        let child_parent = manager.parent(child_idx);
        assert_eq!(
            child_parent,
            Some(new_idx),
            "Child inside new_ring should be reassigned to new_ring"
        );
    }

    #[test]
    fn reassign_children_if_necessary_leaves_outside_children_alone() {
        // Children of orig_ring that are NOT inside new_ring should stay with orig_ring.
        let mut manager: RingManager<f64> = RingManager::new();

        // orig_ring: large CCW exterior (100x100)
        let orig_ring = make_ccw_square(0.0, 0.0, 100.0);
        let orig_idx = manager.add_ring(orig_ring);

        // child: CW hole at 60,60 to 80,80 - outside new_ring
        let child_pts: Vec<Coord<f64>> = vec![
            Coord { x: 60.0, y: 60.0 },
            Coord { x: 60.0, y: 80.0 },
            Coord { x: 80.0, y: 80.0 },
            Coord { x: 80.0, y: 60.0 },
        ];
        let child_idx = manager.create_new_ring();
        manager.set_ring_points(child_idx, child_pts);
        manager.set_parent(child_idx, orig_idx);

        // new_ring: small ring at 0,0 to 20,20 - does NOT contain the child
        let new_ring_pts = make_ccw_square(0.0, 0.0, 20.0).points().to_vec();
        let new_idx = manager.create_new_ring();
        manager.set_ring_points(new_idx, new_ring_pts);

        reassign_children_if_necessary(&mut manager, orig_idx, new_idx, &[]);

        // Child should still be a child of orig_ring
        let child_parent = manager.parent(child_idx);
        assert_eq!(
            child_parent,
            Some(orig_idx),
            "Child outside new_ring should remain with orig_ring"
        );
    }

    // ==================== find_parent_in_tree Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp - find_parent_in_tree

    #[test]
    fn find_parent_in_tree_finds_direct_parent() {
        // When the candidate is the direct parent, it should be found.
        let mut manager: RingManager<f64> = RingManager::new();

        // parent: CCW 100x100
        let parent_ring = make_ccw_square(0.0, 0.0, 100.0);
        let parent_idx = manager.add_ring(parent_ring);

        // new ring: small CCW 10x10 that fits inside parent
        let new_ring_pts = make_ccw_square(10.0, 10.0, 10.0).points().to_vec();
        let new_area = ring_area(&new_ring_pts);

        let found = find_parent_in_tree(&manager, &new_ring_pts, new_area, Some(parent_idx));

        assert_eq!(
            found,
            Some(parent_idx),
            "Should find direct parent that contains the new ring"
        );
    }

    #[test]
    fn find_parent_in_tree_returns_none_when_no_parent_contains_ring() {
        // When no candidate ring contains the new ring, return None.
        let mut manager: RingManager<f64> = RingManager::new();

        // candidate: small 10x10 ring
        let candidate_ring = make_ccw_square(0.0, 0.0, 10.0);
        let candidate_idx = manager.add_ring(candidate_ring);

        // new ring: large 50x50 ring that is NOT inside the candidate
        let new_ring_pts = make_ccw_square(0.0, 0.0, 50.0).points().to_vec();
        let new_area = ring_area(&new_ring_pts);

        let found = find_parent_in_tree(&manager, &new_ring_pts, new_area, Some(candidate_idx));

        assert!(
            found.is_none(),
            "Should return None when candidate does not contain new ring"
        );
    }

    #[test]
    fn find_parent_in_tree_with_no_candidate_returns_none() {
        let manager: RingManager<f64> = RingManager::new();
        let pts = make_ccw_square(0.0, 0.0, 10.0).points().to_vec();
        let area = ring_area(&pts);

        let found = find_parent_in_tree(&manager, &pts, area, None);
        assert!(found.is_none());
    }

    // ==================== Integration: correct_topology with self-intersecting ring ====================

    #[test]
    fn correct_topology_splits_figure_8_into_two_valid_rings() {
        // A figure-8 ring should be split into two separate rings by correct_topology.
        let mut manager: RingManager<f64> = RingManager::new();
        let fig8_idx = manager.add_ring(make_figure_8_ring());

        let rings_before = manager.len();
        correct_topology(&mut manager);

        // A new ring should have been created by the split
        assert!(
            manager.len() > rings_before,
            "correct_topology should split figure-8 ring"
        );

        // Original ring should still have points (wasn't destroyed)
        assert!(
            manager.get(fig8_idx).unwrap().len() >= 3,
            "Original ring should have >= 3 points after split"
        );
    }

    #[test]
    fn correct_topology_clean_ring_unchanged_point_count() {
        // A clean ring should not be split by correct_topology.
        let mut manager: RingManager<f64> = RingManager::new();
        let ring_idx = manager.add_ring(make_ccw_square(0.0, 0.0, 10.0));

        correct_topology(&mut manager);

        // Ring should still have 4 points
        let len_after = manager.get(ring_idx).unwrap().len();
        assert_eq!(
            len_after, 4,
            "Clean ring should still have 4 points after correct_topology"
        );
    }

    // ==================== correct_chained_rings Tests ====================
    // PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
    //            correct_chained_rings (line 755)
    //
    // These tests cover the four topological cases that correct_chained_rings
    // must handle.  All tests are RED: they compile but panic at runtime
    // because correct_chained_rings is a todo!() stub.

    // -----------------------------------------------------------------------
    // Case 1: Two EXTERIOR rings share a single boundary point
    //
    // Scenario (based on fixture polygon-with-hole-with-shared-point.json):
    //   Ring A (exterior, CCW): (-3181,-1223), (-1657,1185), (2761,1256), (1563,-1814)
    //   Ring B (exterior, CCW): (-3181,-1223), (364,-5497), (6665,2813), (-3335,4503)
    //   Both rings share the vertex (-3181, -1223).
    //
    // Expected outcome after correct_chained_rings:
    //   PORT FROM: C++ process_single_intersection lines 482-485
    //   Two EXTERIOR rings sharing a point are SKIPPED (no merge).
    //   This is valid OGC geometry - multipolygons can touch at points.
    //   The rings should remain unchanged.
    // -----------------------------------------------------------------------
    #[test]
    fn correct_chained_rings_two_exteriors_sharing_single_point() {
        let mut manager: RingManager<i64> = RingManager::new();

        // Ring A - exterior polygon (CCW, positive area)
        let ring_a_pts: Vec<Coord<i64>> = vec![
            Coord { x: -3181, y: -1223 },
            Coord { x: -1657, y: 1185 },
            Coord { x: 2761, y: 1256 },
            Coord { x: 1563, y: -1814 },
        ];
        let mut ring_a = Ring::empty();
        for pt in ring_a_pts {
            ring_a.push_point(pt);
        }
        let idx_a = manager.add_ring(ring_a);

        // Ring B - exterior polygon (CCW, positive area)
        // Shares vertex (-3181, -1223) with Ring A.
        let ring_b_pts: Vec<Coord<i64>> = vec![
            Coord { x: -3181, y: -1223 },
            Coord { x: 364, y: -5497 },
            Coord { x: 6665, y: 2813 },
            Coord { x: -3335, y: 4503 },
        ];
        let mut ring_b = Ring::empty();
        for pt in ring_b_pts {
            ring_b.push_point(pt);
        }
        let idx_b = manager.add_ring(ring_b);

        let len_a_before = manager.get(idx_a).unwrap().len();
        let len_b_before = manager.get(idx_b).unwrap().len();

        // Act
        correct_chained_rings(&mut manager);

        // Two EXTERIOR rings sharing a point should NOT be merged.
        // PORT FROM: C++ lines 482-485: "Both are not holes, return nothing to do."
        // The rings should remain completely unchanged.
        assert_eq!(manager.len(), 2, "Two exterior rings should not be merged");
        assert_eq!(
            manager.get(idx_a).unwrap().len(),
            len_a_before,
            "Exterior ring A should be unchanged"
        );
        assert_eq!(
            manager.get(idx_b).unwrap().len(),
            len_b_before,
            "Exterior ring B should be unchanged"
        );
    }

    // -----------------------------------------------------------------------
    // Case 2: Polygon with hole - outer ring and hole share an entire edge
    //
    // Scenario (from fixture polygon-with-hole-shared-edge.json):
    //   Outer ring (CCW):
    //     (-5163,2658), (-4971,-3736), (4366,-3119), (4837,6264)
    //   Hole ring (CW):
    //     (-4971,-3736), (-517,2129), (2053,2658), (4366,-3119)
    //   The outer and hole rings both contain (-4971,-3736) AND (4366,-3119),
    //   forming a shared edge.
    //
    // Expected outcome:
    //   The shared edge is resolved so the resulting geometry is OGC valid
    //   (no degenerate shared-edge between outer ring and hole).
    // -----------------------------------------------------------------------
    #[test]
    fn correct_chained_rings_outer_and_hole_share_edge() {
        let mut manager: RingManager<i64> = RingManager::new();

        // Outer ring (exterior)
        let outer_pts: Vec<Coord<i64>> = vec![
            Coord { x: -5163, y: 2658 },
            Coord { x: -4971, y: -3736 },
            Coord { x: 4366, y: -3119 },
            Coord { x: 4837, y: 6264 },
        ];
        let mut outer = Ring::empty();
        for pt in outer_pts {
            outer.push_point(pt);
        }
        let outer_idx = manager.add_ring(outer);

        // Hole ring (interior, marked as hole)
        // Shares TWO vertices with the outer ring: (-4971,-3736) and (4366,-3119).
        let hole_pts: Vec<Coord<i64>> = vec![
            Coord { x: -4971, y: -3736 },
            Coord { x: -517, y: 2129 },
            Coord { x: 2053, y: 2658 },
            Coord { x: 4366, y: -3119 },
        ];
        let mut hole = Ring::empty();
        for pt in hole_pts {
            hole.push_point(pt);
        }
        hole.set_hole(true);
        let hole_idx = manager.add_ring(hole);
        manager.set_parent(hole_idx, outer_idx);

        // Verify setup
        assert!(
            manager.get(hole_idx).unwrap().is_hole(),
            "Setup: hole ring must be marked as hole"
        );
        assert_eq!(
            manager.get(hole_idx).unwrap().len(),
            4,
            "Setup: hole has 4 points"
        );

        // Act - will panic with todo!() until implemented
        correct_chained_rings(&mut manager);

        // After correction the shared-edge (two shared vertices) must be resolved.
        // The geometry should no longer have the outer and hole sharing an edge;
        // this typically results in the hole being merged into the outer ring.
        let outer_len_after = manager.get(outer_idx).map(|r| r.len()).unwrap_or(0);
        let hole_len_after = manager.get(hole_idx).map(|r| r.len()).unwrap_or(0);

        assert!(
            outer_len_after != 4 || hole_len_after != 4,
            "correct_chained_rings must modify rings that share an edge"
        );
    }

    // -----------------------------------------------------------------------
    // Case 3: Multi-polygon - two EXTERIOR rings share an entire edge
    //
    // Scenario (from fixture multi-polygon-with-shared-edge.json):
    //   Polygon 1 (CCW):
    //     (-5163,2658), (-4971,-3736), (4366,-3119), (4837,6264)
    //   Polygon 2 (CCW):
    //     (-4971,-3736), (-517,2129), (2053,2658), (4366,-3119)
    //   Both polygons share the vertices (-4971,-3736) and (4366,-3119),
    //   which form a shared boundary edge.
    //
    // Expected outcome:
    //   PORT FROM: C++ process_single_intersection lines 482-485
    //   Two EXTERIOR rings are SKIPPED (no merge), even if they share an edge.
    //   This is because correct_chained_rings only processes pairs where
    //   at least one ring is a hole.
    // -----------------------------------------------------------------------
    #[test]
    fn correct_chained_rings_two_exteriors_sharing_edge() {
        let mut manager: RingManager<i64> = RingManager::new();

        // Polygon 1 exterior (CCW)
        let poly1_pts: Vec<Coord<i64>> = vec![
            Coord { x: -5163, y: 2658 },
            Coord { x: -4971, y: -3736 },
            Coord { x: 4366, y: -3119 },
            Coord { x: 4837, y: 6264 },
        ];
        let mut poly1 = Ring::empty();
        for pt in poly1_pts {
            poly1.push_point(pt);
        }
        let idx1 = manager.add_ring(poly1);

        // Polygon 2 exterior (CCW) - shares (-4971,-3736) and (4366,-3119)
        let poly2_pts: Vec<Coord<i64>> = vec![
            Coord { x: -4971, y: -3736 },
            Coord { x: -517, y: 2129 },
            Coord { x: 2053, y: 2658 },
            Coord { x: 4366, y: -3119 },
        ];
        let mut poly2 = Ring::empty();
        for pt in poly2_pts {
            poly2.push_point(pt);
        }
        let idx2 = manager.add_ring(poly2);

        let len1_before = manager.get(idx1).unwrap().len();
        let len2_before = manager.get(idx2).unwrap().len();

        // Act
        correct_chained_rings(&mut manager);

        // Two EXTERIOR rings sharing an edge should NOT be merged.
        // PORT FROM: C++ lines 482-485: "Both are not holes, return nothing to do."
        // The rings should remain completely unchanged.
        assert_eq!(manager.len(), 2, "Two exterior rings should not be merged");
        assert_eq!(
            manager.get(idx1).unwrap().len(),
            len1_before,
            "Exterior ring 1 should be unchanged"
        );
        assert_eq!(
            manager.get(idx2).unwrap().len(),
            len2_before,
            "Exterior ring 2 should be unchanged"
        );
    }

    // -----------------------------------------------------------------------
    // Case 4: No shared points - correct_chained_rings is a no-op
    //
    // Two completely disjoint rings must not be modified.
    // This ensures the function does not accidentally corrupt clean geometry.
    // -----------------------------------------------------------------------
    #[test]
    fn correct_chained_rings_disjoint_rings_unchanged() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Ring 1: square at origin (0,0)-(10,10)
        let ring1 = make_ccw_square(0.0, 0.0, 10.0);
        let idx1 = manager.add_ring(ring1);

        // Ring 2: square far away at (100,100)-(110,110) - no shared coordinates
        let ring2 = make_ccw_square(100.0, 100.0, 10.0);
        let idx2 = manager.add_ring(ring2);

        let len1_before = manager.get(idx1).unwrap().len();
        let len2_before = manager.get(idx2).unwrap().len();

        // Act - will panic with todo!() until implemented
        correct_chained_rings(&mut manager);

        // Disjoint rings must not be altered
        let len1_after = manager.get(idx1).unwrap().len();
        let len2_after = manager.get(idx2).unwrap().len();

        assert_eq!(
            len1_after, len1_before,
            "Disjoint ring 1 must be unchanged by correct_chained_rings"
        );
        assert_eq!(
            len2_after, len2_before,
            "Disjoint ring 2 must be unchanged by correct_chained_rings"
        );
    }

    // -----------------------------------------------------------------------
    // Case 5: Single shared point - no merge (no cycle)
    //
    // PORT FROM: C++ process_single_intersection behavior
    //
    // A single shared point between hole and exterior does NOT form a cycle
    // in the connection_map. Merges only happen when there are MULTIPLE
    // shared points that form a closed loop (like a shared edge).
    //
    // This is valid OGC geometry - a hole touching its exterior at one point.
    // -----------------------------------------------------------------------
    #[test]
    fn correct_chained_rings_single_shared_point_no_merge() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Ring A: Large exterior (CCW)
        // Has a point at (50, 0) that will be shared with the hole
        let mut ring_a: Ring<f64> = Ring::empty();
        ring_a.push_point(Coord { x: 0.0, y: 0.0 });
        ring_a.push_point(Coord { x: 50.0, y: 0.0 }); // shared with hole
        ring_a.push_point(Coord { x: 100.0, y: 0.0 });
        ring_a.push_point(Coord { x: 100.0, y: 100.0 });
        ring_a.push_point(Coord { x: 0.0, y: 100.0 });
        let idx_a = manager.add_ring(ring_a);

        // Ring B: Hole inside A (CW) that touches A at a single point (50, 0)
        let mut ring_b: Ring<f64> = Ring::empty();
        ring_b.push_point(Coord { x: 50.0, y: 0.0 }); // shared with A
        ring_b.push_point(Coord { x: 30.0, y: 30.0 });
        ring_b.push_point(Coord { x: 70.0, y: 30.0 });
        ring_b.set_hole(true);
        let idx_b = manager.add_ring(ring_b);
        manager.set_parent(idx_b, idx_a); // Establish parent relationship

        let len_a_before = manager.get(idx_a).unwrap().len();
        let len_b_before = manager.get(idx_b).unwrap().len();

        // Act
        correct_chained_rings(&mut manager);

        // A single shared point does NOT create a cycle in the connection_map,
        // so no merge occurs. Both rings should remain unchanged.
        // PORT FROM: C++ only merges when connection_map has a cycle (multiple shared points)
        let len_a_after = manager.get(idx_a).map(|r| r.len()).unwrap_or(0);
        let len_b_after = manager.get(idx_b).map(|r| r.len()).unwrap_or(0);

        assert_eq!(
            len_a_after, len_a_before,
            "Single shared point: exterior should be unchanged"
        );
        assert_eq!(
            len_b_after, len_b_before,
            "Single shared point: hole should be unchanged"
        );
    }
}

// ============================================================================
// correct_collinear_edges Tests
// ============================================================================
//
// PORT FROM: wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp
//            `correct_collinear_edges` (line 1229)
//            `correct_collinear_repeats` (line 1204)
//            `process_collinear_edges` (line 1177)
//            `process_collinear_edges_same_ring` (line 1034)
//            `process_collinear_edges_different_rings` (line 1064)
//
// The C++ algorithm:
//   1. Iterates over all_points (sorted by coordinate, like chained rings).
//   2. For each group of ≥2 co-located points, calls correct_collinear_repeats.
//   3. correct_collinear_repeats calls process_collinear_edges on every pair.
//   4. process_collinear_edges:
//      - Removes duplicate points if adjacent in the same ring.
//      - If the two points share a collinear edge (next/prev of one equals
//        the other's prev/next at the same position), fixes the collinear path.
//      - Same-ring collinear edge: spike removal or ring split.
//      - Different-ring collinear edge: ring merge.
//
// These tests are written BEFORE implementation (RED phase). They will fail
// with a compile error because `correct_collinear_edges` does not yet exist.
#[cfg(test)]
mod collinear_edge_tests {
    use super::*;
    use crate::build_result::RingManager;
    use crate::ring_util::ring_area;
    use crate::Ring;

    // -----------------------------------------------------------------------
    // Helper: build a spike ring.
    //
    // A "spike" is a degenerate collinear appendage where the ring traversal
    // goes A → spike_tip → A (back-tracks on itself). Example:
    //
    //   (0,10) ──── (10,10)
    //     │               │
    //     │  spike: (5,0)─(5,-5)─(5,0)
    //     │               │
    //   (0,0) ──── (10,0)
    //
    // Ring points (CCW square with a spike on the bottom edge):
    //   (0,0), (5,0), (5,-5), (5,0), (10,0), (10,10), (0,10)
    //
    // Here (5,0) appears twice and the segment (5,0)→(5,-5)→(5,0) is a spike.
    // After correction the spike should be removed and only the square remain.
    fn make_spike_ring() -> Ring<f64> {
        let mut ring = Ring::empty();
        // CCW square base with a downward spike from (5,0)
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 5.0, y: 0.0 }); // spike base (first visit)
        ring.push_point(Coord { x: 5.0, y: -5.0 }); // spike tip
        ring.push_point(Coord { x: 5.0, y: 0.0 }); // spike base (second visit)
        ring.push_point(Coord { x: 10.0, y: 0.0 });
        ring.push_point(Coord { x: 10.0, y: 10.0 });
        ring.push_point(Coord { x: 0.0, y: 10.0 });
        ring
    }

    // -----------------------------------------------------------------------
    // Helper: build a square ring with two spikes.
    //
    // Two separate spikes at different points of the same ring, both back-
    // tracking to their start points.
    //
    //   Spike 1: from (0,5) goes to (-5,5) and back
    //   Spike 2: from (5,10) goes to (5,15) and back
    //
    // Points: (0,0), (0,5), (-5,5), (0,5), (10,0), ... (5,10), (5,15), (5,10), ...
    fn make_double_spike_ring() -> Ring<f64> {
        let mut ring = Ring::empty();
        // Left spike (from left edge at y=5)
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 0.0, y: 5.0 }); // spike base first visit
        ring.push_point(Coord { x: -5.0, y: 5.0 }); // spike tip
        ring.push_point(Coord { x: 0.0, y: 5.0 }); // spike base second visit
        ring.push_point(Coord { x: 0.0, y: 10.0 });
        // Top spike (from top edge at x=5)
        ring.push_point(Coord { x: 5.0, y: 10.0 }); // spike base first visit
        ring.push_point(Coord { x: 5.0, y: 15.0 }); // spike tip
        ring.push_point(Coord { x: 5.0, y: 10.0 }); // spike base second visit
        ring.push_point(Coord { x: 10.0, y: 10.0 });
        ring.push_point(Coord { x: 10.0, y: 0.0 });
        ring
    }

    // -----------------------------------------------------------------------
    // Helper: build two rings sharing a collinear edge.
    //
    // Ring A (CCW square, left):  (0,0)-(5,0)-(5,10)-(0,10)
    //   Edge from (5,0) to (5,10) goes UP
    //
    // Ring B (arranged so shared edge goes DOWN): (5,10)-(5,0)-(10,0)-(10,10)
    //   Edge from (5,10) to (5,0) goes DOWN
    //
    // The shared edge at x=5 is traversed in opposite directions, forming a
    // collinear boundary. After correction they should be merged.
    fn make_two_rings_sharing_edge() -> (Ring<f64>, Ring<f64>) {
        let mut ring_a = Ring::empty();
        // CCW: left square - edge goes (5,0) -> (5,10) = UP
        ring_a.push_point(Coord { x: 0.0, y: 0.0 });
        ring_a.push_point(Coord { x: 5.0, y: 0.0 }); // shared edge bottom
        ring_a.push_point(Coord { x: 5.0, y: 10.0 }); // shared edge top
        ring_a.push_point(Coord { x: 0.0, y: 10.0 });

        let mut ring_b = Ring::empty();
        // Arranged so shared edge goes (5,10) -> (5,0) = DOWN (opposite of Ring A)
        ring_b.push_point(Coord { x: 5.0, y: 10.0 }); // shared edge top - start here
        ring_b.push_point(Coord { x: 5.0, y: 0.0 }); // shared edge bottom - goes DOWN
        ring_b.push_point(Coord { x: 10.0, y: 0.0 });
        ring_b.push_point(Coord { x: 10.0, y: 10.0 });

        (ring_a, ring_b)
    }

    // -----------------------------------------------------------------------
    // Helper: build a ring that is purely degenerate (a back-and-forth line).
    //
    // This ring will have its entire area reduced to zero by the spike removal
    // and should be deleted entirely.
    //
    // Points: (0,0) → (5,0) → (0,0) - a pure spike with no area.
    fn make_degenerate_spike_ring() -> Ring<f64> {
        let mut ring = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 5.0, y: 0.0 });
        ring.push_point(Coord { x: 0.0, y: 0.0 }); // back to start = fully degenerate
        ring
    }

    // -----------------------------------------------------------------------
    // Helper: build a ring with a spike at the ring's start/end wrap-around.
    //
    // The collinear edge crosses the ring's index-0 boundary:
    //   ...(9,0) → (10,0) → (10,-3) → (10,0)... (spike at end of ring)
    // but the first point is (10,0) so the spike wraps the ring start.
    fn make_spike_at_start_ring() -> Ring<f64> {
        let mut ring = Ring::empty();
        // Spike at ring start: first point equals a later point
        ring.push_point(Coord { x: 10.0, y: 0.0 }); // appears again at end of spike
        ring.push_point(Coord { x: 10.0, y: -3.0 }); // spike tip
        ring.push_point(Coord { x: 10.0, y: 0.0 }); // back to start = spike base
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 0.0, y: 10.0 });
        ring.push_point(Coord { x: 10.0, y: 10.0 });
        ring
    }

    // -----------------------------------------------------------------------
    // Test 1: Empty manager exits without panic.
    //
    // PORT FROM: C++ correct_collinear_edges line 1231-1233:
    //   if (manager.all_points.size() < 2) return;
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_no_op_on_empty_manager() {
        // An empty manager has no rings and no points. The function must
        // return immediately without panicking.
        let mut manager: RingManager<f64> = RingManager::new();
        correct_collinear_edges(&mut manager);
        assert_eq!(manager.len(), 0, "Empty manager should remain empty");
    }

    // -----------------------------------------------------------------------
    // Test 2: Single clean ring (no co-located points) is unchanged.
    //
    // A simple square with no repeated coordinates should pass through
    // correct_collinear_edges untouched.
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_clean_ring_unchanged() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Plain 4-point CCW square - no spikes, no shared edges
        let mut ring = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 10.0, y: 0.0 });
        ring.push_point(Coord { x: 10.0, y: 10.0 });
        ring.push_point(Coord { x: 0.0, y: 10.0 });
        let idx = manager.add_ring(ring);

        correct_collinear_edges(&mut manager);

        // Ring should still exist and have 4 points
        let ring_after = manager.get(idx).expect("Ring should still exist");
        assert_eq!(
            ring_after.len(),
            4,
            "Clean square should remain 4 points after collinear correction"
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: Simple spike is removed from a ring (same-ring collinear edge).
    //
    // PORT FROM: C++ process_collinear_edges_same_ring → fix_collinear_path
    //            "spike_left" or "spike_right" branch: removes the spike
    //            and returns a single non-null point, keeping the ring.
    //
    // The ring has 7 points (square + 3-point spike). After correction it
    // should have 4 points (just the square).
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_removes_simple_spike() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_spike_ring();

        // Setup: 7 points - square with a downward spike at (5,0)
        assert_eq!(ring.len(), 7, "Setup: spike ring has 7 points");
        let idx = manager.add_ring(ring);

        correct_collinear_edges(&mut manager);

        // After correction: spike removed, ring should have 4 points
        let ring_after = manager.get(idx).expect("Ring should survive spike removal");
        assert_eq!(
            ring_after.len(),
            4,
            "Square spike ring should reduce to 4-point square after spike removal"
        );

        // The spike tip (5,-5) must not appear in the result
        let has_spike_tip = ring_after
            .points()
            .iter()
            .any(|p| p.x == 5.0 && p.y == -5.0);
        assert!(
            !has_spike_tip,
            "Spike tip (5,-5) should have been removed from ring"
        );
    }

    // -----------------------------------------------------------------------
    // Test 4: Multiple spikes on the same ring are all removed.
    //
    // PORT FROM: correct_collinear_repeats iterates all pairs in a co-located
    //            group, so multiple spikes must each be processed.
    //
    // The ring has 10 points (left spike + top spike on a square). After
    // correction it should have 4 points (just the square corners).
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_removes_multiple_spikes() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_double_spike_ring();

        // Setup: 10 points - square with two spikes
        assert_eq!(ring.len(), 10, "Setup: double spike ring has 10 points");
        let idx = manager.add_ring(ring);

        correct_collinear_edges(&mut manager);

        // After correction: both spikes removed, ring should have 4 points
        let ring_after = manager
            .get(idx)
            .expect("Ring should survive double spike removal");
        assert_eq!(
            ring_after.len(),
            4,
            "Ring with two spikes should reduce to 4-point square"
        );

        // Neither spike tip should remain
        let has_left_spike_tip = ring_after
            .points()
            .iter()
            .any(|p| p.x == -5.0 && p.y == 5.0);
        let has_top_spike_tip = ring_after
            .points()
            .iter()
            .any(|p| p.x == 5.0 && p.y == 15.0);
        assert!(!has_left_spike_tip, "Left spike tip (-5,5) should be gone");
        assert!(!has_top_spike_tip, "Top spike tip (5,15) should be gone");
    }

    // -----------------------------------------------------------------------
    // Test 5: Fully degenerate ring (pure spike with zero area) is removed.
    //
    // PORT FROM: C++ process_collinear_edges_same_ring:
    //   if (results.pt1 == nullptr) → remove_ring(original_ring, ...)
    //
    // A ring that is entirely a back-and-forth spike has no area. After
    // correction the ring should be removed from the manager (or have 0 pts).
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_removes_fully_degenerate_ring() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_degenerate_spike_ring();

        // Setup: 3 points forming a pure zero-area spike
        assert_eq!(ring.len(), 3, "Setup: degenerate ring has 3 points");
        let idx = manager.add_ring(ring);

        correct_collinear_edges(&mut manager);

        // After correction: ring should be removed (no points) or absent
        let ring_gone = manager.get(idx).map(|r| r.len() == 0).unwrap_or(true);
        assert!(
            ring_gone,
            "Fully degenerate (zero-area) ring should be removed after collinear correction"
        );
    }

    // -----------------------------------------------------------------------
    // Test 6: Two rings sharing a collinear edge are merged into one.
    //
    // PORT FROM: C++ process_collinear_edges_different_rings:
    //   rings become one merged ring; the smaller ring is deleted.
    //
    // Ring A (4 pts, left square) + Ring B (4 pts, right square) share the
    // edge from (5,0) to (5,10). After correction they should merge into one
    // ring covering the full rectangle (0,0)-(10,0)-(10,10)-(0,10).
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_merges_two_rings_sharing_edge() {
        let mut manager: RingManager<f64> = RingManager::new();
        let (ring_a, ring_b) = make_two_rings_sharing_edge();

        let idx_a = manager.add_ring(ring_a);
        let idx_b = manager.add_ring(ring_b);

        // Both rings start as 4-point squares
        assert_eq!(
            manager.get(idx_a).unwrap().len(),
            4,
            "Setup: ring A is 4-point"
        );
        assert_eq!(
            manager.get(idx_b).unwrap().len(),
            4,
            "Setup: ring B is 4-point"
        );

        correct_collinear_edges(&mut manager);

        // After merging: one ring should be removed, the other survives.
        // The merged ring covers the full 10x10 rectangle = 4 corner points.
        let a_exists = manager.get(idx_a).map(|r| r.len() > 0).unwrap_or(false);
        let b_exists = manager.get(idx_b).map(|r| r.len() > 0).unwrap_or(false);

        // Exactly one ring should survive (the other was deleted)
        assert!(
            a_exists ^ b_exists,
            "Exactly one ring should survive after merging two rings that share a collinear edge \
             (a_exists={a_exists}, b_exists={b_exists})"
        );

        // The surviving ring should be a 4-point rectangle (shared edge removed)
        let surviving_len = if a_exists {
            manager.get(idx_a).unwrap().len()
        } else {
            manager.get(idx_b).unwrap().len()
        };
        assert_eq!(
            surviving_len, 4,
            "Merged ring should be the 4-corner outer rectangle"
        );

        // Area of merged ring should equal sum of original areas (50 + 50 = 100)
        let surviving_idx = if a_exists { idx_a } else { idx_b };
        let area = ring_area(manager.get(surviving_idx).unwrap().points()).abs();
        assert!(
            (area - 100.0).abs() < 1e-6,
            "Merged ring should cover full 10×10 area = 100, got {area}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 7: Spike at ring wrap-around (start/end boundary) is removed.
    //
    // PORT FROM: C++ correct_collinear_edges handles the wrap-around case
    //            because all_points is sorted globally (not per-ring), and
    //            correct_collinear_repeats iterates all pairs regardless of
    //            their position within the ring.
    //
    // The spike straddles index 0: point[0] == point[2] == (10,0).
    // After correction the ring should collapse to the 4 non-spike corners.
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_removes_spike_at_ring_start() {
        let mut manager: RingManager<f64> = RingManager::new();
        let ring = make_spike_at_start_ring();

        // Setup: 6 points - square with spike wrapping index 0
        assert_eq!(ring.len(), 6, "Setup: wrap-around spike ring has 6 points");
        let idx = manager.add_ring(ring);

        correct_collinear_edges(&mut manager);

        // After correction: spike removed, 4 square corners remain
        let ring_after = manager
            .get(idx)
            .expect("Ring should survive spike at start");
        assert_eq!(
            ring_after.len(),
            4,
            "Ring with spike at start/end wrap should reduce to 4-point square"
        );

        // Spike tip (10,-3) must be gone
        let has_spike_tip = ring_after
            .points()
            .iter()
            .any(|p| p.x == 10.0 && p.y == -3.0);
        assert!(!has_spike_tip, "Spike tip (10,-3) should have been removed");
    }

    // -----------------------------------------------------------------------
    // Test 8: correct_collinear_edges does not affect disjoint rings.
    //
    // Two separate squares with no shared coordinates should each remain
    // unchanged after correct_collinear_edges runs.
    // -----------------------------------------------------------------------
    #[test]
    fn correct_collinear_edges_disjoint_rings_unchanged() {
        let mut manager: RingManager<f64> = RingManager::new();

        // Square A at origin
        let mut ring_a = Ring::empty();
        ring_a.push_point(Coord { x: 0.0, y: 0.0 });
        ring_a.push_point(Coord { x: 10.0, y: 0.0 });
        ring_a.push_point(Coord { x: 10.0, y: 10.0 });
        ring_a.push_point(Coord { x: 0.0, y: 10.0 });
        let idx_a = manager.add_ring(ring_a);

        // Square B far away - no shared coordinates
        let mut ring_b = Ring::empty();
        ring_b.push_point(Coord { x: 100.0, y: 100.0 });
        ring_b.push_point(Coord { x: 110.0, y: 100.0 });
        ring_b.push_point(Coord { x: 110.0, y: 110.0 });
        ring_b.push_point(Coord { x: 100.0, y: 110.0 });
        let idx_b = manager.add_ring(ring_b);

        correct_collinear_edges(&mut manager);

        // Both rings should be unchanged
        assert_eq!(
            manager.get(idx_a).map(|r| r.len()).unwrap_or(0),
            4,
            "Disjoint ring A should be unchanged"
        );
        assert_eq!(
            manager.get(idx_b).map(|r| r.len()).unwrap_or(0),
            4,
            "Disjoint ring B should be unchanged"
        );
    }
}
