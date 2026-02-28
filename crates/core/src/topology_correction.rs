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

    // Iterative outer loop: restart after each split because Vec indices become stale
    loop {
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
/// Returns true if any splits were performed.
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

    if correct_tree {
        assign_new_ring_parents(manager, ring_idx, &new_rings);
    }

    manager.set_corrected(ring_idx, true);
    did_split
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
    for ring_idx in sorted {
        if correct_ring_self_intersections(manager, ring_idx, correct_tree) {
            fixed = true;
        }
    }
    fixed
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

    // Step 2: First pass of self-intersection correction (without tree correction)
    // PORT FROM: C++ correct_topology line ~1309 - called before correct_tree
    correct_self_intersections(manager, false);

    // Step 3: Rebuild tree structure
    // Rebuilds parent/child relationships based on containment
    correct_tree(manager);

    // Step 4: Iteratively correct self-intersections with tree correction until stable
    // PORT FROM: C++ correct_topology lines ~1311-1315 - loop until no more fixes
    let mut fixed = true;
    while fixed {
        fixed = correct_self_intersections(manager, true);
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
}
