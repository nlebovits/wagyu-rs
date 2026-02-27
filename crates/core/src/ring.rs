//! Ring - A closed ring of points representing a polygon boundary.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring.hpp
//!
//! A Ring stores points forming a polygon boundary. It tracks:
//! - The points themselves (owned directly)
//! - Whether this ring is a hole
//! - Parent/child relationships for nesting
//! - Ring index for identification

use geo_types::{Coord, CoordFloat, CoordNum};

/// A closed ring of points representing a polygon boundary.
///
/// Rings are the fundamental building blocks of polygons in wagyu.
/// An exterior ring defines the outer boundary, while interior rings
/// (holes) are marked with `is_hole = true` and reference their parent.
#[derive(Debug, Clone)]
pub struct Ring<T: CoordNum> {
    /// Points forming this ring
    points: Vec<Coord<T>>,
    /// Whether this ring represents a hole (interior ring)
    is_hole: bool,
    /// Unique identifier for this ring
    ring_index: usize,
    /// Index of parent ring (if this is a hole)
    parent: Option<usize>,
    /// Indices of child rings (holes within this ring)
    children: Vec<usize>,
}

impl<T: CoordNum> Ring<T> {
    /// Create a new empty ring.
    pub fn new(points: Vec<Coord<T>>) -> Self {
        Ring {
            points,
            is_hole: false,
            ring_index: 0,
            parent: None,
            children: Vec::new(),
        }
    }

    /// Create a new empty ring.
    pub fn empty() -> Self {
        Ring {
            points: Vec::new(),
            is_hole: false,
            ring_index: 0,
            parent: None,
            children: Vec::new(),
        }
    }

    /// Add a point to the ring.
    pub fn push_point(&mut self, point: Coord<T>) {
        self.points.push(point);
    }

    /// Add a point to the ring (alias for push_point for compatibility).
    pub fn add_point(&mut self, point: Coord<T>) {
        self.points.push(point);
    }

    /// Returns the number of points in the ring.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Returns true if the ring has no points.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Returns true if this ring is a hole (interior ring).
    pub fn is_hole(&self) -> bool {
        self.is_hole
    }

    /// Set whether this ring is a hole.
    pub fn set_hole(&mut self, is_hole: bool) {
        self.is_hole = is_hole;
    }

    /// Returns the ring's unique index.
    pub fn ring_index(&self) -> usize {
        self.ring_index
    }

    /// Set the ring's unique index.
    pub fn set_ring_index(&mut self, index: usize) {
        self.ring_index = index;
    }

    /// Returns the parent ring's index, if this ring is a hole.
    pub fn parent(&self) -> Option<usize> {
        self.parent
    }

    /// Set the parent ring's index.
    pub fn set_parent(&mut self, parent: Option<usize>) {
        self.parent = parent;
    }

    /// Returns the indices of child rings (holes within this ring).
    pub fn children(&self) -> &[usize] {
        &self.children
    }

    /// Add a child ring index.
    pub fn add_child(&mut self, child_index: usize) {
        self.children.push(child_index);
    }

    /// Clear all children (used during tree rebuilding).
    pub fn clear_children(&mut self) {
        self.children.clear();
    }

    /// Returns a reference to the ring's points.
    pub fn points(&self) -> &[Coord<T>] {
        &self.points
    }

    /// Returns a mutable reference to the ring's points.
    pub fn points_mut(&mut self) -> &mut Vec<Coord<T>> {
        &mut self.points
    }
}

impl<T: CoordFloat> Ring<T> {
    /// Compute the signed area of the ring using the shoelace formula.
    ///
    /// The sign indicates winding direction:
    /// - Positive area: counter-clockwise (CCW) winding
    /// - Negative area: clockwise (CW) winding
    ///
    /// For an empty ring or a ring with fewer than 3 points, returns 0.
    pub fn area(&self) -> T {
        if self.points.len() < 3 {
            return T::zero();
        }

        // Shoelace formula: area = 0.5 * sum(x[i] * y[i+1] - x[i+1] * y[i])
        let mut sum = T::zero();
        let n = self.points.len();

        for i in 0..n {
            let j = (i + 1) % n;
            let pi = &self.points[i];
            let pj = &self.points[j];
            sum = sum + (pi.x * pj.y - pj.x * pi.y);
        }

        sum * T::from(0.5).unwrap()
    }
}

impl<T: CoordNum> Default for Ring<T> {
    fn default() -> Self {
        Self::empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::Coord;

    // ==================== Empty Ring Creation ====================

    #[test]
    fn new_ring_is_empty() {
        let ring: Ring<f64> = Ring::empty();
        assert!(ring.is_empty());
        assert_eq!(ring.len(), 0);
    }

    // ==================== Adding Points ====================

    #[test]
    fn push_point_increases_len() {
        let mut ring: Ring<f64> = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        assert_eq!(ring.len(), 1);
        assert!(!ring.is_empty());

        ring.push_point(Coord { x: 1.0, y: 0.0 });
        ring.push_point(Coord { x: 1.0, y: 1.0 });
        assert_eq!(ring.len(), 3);
    }

    // ==================== Hole Flag ====================

    #[test]
    fn new_ring_is_not_a_hole_by_default() {
        let ring: Ring<f64> = Ring::empty();
        assert!(!ring.is_hole());
    }

    #[test]
    fn set_hole_marks_ring_as_hole() {
        let mut ring: Ring<f64> = Ring::empty();
        ring.set_hole(true);
        assert!(ring.is_hole());

        ring.set_hole(false);
        assert!(!ring.is_hole());
    }

    // ==================== Area Calculation ====================
    // The shoelace formula: area = 0.5 * sum(x[i] * y[i+1] - x[i+1] * y[i])
    // Positive area = counter-clockwise (CCW) winding
    // Negative area = clockwise (CW) winding

    #[test]
    fn area_of_empty_ring_is_zero() {
        let ring: Ring<f64> = Ring::empty();
        assert!((ring.area() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn area_of_ccw_unit_square_is_positive() {
        // Unit square in CCW order: (0,0) -> (1,0) -> (1,1) -> (0,1)
        // Expected area: +1.0 (positive indicates CCW)
        let mut ring: Ring<f64> = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 1.0, y: 0.0 });
        ring.push_point(Coord { x: 1.0, y: 1.0 });
        ring.push_point(Coord { x: 0.0, y: 1.0 });

        let area = ring.area();
        assert!(area > 0.0, "CCW ring should have positive area");
        assert!((area - 1.0).abs() < 1e-10, "Unit square area should be 1.0");
    }

    #[test]
    fn area_of_cw_unit_square_is_negative() {
        // Unit square in CW order: (0,0) -> (0,1) -> (1,1) -> (1,0)
        // Expected area: -1.0 (negative indicates CW)
        let mut ring: Ring<f64> = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 0.0, y: 1.0 });
        ring.push_point(Coord { x: 1.0, y: 1.0 });
        ring.push_point(Coord { x: 1.0, y: 0.0 });

        let area = ring.area();
        assert!(area < 0.0, "CW ring should have negative area");
        assert!(
            (area - (-1.0)).abs() < 1e-10,
            "Unit square area should be -1.0"
        );
    }

    #[test]
    fn area_of_triangle() {
        // Triangle with vertices at (0,0), (4,0), (0,3) in CCW order
        // Expected area: 0.5 * base * height = 0.5 * 4 * 3 = 6.0
        let mut ring: Ring<f64> = Ring::empty();
        ring.push_point(Coord { x: 0.0, y: 0.0 });
        ring.push_point(Coord { x: 4.0, y: 0.0 });
        ring.push_point(Coord { x: 0.0, y: 3.0 });

        let area = ring.area();
        assert!(
            (area - 6.0).abs() < 1e-10,
            "Triangle area should be 6.0, got {}",
            area
        );
    }

    // ==================== Ring Index ====================

    #[test]
    fn new_ring_has_zero_index_by_default() {
        let ring: Ring<f64> = Ring::empty();
        assert_eq!(ring.ring_index(), 0);
    }

    #[test]
    fn set_ring_index_changes_index() {
        let mut ring: Ring<f64> = Ring::empty();
        ring.set_ring_index(42);
        assert_eq!(ring.ring_index(), 42);
    }

    // ==================== Parent/Child Relationships ====================

    #[test]
    fn new_ring_has_no_parent() {
        let ring: Ring<f64> = Ring::empty();
        assert!(ring.parent().is_none());
    }

    #[test]
    fn set_parent_assigns_parent_index() {
        let mut ring: Ring<f64> = Ring::empty();
        ring.set_parent(Some(5));
        assert_eq!(ring.parent(), Some(5));

        ring.set_parent(None);
        assert!(ring.parent().is_none());
    }

    #[test]
    fn new_ring_has_no_children() {
        let ring: Ring<f64> = Ring::empty();
        assert!(ring.children().is_empty());
    }

    #[test]
    fn add_child_adds_child_index() {
        let mut ring: Ring<f64> = Ring::empty();
        ring.add_child(1);
        ring.add_child(3);
        ring.add_child(5);

        assert_eq!(ring.children(), &[1, 3, 5]);
    }
}
