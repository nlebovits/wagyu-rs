//! Intersection node types for the Vatti clipping algorithm.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/intersect.hpp
//!            wagyu/include/mapbox/geometry/wagyu/intersect_util.hpp
//!
//! An `IntersectNode` represents an intersection point between two edges
//! during the sweep. The sweep algorithm detects when two bounds cross
//! and records their intersection for later processing.

use crate::point::Point;
use geo_types::CoordNum;
use num_traits::ToPrimitive;
use std::cmp::Ordering;
use std::ops::Index;

// ============================================================================
// IntersectNode
// ============================================================================

/// Represents an intersection point between two edges during the sweep.
///
/// From C++: `struct intersect_node<T>` with bound pointers and intersection point.
///
/// In Rust, we use indices into a bounds vector instead of raw pointers,
/// following the "Vec + usize" ownership strategy.
#[derive(Debug, Clone)]
pub struct IntersectNode<T: CoordNum> {
    /// The intersection point (always stored as f64 for precision).
    pub point: Point<T>,
    /// Index of the first bound involved in this intersection.
    pub bound1_index: usize,
    /// Index of the second bound involved in this intersection.
    pub bound2_index: usize,
}

impl<T: CoordNum> IntersectNode<T> {
    /// Create a new intersection node.
    ///
    /// # Arguments
    ///
    /// * `point` - The intersection point
    /// * `bound1_index` - Index of the first bound in the active bounds list
    /// * `bound2_index` - Index of the second bound in the active bounds list
    pub fn new(point: Point<T>, bound1_index: usize, bound2_index: usize) -> Self {
        Self {
            point,
            bound1_index,
            bound2_index,
        }
    }
}

impl<T: CoordNum + ToPrimitive> PartialEq for IntersectNode<T> {
    fn eq(&self, other: &Self) -> bool {
        let self_y = self.point.y.to_f64().unwrap_or(0.0);
        let other_y = other.point.y.to_f64().unwrap_or(0.0);
        let self_x = self.point.x.to_f64().unwrap_or(0.0);
        let other_x = other.point.x.to_f64().unwrap_or(0.0);

        (self_y - other_y).abs() < f64::EPSILON && (self_x - other_x).abs() < f64::EPSILON
    }
}

impl<T: CoordNum + ToPrimitive> Eq for IntersectNode<T> {}

impl<T: CoordNum + ToPrimitive> PartialOrd for IntersectNode<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: CoordNum + ToPrimitive> Ord for IntersectNode<T> {
    /// Compare intersection nodes for sorting.
    ///
    /// From C++ `intersect_list_sorter`:
    /// - Primary: sort by Y descending (higher Y values first)
    /// - Secondary: sort by X ascending (when Y values are equal)
    ///
    /// Note: The C++ code uses winding counts as secondary sort, but we use X
    /// as a simpler tiebreaker since we don't have direct access to bounds here.
    fn cmp(&self, other: &Self) -> Ordering {
        let self_y = self.point.y.to_f64().unwrap_or(0.0);
        let other_y = other.point.y.to_f64().unwrap_or(0.0);

        // Primary sort: Y descending (higher Y first)
        // If other_y > self_y, other should come first, so return Greater
        match other_y.partial_cmp(&self_y) {
            Some(Ordering::Equal) | None => {
                // Secondary sort: X ascending (lower X first)
                let self_x = self.point.x.to_f64().unwrap_or(0.0);
                let other_x = other.point.x.to_f64().unwrap_or(0.0);
                self_x.partial_cmp(&other_x).unwrap_or(Ordering::Equal)
            }
            Some(ord) => ord,
        }
    }
}

// ============================================================================
// IntersectList
// ============================================================================

/// A collection of intersection nodes, to be sorted and processed.
///
/// From C++: `using intersect_list = std::vector<intersect_node<T>>;`
#[derive(Debug, Clone)]
pub struct IntersectList<T: CoordNum> {
    nodes: Vec<IntersectNode<T>>,
}

impl<T: CoordNum> IntersectList<T> {
    /// Create a new empty intersection list.
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    /// Push an intersection node onto the list.
    pub fn push(&mut self, node: IntersectNode<T>) {
        self.nodes.push(node);
    }

    /// Returns true if the list is empty.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Returns the number of nodes in the list.
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// Returns an iterator over the intersection nodes.
    pub fn iter(&self) -> impl Iterator<Item = &IntersectNode<T>> {
        self.nodes.iter()
    }
}

impl<T: CoordNum + ToPrimitive> IntersectList<T> {
    /// Sort the intersection list.
    ///
    /// Sorts by Y descending, then by X ascending for equal Y values.
    pub fn sort(&mut self) {
        self.nodes.sort();
    }
}

impl<T: CoordNum> Default for IntersectList<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: CoordNum> Index<usize> for IntersectList<T> {
    type Output = IntersectNode<T>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::point::Point;

    // =========================================================================
    // IntersectNode construction tests
    // =========================================================================

    #[test]
    fn intersect_node_new_creates_node_with_point_and_indices() {
        let pt = Point::new(5.0_f64, 10.0_f64);
        let node = IntersectNode::new(pt, 0, 1);

        assert_eq!(node.point, pt);
        assert_eq!(node.bound1_index, 0);
        assert_eq!(node.bound2_index, 1);
    }

    #[test]
    fn intersect_node_with_different_indices() {
        let pt = Point::new(100.0_f64, 200.0_f64);
        let node = IntersectNode::new(pt, 5, 10);

        assert_eq!(node.point, pt);
        assert_eq!(node.bound1_index, 5);
        assert_eq!(node.bound2_index, 10);
    }

    #[test]
    fn intersect_node_with_negative_coordinates() {
        let pt = Point::new(-5.0_f64, -10.0_f64);
        let node = IntersectNode::new(pt, 0, 1);

        assert_eq!(node.point.x, -5.0);
        assert_eq!(node.point.y, -10.0);
    }

    // =========================================================================
    // Sorting tests - by Y coordinate (descending, higher Y first)
    // =========================================================================

    #[test]
    fn intersect_node_sorts_by_y_descending() {
        // Higher Y values should come first (like the C++ code: node2.pt.y < node1.pt.y)
        let node1 = IntersectNode::new(Point::new(0.0_f64, 10.0_f64), 0, 1);
        let node2 = IntersectNode::new(Point::new(0.0_f64, 20.0_f64), 2, 3);
        let node3 = IntersectNode::new(Point::new(0.0_f64, 15.0_f64), 4, 5);

        let mut nodes = [node1.clone(), node2.clone(), node3.clone()];
        nodes.sort();

        // After sorting: node2 (y=20) should be first, then node3 (y=15), then node1 (y=10)
        assert_eq!(nodes[0].point.y, 20.0);
        assert_eq!(nodes[1].point.y, 15.0);
        assert_eq!(nodes[2].point.y, 10.0);
    }

    #[test]
    fn intersect_node_equal_y_compares_by_x_ascending() {
        // When Y is equal, we'll compare by X ascending as a tiebreaker
        // (The C++ uses winding counts, but we don't have access to bounds here;
        // for now we use X as a simpler tiebreaker)
        let node1 = IntersectNode::new(Point::new(10.0_f64, 20.0_f64), 0, 1);
        let node2 = IntersectNode::new(Point::new(5.0_f64, 20.0_f64), 2, 3);
        let node3 = IntersectNode::new(Point::new(15.0_f64, 20.0_f64), 4, 5);

        let mut nodes = [node1.clone(), node2.clone(), node3.clone()];
        nodes.sort();

        // All have same Y=20, so sort by X ascending: 5, 10, 15
        assert_eq!(nodes[0].point.x, 5.0);
        assert_eq!(nodes[1].point.x, 10.0);
        assert_eq!(nodes[2].point.x, 15.0);
    }

    #[test]
    fn intersect_node_mixed_y_and_x_sorting() {
        // Mix of different Y values and same Y values
        let node_a = IntersectNode::new(Point::new(10.0_f64, 20.0_f64), 0, 1); // y=20, x=10
        let node_b = IntersectNode::new(Point::new(5.0_f64, 20.0_f64), 2, 3); // y=20, x=5
        let node_c = IntersectNode::new(Point::new(0.0_f64, 30.0_f64), 4, 5); // y=30, x=0
        let node_d = IntersectNode::new(Point::new(20.0_f64, 10.0_f64), 6, 7); // y=10, x=20

        let mut nodes = [
            node_a.clone(),
            node_b.clone(),
            node_c.clone(),
            node_d.clone(),
        ];
        nodes.sort();

        // Order: y=30 first, then y=20 (x=5, then x=10), then y=10
        assert_eq!(nodes[0].point.y, 30.0); // node_c
        assert_eq!(nodes[1].point.y, 20.0);
        assert_eq!(nodes[1].point.x, 5.0); // node_b
        assert_eq!(nodes[2].point.y, 20.0);
        assert_eq!(nodes[2].point.x, 10.0); // node_a
        assert_eq!(nodes[3].point.y, 10.0); // node_d
    }

    // =========================================================================
    // IntersectList tests
    // =========================================================================

    #[test]
    fn intersect_list_can_be_created_empty() {
        let list: IntersectList<f64> = IntersectList::new();
        assert!(list.is_empty());
        assert_eq!(list.len(), 0);
    }

    #[test]
    fn intersect_list_can_push_nodes() {
        let mut list: IntersectList<f64> = IntersectList::new();
        let node = IntersectNode::new(Point::new(5.0_f64, 10.0_f64), 0, 1);

        list.push(node);

        assert_eq!(list.len(), 1);
        assert!(!list.is_empty());
    }

    #[test]
    fn intersect_list_can_be_sorted() {
        let mut list: IntersectList<f64> = IntersectList::new();
        list.push(IntersectNode::new(Point::new(0.0, 10.0), 0, 1));
        list.push(IntersectNode::new(Point::new(0.0, 30.0), 2, 3));
        list.push(IntersectNode::new(Point::new(0.0, 20.0), 4, 5));

        list.sort();

        // After sorting: y=30, y=20, y=10 (descending)
        assert_eq!(list[0].point.y, 30.0);
        assert_eq!(list[1].point.y, 20.0);
        assert_eq!(list[2].point.y, 10.0);
    }

    #[test]
    fn intersect_list_iter_works() {
        let mut list: IntersectList<f64> = IntersectList::new();
        list.push(IntersectNode::new(Point::new(1.0, 1.0), 0, 1));
        list.push(IntersectNode::new(Point::new(2.0, 2.0), 2, 3));

        let collected: Vec<_> = list.iter().collect();
        assert_eq!(collected.len(), 2);
    }

    // =========================================================================
    // Clone/Debug tests
    // =========================================================================

    #[test]
    fn intersect_node_is_clone() {
        let node = IntersectNode::new(Point::new(5.0_f64, 10.0_f64), 0, 1);
        let cloned = node.clone();

        assert_eq!(node.point, cloned.point);
        assert_eq!(node.bound1_index, cloned.bound1_index);
        assert_eq!(node.bound2_index, cloned.bound2_index);
    }

    #[test]
    fn intersect_node_debug_format() {
        let node = IntersectNode::new(Point::new(5.0_f64, 10.0_f64), 0, 1);
        let debug_str = format!("{:?}", node);

        // Should contain the key information
        assert!(debug_str.contains("5"));
        assert!(debug_str.contains("10"));
    }
}
