//! Scanbeam - Priority queue of Y coordinates for the Vatti algorithm.
//!
//! The scanbeam maintains a sorted list of Y coordinates where the sweep line
//! needs to stop and process events. Values are stored in ascending order,
//! with duplicates ignored.

use geo_types::CoordNum;

/// A priority queue of Y coordinates for the sweep line algorithm.
///
/// The scanbeam stores unique Y coordinates in sorted order and provides
/// efficient access to the maximum (topmost) Y value. This is used by the
/// Vatti algorithm to determine where the sweep line should stop next.
///
/// # Type Parameters
///
/// * `T` - The coordinate type, typically `i64` or `f64`.
///
/// # Example
///
/// ```
/// use wagyu_rs::Scanbeam;
///
/// let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
/// scanbeam.insert(10);
/// scanbeam.insert(5);
/// scanbeam.insert(15);
///
/// // Values come out in descending order (highest Y first)
/// assert_eq!(scanbeam.pop(), Some(15));
/// assert_eq!(scanbeam.pop(), Some(10));
/// assert_eq!(scanbeam.pop(), Some(5));
/// assert_eq!(scanbeam.pop(), None);
/// ```
#[derive(Debug, Clone)]
pub struct Scanbeam<T: CoordNum> {
    // Internal sorted vector - values stored in ascending order
    // pop() returns from the back (highest Y)
    values: Vec<T>,
}

impl<T: CoordNum> Scanbeam<T> {
    /// Creates a new empty scanbeam.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let scanbeam: Scanbeam<i64> = Scanbeam::new();
    /// assert!(scanbeam.is_empty());
    /// ```
    #[inline]
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }

    /// Creates a new empty scanbeam with the specified capacity.
    ///
    /// The scanbeam will be able to hold at least `capacity` elements
    /// without reallocating.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let scanbeam: Scanbeam<i64> = Scanbeam::with_capacity(100);
    /// assert!(scanbeam.is_empty());
    /// ```
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            values: Vec::with_capacity(capacity),
        }
    }

    /// Inserts a Y coordinate into the scanbeam, maintaining sorted order.
    ///
    /// If the value already exists in the scanbeam, it is not inserted again.
    /// Values are stored in ascending order internally.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
    /// scanbeam.insert(10);
    /// scanbeam.insert(5);
    /// scanbeam.insert(10); // duplicate - ignored
    ///
    /// assert_eq!(scanbeam.len(), 2);
    /// ```
    pub fn insert(&mut self, y: T) {
        // Use binary search to find insertion point (maintaining ascending order)
        match self
            .values
            .binary_search_by(|probe| probe.partial_cmp(&y).unwrap_or(std::cmp::Ordering::Equal))
        {
            Ok(_) => {
                // Value already exists, don't insert duplicate
            }
            Err(pos) => {
                // Insert at the correct position to maintain sorted order
                self.values.insert(pos, y);
            }
        }
    }

    /// Removes and returns the highest Y coordinate from the scanbeam.
    ///
    /// Returns `None` if the scanbeam is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
    /// scanbeam.insert(10);
    /// scanbeam.insert(5);
    ///
    /// assert_eq!(scanbeam.pop(), Some(10));
    /// assert_eq!(scanbeam.pop(), Some(5));
    /// assert_eq!(scanbeam.pop(), None);
    /// ```
    #[inline]
    pub fn pop(&mut self) -> Option<T> {
        self.values.pop()
    }

    /// Returns a reference to the highest Y coordinate without removing it.
    ///
    /// Returns `None` if the scanbeam is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
    /// scanbeam.insert(10);
    ///
    /// assert_eq!(scanbeam.peek(), Some(&10));
    /// assert_eq!(scanbeam.peek(), Some(&10)); // Still there
    /// ```
    #[inline]
    pub fn peek(&self) -> Option<&T> {
        self.values.last()
    }

    /// Returns `true` if the scanbeam contains no elements.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
    /// assert!(scanbeam.is_empty());
    ///
    /// scanbeam.insert(10);
    /// assert!(!scanbeam.is_empty());
    /// ```
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Returns the number of elements in the scanbeam.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
    /// assert_eq!(scanbeam.len(), 0);
    ///
    /// scanbeam.insert(10);
    /// scanbeam.insert(20);
    /// assert_eq!(scanbeam.len(), 2);
    /// ```
    #[inline]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Removes all elements from the scanbeam.
    ///
    /// # Example
    ///
    /// ```
    /// use wagyu_rs::Scanbeam;
    ///
    /// let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
    /// scanbeam.insert(10);
    /// scanbeam.insert(20);
    ///
    /// scanbeam.clear();
    /// assert!(scanbeam.is_empty());
    /// ```
    #[inline]
    pub fn clear(&mut self) {
        self.values.clear();
    }
}

impl<T: CoordNum> Default for Scanbeam<T> {
    /// Creates a new empty scanbeam.
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // Empty scanbeam tests
    // =========================================================================

    #[test]
    fn test_new_scanbeam_is_empty() {
        let scanbeam: Scanbeam<i64> = Scanbeam::new();
        assert!(scanbeam.is_empty());
    }

    #[test]
    fn test_empty_scanbeam_pop_returns_none() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        assert_eq!(scanbeam.pop(), None);
    }

    #[test]
    fn test_empty_scanbeam_peek_returns_none() {
        let scanbeam: Scanbeam<i64> = Scanbeam::new();
        assert_eq!(scanbeam.peek(), None);
    }

    #[test]
    fn test_empty_scanbeam_len_is_zero() {
        let scanbeam: Scanbeam<i64> = Scanbeam::new();
        assert_eq!(scanbeam.len(), 0);
    }

    // =========================================================================
    // Single value tests
    // =========================================================================

    #[test]
    fn test_insert_single_value_not_empty() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(42);
        assert!(!scanbeam.is_empty());
    }

    #[test]
    fn test_insert_single_value_len_is_one() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(42);
        assert_eq!(scanbeam.len(), 1);
    }

    #[test]
    fn test_peek_returns_single_value() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(42);
        assert_eq!(scanbeam.peek(), Some(&42));
    }

    #[test]
    fn test_pop_returns_single_value() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(42);
        assert_eq!(scanbeam.pop(), Some(42));
    }

    #[test]
    fn test_pop_single_value_then_empty() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(42);
        scanbeam.pop();
        assert!(scanbeam.is_empty());
    }

    // =========================================================================
    // Multiple values - ascending order tests (pop returns highest Y first)
    // =========================================================================

    #[test]
    fn test_multiple_values_pop_in_descending_order() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(10);
        scanbeam.insert(5);
        scanbeam.insert(15);

        // Should come out highest first (descending)
        assert_eq!(scanbeam.pop(), Some(15));
        assert_eq!(scanbeam.pop(), Some(10));
        assert_eq!(scanbeam.pop(), Some(5));
        assert_eq!(scanbeam.pop(), None);
    }

    #[test]
    fn test_insert_already_sorted_descending() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(15);
        scanbeam.insert(10);
        scanbeam.insert(5);

        assert_eq!(scanbeam.pop(), Some(15));
        assert_eq!(scanbeam.pop(), Some(10));
        assert_eq!(scanbeam.pop(), Some(5));
    }

    #[test]
    fn test_insert_already_sorted_ascending() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(5);
        scanbeam.insert(10);
        scanbeam.insert(15);

        assert_eq!(scanbeam.pop(), Some(15));
        assert_eq!(scanbeam.pop(), Some(10));
        assert_eq!(scanbeam.pop(), Some(5));
    }

    #[test]
    fn test_peek_returns_highest_value() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(10);
        scanbeam.insert(5);
        scanbeam.insert(15);

        assert_eq!(scanbeam.peek(), Some(&15));
    }

    #[test]
    fn test_peek_does_not_remove_value() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(10);
        scanbeam.insert(5);
        scanbeam.insert(15);

        assert_eq!(scanbeam.peek(), Some(&15));
        assert_eq!(scanbeam.peek(), Some(&15)); // Still there
        assert_eq!(scanbeam.len(), 3);
    }

    // =========================================================================
    // Duplicate handling tests
    // =========================================================================

    #[test]
    fn test_duplicate_values_are_ignored() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(10);
        scanbeam.insert(10);
        scanbeam.insert(10);

        assert_eq!(scanbeam.len(), 1);
        assert_eq!(scanbeam.pop(), Some(10));
        assert!(scanbeam.is_empty());
    }

    #[test]
    fn test_duplicate_mixed_with_unique() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(5);
        scanbeam.insert(10);
        scanbeam.insert(5); // duplicate
        scanbeam.insert(15);
        scanbeam.insert(10); // duplicate

        assert_eq!(scanbeam.len(), 3);
        assert_eq!(scanbeam.pop(), Some(15));
        assert_eq!(scanbeam.pop(), Some(10));
        assert_eq!(scanbeam.pop(), Some(5));
        assert!(scanbeam.is_empty());
    }

    // =========================================================================
    // Negative values tests
    // =========================================================================

    #[test]
    fn test_negative_values() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(-10);
        scanbeam.insert(-5);
        scanbeam.insert(-15);

        // -5 is highest (closest to zero)
        assert_eq!(scanbeam.pop(), Some(-5));
        assert_eq!(scanbeam.pop(), Some(-10));
        assert_eq!(scanbeam.pop(), Some(-15));
    }

    #[test]
    fn test_mixed_positive_and_negative() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(-10);
        scanbeam.insert(0);
        scanbeam.insert(10);

        assert_eq!(scanbeam.pop(), Some(10));
        assert_eq!(scanbeam.pop(), Some(0));
        assert_eq!(scanbeam.pop(), Some(-10));
    }

    // =========================================================================
    // f64 coordinate tests
    // =========================================================================

    #[test]
    fn test_f64_coordinates() {
        let mut scanbeam: Scanbeam<f64> = Scanbeam::new();
        scanbeam.insert(1.5);
        scanbeam.insert(2.5);
        scanbeam.insert(0.5);

        assert_eq!(scanbeam.pop(), Some(2.5));
        assert_eq!(scanbeam.pop(), Some(1.5));
        assert_eq!(scanbeam.pop(), Some(0.5));
    }

    // =========================================================================
    // Clear test
    // =========================================================================

    #[test]
    fn test_clear_empties_scanbeam() {
        let mut scanbeam: Scanbeam<i64> = Scanbeam::new();
        scanbeam.insert(10);
        scanbeam.insert(20);
        scanbeam.insert(30);

        scanbeam.clear();

        assert!(scanbeam.is_empty());
        assert_eq!(scanbeam.len(), 0);
        assert_eq!(scanbeam.pop(), None);
    }

    // =========================================================================
    // Reserve/capacity test
    // =========================================================================

    #[test]
    fn test_with_capacity_creates_empty_scanbeam() {
        let scanbeam: Scanbeam<i64> = Scanbeam::with_capacity(100);
        assert!(scanbeam.is_empty());
    }
}
