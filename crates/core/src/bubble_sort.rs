//! Bubble sort with swap callback.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/bubble_sort.hpp
//!
//! This module provides a bubble sort implementation that calls a callback
//! function whenever two elements are swapped. This is useful in wagyu for
//! maintaining auxiliary data structures (like edge relationships) during sorting.
//!
//! # Why Bubble Sort?
//!
//! 1. **Stability**: Preserves relative order of equal elements
//! 2. **Callback on swap**: Allows updating related data when elements move
//! 3. **Small lists**: For small edge lists, O(n²) is competitive due to low overhead

/// Sorts a slice in place using bubble sort, calling a callback on each swap.
///
/// Unlike standard sorting algorithms, this allows you to observe and react to
/// each swap operation. This is useful when elements have relationships that
/// need to be updated when their positions change.
///
/// # Type Parameters
///
/// * `T` - The element type
///
/// # Arguments
///
/// * `slice` - The mutable slice to sort
/// * `compare` - Comparison function returning `true` if first arg should come before second
/// * `on_swap` - Callback called with mutable references to both elements just before they are swapped
///
/// # Examples
///
/// ```
/// use wagyu_rs::bubble_sort::bubble_sort;
///
/// let mut data = vec![3, 1, 4, 1, 5];
/// let mut swap_count = 0;
///
/// bubble_sort(
///     &mut data,
///     |a, b| a < b,  // ascending order
///     |_a, _b| swap_count += 1,  // count swaps
/// );
///
/// assert_eq!(data, vec![1, 1, 3, 4, 5]);
/// assert!(swap_count > 0);
/// ```
///
/// # Complexity
///
/// - Time: O(n²) worst and average case, O(n) best case (already sorted)
/// - Space: O(1)
/// - Stable: Yes
pub fn bubble_sort<T, C, M>(slice: &mut [T], compare: C, mut on_swap: M)
where
    C: Fn(&T, &T) -> bool,
    M: FnMut(&mut T, &mut T),
{
    if slice.is_empty() {
        return;
    }

    let n = slice.len();
    if n <= 1 {
        return;
    }

    let last = n - 1;
    let mut modified = true;

    while modified {
        modified = false;
        for i in 0..last {
            // Swap if the NEXT element should come BEFORE the current element
            // Using compare(next, current) instead of !compare(current, next)
            // avoids infinite loops with equal elements.
            // If compare is "<", we swap when slice[i+1] < slice[i] (out of order)
            if compare(&slice[i + 1], &slice[i]) {
                // Call the callback before swapping
                // We need to split the slice to get two mutable references
                let (left, right) = slice.split_at_mut(i + 1);
                on_swap(&mut left[i], &mut right[0]);
                // Now swap them
                slice.swap(i, i + 1);
                modified = true;
            }
        }
    }
}

/// Convenience wrapper that sorts without a callback.
///
/// This is equivalent to `bubble_sort` with a no-op callback.
///
/// # Examples
///
/// ```
/// use wagyu_rs::bubble_sort::bubble_sort_simple;
///
/// let mut data = vec![3, 1, 4, 1, 5];
/// bubble_sort_simple(&mut data, |a, b| a < b);
/// assert_eq!(data, vec![1, 1, 3, 4, 5]);
/// ```
pub fn bubble_sort_simple<T, C>(slice: &mut [T], compare: C)
where
    C: Fn(&T, &T) -> bool,
{
    bubble_sort(slice, compare, |_, _| {})
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // Basic sorting tests
    // =========================================================================

    #[test]
    fn test_empty_slice() {
        let mut data: Vec<i32> = vec![];
        bubble_sort(&mut data, |a, b| a < b, |_, _| {});
        assert!(data.is_empty());
    }

    #[test]
    fn test_single_element() {
        let mut data = vec![42];
        bubble_sort(&mut data, |a, b| a < b, |_, _| {});
        assert_eq!(data, vec![42]);
    }

    #[test]
    fn test_two_elements_sorted() {
        let mut data = vec![1, 2];
        bubble_sort(&mut data, |a, b| a < b, |_, _| {});
        assert_eq!(data, vec![1, 2]);
    }

    #[test]
    fn test_two_elements_unsorted() {
        let mut data = vec![2, 1];
        bubble_sort(&mut data, |a, b| a < b, |_, _| {});
        assert_eq!(data, vec![1, 2]);
    }

    #[test]
    fn test_ascending_order() {
        let mut data = vec![3, 1, 4, 1, 5, 9, 2, 6];
        bubble_sort(&mut data, |a, b| a < b, |_, _| {});
        assert_eq!(data, vec![1, 1, 2, 3, 4, 5, 6, 9]);
    }

    #[test]
    fn test_descending_order() {
        let mut data = vec![3, 1, 4, 1, 5, 9, 2, 6];
        bubble_sort(&mut data, |a, b| a > b, |_, _| {});
        assert_eq!(data, vec![9, 6, 5, 4, 3, 2, 1, 1]);
    }

    #[test]
    fn test_already_sorted() {
        let mut data = vec![1, 2, 3, 4, 5];
        let mut swaps = 0;
        bubble_sort(&mut data, |a, b| a < b, |_, _| swaps += 1);
        assert_eq!(data, vec![1, 2, 3, 4, 5]);
        assert_eq!(swaps, 0); // No swaps needed
    }

    #[test]
    fn test_reverse_sorted() {
        let mut data = vec![5, 4, 3, 2, 1];
        bubble_sort(&mut data, |a, b| a < b, |_, _| {});
        assert_eq!(data, vec![1, 2, 3, 4, 5]);
    }

    // =========================================================================
    // Callback tests
    // =========================================================================

    #[test]
    fn test_swap_callback_is_called() {
        let mut data = vec![3, 1, 2];
        let mut swaps = 0;
        bubble_sort(&mut data, |a, b| a < b, |_, _| swaps += 1);
        assert_eq!(data, vec![1, 2, 3]);
        assert!(swaps > 0);
    }

    #[test]
    fn test_swap_callback_receives_correct_elements() {
        let mut data = vec![2, 1];
        let mut swapped_values = Vec::new();

        bubble_sort(
            &mut data,
            |a, b| a < b,
            |a, b| {
                swapped_values.push((*a, *b));
            },
        );

        assert_eq!(data, vec![1, 2]);
        // Before swap: a=2, b=1 (they're out of order)
        assert_eq!(swapped_values, vec![(2, 1)]);
    }

    #[test]
    fn test_callback_can_modify_elements() {
        // Use a struct that tracks if it was involved in a swap
        #[derive(Debug, PartialEq, Clone)]
        struct Item {
            value: i32,
            swapped: bool,
        }

        let mut data = vec![
            Item {
                value: 3,
                swapped: false,
            },
            Item {
                value: 1,
                swapped: false,
            },
            Item {
                value: 2,
                swapped: false,
            },
        ];

        bubble_sort(
            &mut data,
            |a, b| a.value < b.value,
            |a, b| {
                a.swapped = true;
                b.swapped = true;
            },
        );

        // All elements were involved in swaps
        assert!(data[0].value == 1);
        assert!(data[1].value == 2);
        assert!(data[2].value == 3);
        // At least some elements should be marked as swapped
        let swapped_count = data.iter().filter(|item| item.swapped).count();
        assert!(swapped_count > 0);
    }

    // =========================================================================
    // Stability tests
    // =========================================================================

    #[test]
    fn test_stability() {
        // Items with same sort key but different ids
        #[derive(Debug, Clone)]
        struct Item {
            key: i32,
            id: i32,
        }

        let mut data = vec![
            Item { key: 2, id: 0 },
            Item { key: 1, id: 1 },
            Item { key: 2, id: 2 },
            Item { key: 1, id: 3 },
        ];

        bubble_sort(&mut data, |a, b| a.key < b.key, |_, _| {});

        // Check that items with key=1 maintain relative order (id: 1, 3)
        let key_1_items: Vec<i32> = data.iter().filter(|i| i.key == 1).map(|i| i.id).collect();
        assert_eq!(key_1_items, vec![1, 3]);

        // Check that items with key=2 maintain relative order (id: 0, 2)
        let key_2_items: Vec<i32> = data.iter().filter(|i| i.key == 2).map(|i| i.id).collect();
        assert_eq!(key_2_items, vec![0, 2]);
    }

    // =========================================================================
    // Simple wrapper tests
    // =========================================================================

    #[test]
    fn test_bubble_sort_simple() {
        let mut data = vec![3, 1, 4, 1, 5];
        bubble_sort_simple(&mut data, |a, b| a < b);
        assert_eq!(data, vec![1, 1, 3, 4, 5]);
    }

    // =========================================================================
    // Edge cases
    // =========================================================================

    #[test]
    fn test_all_equal_elements() {
        let mut data = vec![5, 5, 5, 5, 5];
        let mut swaps = 0;
        bubble_sort(&mut data, |a, b| a < b, |_, _| swaps += 1);
        assert_eq!(data, vec![5, 5, 5, 5, 5]);
        assert_eq!(swaps, 0); // No swaps needed for equal elements
    }

    #[test]
    fn test_duplicates() {
        let mut data = vec![3, 1, 3, 1, 3];
        bubble_sort(&mut data, |a, b| a < b, |_, _| {});
        assert_eq!(data, vec![1, 1, 3, 3, 3]);
    }
}
