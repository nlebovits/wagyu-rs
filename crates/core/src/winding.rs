//! Winding count calculation and contribution logic.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/active_bound_list.hpp
//!
//! This module contains functions for calculating winding counts based on
//! the active edge list, and determining whether a bound contributes to
//! the output based on fill rules and operation type.

use crate::bound::Bound;
use crate::config::{FillType, PolygonType};
use crate::Operation;
use geo_types::CoordNum;

// ============================================================================
// Helper functions for fill type determination
// ============================================================================

/// Check if a bound uses even-odd fill type.
///
/// From C++: `is_even_odd_fill_type(bound<T> const& bound, fill_type subject_fill_type, fill_type clip_fill_type)`
pub fn is_even_odd_fill_type(
    bound: &Bound<impl CoordNum>,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> bool {
    if bound.poly_type == PolygonType::Subject {
        subject_fill_type == FillType::EvenOdd
    } else {
        clip_fill_type == FillType::EvenOdd
    }
}

/// Check if a bound uses even-odd fill type for the alternate polygon type.
///
/// From C++: `is_even_odd_alt_fill_type(bound<T> const& bound, fill_type subject_fill_type, fill_type clip_fill_type)`
pub fn is_even_odd_alt_fill_type(
    bound: &Bound<impl CoordNum>,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> bool {
    if bound.poly_type == PolygonType::Subject {
        clip_fill_type == FillType::EvenOdd
    } else {
        subject_fill_type == FillType::EvenOdd
    }
}

// ============================================================================
// set_winding_count
// ============================================================================

/// Set the winding count for a bound based on the bounds that precede it in the AEL.
///
/// From C++: `set_winding_count(active_bound_list_itr<T> bnd_itr, active_bound_list<T>& active_bounds, ...)`
///
/// This function calculates:
/// 1. `winding_count` - The winding count for this polygon type
/// 2. `winding_count2` - The winding count for the other polygon type
///
/// # Arguments
///
/// * `bound_position` - Position of the bound in the AEL
/// * `ael_indices` - The active edge list indices
/// * `bounds` - The bounds storage
/// * `subject_fill_type` - Fill rule for subject polygons
/// * `clip_fill_type` - Fill rule for clip polygons
pub fn set_winding_count<T: CoordNum>(
    bound_position: usize,
    ael_indices: &[usize],
    bounds: &mut [Bound<T>],
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) {
    let bound_index = ael_indices[bound_position];
    let bnd_poly_type = bounds[bound_index].poly_type;
    let bnd_winding_delta = bounds[bound_index].winding_delta;

    // Check if this is the first bound (no predecessors)
    if bound_position == 0 {
        bounds[bound_index].winding_count = bnd_winding_delta;
        bounds[bound_index].winding_count2 = 0;
        return;
    }

    // Find the edge of the same polytype that immediately precedes this bound in AEL
    let mut same_type_prev_pos: Option<usize> = None;
    for pos in (0..bound_position).rev() {
        let idx = ael_indices[pos];
        if bounds[idx].poly_type == bnd_poly_type {
            same_type_prev_pos = Some(pos);
            break;
        }
    }

    // Determine if this bound uses even-odd fill type
    let is_even_odd = if bnd_poly_type == PolygonType::Subject {
        subject_fill_type == FillType::EvenOdd
    } else {
        clip_fill_type == FillType::EvenOdd
    };

    // Calculate winding_count
    match same_type_prev_pos {
        None => {
            // No same-type predecessor
            bounds[bound_index].winding_count = bnd_winding_delta;
            bounds[bound_index].winding_count2 = 0;
        }
        Some(prev_pos) => {
            let prev_idx = ael_indices[prev_pos];
            if is_even_odd {
                // EvenOdd filling: winding_count = winding_delta, copy winding_count2
                bounds[bound_index].winding_count = bnd_winding_delta;
                bounds[bound_index].winding_count2 = bounds[prev_idx].winding_count2;
            } else {
                // NonZero, Positive, or Negative filling
                let prev_winding_count = bounds[prev_idx].winding_count;
                let prev_winding_delta = bounds[prev_idx].winding_delta;

                if prev_winding_count * prev_winding_delta < 0 {
                    // Prev edge is "decreasing" WindCount toward zero
                    // So we're outside the previous polygon
                    if prev_winding_count.abs() > 1 {
                        // Outside prev poly but still inside another
                        if prev_winding_delta * bnd_winding_delta < 0 {
                            // When reversing direction of prev poly, use same WC
                            bounds[bound_index].winding_count = prev_winding_count;
                        } else {
                            // Otherwise continue to "decrease" WC
                            bounds[bound_index].winding_count =
                                prev_winding_count + bnd_winding_delta;
                        }
                    } else {
                        // Now outside all polys of same polytype, set own WC
                        bounds[bound_index].winding_count = bnd_winding_delta;
                    }
                } else {
                    // Prev edge is "increasing" WindCount away from zero
                    // So we're inside the previous polygon
                    if prev_winding_delta * bnd_winding_delta < 0 {
                        // Wind direction is reversing, use same WC
                        bounds[bound_index].winding_count = prev_winding_count;
                    } else {
                        // Add to WC
                        bounds[bound_index].winding_count =
                            prev_winding_count + bnd_winding_delta;
                    }
                }
                bounds[bound_index].winding_count2 = bounds[prev_idx].winding_count2;
            }
        }
    }

    // Update winding_count2 by iterating from same_type_prev to this bound
    // and accumulating winding_deltas of alternate type bounds
    let start_pos = same_type_prev_pos.map(|p| p + 1).unwrap_or(0);

    // Determine if this bound uses even-odd alt fill type
    let is_even_odd_alt = if bnd_poly_type == PolygonType::Subject {
        clip_fill_type == FillType::EvenOdd
    } else {
        subject_fill_type == FillType::EvenOdd
    };

    if is_even_odd_alt {
        // EvenOdd filling for alt type: toggle winding_count2
        for &idx in ael_indices.iter().take(bound_position).skip(start_pos) {
            if bounds[idx].poly_type != bnd_poly_type {
                bounds[bound_index].winding_count2 = if bounds[bound_index].winding_count2 == 0 {
                    1
                } else {
                    0
                };
            }
        }
    } else {
        // NonZero, Positive, or Negative filling: accumulate winding_delta
        for &idx in ael_indices.iter().take(bound_position).skip(start_pos) {
            if bounds[idx].poly_type != bnd_poly_type {
                bounds[bound_index].winding_count2 += bounds[idx].winding_delta;
            }
        }
    }
}

// ============================================================================
// is_contributing
// ============================================================================

/// Determine if a bound contributes to the output polygon.
///
/// From C++: `is_contributing(bound<T> const& bnd, clip_type cliptype, fill_type subject_fill_type, fill_type clip_fill_type)`
///
/// A bound contributes based on:
/// 1. Its winding count and the fill rule for its polygon type
/// 2. The winding count for the other polygon type
/// 3. The type of boolean operation being performed
///
/// # Arguments
///
/// * `bound` - The bound to check
/// * `operation` - The boolean operation type
/// * `subject_fill_type` - Fill rule for subject polygons
/// * `clip_fill_type` - Fill rule for clip polygons
///
/// # Returns
///
/// `true` if the bound contributes to the output, `false` otherwise.
pub fn is_contributing<T: CoordNum>(
    bound: &Bound<T>,
    operation: Operation,
    subject_fill_type: FillType,
    clip_fill_type: FillType,
) -> bool {
    // Determine fill types based on polygon type
    let (pft, pft2) = if bound.poly_type == PolygonType::Subject {
        (subject_fill_type, clip_fill_type)
    } else {
        (clip_fill_type, subject_fill_type)
    };

    // First check: does the winding count satisfy the fill rule for this polygon type?
    match pft {
        FillType::EvenOdd => {
            // EvenOdd: any non-zero winding count contributes
            // (the actual check is implicit in the operation logic below)
        }
        FillType::NonZero => {
            if bound.winding_count.abs() != 1 {
                return false;
            }
        }
        FillType::Positive => {
            if bound.winding_count != 1 {
                return false;
            }
        }
        FillType::Negative => {
            if bound.winding_count != -1 {
                return false;
            }
        }
    }

    // Second check: based on operation and winding_count2
    match operation {
        Operation::Intersection => match pft2 {
            FillType::EvenOdd | FillType::NonZero => bound.winding_count2 != 0,
            FillType::Positive => bound.winding_count2 > 0,
            FillType::Negative => bound.winding_count2 < 0,
        },
        Operation::Union => match pft2 {
            FillType::EvenOdd | FillType::NonZero => bound.winding_count2 == 0,
            FillType::Positive => bound.winding_count2 <= 0,
            FillType::Negative => bound.winding_count2 >= 0,
        },
        Operation::Difference => {
            if bound.poly_type == PolygonType::Subject {
                // Subject: contributes when outside clip
                match pft2 {
                    FillType::EvenOdd | FillType::NonZero => bound.winding_count2 == 0,
                    FillType::Positive => bound.winding_count2 <= 0,
                    FillType::Negative => bound.winding_count2 >= 0,
                }
            } else {
                // Clip: contributes when inside subject (creates hole)
                match pft2 {
                    FillType::EvenOdd | FillType::NonZero => bound.winding_count2 != 0,
                    FillType::Positive => bound.winding_count2 > 0,
                    FillType::Negative => bound.winding_count2 < 0,
                }
            }
        }
        Operation::Xor => true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bound::Edge;
    use crate::config::EdgeSide;
    use crate::point::Point;

    // ==================== Helper Functions ====================

    fn make_bound_with_delta(poly_type: PolygonType, winding_delta: i32) -> Bound<f64> {
        let edge = Edge::new(Point::new(0.0_f64, 0.0), Point::new(10.0_f64, 10.0));
        Bound::new_with_delta(vec![edge], poly_type, EdgeSide::Left, winding_delta)
    }

    fn make_subject_bound(winding_delta: i32) -> Bound<f64> {
        make_bound_with_delta(PolygonType::Subject, winding_delta)
    }

    fn make_clip_bound(winding_delta: i32) -> Bound<f64> {
        make_bound_with_delta(PolygonType::Clip, winding_delta)
    }

    // ==================== is_even_odd_fill_type Tests ====================

    #[test]
    fn is_even_odd_fill_type_subject_with_even_odd() {
        let bound = make_subject_bound(1);
        assert!(is_even_odd_fill_type(
            &bound,
            FillType::EvenOdd,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_even_odd_fill_type_subject_with_non_zero() {
        let bound = make_subject_bound(1);
        assert!(!is_even_odd_fill_type(
            &bound,
            FillType::NonZero,
            FillType::EvenOdd
        ));
    }

    #[test]
    fn is_even_odd_fill_type_clip_with_even_odd() {
        let bound = make_clip_bound(1);
        assert!(is_even_odd_fill_type(
            &bound,
            FillType::NonZero,
            FillType::EvenOdd
        ));
    }

    #[test]
    fn is_even_odd_fill_type_clip_with_non_zero() {
        let bound = make_clip_bound(1);
        assert!(!is_even_odd_fill_type(
            &bound,
            FillType::EvenOdd,
            FillType::NonZero
        ));
    }

    // ==================== is_even_odd_alt_fill_type Tests ====================

    #[test]
    fn is_even_odd_alt_fill_type_subject_checks_clip_fill() {
        // For subject bounds, alt fill type is the clip fill type
        let bound = make_subject_bound(1);
        assert!(is_even_odd_alt_fill_type(
            &bound,
            FillType::NonZero,
            FillType::EvenOdd
        ));
        assert!(!is_even_odd_alt_fill_type(
            &bound,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_even_odd_alt_fill_type_clip_checks_subject_fill() {
        // For clip bounds, alt fill type is the subject fill type
        let bound = make_clip_bound(1);
        assert!(is_even_odd_alt_fill_type(
            &bound,
            FillType::EvenOdd,
            FillType::NonZero
        ));
        assert!(!is_even_odd_alt_fill_type(
            &bound,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    // ==================== set_winding_count Tests ====================

    #[test]
    fn set_winding_count_first_bound_in_ael() {
        // First bound in AEL: winding_count = winding_delta, winding_count2 = 0
        let mut bounds = vec![make_subject_bound(1)];
        let ael_indices = vec![0];

        set_winding_count(
            0,
            &ael_indices,
            &mut bounds,
            FillType::NonZero,
            FillType::NonZero,
        );

        assert_eq!(bounds[0].winding_count, 1);
        assert_eq!(bounds[0].winding_count2, 0);
    }

    #[test]
    fn set_winding_count_first_bound_negative_delta() {
        // First bound with negative winding delta
        let mut bounds = vec![make_subject_bound(-1)];
        let ael_indices = vec![0];

        set_winding_count(
            0,
            &ael_indices,
            &mut bounds,
            FillType::NonZero,
            FillType::NonZero,
        );

        assert_eq!(bounds[0].winding_count, -1);
        assert_eq!(bounds[0].winding_count2, 0);
    }

    #[test]
    fn set_winding_count_second_subject_bound_non_zero() {
        // Second subject bound after a subject bound with non-zero fill
        // Should accumulate winding counts
        let mut bounds = vec![
            {
                let mut b = make_subject_bound(1);
                b.winding_count = 1;
                b.winding_count2 = 0;
                b
            },
            make_subject_bound(1),
        ];
        let ael_indices = vec![0, 1];

        set_winding_count(
            1,
            &ael_indices,
            &mut bounds,
            FillType::NonZero,
            FillType::NonZero,
        );

        // With non-zero fill and same poly_type, winding_count accumulates
        assert_eq!(bounds[1].winding_count, 2);
        assert_eq!(bounds[1].winding_count2, 0);
    }

    #[test]
    fn set_winding_count_second_subject_bound_even_odd() {
        // Second subject bound with even-odd fill
        // Should use winding_delta, not accumulate
        let mut bounds = vec![
            {
                let mut b = make_subject_bound(1);
                b.winding_count = 1;
                b.winding_count2 = 0;
                b
            },
            make_subject_bound(1),
        ];
        let ael_indices = vec![0, 1];

        set_winding_count(
            1,
            &ael_indices,
            &mut bounds,
            FillType::EvenOdd,
            FillType::NonZero,
        );

        // With even-odd fill, winding_count = winding_delta
        assert_eq!(bounds[1].winding_count, 1);
        assert_eq!(bounds[1].winding_count2, 0);
    }

    #[test]
    fn set_winding_count_subject_after_clip_non_zero() {
        // Subject bound after clip bound - should set winding_count2
        let mut bounds = vec![
            {
                let mut b = make_clip_bound(1);
                b.winding_count = 1;
                b.winding_count2 = 0;
                b
            },
            make_subject_bound(1),
        ];
        let ael_indices = vec![0, 1];

        set_winding_count(
            1,
            &ael_indices,
            &mut bounds,
            FillType::NonZero,
            FillType::NonZero,
        );

        // Subject bound's winding_count = winding_delta (no same-type predecessor)
        assert_eq!(bounds[1].winding_count, 1);
        // winding_count2 should accumulate the clip's winding_delta
        assert_eq!(bounds[1].winding_count2, 1);
    }

    #[test]
    fn set_winding_count_clip_after_subject_non_zero() {
        // Clip bound after subject bound
        let mut bounds = vec![
            {
                let mut b = make_subject_bound(1);
                b.winding_count = 1;
                b.winding_count2 = 0;
                b
            },
            make_clip_bound(1),
        ];
        let ael_indices = vec![0, 1];

        set_winding_count(
            1,
            &ael_indices,
            &mut bounds,
            FillType::NonZero,
            FillType::NonZero,
        );

        // Clip bound's winding_count = winding_delta (no same-type predecessor)
        assert_eq!(bounds[1].winding_count, 1);
        // winding_count2 should accumulate the subject's winding_delta
        assert_eq!(bounds[1].winding_count2, 1);
    }

    #[test]
    fn set_winding_count_subject_after_two_clips_even_odd_alt() {
        // Subject bound after two clip bounds with even-odd fill for clips
        let mut bounds = vec![
            {
                let mut b = make_clip_bound(1);
                b.winding_count = 1;
                b.winding_count2 = 0;
                b
            },
            {
                let mut b = make_clip_bound(1);
                b.winding_count = 1;
                b.winding_count2 = 0;
                b
            },
            make_subject_bound(1),
        ];
        let ael_indices = vec![0, 1, 2];

        // Subject uses NonZero, Clip uses EvenOdd
        // For subject, alt fill type is clip fill type (EvenOdd)
        set_winding_count(
            2,
            &ael_indices,
            &mut bounds,
            FillType::NonZero,
            FillType::EvenOdd,
        );

        assert_eq!(bounds[2].winding_count, 1);
        // With even-odd alt fill, winding_count2 should toggle: 0 -> 1 -> 0
        assert_eq!(bounds[2].winding_count2, 0);
    }

    #[test]
    fn set_winding_count_opposite_winding_deltas() {
        // Two subject bounds with opposite winding deltas
        let mut bounds = vec![
            {
                let mut b = make_subject_bound(1);
                b.winding_count = 1;
                b.winding_count2 = 0;
                b
            },
            make_subject_bound(-1), // Opposite direction
        ];
        let ael_indices = vec![0, 1];

        set_winding_count(
            1,
            &ael_indices,
            &mut bounds,
            FillType::NonZero,
            FillType::NonZero,
        );

        // When deltas are opposite and prev is "decreasing" toward zero
        // (winding_count * winding_delta > 0 means "increasing")
        // winding_count=1, delta=1 -> 1*1 > 0, so prev is "increasing"
        // When prev is increasing and new delta is opposite, use same WC
        assert_eq!(bounds[1].winding_count, 1);
    }

    // ==================== is_contributing Tests ====================

    #[test]
    fn is_contributing_xor_always_returns_true() {
        // XOR operation always contributes
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 0;

        assert!(is_contributing(
            &bound,
            Operation::Xor,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_union_non_zero_inside_other() {
        // Union: bound is inside other polygon (winding_count2 != 0)
        // Should NOT contribute
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 1; // Inside clip polygon

        assert!(!is_contributing(
            &bound,
            Operation::Union,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_union_non_zero_outside_other() {
        // Union: bound is outside other polygon (winding_count2 == 0)
        // Should contribute
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 0; // Outside clip polygon

        assert!(is_contributing(
            &bound,
            Operation::Union,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_intersection_non_zero_inside_other() {
        // Intersection: bound is inside other polygon (winding_count2 != 0)
        // Should contribute
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 1; // Inside clip polygon

        assert!(is_contributing(
            &bound,
            Operation::Intersection,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_intersection_non_zero_outside_other() {
        // Intersection: bound is outside other polygon (winding_count2 == 0)
        // Should NOT contribute
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 0; // Outside clip polygon

        assert!(!is_contributing(
            &bound,
            Operation::Intersection,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_difference_subject_outside_clip() {
        // Difference: subject bound outside clip (winding_count2 == 0)
        // Should contribute
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 0;

        assert!(is_contributing(
            &bound,
            Operation::Difference,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_difference_subject_inside_clip() {
        // Difference: subject bound inside clip (winding_count2 != 0)
        // Should NOT contribute
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 1;

        assert!(!is_contributing(
            &bound,
            Operation::Difference,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_difference_clip_inside_subject() {
        // Difference: clip bound inside subject (winding_count2 != 0)
        // Should contribute (cuts a hole)
        let mut bound = make_clip_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 1;

        assert!(is_contributing(
            &bound,
            Operation::Difference,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_difference_clip_outside_subject() {
        // Difference: clip bound outside subject (winding_count2 == 0)
        // Should NOT contribute
        let mut bound = make_clip_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 0;

        assert!(!is_contributing(
            &bound,
            Operation::Difference,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_non_zero_fill_winding_count_not_one() {
        // Non-zero fill: only winding count of 1 or -1 contributes
        let mut bound = make_subject_bound(1);
        bound.winding_count = 2; // Not 1 or -1
        bound.winding_count2 = 0;

        assert!(!is_contributing(
            &bound,
            Operation::Union,
            FillType::NonZero,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_even_odd_fill_any_winding_count() {
        // Even-odd fill: any odd winding count contributes (handled as toggle)
        let mut bound = make_subject_bound(1);
        bound.winding_count = 3; // Odd number
        bound.winding_count2 = 0;

        // With even-odd, the contribution logic is simplified
        assert!(is_contributing(
            &bound,
            Operation::Union,
            FillType::EvenOdd,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_positive_fill_only_positive_winding() {
        // Positive fill: only positive winding count contributes
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 0;

        assert!(is_contributing(
            &bound,
            Operation::Union,
            FillType::Positive,
            FillType::NonZero
        ));

        bound.winding_count = -1;
        assert!(!is_contributing(
            &bound,
            Operation::Union,
            FillType::Positive,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_negative_fill_only_negative_winding() {
        // Negative fill: only negative winding count contributes
        let mut bound = make_subject_bound(1);
        bound.winding_count = -1;
        bound.winding_count2 = 0;

        assert!(is_contributing(
            &bound,
            Operation::Union,
            FillType::Negative,
            FillType::NonZero
        ));

        bound.winding_count = 1;
        assert!(!is_contributing(
            &bound,
            Operation::Union,
            FillType::Negative,
            FillType::NonZero
        ));
    }

    #[test]
    fn is_contributing_union_positive_alt_fill() {
        // Union with positive fill for alt polygon type
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = 1; // Positive winding for clip

        // Subject uses NonZero, Clip uses Positive
        // For union, contributes when winding_count2 <= 0
        assert!(!is_contributing(
            &bound,
            Operation::Union,
            FillType::NonZero,
            FillType::Positive
        ));

        bound.winding_count2 = 0;
        assert!(is_contributing(
            &bound,
            Operation::Union,
            FillType::NonZero,
            FillType::Positive
        ));

        bound.winding_count2 = -1;
        assert!(is_contributing(
            &bound,
            Operation::Union,
            FillType::NonZero,
            FillType::Positive
        ));
    }

    #[test]
    fn is_contributing_intersection_negative_alt_fill() {
        // Intersection with negative fill for alt polygon type
        let mut bound = make_subject_bound(1);
        bound.winding_count = 1;
        bound.winding_count2 = -1; // Negative winding for clip

        // Subject uses NonZero, Clip uses Negative
        // For intersection, contributes when winding_count2 < 0
        assert!(is_contributing(
            &bound,
            Operation::Intersection,
            FillType::NonZero,
            FillType::Negative
        ));

        bound.winding_count2 = 0;
        assert!(!is_contributing(
            &bound,
            Operation::Intersection,
            FillType::NonZero,
            FillType::Negative
        ));
    }
}
