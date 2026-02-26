//! Configuration types for wagyu boolean operations.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/config.hpp
//!
//! These enums define the operation modes, fill rules, and internal
//! edge state used by the Vatti clipping algorithm.

// NOTE: C++ defines `clip_type` here, but we use `Operation` in lib.rs instead.
// DIVERGENCE FROM WAGYU: We renamed `clip_type` to `Operation` for a more Rust-idiomatic API.
// The `Operation` enum is exported from lib.rs and used throughout the codebase.

/// Identifies which operand a polygon belongs to in a boolean operation.
///
/// From C++: `enum polygon_type : std::uint8_t { polygon_type_subject = 0, polygon_type_clip };`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum PolygonType {
    /// The subject polygon (first operand)
    #[default]
    Subject = 0,
    /// The clip polygon (second operand)
    Clip = 1,
}

/// Fill rule for determining polygon interior.
///
/// From C++: `enum fill_type : std::uint8_t { fill_type_even_odd = 0, fill_type_non_zero, fill_type_positive, fill_type_negative };`
///
/// See: <https://en.wikipedia.org/wiki/Nonzero-rule>
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum FillType {
    /// A point is inside if a ray crosses an odd number of edges
    #[default]
    EvenOdd = 0,
    /// A point is inside if the winding number is non-zero
    NonZero = 1,
    /// A point is inside if the winding number is positive
    Positive = 2,
    /// A point is inside if the winding number is negative
    Negative = 3,
}

/// Direction of horizontal edge traversal.
///
/// From C++: `enum horizontal_direction : std::uint8_t { right_to_left = 0, left_to_right = 1 };`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum HorizontalDirection {
    /// Processing from right to left (decreasing x)
    #[default]
    RightToLeft = 0,
    /// Processing from left to right (increasing x)
    LeftToRight = 1,
}

/// Which side of an edge we're on during processing.
///
/// From C++: `enum edge_side : std::uint8_t { edge_left = 0, edge_right };`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum EdgeSide {
    /// Left side of the edge
    #[default]
    Left = 0,
    /// Right side of the edge
    Right = 1,
}

/// Join style for offset operations.
///
/// From C++: `enum join_type : std::uint8_t { join_type_square = 0, join_type_round, join_type_miter };`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum JoinType {
    /// Square join
    #[default]
    Square = 0,
    /// Round join
    Round = 1,
    /// Miter join
    Miter = 2,
}

/// End cap style for open paths in offset operations.
///
/// From C++: `enum end_type { end_type_closed_polygon = 0, ..., end_type_open_round };`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum EndType {
    /// Closed polygon (no end caps)
    #[default]
    ClosedPolygon = 0,
    /// Closed line
    ClosedLine = 1,
    /// Open path with butt end caps
    OpenButt = 2,
    /// Open path with square end caps
    OpenSquare = 3,
    /// Open path with round end caps
    OpenRound = 4,
}

// ==================== Constants ====================
// From C++ config.hpp

/// Default arc tolerance for curve approximation
pub const DEF_ARC_TOLERANCE: f64 = 0.25;

/// Marker for an edge not currently owning a solution
pub const EDGE_UNASSIGNED: i32 = -1;

/// Marker for an edge that would otherwise close a path
pub const EDGE_SKIP: i32 = -2;

/// Low range threshold for coordinate validation
pub const LOW_RANGE: i64 = 0x3FFFFFFF;

/// High range threshold for coordinate validation
pub const HIGH_RANGE: i64 = 0x3FFFFFFFFFFFFFFF;

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== PolygonType Tests ====================

    #[test]
    fn polygon_type_has_subject_and_clip_variants() {
        // From C++: polygon_type_subject = 0, polygon_type_clip
        let subject = PolygonType::Subject;
        let clip = PolygonType::Clip;
        assert_ne!(subject, clip);
    }

    #[test]
    fn polygon_type_default_is_subject() {
        // Subject is the first variant (value 0 in C++)
        assert_eq!(PolygonType::default(), PolygonType::Subject);
    }

    #[test]
    fn polygon_type_is_copy_and_debug() {
        let pt = PolygonType::Subject;
        let pt_copy = pt; // Should compile - Copy
        assert_eq!(pt, pt_copy);
        let _debug = format!("{:?}", pt); // Should compile - Debug
    }

    // ==================== FillType Tests ====================

    #[test]
    fn fill_type_has_all_variants() {
        // From C++: fill_type_even_odd = 0, fill_type_non_zero, fill_type_positive, fill_type_negative
        let even_odd = FillType::EvenOdd;
        let non_zero = FillType::NonZero;
        let positive = FillType::Positive;
        let negative = FillType::Negative;

        // All should be distinct
        assert_ne!(even_odd, non_zero);
        assert_ne!(non_zero, positive);
        assert_ne!(positive, negative);
    }

    #[test]
    fn fill_type_default_is_even_odd() {
        // EvenOdd is the first variant (value 0 in C++)
        assert_eq!(FillType::default(), FillType::EvenOdd);
    }

    #[test]
    fn fill_type_is_copy_and_debug() {
        let ft = FillType::NonZero;
        let ft_copy = ft;
        assert_eq!(ft, ft_copy);
        let _debug = format!("{:?}", ft);
    }

    // ==================== HorizontalDirection Tests ====================

    #[test]
    fn horizontal_direction_has_both_variants() {
        // From C++: right_to_left = 0, left_to_right = 1
        let rtl = HorizontalDirection::RightToLeft;
        let ltr = HorizontalDirection::LeftToRight;
        assert_ne!(rtl, ltr);
    }

    #[test]
    fn horizontal_direction_default_is_right_to_left() {
        assert_eq!(
            HorizontalDirection::default(),
            HorizontalDirection::RightToLeft
        );
    }

    // ==================== EdgeSide Tests ====================

    #[test]
    fn edge_side_has_left_and_right() {
        // From C++: edge_left = 0, edge_right
        let left = EdgeSide::Left;
        let right = EdgeSide::Right;
        assert_ne!(left, right);
    }

    #[test]
    fn edge_side_default_is_left() {
        assert_eq!(EdgeSide::default(), EdgeSide::Left);
    }

    // ==================== JoinType Tests ====================

    #[test]
    fn join_type_has_all_variants() {
        // From C++: join_type_square = 0, join_type_round, join_type_miter
        let square = JoinType::Square;
        let round = JoinType::Round;
        let miter = JoinType::Miter;
        assert_ne!(square, round);
        assert_ne!(round, miter);
    }

    #[test]
    fn join_type_default_is_square() {
        assert_eq!(JoinType::default(), JoinType::Square);
    }

    // ==================== EndType Tests ====================

    #[test]
    fn end_type_has_all_variants() {
        // From C++: end_type_closed_polygon = 0, ..., end_type_open_round
        let closed_polygon = EndType::ClosedPolygon;
        let closed_line = EndType::ClosedLine;
        let open_butt = EndType::OpenButt;
        let open_square = EndType::OpenSquare;
        let open_round = EndType::OpenRound;

        // All should be distinct
        assert_ne!(closed_polygon, closed_line);
        assert_ne!(closed_line, open_butt);
        assert_ne!(open_butt, open_square);
        assert_ne!(open_square, open_round);
    }

    #[test]
    fn end_type_default_is_closed_polygon() {
        assert_eq!(EndType::default(), EndType::ClosedPolygon);
    }

    // ==================== Constants Tests ====================

    #[test]
    fn constants_match_cpp_values() {
        // From C++:
        // static double const def_arc_tolerance = 0.25;
        // static int const EDGE_UNASSIGNED = -1;
        // static int const EDGE_SKIP = -2;
        // static std::int64_t const LOW_RANGE = 0x3FFFFFFF;
        // static std::int64_t const HIGH_RANGE = 0x3FFFFFFFFFFFFFFFLL;

        assert!((DEF_ARC_TOLERANCE - 0.25).abs() < f64::EPSILON);
        assert_eq!(EDGE_UNASSIGNED, -1);
        assert_eq!(EDGE_SKIP, -2);
        assert_eq!(LOW_RANGE, 0x3FFFFFFF_i64);
        assert_eq!(HIGH_RANGE, 0x3FFFFFFFFFFFFFFF_i64);
    }
}
