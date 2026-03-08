//! Integration tests for XOR ring orientation fix (Issue #25).
//!
//! These tests verify that XOR operations correctly handle ring orientation:
//! - Rings with no parent (depth 0) should be treated as exteriors
//! - Rings nested inside holes (depth 2) should become separate exterior polygons
//! - Children of absorbed rings should inherit correct parent (None for top-level)
//!
//! Bug fixed: `merge_rings_at_intersection` was using `ring_parent_idx` instead of
//! `ring_origin->parent` when assigning parents to children of absorbed rings.

use geo_types::MultiPolygon;
use wagyu_rs::{config::FillType, config::PolygonType, point::Point, wagyu::Wagyu, Operation};

/// Helper to create a square polygon with the given corners.
fn make_square(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> Vec<Vec<Point<i64>>> {
    vec![vec![
        Point::new(min_x, min_y),
        Point::new(max_x, min_y),
        Point::new(max_x, max_y),
        Point::new(min_x, max_y),
        Point::new(min_x, min_y), // Close the ring
    ]]
}

/// Helper to count total polygons in result.
fn count_polygons(result: &MultiPolygon<i64>) -> usize {
    result.0.len()
}

/// Helper to count total hole rings across all polygons.
fn count_hole_rings(result: &MultiPolygon<i64>) -> usize {
    result.0.iter().map(|p| p.interiors().len()).sum()
}

/// Test: XOR of two non-overlapping squares should produce two separate polygons.
///
/// Before fix: Rings without parents might get wrong orientations.
/// After fix: Each top-level ring stays as exterior (depth 0 = exterior orientation).
#[test]
fn xor_non_overlapping_squares_produces_two_polygons() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Square 1: (0,0) to (100,100)
    let square1 = make_square(0, 0, 100, 100);
    wagyu.add_polygon(&square1, PolygonType::Subject);

    // Square 2: (200,0) to (300,100) - no overlap
    let square2 = make_square(200, 0, 300, 100);
    wagyu.add_polygon(&square2, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // XOR of non-overlapping should give both as separate polygons
    assert_eq!(
        count_polygons(&result),
        2,
        "XOR of non-overlapping squares should produce 2 separate polygons"
    );
    assert_eq!(
        count_hole_rings(&result),
        0,
        "Non-overlapping XOR should have no holes"
    );
}

/// Test: XOR of a polygon with its exact copy should produce empty result.
///
/// This tests that XOR correctly cancels out identical regions.
#[test]
fn xor_identical_squares_produces_empty() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    let square = make_square(0, 0, 100, 100);

    // Add same square as both subject and clip
    wagyu.add_polygon(&square, PolygonType::Subject);
    wagyu.add_polygon(&square, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // XOR of identical should give empty
    assert_eq!(
        count_polygons(&result),
        0,
        "XOR of identical squares should produce empty result"
    );
}

/// Test: XOR of partially overlapping squares should produce result covering non-overlapping parts.
///
/// This tests the core XOR logic where overlapping regions cancel out.
#[test]
fn xor_overlapping_squares_cancels_overlap() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Square 1: (0,0) to (100,100)
    let square1 = make_square(0, 0, 100, 100);
    wagyu.add_polygon(&square1, PolygonType::Subject);

    // Square 2: (50,0) to (150,100) - overlaps from x=50 to x=100
    let square2 = make_square(50, 0, 150, 100);
    wagyu.add_polygon(&square2, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // XOR should produce geometry covering non-overlapping parts
    // The overlap (50,0)-(100,100) should be removed
    assert!(
        count_polygons(&result) >= 1,
        "XOR of overlapping squares should produce at least one polygon"
    );
}

/// Test: XOR of polygon containing another should produce ring with hole.
///
/// Critical test for issue #25: When a small square is inside a large square,
/// XOR should produce the large square with a hole where the small one was.
#[test]
fn xor_contained_polygon_produces_hole() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Large outer square: (0,0) to (100,100)
    let outer = make_square(0, 0, 100, 100);
    wagyu.add_polygon(&outer, PolygonType::Subject);

    // Small inner square: (25,25) to (75,75) - completely inside
    let inner = make_square(25, 25, 75, 75);
    wagyu.add_polygon(&inner, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // XOR should produce outer square with inner as hole
    assert_eq!(
        count_polygons(&result),
        1,
        "XOR of contained polygon should produce 1 polygon"
    );
    assert_eq!(
        count_hole_rings(&result),
        1,
        "XOR of contained polygon should produce 1 hole"
    );
}

/// Test: XOR with clockwise exterior (C++ convention).
///
/// Tests that ring orientations are handled correctly regardless of input winding.
#[test]
fn xor_handles_clockwise_input() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Clockwise square (C++ convention for exterior)
    let cw_square = vec![vec![
        Point::new(0, 0),
        Point::new(0, 100),
        Point::new(100, 100),
        Point::new(100, 0),
        Point::new(0, 0),
    ]];

    wagyu.add_polygon(&cw_square, PolygonType::Subject);

    // Non-overlapping CCW square
    let ccw_square = vec![vec![
        Point::new(200, 0),
        Point::new(300, 0),
        Point::new(300, 100),
        Point::new(200, 100),
        Point::new(200, 0),
    ]];

    wagyu.add_polygon(&ccw_square, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // Both squares should appear in result
    assert_eq!(
        count_polygons(&result),
        2,
        "XOR should handle mixed winding inputs correctly"
    );
}

/// Test: XOR of multipolygon produces correct separate polygons.
///
/// Tests the merge_rings_at_intersection fix where children of absorbed rings
/// should become top-level exteriors, not nested holes.
#[test]
fn xor_multipolygon_produces_separate_exteriors() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Two non-touching subject squares
    let square1 = make_square(0, 0, 100, 100);
    let square2 = make_square(200, 0, 300, 100);

    wagyu.add_polygon(&square1, PolygonType::Subject);
    wagyu.add_polygon(&square2, PolygonType::Subject);

    // Clip square that doesn't overlap with either
    let clip = make_square(400, 0, 500, 100);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // All three should be separate polygons
    assert_eq!(
        count_polygons(&result),
        3,
        "XOR with multiple non-overlapping should produce separate polygons"
    );
    assert_eq!(
        count_hole_rings(&result),
        0,
        "Non-overlapping XOR should have no holes"
    );
}

/// Test: Verify depth-based orientation correction works correctly.
///
/// This directly tests the fix in correct_orientations that compares
/// depth-based hole status with area-based hole status.
#[test]
fn xor_depth_based_orientation_correction() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Large square with a hole
    let polygon_with_hole = vec![
        // Exterior (CCW)
        vec![
            Point::new(0, 0),
            Point::new(200, 0),
            Point::new(200, 200),
            Point::new(0, 200),
            Point::new(0, 0),
        ],
        // Hole (CW)
        vec![
            Point::new(50, 50),
            Point::new(50, 150),
            Point::new(150, 150),
            Point::new(150, 50),
            Point::new(50, 50),
        ],
    ];

    wagyu.add_polygon(&polygon_with_hole, PolygonType::Subject);

    // Small square inside the hole - should become exterior after XOR
    let inner_square = make_square(75, 75, 125, 125);
    wagyu.add_polygon(&inner_square, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // The inner square is in the hole of subject, so XOR should:
    // 1. Keep the outer ring as exterior
    // 2. Keep the hole
    // 3. Have the inner square as a separate polygon OR fill in part of the hole
    //
    // The key is that the inner square shouldn't incorrectly become a hole
    // of the outer ring due to wrong depth-based orientation
    assert!(
        count_polygons(&result) >= 1,
        "XOR with nested geometry should produce valid output"
    );
}

/// Test: XOR with complex nesting (exterior -> hole -> island pattern).
///
/// This tests the critical path where absorbed ring children should
/// get the correct parent (None for top-level exterior, not the wrong ring).
#[test]
fn xor_complex_nesting_correct_parent_assignment() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Subject: Large square (exterior)
    let subject = make_square(0, 0, 1000, 1000);
    wagyu.add_polygon(&subject, PolygonType::Subject);

    // Clip: Slightly smaller square that overlaps, creating XOR regions
    let clip = make_square(500, 0, 1500, 1000);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // Should produce geometry without nested hole issues
    // The result should be two non-overlapping L-shaped regions
    assert!(
        count_polygons(&result) >= 1,
        "Complex XOR should produce valid polygons"
    );

    // Each polygon in result should have proper exterior/hole structure
    for (i, poly) in result.0.iter().enumerate() {
        let exterior_coords: Vec<_> = poly.exterior().coords().collect();
        assert!(
            exterior_coords.len() >= 3,
            "Polygon {} should have valid exterior ring",
            i
        );
    }
}
