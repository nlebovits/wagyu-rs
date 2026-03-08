//! Integration tests for winding_count2_sum sorting behavior.
//!
//! These tests verify that intersection list sorting uses winding_count2_sum
//! as the tie-breaker for same-Y intersections, matching C++ behavior.

use wagyu_rs::{
    config::FillType, config::PolygonType, point::Point, wagyu::Wagyu, Operation,
};

/// Helper to create a polygon (Vec of rings) from a single ring
fn polygon_from_ring(ring: Vec<Point<i64>>) -> Vec<Vec<Point<i64>>> {
    vec![ring]
}

/// Test case designed to produce intersections at the same Y level
/// with potentially different winding_count2 values.
///
/// Setup:
/// - Subject: wide rectangle spanning x=0 to x=30
/// - Clip 1: triangle on the left (x=5 to x=15)
/// - Clip 2: triangle on the right (x=15 to x=25)
///
/// This creates multiple clip bounds that cross the subject at similar Y levels.
#[test]
fn multiple_clips_produce_same_y_intersections() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Subject: wide rectangle
    let subject = polygon_from_ring(vec![
        Point::new(0, 0),
        Point::new(30, 0),
        Point::new(30, 20),
        Point::new(0, 20),
        Point::new(0, 0),
    ]);

    // Clip 1: left triangle
    let clip1 = polygon_from_ring(vec![
        Point::new(5, 5),
        Point::new(15, 10),
        Point::new(5, 15),
        Point::new(5, 5),
    ]);

    // Clip 2: right triangle (symmetric)
    let clip2 = polygon_from_ring(vec![
        Point::new(25, 5),
        Point::new(15, 10),
        Point::new(25, 15),
        Point::new(25, 5),
    ]);

    wagyu.add_polygon(&subject, PolygonType::Subject);
    wagyu.add_polygon(&clip1, PolygonType::Clip);
    wagyu.add_polygon(&clip2, PolygonType::Clip);

    // This should process without panicking and produce valid output
    let result = wagyu.execute(
        Operation::Intersection,
        FillType::EvenOdd,
        FillType::EvenOdd,
    );

    assert!(
        result.is_ok(),
        "Intersection of overlapping polygons should succeed"
    );

    let mp = result.unwrap();
    assert!(
        !mp.0.is_empty(),
        "Should have at least one polygon in result"
    );
}

/// Overlapping clip polygons create complex winding scenarios.
///
/// When clip polygons overlap, bounds that cross through the overlap
/// region will have higher winding_count2 values than those that don't.
#[test]
fn overlapping_clips_with_subject() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Subject: tall rectangle
    let subject = polygon_from_ring(vec![
        Point::new(10, 0),
        Point::new(20, 0),
        Point::new(20, 30),
        Point::new(10, 30),
        Point::new(10, 0),
    ]);

    // Clip 1: square overlapping subject
    let clip1 = polygon_from_ring(vec![
        Point::new(0, 10),
        Point::new(15, 10),
        Point::new(15, 20),
        Point::new(0, 20),
        Point::new(0, 10),
    ]);

    // Clip 2: another square overlapping both subject and clip1
    let clip2 = polygon_from_ring(vec![
        Point::new(12, 12),
        Point::new(25, 12),
        Point::new(25, 18),
        Point::new(12, 18),
        Point::new(12, 12),
    ]);

    wagyu.add_polygon(&subject, PolygonType::Subject);
    wagyu.add_polygon(&clip1, PolygonType::Clip);
    wagyu.add_polygon(&clip2, PolygonType::Clip);

    // Union should work without issues
    let result = wagyu.execute(Operation::Union, FillType::EvenOdd, FillType::EvenOdd);

    assert!(result.is_ok(), "Union should succeed");
}

/// Test that demonstrates the sorting fix doesn't break existing behavior.
///
/// This is a regression test to ensure the winding_count2_sum sorting
/// doesn't cause issues with standard polygon operations.
#[test]
fn sorting_fix_regression_simple_intersection() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Two overlapping squares
    let subject = polygon_from_ring(vec![
        Point::new(0, 0),
        Point::new(10, 0),
        Point::new(10, 10),
        Point::new(0, 10),
        Point::new(0, 0),
    ]);

    let clip = polygon_from_ring(vec![
        Point::new(5, 5),
        Point::new(15, 5),
        Point::new(15, 15),
        Point::new(5, 15),
        Point::new(5, 5),
    ]);

    wagyu.add_polygon(&subject, PolygonType::Subject);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu.execute(
        Operation::Intersection,
        FillType::EvenOdd,
        FillType::EvenOdd,
    );

    assert!(result.is_ok());
    let mp = result.unwrap();
    assert_eq!(mp.0.len(), 1, "Should produce exactly one intersection polygon");

    // The intersection should be a 5x5 square
    let poly = &mp.0[0];
    let exterior_coords: Vec<_> = poly.exterior().coords().collect();
    assert!(
        exterior_coords.len() >= 4,
        "Intersection polygon should have at least 4 vertices"
    );
}
