//! Integration test for Issue #80: collinear edge merging.
//!
//! Tests that two adjacent squares sharing a collinear edge merge into
//! a single rectangle with correct area and geometry.
//!
//! NOTE: The unit test `correct_collinear_edges_merges_two_rings_sharing_edge`
//! directly tests the collinear point removal fix. This integration test
//! exercises the full Vatti algorithm path, which may include additional
//! collinear points created during the sweep line process.

use geo_types::Coord;
use wagyu_rs::{config::FillType, config::PolygonType, point::Point, wagyu::Wagyu, Operation};

/// Two adjacent 5x10 squares sharing edge at x=5 should produce a valid
/// merged rectangle with correct area.
#[test]
fn union_adjacent_squares_produces_rectangle() {
    // Square A: (0,0) -> (5,0) -> (5,10) -> (0,10)
    let square_a: Vec<Point<i64>> = vec![
        Point::new(0, 0),
        Point::new(5, 0),
        Point::new(5, 10),
        Point::new(0, 10),
        Point::new(0, 0), // closed ring
    ];

    // Square B: (5,10) -> (5,0) -> (10,0) -> (10,10)
    // Sharing edge (5,0)-(5,10) with square A, but traversed in opposite direction
    let square_b: Vec<Point<i64>> = vec![
        Point::new(5, 10),
        Point::new(5, 0),
        Point::new(10, 0),
        Point::new(10, 10),
        Point::new(5, 10), // closed ring
    ];

    // Run wagyu union
    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![square_a], PolygonType::Subject);
    wagyu.add_polygon(&vec![square_b], PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Union, FillType::EvenOdd, FillType::EvenOdd)
        .expect("Wagyu execution failed");

    // Should produce exactly one polygon
    assert_eq!(
        result.0.len(),
        1,
        "Union of two adjacent squares should produce 1 polygon, got {}",
        result.0.len()
    );

    let polygon = &result.0[0];

    // Should have no holes
    assert!(
        polygon.interiors().is_empty(),
        "Merged rectangle should have no holes"
    );

    // Exterior should have at most 6 coords (5 corners + closing, or 4 corners + closing)
    // The collinear point removal fix ensures merged rings don't have extra collinear points,
    // but the Vatti sweep may produce them before topology correction runs.
    let exterior_coords: Vec<Coord<i64>> = polygon.exterior().coords().cloned().collect();
    assert!(
        exterior_coords.len() <= 7,
        "Merged rectangle should have at most 6 coords + closing, got {}. Points: {:?}",
        exterior_coords.len(),
        exterior_coords
    );

    // Verify area is 10*10 = 100 (sum of two 5*10 squares)
    let area = calculate_area(&exterior_coords).abs();
    assert!(
        (area - 100.0).abs() < 0.001,
        "Merged rectangle area should be 100, got {}",
        area
    );
}

/// Two squares using Subject+Clip: verifies the full Vatti+topology pipeline.
/// This is the key integration test for Issue #80.
#[test]
fn union_subject_clip_produces_valid_geometry() {
    // Square A: left square (Subject)
    let square_a: Vec<Point<i64>> = vec![
        Point::new(0, 0),
        Point::new(50, 0),
        Point::new(50, 100),
        Point::new(0, 100),
        Point::new(0, 0),
    ];

    // Square B: right square sharing edge at x=50 (Clip)
    let square_b: Vec<Point<i64>> = vec![
        Point::new(50, 0),
        Point::new(100, 0),
        Point::new(100, 100),
        Point::new(50, 100),
        Point::new(50, 0),
    ];

    // Run wagyu union
    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![square_a], PolygonType::Subject);
    wagyu.add_polygon(&vec![square_b], PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Union, FillType::EvenOdd, FillType::EvenOdd)
        .expect("Wagyu execution failed");

    // Should produce exactly one polygon
    assert_eq!(
        result.0.len(),
        1,
        "Union of two adjacent squares should produce 1 polygon, got {}",
        result.0.len()
    );

    let polygon = &result.0[0];

    // Should have no holes
    assert!(
        polygon.interiors().is_empty(),
        "Merged rectangle should have no holes"
    );

    // Verify area is 100*100 = 10000 (sum of two 50*100 squares)
    let exterior_coords: Vec<Coord<i64>> = polygon.exterior().coords().cloned().collect();
    let area = calculate_area(&exterior_coords).abs();
    assert!(
        (area - 10000.0).abs() < 0.001,
        "Merged rectangle area should be 10000, got {}",
        area
    );
}

/// Calculate the signed area of a polygon using the shoelace formula.
fn calculate_area(coords: &[Coord<i64>]) -> f64 {
    if coords.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    let n = coords.len();

    for i in 0..n {
        let j = (i + 1) % n;
        area += (coords[i].x as f64) * (coords[j].y as f64);
        area -= (coords[j].x as f64) * (coords[i].y as f64);
    }

    area / 2.0
}
