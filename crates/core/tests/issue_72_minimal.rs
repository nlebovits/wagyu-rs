//! Minimal test case for issue #72 - XOR with two intersecting holes
//!
//! This test verifies that corner-touching polygons remain separate after XOR.

use wagyu_rs::{config::FillType, config::PolygonType, point::Point, wagyu::Wagyu, Operation};

/// Minimal reproduction: Two triangles that share only a single vertex should remain separate.
#[test]
fn xor_corner_touching_triangles_stay_separate() {
    // Two triangles that share vertex (0, 0)
    // Triangle 1: (0,0), (10,0), (5,10)
    // Triangle 2: (0,0), (-10,0), (-5,-10)
    // After XOR, both should be separate exterior polygons

    let subject: Vec<Vec<Point<i64>>> = vec![vec![
        Point::new(0, 0),
        Point::new(10, 0),
        Point::new(5, 10),
        Point::new(0, 0),
    ]];

    // Clip triangle shares vertex (0,0) with subject
    let clip: Vec<Vec<Point<i64>>> = vec![vec![
        Point::new(0, 0),
        Point::new(-10, 0),
        Point::new(-5, -10),
        Point::new(0, 0),
    ]];

    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&subject, PolygonType::Subject);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR should succeed");

    println!("Result has {} polygons", result.0.len());
    for (i, poly) in result.0.iter().enumerate() {
        println!(
            "  Polygon {}: {} exterior coords",
            i,
            poly.exterior().0.len()
        );
    }

    // XOR of two non-overlapping triangles that share a vertex should produce 2 separate polygons
    assert_eq!(
        result.0.len(),
        2,
        "Corner-touching triangles should remain as 2 separate polygons after XOR, got {}",
        result.0.len()
    );
}

/// Test case matching the golden test: polygon with two intersecting holes and self-intersection
/// Clips against a square, XOR should produce 12 separate polygons
#[test]
fn xor_polygon_with_intersecting_holes_against_square() {
    // From polygon-two-intersecting-holes-and-self-intersection.json
    // Subject has 3 rings that form a self-intersecting polygon with holes
    let subject: Vec<Vec<Point<i64>>> = vec![
        // Ring 0: exterior with self-intersection
        vec![
            Point::new(-3580, -406),
            Point::new(-392, 4575),
            Point::new(1263, -964),
            Point::new(-1211, -2262),
            Point::new(-3580, -406),
        ],
        // Ring 1: hole that intersects
        vec![
            Point::new(1071, -2620),
            Point::new(-810, -161),
            Point::new(1681, 662),
            Point::new(2953, -1635),
            Point::new(1071, -2620),
        ],
        // Ring 2: exterior that creates complex topology
        vec![
            Point::new(-5096, 5738),
            Point::new(4591, 5190),
            Point::new(-5235, -3856),
            Point::new(4765, -4262),
            Point::new(-5096, 5738),
        ],
    ];

    // Clip: clockwise square
    let clip: Vec<Vec<Point<i64>>> = vec![vec![
        Point::new(-2500, -2500),
        Point::new(-2500, 2500),
        Point::new(2500, 2500),
        Point::new(2500, -2500),
        Point::new(-2500, -2500),
    ]];

    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&subject, PolygonType::Subject);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR should succeed");

    println!("Result has {} polygons", result.0.len());
    for (i, poly) in result.0.iter().enumerate() {
        let ext_coords: Vec<_> = poly.exterior().0.iter().collect();
        println!(
            "  Polygon {}: {} exterior coords, starts at ({}, {})",
            i,
            ext_coords.len(),
            ext_coords.first().map(|c| c.x).unwrap_or(0),
            ext_coords.first().map(|c| c.y).unwrap_or(0)
        );
    }

    // C++ wagyu produces 12 polygons for this XOR
    assert_eq!(
        result.0.len(),
        12,
        "XOR should produce 12 separate polygons, got {}",
        result.0.len()
    );
}
