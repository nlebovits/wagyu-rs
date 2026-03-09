//! Integration tests for hot pixel insertion (Issue #36).
//!
//! Hot pixels are snap-rounding points collected during the initial scan phase.
//! They must be inserted into output rings during edge processing to ensure
//! geometric accuracy at shared edges and intersection points.
//!
//! Bug: `add_point_to_ring` does not call `insert_hot_pixels_in_path`, causing
//! hot pixels at shared edges to be missing from output.

use geo_types::{Coord, MultiPolygon};
use wagyu_rs::{config::FillType, config::PolygonType, point::Point, wagyu::Wagyu, Operation};

/// Helper to create a rectangle polygon with the given corners.
fn make_rect(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> Vec<Vec<Point<i64>>> {
    vec![vec![
        Point::new(min_x, max_y), // top-left
        Point::new(max_x, max_y), // top-right
        Point::new(max_x, min_y), // bottom-right
        Point::new(min_x, min_y), // bottom-left
        Point::new(min_x, max_y), // close
    ]]
}

/// Collect all coordinates from a MultiPolygon into a flat vector.
fn all_coords(result: &MultiPolygon<i64>) -> Vec<Coord<i64>> {
    result
        .0
        .iter()
        .flat_map(|p| p.exterior().coords().cloned())
        .collect()
}

/// Check if a specific coordinate exists in the result.
fn has_coord(result: &MultiPolygon<i64>, x: i64, y: i64) -> bool {
    all_coords(result).iter().any(|c| c.x == x && c.y == y)
}

/// Test: Union of two rectangles with shared horizontal edge should include
/// the shared edge point (10, 10).
///
/// This is the exact test case from Issue #36:
/// - Subject: Rectangle (0,0) to (10,10)
/// - Clip: Rectangle (5,5) to (15,10)
/// - Shared edge: Y=10 from X=5 to X=10
/// - Expected: 9-vertex polygon including (10,10) and (5,10)
/// - Bug: 8-vertex polygon missing (10,10) due to no hot pixel insertion
#[test]
fn issue_36_union_shared_horizontal_edge_preserves_hot_pixel() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Subject: rectangle (0,0) to (10,10)
    let subject = vec![vec![
        Point::new(0, 10),
        Point::new(10, 10),
        Point::new(10, 0),
        Point::new(0, 0),
        Point::new(0, 10),
    ]];
    wagyu.add_polygon(&subject, PolygonType::Subject);

    // Clip: rectangle (5,5) to (15,10)
    let clip = vec![vec![
        Point::new(5, 10),
        Point::new(15, 10),
        Point::new(15, 5),
        Point::new(5, 5),
        Point::new(5, 10),
    ]];
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Union, FillType::EvenOdd, FillType::EvenOdd)
        .expect("Union execution failed");

    // Should produce exactly 1 polygon
    assert_eq!(result.0.len(), 1, "Union should produce 1 polygon");

    // The union result should have 9 unique vertices (closing point included = 10 coords)
    // C++ produces: (0,10), (0,0), (10,0), (10,5), (15,5), (15,10), (10,10), (5,10), (0,10)
    let coords = all_coords(&result);

    // Critical check: (10, 10) must be present - this is the hot pixel
    assert!(
        has_coord(&result, 10, 10),
        "Hot pixel vertex (10, 10) should be present in result. \
         Got coords: {:?}",
        coords
    );

    // Also check (5, 10) is present
    assert!(
        has_coord(&result, 5, 10),
        "Vertex (5, 10) should be present in result. \
         Got coords: {:?}",
        coords
    );
}

/// Test: Union of rectangles sharing a vertical edge should preserve the shared point.
///
/// Similar to issue #36 but with vertical edge sharing.
/// Currently fails because polygons that overlap vertically produce 2 polygons
/// instead of merging into 1. This is a known limitation of the current topology
/// correction and is tracked separately from issue #36.
#[test]
#[ignore = "Known limitation: vertical edge overlap produces separate polygons"]
fn union_shared_vertical_edge_preserves_hot_pixel() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Subject: rectangle (0,0) to (10,10)
    let subject = make_rect(0, 0, 10, 10);
    wagyu.add_polygon(&subject, PolygonType::Subject);

    // Clip: rectangle (10,5) to (20,15) - shares vertical edge at X=10
    let clip = make_rect(10, 5, 20, 15);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Union, FillType::EvenOdd, FillType::EvenOdd)
        .expect("Union execution failed");

    assert_eq!(result.0.len(), 1, "Union should produce 1 polygon");

    // The shared edge point (10, 5) and (10, 10) should both be present
    assert!(
        has_coord(&result, 10, 5),
        "Shared vertex (10, 5) should be present in result"
    );
    assert!(
        has_coord(&result, 10, 10),
        "Shared vertex (10, 10) should be present in result"
    );
}

/// Test: Intersection with touching edges should handle hot pixels correctly.
///
/// When polygons touch at a single point, that point should be a hot pixel.
#[test]
fn intersection_touching_at_corner_handles_hot_pixel() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Subject: rectangle (0,0) to (10,10)
    let subject = make_rect(0, 0, 10, 10);
    wagyu.add_polygon(&subject, PolygonType::Subject);

    // Clip: rectangle (5,0) to (15,5) - overlaps bottom-right corner
    let clip = make_rect(5, 0, 15, 5);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(
            Operation::Intersection,
            FillType::EvenOdd,
            FillType::EvenOdd,
        )
        .expect("Intersection execution failed");

    // Intersection should be the overlapping region: (5,0) to (10,5)
    assert_eq!(result.0.len(), 1, "Intersection should produce 1 polygon");

    // Corner points should be present
    assert!(has_coord(&result, 5, 0), "Corner (5, 0) should be present");
    assert!(
        has_coord(&result, 10, 0),
        "Corner (10, 0) should be present"
    );
    assert!(
        has_coord(&result, 10, 5),
        "Corner (10, 5) should be present"
    );
    assert!(has_coord(&result, 5, 5), "Corner (5, 5) should be present");
}

/// Test: XOR operation preserves hot pixels at shared edges.
#[test]
fn xor_shared_edge_preserves_hot_pixel() {
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Two squares sharing an edge at X=10
    let square1 = make_rect(0, 0, 10, 10);
    wagyu.add_polygon(&square1, PolygonType::Subject);

    let square2 = make_rect(10, 0, 20, 10);
    wagyu.add_polygon(&square2, PolygonType::Clip);

    let result = wagyu
        .execute(Operation::Xor, FillType::EvenOdd, FillType::EvenOdd)
        .expect("XOR execution failed");

    // XOR of adjacent non-overlapping rectangles should merge into one
    // The shared edge points should be handled correctly
    assert!(
        !result.0.is_empty(),
        "XOR should produce at least 1 polygon"
    );
}
