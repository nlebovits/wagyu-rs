//! Golden tests comparing Rust wagyu output to C++ wagyu expected results.
//!
//! These tests load fixtures from `tests/fixtures/` and compare output against
//! expected results in `tests/expected/`.
//!
//! Test fixtures sourced from: https://github.com/mapbox/wagyu

use geo_types::{Coord, MultiPolygon, Polygon as GeoPolygon};
use serde_json::Value;
use std::fs;
use std::path::Path;
use wagyu_rs::{config::FillType, config::PolygonType, point::Point, wagyu::Wagyu, Operation};

/// Load a JSON polygon fixture file.
///
/// Format: `[[[x,y], [x,y], ...], [[x,y], ...]]`
/// First array is rings, each ring is an array of [x,y] coordinate pairs.
fn load_fixture(path: &Path) -> Vec<Vec<Point<i64>>> {
    let content =
        fs::read_to_string(path).unwrap_or_else(|_| panic!("Failed to read fixture: {:?}", path));
    let json: Value = serde_json::from_str(&content).expect("Failed to parse JSON");

    let mut rings = Vec::new();
    if let Value::Array(polygon) = json {
        for ring_val in polygon {
            let mut ring = Vec::new();
            if let Value::Array(points) = ring_val {
                for pt in points {
                    if let Value::Array(coords) = pt {
                        let x = coords[0].as_i64().expect("x coord");
                        let y = coords[1].as_i64().expect("y coord");
                        ring.push(Point::new(x, y));
                    }
                }
            }
            rings.push(ring);
        }
    }
    rings
}

/// Load expected output (MultiPolygon format).
///
/// Format: `[[[[x,y], ...], ...], ...]` - array of polygons, each polygon is array of rings.
fn load_expected(path: &Path) -> MultiPolygon<i64> {
    let content =
        fs::read_to_string(path).unwrap_or_else(|_| panic!("Failed to read expected: {:?}", path));
    let json: Value = serde_json::from_str(&content).expect("Failed to parse JSON");

    let mut polygons: Vec<GeoPolygon<i64>> = Vec::new();
    if let Value::Array(multi) = json {
        for poly_val in multi {
            let mut rings: Vec<geo_types::LineString<i64>> = Vec::new();
            if let Value::Array(poly) = poly_val {
                for ring_val in poly {
                    let mut coords: Vec<Coord<i64>> = Vec::new();
                    if let Value::Array(points) = ring_val {
                        for pt in points {
                            if let Value::Array(c) = pt {
                                let x = c[0].as_i64().expect("x coord");
                                let y = c[1].as_i64().expect("y coord");
                                coords.push(Coord { x, y });
                            }
                        }
                    }
                    rings.push(geo_types::LineString::new(coords));
                }
            }
            if !rings.is_empty() {
                let exterior = rings.remove(0);
                let interiors = rings;
                polygons.push(GeoPolygon::new(exterior, interiors));
            }
        }
    }
    MultiPolygon::new(polygons)
}

/// Convert wagyu MultiPolygon<i64> to sorted representation for comparison.
/// Sorts polygons and rings to handle different orderings.
fn normalize_result(mp: &MultiPolygon<i64>) -> Vec<Vec<Vec<(i64, i64)>>> {
    let mut result: Vec<Vec<Vec<(i64, i64)>>> =
        mp.0.iter()
            .map(|poly| {
                let mut rings: Vec<Vec<(i64, i64)>> = std::iter::once(poly.exterior())
                    .chain(poly.interiors().iter())
                    .map(|ring| ring.coords().map(|c| (c.x, c.y)).collect::<Vec<_>>())
                    .collect();
                // Sort rings within polygon for consistent comparison
                rings.sort();
                rings
            })
            .collect();
    // Sort polygons for consistent comparison
    result.sort();
    result
}

/// Get operation name for file lookup
fn op_name(op: Operation) -> &'static str {
    match op {
        Operation::Union => "union",
        Operation::Intersection => "intersection",
        Operation::Difference => "difference",
        Operation::Xor => "x_or",
    }
}

/// Run a single golden test case.
fn run_golden_test(test_name: &str, operation: Operation) {
    let fixtures_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let expected_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/expected");

    // Load subject polygon
    let subject_path = fixtures_dir.join(format!("{}.json", test_name));
    let subject = load_fixture(&subject_path);

    // Load clip polygon (always the square)
    let clip_path = fixtures_dir.join("clip-clockwise-square.json");
    let clip = load_fixture(&clip_path);

    // Load expected output
    let expected_path = expected_dir.join(format!("{}-{}.json", op_name(operation), test_name));
    let expected = load_expected(&expected_path);

    // Run wagyu
    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&subject, PolygonType::Subject);
    wagyu.add_polygon(&clip, PolygonType::Clip);

    let result = wagyu
        .execute(operation, FillType::EvenOdd, FillType::EvenOdd)
        .expect("Wagyu execution failed");

    // Compare normalized results
    let expected_norm = normalize_result(&expected);
    let result_norm = normalize_result(&result);

    if expected_norm != result_norm {
        eprintln!("\n=== GOLDEN TEST FAILURE ===");
        eprintln!("Test: {} / {:?}", test_name, operation);
        eprintln!("\nExpected ({} polygons):", expected.0.len());
        for (i, poly) in expected.0.iter().enumerate() {
            eprintln!("  Polygon {}: {} rings", i, poly.interiors().len() + 1);
            let coords: Vec<_> = poly.exterior().coords().collect();
            eprintln!("  Exterior coords ({}):", coords.len());
            for (j, c) in coords.iter().take(10).enumerate() {
                eprintln!("    {}: ({}, {})", j, c.x, c.y);
            }
            if coords.len() > 10 {
                eprintln!("    ... and {} more", coords.len() - 10);
            }
        }
        eprintln!("\nGot ({} polygons):", result.0.len());
        for (i, poly) in result.0.iter().enumerate() {
            eprintln!("  Polygon {}: {} rings", i, poly.interiors().len() + 1);
            let coords: Vec<_> = poly.exterior().coords().collect();
            eprintln!("  Exterior coords ({}):", coords.len());
            for (j, c) in coords.iter().take(10).enumerate() {
                eprintln!("    {}: ({}, {})", j, c.x, c.y);
            }
            if coords.len() > 10 {
                eprintln!("    ... and {} more", coords.len() - 10);
            }
        }
        panic!("Golden test failed: {}-{}", op_name(operation), test_name);
    }
}

// ============================================================================
// GENERATED TESTS
// ============================================================================

macro_rules! golden_test {
    ($name:ident, $test_name:expr, $op:expr) => {
        #[test]
        fn $name() {
            run_golden_test($test_name, $op);
        }
    };
}

// --- clockwise-polygon ---
golden_test!(
    union_clockwise_polygon,
    "clockwise-polygon",
    Operation::Union
);
golden_test!(
    intersection_clockwise_polygon,
    "clockwise-polygon",
    Operation::Intersection
);
golden_test!(
    difference_clockwise_polygon,
    "clockwise-polygon",
    Operation::Difference
);
golden_test!(xor_clockwise_polygon, "clockwise-polygon", Operation::Xor);

// --- clockwise-polygon-clockwise-hole ---
golden_test!(
    union_clockwise_polygon_clockwise_hole,
    "clockwise-polygon-clockwise-hole",
    Operation::Union
);
golden_test!(
    intersection_clockwise_polygon_clockwise_hole,
    "clockwise-polygon-clockwise-hole",
    Operation::Intersection
);
golden_test!(
    difference_clockwise_polygon_clockwise_hole,
    "clockwise-polygon-clockwise-hole",
    Operation::Difference
);
golden_test!(
    xor_clockwise_polygon_clockwise_hole,
    "clockwise-polygon-clockwise-hole",
    Operation::Xor
);

// --- clockwise-polygon-counter-clockwise-hole ---
golden_test!(
    union_clockwise_polygon_counter_clockwise_hole,
    "clockwise-polygon-counter-clockwise-hole",
    Operation::Union
);
golden_test!(
    intersection_clockwise_polygon_counter_clockwise_hole,
    "clockwise-polygon-counter-clockwise-hole",
    Operation::Intersection
);
golden_test!(
    difference_clockwise_polygon_counter_clockwise_hole,
    "clockwise-polygon-counter-clockwise-hole",
    Operation::Difference
);
golden_test!(
    xor_clockwise_polygon_counter_clockwise_hole,
    "clockwise-polygon-counter-clockwise-hole",
    Operation::Xor
);

// --- counter-clockwise-polygon ---
golden_test!(
    union_counter_clockwise_polygon,
    "counter-clockwise-polygon",
    Operation::Union
);
golden_test!(
    intersection_counter_clockwise_polygon,
    "counter-clockwise-polygon",
    Operation::Intersection
);
golden_test!(
    difference_counter_clockwise_polygon,
    "counter-clockwise-polygon",
    Operation::Difference
);
golden_test!(
    xor_counter_clockwise_polygon,
    "counter-clockwise-polygon",
    Operation::Xor
);

// --- counter-clockwise-polygon-clockwise-hole ---
golden_test!(
    union_counter_clockwise_polygon_clockwise_hole,
    "counter-clockwise-polygon-clockwise-hole",
    Operation::Union
);
golden_test!(
    intersection_counter_clockwise_polygon_clockwise_hole,
    "counter-clockwise-polygon-clockwise-hole",
    Operation::Intersection
);
golden_test!(
    difference_counter_clockwise_polygon_clockwise_hole,
    "counter-clockwise-polygon-clockwise-hole",
    Operation::Difference
);
golden_test!(
    xor_counter_clockwise_polygon_clockwise_hole,
    "counter-clockwise-polygon-clockwise-hole",
    Operation::Xor
);

// --- counter-clockwise-polygon-counter-clockwise-hole ---
golden_test!(
    union_counter_clockwise_polygon_counter_clockwise_hole,
    "counter-clockwise-polygon-counter-clockwise-hole",
    Operation::Union
);
golden_test!(
    intersection_counter_clockwise_polygon_counter_clockwise_hole,
    "counter-clockwise-polygon-counter-clockwise-hole",
    Operation::Intersection
);
golden_test!(
    difference_counter_clockwise_polygon_counter_clockwise_hole,
    "counter-clockwise-polygon-counter-clockwise-hole",
    Operation::Difference
);
golden_test!(
    xor_counter_clockwise_polygon_counter_clockwise_hole,
    "counter-clockwise-polygon-counter-clockwise-hole",
    Operation::Xor
);

// --- multipolygon-both-clockwise ---
golden_test!(
    union_multipolygon_both_clockwise,
    "multipolygon-both-clockwise",
    Operation::Union
);
golden_test!(
    intersection_multipolygon_both_clockwise,
    "multipolygon-both-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_multipolygon_both_clockwise,
    "multipolygon-both-clockwise",
    Operation::Difference
);
golden_test!(
    xor_multipolygon_both_clockwise,
    "multipolygon-both-clockwise",
    Operation::Xor
);

// --- multipolygon-both-counter-clockwise ---
golden_test!(
    union_multipolygon_both_counter_clockwise,
    "multipolygon-both-counter-clockwise",
    Operation::Union
);
golden_test!(
    intersection_multipolygon_both_counter_clockwise,
    "multipolygon-both-counter-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_multipolygon_both_counter_clockwise,
    "multipolygon-both-counter-clockwise",
    Operation::Difference
);
golden_test!(
    xor_multipolygon_both_counter_clockwise,
    "multipolygon-both-counter-clockwise",
    Operation::Xor
);

// --- multipolygon-overlap-different-orientations ---
golden_test!(
    union_multipolygon_overlap_different_orientations,
    "multipolygon-overlap-different-orientations",
    Operation::Union
);
golden_test!(
    intersection_multipolygon_overlap_different_orientations,
    "multipolygon-overlap-different-orientations",
    Operation::Intersection
);
golden_test!(
    difference_multipolygon_overlap_different_orientations,
    "multipolygon-overlap-different-orientations",
    Operation::Difference
);
golden_test!(
    xor_multipolygon_overlap_different_orientations,
    "multipolygon-overlap-different-orientations",
    Operation::Xor
);

// --- multi-polygon-with-duplicate-polygon ---
golden_test!(
    union_multi_polygon_with_duplicate_polygon,
    "multi-polygon-with-duplicate-polygon",
    Operation::Union
);
golden_test!(
    intersection_multi_polygon_with_duplicate_polygon,
    "multi-polygon-with-duplicate-polygon",
    Operation::Intersection
);
golden_test!(
    difference_multi_polygon_with_duplicate_polygon,
    "multi-polygon-with-duplicate-polygon",
    Operation::Difference
);
golden_test!(
    xor_multi_polygon_with_duplicate_polygon,
    "multi-polygon-with-duplicate-polygon",
    Operation::Xor
);

// --- multi-polygon-with-shared-edge ---
golden_test!(
    union_multi_polygon_with_shared_edge,
    "multi-polygon-with-shared-edge",
    Operation::Union
);
golden_test!(
    intersection_multi_polygon_with_shared_edge,
    "multi-polygon-with-shared-edge",
    Operation::Intersection
);
golden_test!(
    difference_multi_polygon_with_shared_edge,
    "multi-polygon-with-shared-edge",
    Operation::Difference
);
golden_test!(
    xor_multi_polygon_with_shared_edge,
    "multi-polygon-with-shared-edge",
    Operation::Xor
);

// --- multi-polygon-with-spikes ---
golden_test!(
    union_multi_polygon_with_spikes,
    "multi-polygon-with-spikes",
    Operation::Union
);
golden_test!(
    intersection_multi_polygon_with_spikes,
    "multi-polygon-with-spikes",
    Operation::Intersection
);
golden_test!(
    difference_multi_polygon_with_spikes,
    "multi-polygon-with-spikes",
    Operation::Difference
);
golden_test!(
    xor_multi_polygon_with_spikes,
    "multi-polygon-with-spikes",
    Operation::Xor
);

// --- nested-multi-polygon-outer-clockwise-inner-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-clockwise-inner-clockwise-hole-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-clockwise-inner-clockwise-hole-counter-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-counter-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-counter-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-counter-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_clockwise_inner_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-clockwise-hole-counter-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-clockwise-inner-counter-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-counter-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-counter-clockwise",
    Operation::Union
);
golden_test!(intersection_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_counter_clockwise, "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-counter-clockwise", Operation::Intersection);
golden_test!(
    difference_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-counter-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_clockwise_inner_counter_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-clockwise-inner-counter-clockwise-hole-counter-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-counter-clockwise-inner-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_counter_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_counter_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_counter_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_counter_clockwise_inner_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-counter-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-counter-clockwise",
    Operation::Union
);
golden_test!(intersection_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_counter_clockwise, "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-counter-clockwise", Operation::Intersection);
golden_test!(
    difference_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-counter-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_counter_clockwise_inner_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-clockwise-hole-counter-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise",
    Operation::Union
);
golden_test!(
    intersection_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise",
    Operation::Intersection
);
golden_test!(
    difference_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-clockwise ---
golden_test!(
    union_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-clockwise",
    Operation::Union
);
golden_test!(intersection_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_clockwise, "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-clockwise", Operation::Intersection);
golden_test!(
    difference_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-clockwise",
    Operation::Difference
);
golden_test!(
    xor_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-clockwise",
    Operation::Xor
);

// --- nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-counter-clockwise ---
golden_test!(union_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_counter_clockwise, "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-counter-clockwise", Operation::Union);
golden_test!(intersection_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_counter_clockwise, "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-counter-clockwise", Operation::Intersection);
golden_test!(difference_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_counter_clockwise, "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-counter-clockwise", Operation::Difference);
golden_test!(
    xor_nested_multi_polygon_outer_counter_clockwise_inner_counter_clockwise_hole_counter_clockwise,
    "nested-multi-polygon-outer-counter-clockwise-inner-counter-clockwise-hole-counter-clockwise",
    Operation::Xor
);

// --- overlapping-multi-polygon ---
golden_test!(
    union_overlapping_multi_polygon,
    "overlapping-multi-polygon",
    Operation::Union
);
golden_test!(
    intersection_overlapping_multi_polygon,
    "overlapping-multi-polygon",
    Operation::Intersection
);
golden_test!(
    difference_overlapping_multi_polygon,
    "overlapping-multi-polygon",
    Operation::Difference
);
golden_test!(
    xor_overlapping_multi_polygon,
    "overlapping-multi-polygon",
    Operation::Xor
);

// --- polygon-covered-with-hole ---
golden_test!(
    union_polygon_covered_with_hole,
    "polygon-covered-with-hole",
    Operation::Union
);
golden_test!(
    intersection_polygon_covered_with_hole,
    "polygon-covered-with-hole",
    Operation::Intersection
);
golden_test!(
    difference_polygon_covered_with_hole,
    "polygon-covered-with-hole",
    Operation::Difference
);
golden_test!(
    xor_polygon_covered_with_hole,
    "polygon-covered-with-hole",
    Operation::Xor
);

// --- polygon-no-interior ---
golden_test!(
    union_polygon_no_interior,
    "polygon-no-interior",
    Operation::Union
);
golden_test!(
    intersection_polygon_no_interior,
    "polygon-no-interior",
    Operation::Intersection
);
golden_test!(
    difference_polygon_no_interior,
    "polygon-no-interior",
    Operation::Difference
);
golden_test!(
    xor_polygon_no_interior,
    "polygon-no-interior",
    Operation::Xor
);

// --- polygon-two-intersecting-holes ---
golden_test!(
    union_polygon_two_intersecting_holes,
    "polygon-two-intersecting-holes",
    Operation::Union
);
golden_test!(
    intersection_polygon_two_intersecting_holes,
    "polygon-two-intersecting-holes",
    Operation::Intersection
);
golden_test!(
    difference_polygon_two_intersecting_holes,
    "polygon-two-intersecting-holes",
    Operation::Difference
);
golden_test!(
    xor_polygon_two_intersecting_holes,
    "polygon-two-intersecting-holes",
    Operation::Xor
);

// --- polygon-two-intersecting-holes-and-self-intersection ---
golden_test!(
    union_polygon_two_intersecting_holes_and_self_intersection,
    "polygon-two-intersecting-holes-and-self-intersection",
    Operation::Union
);
golden_test!(
    intersection_polygon_two_intersecting_holes_and_self_intersection,
    "polygon-two-intersecting-holes-and-self-intersection",
    Operation::Intersection
);
golden_test!(
    difference_polygon_two_intersecting_holes_and_self_intersection,
    "polygon-two-intersecting-holes-and-self-intersection",
    Operation::Difference
);
golden_test!(
    xor_polygon_two_intersecting_holes_and_self_intersection,
    "polygon-two-intersecting-holes-and-self-intersection",
    Operation::Xor
);

// --- polygon-with-double-nested-holes ---
golden_test!(
    union_polygon_with_double_nested_holes,
    "polygon-with-double-nested-holes",
    Operation::Union
);
golden_test!(
    intersection_polygon_with_double_nested_holes,
    "polygon-with-double-nested-holes",
    Operation::Intersection
);
golden_test!(
    difference_polygon_with_double_nested_holes,
    "polygon-with-double-nested-holes",
    Operation::Difference
);
golden_test!(
    xor_polygon_with_double_nested_holes,
    "polygon-with-double-nested-holes",
    Operation::Xor
);

// --- polygon-with-extending-hole ---
golden_test!(
    union_polygon_with_extending_hole,
    "polygon-with-extending-hole",
    Operation::Union
);
golden_test!(
    intersection_polygon_with_extending_hole,
    "polygon-with-extending-hole",
    Operation::Intersection
);
golden_test!(
    difference_polygon_with_extending_hole,
    "polygon-with-extending-hole",
    Operation::Difference
);
golden_test!(
    xor_polygon_with_extending_hole,
    "polygon-with-extending-hole",
    Operation::Xor
);

// --- polygon-with-exterior-hole ---
golden_test!(
    union_polygon_with_exterior_hole,
    "polygon-with-exterior-hole",
    Operation::Union
);
golden_test!(
    intersection_polygon_with_exterior_hole,
    "polygon-with-exterior-hole",
    Operation::Intersection
);
golden_test!(
    difference_polygon_with_exterior_hole,
    "polygon-with-exterior-hole",
    Operation::Difference
);
golden_test!(
    xor_polygon_with_exterior_hole,
    "polygon-with-exterior-hole",
    Operation::Xor
);

// --- polygon-with-hole-shared-edge ---
golden_test!(
    union_polygon_with_hole_shared_edge,
    "polygon-with-hole-shared-edge",
    Operation::Union
);
golden_test!(
    intersection_polygon_with_hole_shared_edge,
    "polygon-with-hole-shared-edge",
    Operation::Intersection
);
golden_test!(
    difference_polygon_with_hole_shared_edge,
    "polygon-with-hole-shared-edge",
    Operation::Difference
);
golden_test!(
    xor_polygon_with_hole_shared_edge,
    "polygon-with-hole-shared-edge",
    Operation::Xor
);

// --- polygon-with-hole-with-shared-point ---
golden_test!(
    union_polygon_with_hole_with_shared_point,
    "polygon-with-hole-with-shared-point",
    Operation::Union
);
golden_test!(
    intersection_polygon_with_hole_with_shared_point,
    "polygon-with-hole-with-shared-point",
    Operation::Intersection
);
golden_test!(
    difference_polygon_with_hole_with_shared_point,
    "polygon-with-hole-with-shared-point",
    Operation::Difference
);
golden_test!(
    xor_polygon_with_hole_with_shared_point,
    "polygon-with-hole-with-shared-point",
    Operation::Xor
);

// --- polygon-with-spike ---
golden_test!(
    union_polygon_with_spike,
    "polygon-with-spike",
    Operation::Union
);
golden_test!(
    intersection_polygon_with_spike,
    "polygon-with-spike",
    Operation::Intersection
);
golden_test!(
    difference_polygon_with_spike,
    "polygon-with-spike",
    Operation::Difference
);
golden_test!(xor_polygon_with_spike, "polygon-with-spike", Operation::Xor);

// --- polygon-with-two-holes-outside-exterior-ring ---
golden_test!(
    union_polygon_with_two_holes_outside_exterior_ring,
    "polygon-with-two-holes-outside-exterior-ring",
    Operation::Union
);
golden_test!(
    intersection_polygon_with_two_holes_outside_exterior_ring,
    "polygon-with-two-holes-outside-exterior-ring",
    Operation::Intersection
);
golden_test!(
    difference_polygon_with_two_holes_outside_exterior_ring,
    "polygon-with-two-holes-outside-exterior-ring",
    Operation::Difference
);
golden_test!(
    xor_polygon_with_two_holes_outside_exterior_ring,
    "polygon-with-two-holes-outside-exterior-ring",
    Operation::Xor
);

// --- self-intersecting-ring-polygon ---
golden_test!(
    union_self_intersecting_ring_polygon,
    "self-intersecting-ring-polygon",
    Operation::Union
);
golden_test!(
    intersection_self_intersecting_ring_polygon,
    "self-intersecting-ring-polygon",
    Operation::Intersection
);
golden_test!(
    difference_self_intersecting_ring_polygon,
    "self-intersecting-ring-polygon",
    Operation::Difference
);
golden_test!(
    xor_self_intersecting_ring_polygon,
    "self-intersecting-ring-polygon",
    Operation::Xor
);
