//! Real-world data tests using OSM building footprints.
//!
//! These tests validate wagyu-rs against actual production geometry
//! from OpenStreetMap building data.

use serde_json::Value;
use std::fs;
use wagyu_rs::{
    config::{FillType, PolygonType},
    point::Point,
    wagyu::{Polygon, Ring, Wagyu},
    Operation,
};

/// Scale factor to convert lat/lon to integers (6 decimal places precision)
const SCALE: f64 = 1_000_000.0;

/// Parse a GeoJSON polygon coordinates array into wagyu's Polygon type
fn parse_polygon(coords: &Value) -> Option<Polygon<i64>> {
    let rings: Vec<Ring<i64>> = coords
        .as_array()?
        .iter()
        .map(|ring_val| {
            ring_val
                .as_array()
                .unwrap()
                .iter()
                .map(|pt| {
                    let arr = pt.as_array().unwrap();
                    let x = (arr[0].as_f64().unwrap() * SCALE).round() as i64;
                    let y = (arr[1].as_f64().unwrap() * SCALE).round() as i64;
                    Point::new(x, y)
                })
                .collect()
        })
        .collect();

    if rings.is_empty() {
        return None;
    }

    Some(rings)
}

/// Load building polygons from test fixtures
fn load_buildings() -> Vec<Polygon<i64>> {
    // Load from fixtures directory (copied from geoparquet-io)
    let content = fs::read_to_string("tests/fixtures/buildings_test.geojson")
        .expect("Could not find buildings_test.geojson in tests/fixtures/");

    let geojson: Value = serde_json::from_str(&content).expect("Invalid GeoJSON");

    let features = geojson["features"].as_array().expect("No features array");

    features
        .iter()
        .filter_map(|feature| {
            let geom = &feature["geometry"];
            if geom["type"].as_str()? == "Polygon" {
                parse_polygon(&geom["coordinates"])
            } else {
                None
            }
        })
        .collect()
}

/// Check if result has valid polygons (non-empty rings with proper structure)
fn is_valid_result(result: &geo_types::MultiPolygon<i64>) -> bool {
    // Empty result is valid (e.g., for non-overlapping difference)
    if result.0.is_empty() {
        return true;
    }

    for poly in &result.0 {
        let ext = poly.exterior();
        // Need at least 3 points + closing point
        if ext.0.len() < 4 {
            return false;
        }
        // Check if closed
        if ext.0.first() != ext.0.last() {
            return false;
        }
    }
    true
}

/// Run a clipping operation on two polygons
fn run_clip(
    poly1: &Polygon<i64>,
    poly2: &Polygon<i64>,
    operation: Operation,
) -> geo_types::MultiPolygon<i64> {
    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(poly1, PolygonType::Subject);
    wagyu.add_polygon(poly2, PolygonType::Clip);

    wagyu
        .execute(operation, FillType::EvenOdd, FillType::EvenOdd)
        .expect("Wagyu execution failed")
}

#[test]
fn real_world_buildings_load_test() {
    let buildings = load_buildings();
    println!("Loaded {} building polygons", buildings.len());
    assert!(buildings.len() >= 40, "Expected at least 40 buildings");

    // Verify each building has at least one ring with proper structure
    for (i, building) in buildings.iter().enumerate() {
        assert!(!building.is_empty(), "Building {} has no rings", i);
        for (j, ring) in building.iter().enumerate() {
            assert!(
                ring.len() >= 4,
                "Building {} ring {} has only {} points",
                i,
                j,
                ring.len()
            );
        }
    }
}

#[test]
fn real_world_buildings_union_pairwise() {
    let buildings = load_buildings();
    println!("Testing union on {} building pairs", buildings.len() - 1);

    let mut success_count = 0;
    let mut total_pairs = 0;

    // Test union of adjacent building pairs
    for i in 0..buildings.len().saturating_sub(1) {
        let result = run_clip(&buildings[i], &buildings[i + 1], Operation::Union);
        total_pairs += 1;

        if is_valid_result(&result) {
            success_count += 1;
        } else {
            println!("Invalid union result for buildings {} and {}", i, i + 1);
        }
    }

    println!("Union: {}/{} valid results", success_count, total_pairs);
    assert_eq!(
        success_count, total_pairs,
        "All union operations should produce valid results"
    );
}

#[test]
fn real_world_buildings_intersection_pairwise() {
    let buildings = load_buildings();

    let mut success_count = 0;
    let mut empty_count = 0;
    let mut total_pairs = 0;

    for i in 0..buildings.len().saturating_sub(1) {
        let result = run_clip(&buildings[i], &buildings[i + 1], Operation::Intersection);
        total_pairs += 1;

        if is_valid_result(&result) {
            success_count += 1;
            if result.0.is_empty() {
                empty_count += 1;
            }
        } else {
            println!(
                "Invalid intersection result for buildings {} and {}",
                i,
                i + 1
            );
        }
    }

    println!(
        "Intersection: {}/{} valid results ({} empty - expected for non-overlapping buildings)",
        success_count, total_pairs, empty_count
    );
    assert_eq!(
        success_count, total_pairs,
        "All intersection operations should produce valid results"
    );
}

#[test]
fn real_world_buildings_difference_pairwise() {
    let buildings = load_buildings();

    let mut success_count = 0;
    let mut total_pairs = 0;

    for i in 0..buildings.len().saturating_sub(1) {
        let result = run_clip(&buildings[i], &buildings[i + 1], Operation::Difference);
        total_pairs += 1;

        if is_valid_result(&result) {
            success_count += 1;
        } else {
            println!(
                "Invalid difference result for buildings {} and {}",
                i,
                i + 1
            );
        }
    }

    println!("Difference: {}/{} valid results", success_count, total_pairs);
    assert_eq!(
        success_count, total_pairs,
        "All difference operations should produce valid results"
    );
}

#[test]
fn real_world_buildings_xor_pairwise() {
    let buildings = load_buildings();

    let mut success_count = 0;
    let mut total_pairs = 0;

    for i in 0..buildings.len().saturating_sub(1) {
        let result = run_clip(&buildings[i], &buildings[i + 1], Operation::Xor);
        total_pairs += 1;

        if is_valid_result(&result) {
            success_count += 1;
        } else {
            println!("Invalid xor result for buildings {} and {}", i, i + 1);
        }
    }

    println!("Xor: {}/{} valid results", success_count, total_pairs);
    assert_eq!(
        success_count, total_pairs,
        "All xor operations should produce valid results"
    );
}

#[test]
fn real_world_buildings_all_operations_random_pairs() {
    let buildings = load_buildings();
    if buildings.len() < 10 {
        println!("Skipping random pairs test - not enough buildings");
        return;
    }

    // Test some specific random-ish pairs (using indices that space out across the dataset)
    let pairs: Vec<(usize, usize)> = vec![
        (0, 10),
        (5, 15),
        (10, 20),
        (15, 25),
        (20, 30),
        (25, 35),
        (0, 41),
        (1, 40),
        (2, 39),
        (3, 38),
    ];

    for (i, j) in pairs {
        if i >= buildings.len() || j >= buildings.len() {
            continue;
        }

        for op in [
            Operation::Union,
            Operation::Intersection,
            Operation::Difference,
            Operation::Xor,
        ] {
            let result = run_clip(&buildings[i], &buildings[j], op);
            assert!(
                is_valid_result(&result),
                "Invalid result for {:?} on buildings {} and {}",
                op,
                i,
                j
            );
        }
    }

    println!("Random pairs test passed for all operations");
}

#[test]
fn real_world_buildings_union_area_invariant() {
    let buildings = load_buildings();

    // Simple area calculation for a polygon ring (shoelace formula)
    fn ring_area(ring: &Ring<i64>) -> f64 {
        if ring.len() < 3 {
            return 0.0;
        }
        let mut sum = 0i64;
        for i in 0..ring.len() - 1 {
            sum += (ring[i + 1].x - ring[i].x) * (ring[i + 1].y + ring[i].y);
        }
        (sum as f64 / 2.0).abs()
    }

    fn polygon_area(poly: &Polygon<i64>) -> f64 {
        if poly.is_empty() {
            return 0.0;
        }
        // First ring is exterior, rest are holes
        let ext_area = ring_area(&poly[0]);
        let hole_area: f64 = poly.iter().skip(1).map(|r| ring_area(r)).sum();
        ext_area - hole_area
    }

    fn result_area(mp: &geo_types::MultiPolygon<i64>) -> f64 {
        mp.0.iter()
            .map(|p| {
                let ext: Vec<Point<i64>> =
                    p.exterior().coords().map(|c| Point::new(c.x, c.y)).collect();
                let ext_area = ring_area(&ext);
                let hole_area: f64 = p
                    .interiors()
                    .iter()
                    .map(|h| {
                        let ring: Vec<Point<i64>> =
                            h.coords().map(|c| Point::new(c.x, c.y)).collect();
                        ring_area(&ring)
                    })
                    .sum();
                ext_area - hole_area
            })
            .sum()
    }

    let mut violations = 0;
    let mut total = 0;

    for i in 0..buildings.len().saturating_sub(1) {
        let area1 = polygon_area(&buildings[i]);
        let area2 = polygon_area(&buildings[i + 1]);
        let max_input_area = area1.max(area2);

        let result = run_clip(&buildings[i], &buildings[i + 1], Operation::Union);
        let union_area = result_area(&result);

        total += 1;

        // Union area should be >= max input area (with small tolerance for floating point)
        if union_area < max_input_area * 0.999 {
            println!(
                "Area invariant violation: union({}, {}) = {:.2} < max({:.2}, {:.2}) = {:.2}",
                i,
                i + 1,
                union_area,
                area1,
                area2,
                max_input_area
            );
            violations += 1;
        }
    }

    println!(
        "Area invariant: {}/{} passed ({} violations)",
        total - violations,
        total,
        violations
    );
    assert_eq!(violations, 0, "Union area should be >= max input area");
}
