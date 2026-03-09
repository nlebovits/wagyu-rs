//! Real-world data tests using production building footprints.
//!
//! These tests validate wagyu-rs against actual production geometry from:
//! - Google-Microsoft-OSM Open Buildings (Andorra subset, 1000 features)
//! - National Wetlands Inventory (DC subset, 1000 features with complex holes)
//!
//! Data provenance documented at:
//! https://github.com/nlebovits/portolan-cli/blob/main/context/shared/documentation/test-fixtures.md

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
fn parse_polygon_coords(coords: &Value, scale: f64) -> Option<Polygon<i64>> {
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
                    let x = (arr[0].as_f64().unwrap() * scale).round() as i64;
                    let y = (arr[1].as_f64().unwrap() * scale).round() as i64;
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

/// Parse a GeoJSON geometry into wagyu Polygon(s)
/// Handles both Polygon and MultiPolygon types
fn parse_geometry(geom: &Value, scale: f64) -> Vec<Polygon<i64>> {
    let geom_type = geom["type"].as_str().unwrap_or("");
    match geom_type {
        "Polygon" => parse_polygon_coords(&geom["coordinates"], scale)
            .into_iter()
            .collect(),
        "MultiPolygon" => geom["coordinates"]
            .as_array()
            .unwrap_or(&vec![])
            .iter()
            .filter_map(|poly_coords| parse_polygon_coords(poly_coords, scale))
            .collect(),
        _ => vec![],
    }
}

/// Load polygons from a GeoJSON fixture file
/// `scale` converts coordinates to integers (use SCALE for lat/lon, 1.0 for projected)
fn load_geojson_polygons(filename: &str, scale: f64) -> Vec<Polygon<i64>> {
    let path = format!("tests/fixtures/{}", filename);
    let content = fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("Could not find {} in tests/fixtures/", filename));

    let geojson: Value = serde_json::from_str(&content).expect("Invalid GeoJSON");

    let features = geojson["features"].as_array().expect("No features array");

    features
        .iter()
        .flat_map(|feature| {
            let geom = &feature["geometry"];
            parse_geometry(geom, scale)
        })
        .collect()
}

/// Load building polygons from Google-Microsoft-OSM Open Buildings (Andorra)
/// These are in WGS84 (lat/lon), so we scale to integers.
fn load_buildings() -> Vec<Polygon<i64>> {
    load_geojson_polygons("open_buildings_andorra.geojson", SCALE)
}

/// Load wetland polygons from National Wetlands Inventory (DC)
/// These are in a projected CRS (EPSG:5070), already in meters.
/// We use scale=1.0 and round to integers.
fn load_wetlands() -> Vec<Polygon<i64>> {
    load_geojson_polygons("nwi_wetlands_dc.geojson", 1.0)
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

// =============================================================================
// Building footprint tests (simple polygons)
// =============================================================================

#[test]
fn real_world_buildings_load_test() {
    let buildings = load_buildings();
    println!("Loaded {} building polygons", buildings.len());
    assert!(buildings.len() >= 100, "Expected at least 100 buildings");

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
    // Test first 100 pairs (adjacent buildings)
    let test_count = buildings.len().min(100);
    println!("Testing union on {} building pairs", test_count - 1);

    let mut success_count = 0;
    let mut total_pairs = 0;

    for i in 0..test_count.saturating_sub(1) {
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
    let test_count = buildings.len().min(100);

    let mut success_count = 0;
    let mut empty_count = 0;
    let mut total_pairs = 0;

    for i in 0..test_count.saturating_sub(1) {
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
    let test_count = buildings.len().min(100);

    let mut success_count = 0;
    let mut total_pairs = 0;

    for i in 0..test_count.saturating_sub(1) {
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

    println!(
        "Difference: {}/{} valid results",
        success_count, total_pairs
    );
    assert_eq!(
        success_count, total_pairs,
        "All difference operations should produce valid results"
    );
}

#[test]
fn real_world_buildings_xor_pairwise() {
    let buildings = load_buildings();
    let test_count = buildings.len().min(100);

    let mut success_count = 0;
    let mut total_pairs = 0;

    for i in 0..test_count.saturating_sub(1) {
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
    if buildings.len() < 100 {
        println!("Skipping random pairs test - not enough buildings");
        return;
    }

    // Test spaced-out pairs across the 1000-feature dataset
    let pairs: Vec<(usize, usize)> = vec![
        (0, 100),
        (50, 150),
        (100, 200),
        (200, 400),
        (300, 600),
        (400, 800),
        (0, 999),
        (1, 500),
        (250, 750),
        (333, 666),
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
    let test_count = buildings.len().min(100);

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
                let ext: Vec<Point<i64>> = p
                    .exterior()
                    .coords()
                    .map(|c| Point::new(c.x, c.y))
                    .collect();
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

    for i in 0..test_count.saturating_sub(1) {
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

// =============================================================================
// Wetland tests (complex polygons with holes)
// =============================================================================

#[test]
fn real_world_wetlands_load_test() {
    let wetlands = load_wetlands();
    println!("Loaded {} wetland polygons", wetlands.len());
    assert!(wetlands.len() >= 100, "Expected at least 100 wetlands");

    // Count polygons with holes
    let with_holes = wetlands.iter().filter(|p| p.len() > 1).count();
    println!("{} wetlands have holes", with_holes);

    // Verify structure
    for (i, wetland) in wetlands.iter().enumerate() {
        assert!(!wetland.is_empty(), "Wetland {} has no rings", i);
        for (j, ring) in wetland.iter().enumerate() {
            assert!(
                ring.len() >= 4,
                "Wetland {} ring {} has only {} points",
                i,
                j,
                ring.len()
            );
        }
    }
}

#[test]
fn real_world_wetlands_union_pairwise() {
    let wetlands = load_wetlands();
    let test_count = wetlands.len().min(50); // Fewer tests since wetlands are more complex
    println!("Testing union on {} wetland pairs", test_count - 1);

    let mut success_count = 0;
    let mut total_pairs = 0;

    for i in 0..test_count.saturating_sub(1) {
        let result = run_clip(&wetlands[i], &wetlands[i + 1], Operation::Union);
        total_pairs += 1;

        if is_valid_result(&result) {
            success_count += 1;
        } else {
            println!("Invalid union result for wetlands {} and {}", i, i + 1);
        }
    }

    println!(
        "Wetland union: {}/{} valid results",
        success_count, total_pairs
    );
    assert_eq!(
        success_count, total_pairs,
        "All wetland union operations should produce valid results"
    );
}

#[test]
fn real_world_wetlands_all_operations() {
    let wetlands = load_wetlands();
    let test_count = wetlands.len().min(20);

    let mut success_count = 0;
    let mut total = 0;

    for i in 0..test_count.saturating_sub(1) {
        for op in [
            Operation::Union,
            Operation::Intersection,
            Operation::Difference,
            Operation::Xor,
        ] {
            let result = run_clip(&wetlands[i], &wetlands[i + 1], op);
            total += 1;

            if is_valid_result(&result) {
                success_count += 1;
            } else {
                println!("Invalid {:?} result for wetlands {} and {}", op, i, i + 1);
            }
        }
    }

    println!(
        "Wetland operations: {}/{} valid results",
        success_count, total
    );
    assert_eq!(
        success_count, total,
        "All wetland operations should produce valid results"
    );
}
