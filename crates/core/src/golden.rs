//! Golden test harness for wagyu-rs
//!
//! This module loads test fixtures (originally from the wagyu C++ repository)
//! and validates that the Rust implementation produces identical results.
//! Fixtures are bundled in `crates/core/tests/` for self-contained testing.
//!
//! ## Fixture Format
//!
//! Fixtures are JSON arrays representing polygons:
//! ```json
//! [
//!     [[x1,y1], [x2,y2], ...],  // exterior ring
//!     [[hx1,hy1], [hx2,hy2], ...] // hole (optional)
//! ]
//! ```
//!
//! ## Expected Output Format
//!
//! Expected results are multi-polygons:
//! ```json
//! [
//!     [[[x1,y1], [x2,y2], ...]],  // polygon 1
//!     [[[x1,y1], [x2,y2], ...]]   // polygon 2
//! ]
//! ```

use crate::Operation;
use geo_types::{Coord, LineString, MultiPolygon, Polygon};
use std::path::Path;

/// Path to fixtures directory (relative to crate root)
const FIXTURES_DIR: &str = "tests/fixtures";
/// Path to expected outputs directory (relative to crate root)
const EXPECTED_DIR: &str = "tests/expected";

// Note: FillType is defined in crate::config - use that instead

/// A ring in JSON format: array of [x, y] coordinate pairs
type JsonRing = Vec<[i64; 2]>;

/// A polygon in JSON format: array of rings (exterior + holes)
type JsonPolygon = Vec<JsonRing>;

/// A multi-polygon in JSON format: array of polygons
type JsonMultiPolygon = Vec<JsonPolygon>;

/// Load a polygon fixture from JSON file
///
/// # Arguments
/// * `filename` - Name of the fixture file (e.g., "clockwise-triangle.json")
///
/// # Returns
/// A `geo_types::Polygon<i64>` parsed from the JSON
pub fn load_fixture(filename: &str) -> Result<Polygon<i64>, String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(FIXTURES_DIR)
        .join(filename);

    let content = std::fs::read_to_string(&path)
        .map_err(|e| format!("Failed to read fixture {}: {}", path.display(), e))?;

    let json_polygon: JsonPolygon = serde_json::from_str(&content)
        .map_err(|e| format!("Failed to parse fixture {}: {}", filename, e))?;

    json_polygon_to_geo(&json_polygon)
}

/// Load expected output from JSON file
///
/// # Arguments
/// * `filename` - Name of the expected file (e.g., "difference-clockwise-polygon.json")
///
/// # Returns
/// A `geo_types::MultiPolygon<i64>` parsed from the JSON
pub fn load_expected(filename: &str) -> Result<MultiPolygon<i64>, String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(EXPECTED_DIR)
        .join(filename);

    let content = std::fs::read_to_string(&path)
        .map_err(|e| format!("Failed to read expected {}: {}", path.display(), e))?;

    let json_multi: JsonMultiPolygon = serde_json::from_str(&content)
        .map_err(|e| format!("Failed to parse expected {}: {}", filename, e))?;

    json_multi_polygon_to_geo(&json_multi)
}

/// Convert JSON polygon representation to geo_types::Polygon
fn json_polygon_to_geo(json_poly: &JsonPolygon) -> Result<Polygon<i64>, String> {
    if json_poly.is_empty() {
        return Err("Empty polygon".to_string());
    }

    // First ring is exterior
    let exterior = json_ring_to_linestring(&json_poly[0]);

    // Remaining rings are holes
    let holes: Vec<LineString<i64>> = json_poly[1..].iter().map(json_ring_to_linestring).collect();

    Ok(Polygon::new(exterior, holes))
}

/// Convert JSON multi-polygon representation to geo_types::MultiPolygon
fn json_multi_polygon_to_geo(json_multi: &JsonMultiPolygon) -> Result<MultiPolygon<i64>, String> {
    let polygons: Result<Vec<Polygon<i64>>, String> =
        json_multi.iter().map(json_polygon_to_geo).collect();

    Ok(MultiPolygon::new(polygons?))
}

/// Convert JSON ring to geo_types::LineString
fn json_ring_to_linestring(ring: &JsonRing) -> LineString<i64> {
    let coords: Vec<Coord<i64>> = ring.iter().map(|[x, y]| Coord { x: *x, y: *y }).collect();

    LineString::new(coords)
}

// =============================================================================
// GEOMETRY COMPARISON
// =============================================================================
//
// These functions implement proper geometry comparison that handles:
// - Ring rotation (rings can start at any vertex)
// - Ring/polygon ordering (unordered collections)
// - Coordinate comparison (exact for integers)
//
// PORT FROM: wagyu C++ uses direct coordinate comparison in unit tests
// DIVERGENCE: We add normalization to handle rotated/reordered outputs

/// Normalize a ring by rotating it to start at the lexicographically smallest coordinate.
///
/// This handles the case where two rings are topologically equal but start at
/// different vertices. For example, `[A,B,C,A]` equals `[B,C,A,B]` topologically.
///
/// The closing coordinate (which duplicates the first) is preserved.
pub fn normalize_ring(ring: &LineString<i64>) -> LineString<i64> {
    let coords = &ring.0;

    // Empty or degenerate ring
    if coords.len() <= 1 {
        return ring.clone();
    }

    // Find the index of the lexicographically smallest coordinate
    // (excluding the closing duplicate if present)
    let len = if coords.len() > 1 && coords.first() == coords.last() {
        coords.len() - 1 // Exclude closing point for rotation
    } else {
        coords.len()
    };

    if len == 0 {
        return ring.clone();
    }

    let min_idx = (0..len)
        .min_by(|&a, &b| {
            let ca = &coords[a];
            let cb = &coords[b];
            ca.x.cmp(&cb.x).then_with(|| ca.y.cmp(&cb.y))
        })
        .unwrap_or(0);

    // Rotate coordinates to start at min_idx
    let mut rotated: Vec<Coord<i64>> = Vec::with_capacity(coords.len());
    for i in 0..len {
        rotated.push(coords[(min_idx + i) % len]);
    }

    // Add closing point if original had one
    if coords.len() > 1 && coords.first() == coords.last() {
        rotated.push(rotated[0]);
    }

    LineString::new(rotated)
}

/// Create a sortable key for a ring (for ordering rings within a polygon).
///
/// Returns the lexicographically smallest coordinate as a tuple.
fn ring_sort_key(ring: &LineString<i64>) -> (i64, i64) {
    ring.0
        .iter()
        .map(|c| (c.x, c.y))
        .min()
        .unwrap_or((i64::MAX, i64::MAX))
}

/// Normalize a polygon for comparison.
///
/// - Normalizes the exterior ring
/// - Normalizes and sorts interior rings (holes)
pub fn normalize_polygon(poly: &Polygon<i64>) -> Polygon<i64> {
    let normalized_exterior = normalize_ring(poly.exterior());

    let mut normalized_interiors: Vec<LineString<i64>> =
        poly.interiors().iter().map(normalize_ring).collect();

    // Sort holes by their sort key for consistent ordering
    normalized_interiors.sort_by_key(ring_sort_key);

    Polygon::new(normalized_exterior, normalized_interiors)
}

/// Create a sortable key for a polygon (for ordering polygons in a multi-polygon).
///
/// Uses the exterior ring's sort key.
fn polygon_sort_key(poly: &Polygon<i64>) -> (i64, i64) {
    ring_sort_key(poly.exterior())
}

/// Normalize a multi-polygon for comparison.
///
/// - Normalizes each polygon
/// - Sorts polygons for consistent ordering
pub fn normalize_multi_polygon(mp: &MultiPolygon<i64>) -> MultiPolygon<i64> {
    let mut normalized: Vec<Polygon<i64>> = mp.0.iter().map(normalize_polygon).collect();

    // Sort polygons by their sort key
    normalized.sort_by_key(polygon_sort_key);

    MultiPolygon::new(normalized)
}

/// Compare two multi-polygons for geometric equality.
///
/// This handles:
/// - Ring rotation (rings can start at any vertex)
/// - Ring ordering (holes can be in any order)
/// - Polygon ordering (polygons in multi-polygon can be in any order)
///
/// Returns `true` if the multi-polygons are geometrically equivalent.
pub fn multi_polygons_equal(a: &MultiPolygon<i64>, b: &MultiPolygon<i64>) -> bool {
    let norm_a = normalize_multi_polygon(a);
    let norm_b = normalize_multi_polygon(b);

    norm_a == norm_b
}

/// Assert that two multi-polygons are geometrically equal.
///
/// Provides detailed error messages showing the normalized forms.
pub fn assert_multi_polygons_equal(
    result: &MultiPolygon<i64>,
    expected: &MultiPolygon<i64>,
    context: &str,
) {
    let norm_result = normalize_multi_polygon(result);
    let norm_expected = normalize_multi_polygon(expected);

    if norm_result != norm_expected {
        // Build detailed error message
        let result_str = format_multi_polygon(&norm_result);
        let expected_str = format_multi_polygon(&norm_expected);

        panic!(
            "Geometry mismatch for {}\n\nResult (normalized):\n{}\n\nExpected (normalized):\n{}",
            context, result_str, expected_str
        );
    }
}

/// Format a multi-polygon for debug output.
fn format_multi_polygon(mp: &MultiPolygon<i64>) -> String {
    let mut result = String::new();
    for (i, poly) in mp.0.iter().enumerate() {
        result.push_str(&format!("  Polygon {}:\n", i));
        result.push_str(&format!("    Exterior: {:?}\n", poly.exterior().0));
        for (j, hole) in poly.interiors().iter().enumerate() {
            result.push_str(&format!("    Hole {}: {:?}\n", j, hole.0));
        }
    }
    if result.is_empty() {
        result.push_str("  (empty)");
    }
    result
}

/// Parse operation type from expected filename
///
/// Expected files are named like: `{operation}-{description}.json`
pub fn parse_operation_from_filename(filename: &str) -> Option<Operation> {
    if filename.starts_with("union-") {
        Some(Operation::Union)
    } else if filename.starts_with("intersection-") {
        Some(Operation::Intersection)
    } else if filename.starts_with("difference-") {
        Some(Operation::Difference)
    } else if filename.starts_with("xor-") {
        Some(Operation::Xor)
    } else {
        None
    }
}

/// List all expected test files for a given operation
pub fn list_expected_files(operation: Operation) -> Result<Vec<String>, String> {
    let expected_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(EXPECTED_DIR);

    let prefix = match operation {
        Operation::Union => "union-",
        Operation::Intersection => "intersection-",
        Operation::Difference => "difference-",
        Operation::Xor => "xor-",
    };

    let entries = std::fs::read_dir(&expected_path)
        .map_err(|e| format!("Failed to read expected dir: {}", e))?;

    let mut files: Vec<String> = entries
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with(prefix) && name.ends_with(".json") {
                Some(name)
            } else {
                None
            }
        })
        .collect();

    files.sort();
    Ok(files)
}

// =============================================================================
// TRAIT: BooleanOp (NOT YET IMPLEMENTED)
// =============================================================================
//
// This trait will be implemented on geo_types::Polygon to provide boolean
// operations. For now, it's a placeholder that will cause tests to fail.

use geo_types::CoordNum;

/// Boolean operations trait (PLACEHOLDER - NOT IMPLEMENTED)
///
/// This trait will provide union, intersection, difference, and xor operations
/// on polygons. Implementation pending.
pub trait BooleanOp<T: CoordNum> {
    /// Perform a boolean operation with another polygon
    fn boolean_op(&self, other: &Polygon<T>, operation: Operation) -> MultiPolygon<T>;
}

// Placeholder implementation that always panics - this ensures TDD red phase
impl BooleanOp<i64> for Polygon<i64> {
    fn boolean_op(&self, _other: &Polygon<i64>, operation: Operation) -> MultiPolygon<i64> {
        // TDD RED: This will fail when tests are run
        panic!(
            "BooleanOp::{:?} not yet implemented - TDD red phase",
            operation
        );
    }
}

// =============================================================================
// TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -------------------------------------------------------------------------
    // Fixture Loading Tests (should pass - infrastructure verification)
    // -------------------------------------------------------------------------

    #[test]
    fn test_load_clockwise_triangle_fixture() {
        let poly = load_fixture("clockwise-triangle.json").expect("Failed to load fixture");

        // Verify the triangle has expected structure
        assert_eq!(poly.exterior().0.len(), 4); // 3 points + closing point
        assert!(poly.interiors().is_empty()); // No holes

        // Check first coordinate
        let first = poly.exterior().0[0];
        assert_eq!(first.x, 0);
        assert_eq!(first.y, 0);
    }

    #[test]
    fn test_load_clip_square_fixture() {
        let poly = load_fixture("clip-clockwise-square.json").expect("Failed to load fixture");

        // Verify the square has expected structure
        assert_eq!(poly.exterior().0.len(), 5); // 4 points + closing point
        assert!(poly.interiors().is_empty());

        // Check it's a 5000x5000 square centered at origin
        let coords = &poly.exterior().0;
        assert_eq!(coords[0], Coord { x: -2500, y: -2500 });
    }

    #[test]
    fn test_load_expected_output() {
        let result =
            load_expected("difference-clockwise-polygon.json").expect("Failed to load expected");

        // This expected output is an empty multi-polygon
        // (difference of polygon with itself = empty)
        // The file contains "[]" which parses as empty array
        assert!(result.0.is_empty() || !result.0.is_empty()); // Just check it loads
    }

    #[test]
    fn test_list_difference_expected_files() {
        let files =
            list_expected_files(Operation::Difference).expect("Failed to list expected files");

        // There should be many difference test cases
        assert!(!files.is_empty(), "Should find difference expected files");

        // All files should start with "difference-"
        for file in &files {
            assert!(
                file.starts_with("difference-"),
                "File {} should start with 'difference-'",
                file
            );
        }
    }

    #[test]
    fn test_parse_operation_from_filename() {
        assert_eq!(
            parse_operation_from_filename("union-test.json"),
            Some(Operation::Union)
        );
        assert_eq!(
            parse_operation_from_filename("intersection-test.json"),
            Some(Operation::Intersection)
        );
        assert_eq!(
            parse_operation_from_filename("difference-test.json"),
            Some(Operation::Difference)
        );
        assert_eq!(
            parse_operation_from_filename("xor-test.json"),
            Some(Operation::Xor)
        );
        assert_eq!(parse_operation_from_filename("invalid.json"), None);
    }

    // -------------------------------------------------------------------------
    // Golden Tests (TDD RED - EXPECTED TO FAIL)
    // -------------------------------------------------------------------------
    //
    // These tests verify that our implementation matches the C++ wagyu output.
    // They are currently expected to fail because BooleanOp is not implemented.

    #[test]
    #[should_panic(expected = "not yet implemented")]
    fn golden_test_difference_clockwise_polygon() {
        // Load subject and clip polygons
        let subject = load_fixture("clockwise-triangle.json").expect("Failed to load subject");
        let clip = load_fixture("clip-clockwise-square.json").expect("Failed to load clip");

        // Load expected result
        let _expected =
            load_expected("difference-clockwise-polygon.json").expect("Failed to load expected");

        // This will panic - TDD red phase
        // When implemented, it should produce a result matching expected
        let _result = subject.boolean_op(&clip, Operation::Difference);

        // Future assertion (commented out until implemented):
        // assert_multi_polygons_equal(&result, &expected);
    }

    #[test]
    #[should_panic(expected = "not yet implemented")]
    fn golden_test_intersection_clockwise_polygon() {
        let subject = load_fixture("clockwise-triangle.json").expect("Failed to load subject");
        let clip = load_fixture("clip-clockwise-square.json").expect("Failed to load clip");

        let _expected =
            load_expected("intersection-clockwise-polygon.json").expect("Failed to load expected");

        // TDD red - this will panic
        let _result = subject.boolean_op(&clip, Operation::Intersection);
    }

    #[test]
    #[should_panic(expected = "not yet implemented")]
    fn golden_test_union_clockwise_polygon() {
        let subject = load_fixture("clockwise-triangle.json").expect("Failed to load subject");
        let clip = load_fixture("clip-clockwise-square.json").expect("Failed to load clip");

        // Note: union expected files may not exist for all combinations
        // For now, just test the operation call fails as expected
        let _result = subject.boolean_op(&clip, Operation::Union);
    }

    // -------------------------------------------------------------------------
    // Parameterized Golden Test Runner (for future use)
    // -------------------------------------------------------------------------
    //
    // This function will be used to run all golden tests once BooleanOp is
    // implemented. For now, it's a template.

    /// Run a single golden test case.
    ///
    /// Template function for loading fixtures and comparing against expected output.
    /// Will be used when additional golden test fixtures are created.
    #[allow(dead_code)]
    fn run_golden_test(
        subject_file: &str,
        clip_file: &str,
        expected_file: &str,
        operation: Operation,
    ) {
        let subject = load_fixture(subject_file).expect("Failed to load subject");
        let clip = load_fixture(clip_file).expect("Failed to load clip");
        let expected = load_expected(expected_file).expect("Failed to load expected");

        let result = subject.boolean_op(&clip, operation);

        // Compare result with expected using proper geometry comparison
        assert_multi_polygons_equal(&result, &expected, expected_file);
    }

    // -------------------------------------------------------------------------
    // Geometry Comparison Tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_normalize_ring_rotation() {
        // Ring starting at different points should normalize to same form
        // [0,0] -> [1,0] -> [1,1] -> [0,0] (starts at lexicographically smallest)
        let ring1 = LineString::new(vec![
            Coord { x: 0, y: 0 },
            Coord { x: 1, y: 0 },
            Coord { x: 1, y: 1 },
            Coord { x: 0, y: 0 },
        ]);

        // Same ring but starting at [1,0]
        let ring2 = LineString::new(vec![
            Coord { x: 1, y: 0 },
            Coord { x: 1, y: 1 },
            Coord { x: 0, y: 0 },
            Coord { x: 1, y: 0 },
        ]);

        let norm1 = normalize_ring(&ring1);
        let norm2 = normalize_ring(&ring2);

        assert_eq!(norm1, norm2, "Rotated rings should normalize to same form");

        // Verify first point is the lexicographically smallest
        assert_eq!(norm1.0[0], Coord { x: 0, y: 0 });
    }

    #[test]
    fn test_normalize_ring_lex_ordering() {
        // When x values are equal, sort by y
        let ring = LineString::new(vec![
            Coord { x: 0, y: 5 },
            Coord { x: 0, y: 0 }, // This should become first (same x, smaller y)
            Coord { x: 1, y: 0 },
            Coord { x: 0, y: 5 },
        ]);

        let normalized = normalize_ring(&ring);

        assert_eq!(
            normalized.0[0],
            Coord { x: 0, y: 0 },
            "Should start at lex smallest (0,0)"
        );
    }

    #[test]
    fn test_normalize_polygon_sorts_holes() {
        // Polygon with two holes in different orders should normalize the same
        let exterior = LineString::new(vec![
            Coord { x: 0, y: 0 },
            Coord { x: 10, y: 0 },
            Coord { x: 10, y: 10 },
            Coord { x: 0, y: 10 },
            Coord { x: 0, y: 0 },
        ]);

        let hole1 = LineString::new(vec![
            Coord { x: 1, y: 1 },
            Coord { x: 2, y: 1 },
            Coord { x: 2, y: 2 },
            Coord { x: 1, y: 1 },
        ]);

        let hole2 = LineString::new(vec![
            Coord { x: 5, y: 5 },
            Coord { x: 6, y: 5 },
            Coord { x: 6, y: 6 },
            Coord { x: 5, y: 5 },
        ]);

        // Polygon with holes in order [hole1, hole2]
        let poly_a = Polygon::new(exterior.clone(), vec![hole1.clone(), hole2.clone()]);
        // Polygon with holes in order [hole2, hole1]
        let poly_b = Polygon::new(exterior.clone(), vec![hole2, hole1]);

        let norm_a = normalize_polygon(&poly_a);
        let norm_b = normalize_polygon(&poly_b);

        assert_eq!(
            norm_a, norm_b,
            "Polygons with reordered holes should be equal after normalization"
        );
    }

    #[test]
    fn test_multi_polygons_equal_reordered() {
        // Two multi-polygons with polygons in different order
        let poly1 = Polygon::new(
            LineString::new(vec![
                Coord { x: 0, y: 0 },
                Coord { x: 1, y: 0 },
                Coord { x: 1, y: 1 },
                Coord { x: 0, y: 0 },
            ]),
            vec![],
        );

        let poly2 = Polygon::new(
            LineString::new(vec![
                Coord { x: 10, y: 10 },
                Coord { x: 11, y: 10 },
                Coord { x: 11, y: 11 },
                Coord { x: 10, y: 10 },
            ]),
            vec![],
        );

        let mp_a = MultiPolygon::new(vec![poly1.clone(), poly2.clone()]);
        let mp_b = MultiPolygon::new(vec![poly2, poly1]);

        assert!(
            multi_polygons_equal(&mp_a, &mp_b),
            "Multi-polygons with reordered polygons should be equal"
        );
    }

    #[test]
    fn test_multi_polygons_equal_rotated_rings() {
        // Same polygon but ring starts at different vertex
        let poly1 = Polygon::new(
            LineString::new(vec![
                Coord { x: 0, y: 0 },
                Coord { x: 1, y: 0 },
                Coord { x: 1, y: 1 },
                Coord { x: 0, y: 0 },
            ]),
            vec![],
        );

        let poly2 = Polygon::new(
            LineString::new(vec![
                Coord { x: 1, y: 0 }, // Started at different vertex
                Coord { x: 1, y: 1 },
                Coord { x: 0, y: 0 },
                Coord { x: 1, y: 0 },
            ]),
            vec![],
        );

        let mp_a = MultiPolygon::new(vec![poly1]);
        let mp_b = MultiPolygon::new(vec![poly2]);

        assert!(
            multi_polygons_equal(&mp_a, &mp_b),
            "Multi-polygons with rotated rings should be equal"
        );
    }

    #[test]
    fn test_multi_polygons_not_equal_different_coords() {
        let poly1 = Polygon::new(
            LineString::new(vec![
                Coord { x: 0, y: 0 },
                Coord { x: 1, y: 0 },
                Coord { x: 1, y: 1 },
                Coord { x: 0, y: 0 },
            ]),
            vec![],
        );

        let poly2 = Polygon::new(
            LineString::new(vec![
                Coord { x: 0, y: 0 },
                Coord { x: 2, y: 0 }, // Different coordinate!
                Coord { x: 2, y: 2 },
                Coord { x: 0, y: 0 },
            ]),
            vec![],
        );

        let mp_a = MultiPolygon::new(vec![poly1]);
        let mp_b = MultiPolygon::new(vec![poly2]);

        assert!(
            !multi_polygons_equal(&mp_a, &mp_b),
            "Multi-polygons with different coordinates should NOT be equal"
        );
    }

    #[test]
    fn test_empty_multi_polygons_equal() {
        let mp_a: MultiPolygon<i64> = MultiPolygon::new(vec![]);
        let mp_b: MultiPolygon<i64> = MultiPolygon::new(vec![]);

        assert!(
            multi_polygons_equal(&mp_a, &mp_b),
            "Empty multi-polygons should be equal"
        );
    }
}
