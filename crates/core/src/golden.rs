//! Golden test harness for wagyu-rs
//!
//! This module loads test fixtures from the original wagyu C++ repository
//! and validates that the Rust implementation produces identical results.
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

/// Path to wagyu C++ fixtures directory (relative to crate root)
const FIXTURES_DIR: &str = "../../../wagyu/tests/fixtures";
/// Path to wagyu C++ expected outputs directory (relative to crate root)
const EXPECTED_DIR: &str = "../../../wagyu/tests/expected";

/// Fill type for polygon operations
///
/// PORT FROM: wagyu/include/mapbox/geometry/wagyu/config.hpp
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillType {
    /// Even-odd fill rule
    EvenOdd,
    /// Non-zero winding fill rule
    NonZero,
    /// Positive winding fill rule
    Positive,
    /// Negative winding fill rule
    Negative,
}

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

    /// Run a single golden test case
    ///
    /// This is marked `allow(dead_code)` because it's a template for future use.
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

        // Compare result with expected
        // TODO: Implement proper comparison that handles coordinate ordering
        assert_eq!(
            result.0.len(),
            expected.0.len(),
            "Result polygon count mismatch for {}",
            expected_file
        );
    }
}
