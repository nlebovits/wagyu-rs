//! Issue #106: Clip operation produces coordinates outside clip box.
//!
//! When clipping a U-shaped polygon that extends significantly outside the clip
//! box, the output contains coordinates that are outside the clip boundary.
//!
//! **Bug found during reproduction**: The intersection operation loses the
//! entire left arm of the U-shape. Only the right arm appears in the output.
//! This is a more severe manifestation of issue #106 -- rather than producing
//! slightly out-of-bounds coordinates, entire geometry is dropped.
//!
//! Scenario from the issue:
//! - Subject: U-shaped polygon with vertices at y=0.0 to y=2.0 (geographic coords)
//! - Clip box: y=1.0 to y=2.5 (mapped to tile coords 0-4096)
//! - Operation: Intersection with EvenOdd fill
//! - Expected: Two rectangles (both arms of U above clip line), all coordinates
//!   within clip boundary [0, 4096]
//! - Actual: Only one rectangle (right arm); left arm is completely missing

use wagyu_rs::{config::FillType, config::PolygonType, point::Point, wagyu::Wagyu, Operation};

/// Map geographic coordinates to tile coordinates.
fn geo_to_tile(
    geo_x: f64,
    geo_y: f64,
    x_min: f64,
    y_min: f64,
    x_max: f64,
    y_max: f64,
) -> (i64, i64) {
    let tile_x = ((geo_x - x_min) / (x_max - x_min) * 4096.0).round() as i64;
    let tile_y = ((geo_y - y_min) / (y_max - y_min) * 4096.0).round() as i64;
    (tile_x, tile_y)
}

/// Build the U-shaped polygon and clip box from the issue description.
///
/// Geographic U-shape:
///   (0.0, 0.0) -> (0.0, 2.0) -> (0.3, 2.0) -> (0.3, 0.5) ->
///   (0.7, 0.5) -> (0.7, 2.0) -> (1.0, 2.0) -> (1.0, 0.0)
///
/// Clip box: x=[0.0, 1.0], y=[1.0, 2.5]
/// Mapped to tile coords: [0, 4096] x [0, 4096]
///
/// After clipping, the expected result is two rectangles:
///   Left arm:  (0, 0) -> (0, 2731) -> (1229, 2731) -> (1229, 0)
///   Right arm: (2867, 0) -> (2867, 2731) -> (4096, 2731) -> (4096, 0)
fn build_issue_106_polygons() -> (Vec<Point<i64>>, Vec<Point<i64>>) {
    let x_min = 0.0_f64;
    let y_min = 1.0_f64;
    let x_max = 1.0_f64;
    let y_max = 2.5_f64;

    let u_geo = vec![
        (0.0, 0.0),
        (0.0, 2.0),
        (0.3, 2.0),
        (0.3, 0.5),
        (0.7, 0.5),
        (0.7, 2.0),
        (1.0, 2.0),
        (1.0, 0.0),
        (0.0, 0.0),
    ];

    let subject: Vec<Point<i64>> = u_geo
        .iter()
        .map(|&(x, y)| {
            let (tx, ty) = geo_to_tile(x, y, x_min, y_min, x_max, y_max);
            Point::new(tx, ty)
        })
        .collect();

    let clip = vec![
        Point::new(0_i64, 0),
        Point::new(4096, 0),
        Point::new(4096, 4096),
        Point::new(0, 4096),
        Point::new(0, 0),
    ];

    (subject, clip)
}

/// PRIMARY FAILING TEST: The U-shape clipped by the box should produce
/// BOTH arms of the U, not just one.
///
/// The Vatti sweep only creates a ring for the right arm (x=2867 to x=4096).
/// The left arm (x=0 to x=1229) is completely lost.
///
/// Debug output shows:
///   [VATTI_START] minima=2 -- only 2 local minima found
///   [VATTI_END] rings=1 -- only 1 ring produced
///   Only ring: (4096,2731) -> (2867,2731) -> (2867,0) -> (4096,0)
///   Missing: the left arm rectangle
#[test]
fn issue_106_u_shape_intersection_drops_left_arm() {
    let (subject, clip) = build_issue_106_polygons();

    println!("Subject (U-shape, tile coords):");
    for (i, p) in subject.iter().enumerate() {
        println!("  [{}] ({}, {})", i, p.x, p.y);
    }

    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![subject], PolygonType::Subject);
    wagyu.add_polygon(&vec![clip], PolygonType::Clip);

    let result = wagyu
        .execute(
            Operation::Intersection,
            FillType::EvenOdd,
            FillType::EvenOdd,
        )
        .expect("Intersection should succeed");

    println!("\nResult: {} polygon(s)", result.0.len());
    for (i, poly) in result.0.iter().enumerate() {
        let ext = poly.exterior();
        println!("  Polygon {} ({} coords):", i, ext.0.len());
        for (j, coord) in ext.0.iter().enumerate() {
            println!("    [{}] ({}, {})", j, coord.x, coord.y);
        }
    }

    // The intersection of the U-shape with the clip box should produce
    // 2 separate polygons (the two arms of the U above the clip line):
    //   Left arm:  (0, 0)-(0, 2731)-(1229, 2731)-(1229, 0)
    //   Right arm: (2867, 0)-(2867, 2731)-(4096, 2731)-(4096, 0)
    assert!(
        result.0.len() >= 2,
        "Intersection of U-shape with clip box should produce at least 2 polygons \
         (both arms of the U above the clip line), but got {}.\n\
         The Vatti sweep is only finding the right arm and dropping the left arm entirely.\n\
         Debug: run with WAGYU_DEBUG=1 to see sweep details.",
        result.0.len()
    );
}

/// SECONDARY TEST: Even if we accept the dropped left arm, verify that all
/// coordinates in the output are within the clip box bounds.
#[test]
fn issue_106_all_output_coords_within_clip_bounds() {
    let (subject, clip) = build_issue_106_polygons();

    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![subject], PolygonType::Subject);
    wagyu.add_polygon(&vec![clip], PolygonType::Clip);

    let result = wagyu
        .execute(
            Operation::Intersection,
            FillType::EvenOdd,
            FillType::EvenOdd,
        )
        .expect("Intersection should succeed");

    let mut out_of_bounds = Vec::new();

    for (i, poly) in result.0.iter().enumerate() {
        for (j, coord) in poly.exterior().0.iter().enumerate() {
            if coord.x < 0 || coord.x > 4096 || coord.y < 0 || coord.y > 4096 {
                out_of_bounds.push(format!(
                    "  Polygon {} ext coord [{}]: ({}, {})",
                    i, j, coord.x, coord.y
                ));
            }
        }
        for (k, hole) in poly.interiors().iter().enumerate() {
            for (j, coord) in hole.0.iter().enumerate() {
                if coord.x < 0 || coord.x > 4096 || coord.y < 0 || coord.y > 4096 {
                    out_of_bounds.push(format!(
                        "  Polygon {} hole {} coord [{}]: ({}, {})",
                        i, k, j, coord.x, coord.y
                    ));
                }
            }
        }
    }

    assert!(
        out_of_bounds.is_empty(),
        "Clip produced coordinates outside [0, 4096]:\n{}",
        out_of_bounds.join("\n")
    );
}

/// SIMPLEST REPRODUCTION: Two separate rectangles as subject, clip box
/// that should preserve both.
///
/// If the issue is specific to the U-shape (concave polygon), this simpler
/// test with two separate rectangles should pass. If it also fails, the
/// bug is more fundamental.
#[test]
fn issue_106_two_separate_rectangles_both_preserved() {
    // Left rectangle: entirely within clip box
    let left_rect = vec![
        Point::new(0_i64, 0),
        Point::new(1229, 0),
        Point::new(1229, 2731),
        Point::new(0, 2731),
        Point::new(0, 0),
    ];

    // Right rectangle: entirely within clip box
    let right_rect = vec![
        Point::new(2867_i64, 0),
        Point::new(4096, 0),
        Point::new(4096, 2731),
        Point::new(2867, 2731),
        Point::new(2867, 0),
    ];

    let clip = vec![
        Point::new(0_i64, 0),
        Point::new(4096, 0),
        Point::new(4096, 4096),
        Point::new(0, 4096),
        Point::new(0, 0),
    ];

    // Add both rectangles as separate subject polygons
    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![left_rect], PolygonType::Subject);
    wagyu.add_polygon(&vec![right_rect], PolygonType::Subject);
    wagyu.add_polygon(&vec![clip], PolygonType::Clip);

    let result = wagyu
        .execute(
            Operation::Intersection,
            FillType::EvenOdd,
            FillType::EvenOdd,
        )
        .expect("Intersection should succeed");

    println!("Two separate rectangles result: {} polygon(s)", result.0.len());
    for (i, poly) in result.0.iter().enumerate() {
        let ext = poly.exterior();
        println!("  Polygon {} ({} coords):", i, ext.0.len());
        for (j, coord) in ext.0.iter().enumerate() {
            println!("    [{}] ({}, {})", j, coord.x, coord.y);
        }
    }

    // Both rectangles are entirely within the clip box, so intersection
    // should preserve both.
    assert_eq!(
        result.0.len(),
        2,
        "Intersection with two separate subject rectangles inside clip box \
         should produce 2 polygons, got {}",
        result.0.len()
    );
}

/// MINIMAL U-SHAPE: Axis-aligned U in tile coordinates (no geographic mapping).
///
/// This is the simplest possible reproduction of the U-shape clipping bug,
/// using direct integer coordinates to eliminate any coordinate mapping issues.
#[test]
fn issue_106_minimal_u_shape_direct_coords() {
    // Simple U-shape in tile coordinates:
    //
    //  (0,1000)----(300,1000)       (700,1000)----(1000,1000)
    //     |             |               |              |
    //     |             |               |              |
    //  (0,-1000)        (300,-500)---(700,-500)    (1000,-1000)
    //     |                                            |
    //     +--------------------------------------------+
    //
    // This U extends from y=-1000 to y=1000.
    // Clip box: (0, 0) to (1000, 1000)
    // Expected result: two rectangles above y=0

    let subject = vec![
        Point::new(0_i64, -1000),     // bottom-left
        Point::new(0, 1000),          // top-left outer
        Point::new(300, 1000),        // top-left inner
        Point::new(300, -500),        // inside left arm
        Point::new(700, -500),        // inside bottom
        Point::new(700, 1000),        // top-right inner
        Point::new(1000, 1000),       // top-right outer
        Point::new(1000, -1000),      // bottom-right
        Point::new(0, -1000),         // close
    ];

    let clip = vec![
        Point::new(0_i64, 0),
        Point::new(1000, 0),
        Point::new(1000, 1000),
        Point::new(0, 1000),
        Point::new(0, 0),
    ];

    println!("=== Minimal U-shape (direct coords) ===");
    println!("Subject:");
    for (i, p) in subject.iter().enumerate() {
        println!("  [{}] ({}, {})", i, p.x, p.y);
    }
    println!("Clip: (0, 0) to (1000, 1000)");

    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![subject], PolygonType::Subject);
    wagyu.add_polygon(&vec![clip], PolygonType::Clip);

    let result = wagyu
        .execute(
            Operation::Intersection,
            FillType::EvenOdd,
            FillType::EvenOdd,
        )
        .expect("Intersection should succeed");

    println!("\nResult: {} polygon(s)", result.0.len());
    for (i, poly) in result.0.iter().enumerate() {
        let ext = poly.exterior();
        println!("  Polygon {} ({} coords):", i, ext.0.len());
        for (j, coord) in ext.0.iter().enumerate() {
            println!("    [{}] ({}, {})", j, coord.x, coord.y);
        }
    }

    // Check 1: All coordinates within clip bounds
    let mut out_of_bounds = Vec::new();
    for (i, poly) in result.0.iter().enumerate() {
        for (j, coord) in poly.exterior().0.iter().enumerate() {
            if coord.x < 0 || coord.x > 1000 || coord.y < 0 || coord.y > 1000 {
                out_of_bounds.push(format!(
                    "  Polygon {} coord [{}]: ({}, {})",
                    i, j, coord.x, coord.y
                ));
            }
        }
    }
    assert!(
        out_of_bounds.is_empty(),
        "Clip produced coordinates outside [0, 1000]:\n{}",
        out_of_bounds.join("\n")
    );

    // Check 2: Should produce 2 polygons (both arms of U above clip line)
    assert_eq!(
        result.0.len(),
        2,
        "U-shape intersection with clip box should produce 2 polygons \
         (left arm and right arm above clip line), got {}.\n\
         The Vatti sweep is dropping one arm of the U.",
        result.0.len()
    );
}

/// Verify the U-shape works correctly with Union (to rule out operation-specific bugs).
#[test]
fn issue_106_u_shape_union_for_comparison() {
    let (subject, clip) = build_issue_106_polygons();

    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![subject], PolygonType::Subject);
    wagyu.add_polygon(&vec![clip], PolygonType::Clip);

    let result = wagyu
        .execute(
            Operation::Union,
            FillType::EvenOdd,
            FillType::EvenOdd,
        )
        .expect("Union should succeed");

    println!("Union result: {} polygon(s)", result.0.len());
    for (i, poly) in result.0.iter().enumerate() {
        let ext = poly.exterior();
        println!("  Polygon {} ({} coords):", i, ext.0.len());
        for (j, coord) in ext.0.iter().enumerate() {
            println!("    [{}] ({}, {})", j, coord.x, coord.y);
        }
    }

    // Union should produce at least 1 polygon
    assert!(
        !result.0.is_empty(),
        "Union of U-shape with clip box should produce at least 1 polygon"
    );
}

/// DIAGNOSTIC TEST: U-shape offset from clip box edges.
///
/// The original U-shape has its left outer edge at x=0, which coincides with
/// the clip box's left edge. This test offsets the U-shape inward so no
/// edges share coordinates with the clip boundary.
///
/// If this test passes (both arms present), the bug is specific to shared
/// boundary edges. If it also fails, the bug is in concave polygon handling.
#[test]
fn issue_106_u_shape_offset_from_clip_edges() {
    // U-shape offset by 100 units from all clip box edges
    let subject = vec![
        Point::new(100_i64, -1000),
        Point::new(100, 900),
        Point::new(300, 900),
        Point::new(300, -500),
        Point::new(700, -500),
        Point::new(700, 900),
        Point::new(900, 900),
        Point::new(900, -1000),
        Point::new(100, -1000),
    ];

    let clip = vec![
        Point::new(0_i64, 0),
        Point::new(1000, 0),
        Point::new(1000, 1000),
        Point::new(0, 1000),
        Point::new(0, 0),
    ];

    println!("=== Offset U-shape ===");
    println!("Subject:");
    for (i, p) in subject.iter().enumerate() {
        println!("  [{}] ({}, {})", i, p.x, p.y);
    }
    println!("Clip: (0, 0) to (1000, 1000)");

    let mut wagyu: Wagyu<i64> = Wagyu::new();
    wagyu.add_polygon(&vec![subject], PolygonType::Subject);
    wagyu.add_polygon(&vec![clip], PolygonType::Clip);

    let result = wagyu
        .execute(
            Operation::Intersection,
            FillType::EvenOdd,
            FillType::EvenOdd,
        )
        .expect("Intersection should succeed");

    println!("\nResult: {} polygon(s)", result.0.len());
    for (i, poly) in result.0.iter().enumerate() {
        let ext = poly.exterior();
        println!("  Polygon {} ({} coords):", i, ext.0.len());
        for (j, coord) in ext.0.iter().enumerate() {
            println!("    [{}] ({}, {})", j, coord.x, coord.y);
        }
    }

    // Should produce 2 polygons (both arms of the U above clip line)
    assert_eq!(
        result.0.len(),
        2,
        "Offset U-shape intersection should produce 2 polygons, got {}.\n\
         If this fails, the bug is in concave polygon handling (not shared edges).",
        result.0.len()
    );
}
