#!/usr/bin/env python3
"""
Oracle fuzz tester for wagyu-rs.

Generates random polygon pairs and validates Rust output matches C++ oracle.

Usage:
    ./fuzz.py [--count N] [--parallel N] [--seed SEED] [--save-all]

Options:
    --count N       Number of test cases to generate (default: 500)
    --parallel N    Number of parallel workers (default: CPU count)
    --seed SEED     Random seed for reproducibility
    --save-all      Save all test cases, not just failures
"""

import argparse
import json
import math
import os
import random
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional

# Polygon types for generation
POLYGON_TYPES = ["convex", "concave", "with_holes", "multi"]
OPERATIONS = ["union", "intersection", "difference", "xor"]
FILL_TYPES = ["evenodd", "nonzero"]

# Coordinate ranges for variety
COORD_RANGES = [
    (-10, 10),        # Small integers
    (-100, 100),      # Medium integers
    (-1000, 1000),    # Large integers
    (0, 100),         # Positive only
]


def generate_convex_polygon(
    num_vertices: int,
    center: Tuple[int, int],
    radius: int,
    rng: random.Random
) -> List[List[int]]:
    """Generate a convex polygon by placing points on a circle."""
    cx, cy = center
    angles = sorted([rng.uniform(0, 2 * math.pi) for _ in range(num_vertices)])

    points = []
    for angle in angles:
        x = int(cx + radius * math.cos(angle))
        y = int(cy + radius * math.sin(angle))
        points.append([x, y])

    # Close the ring
    points.append(points[0].copy())
    return points


def generate_concave_polygon(
    num_vertices: int,
    center: Tuple[int, int],
    radius: int,
    rng: random.Random
) -> List[List[int]]:
    """Generate a concave (star-like) polygon by varying radii."""
    cx, cy = center
    angles = sorted([rng.uniform(0, 2 * math.pi) for _ in range(num_vertices)])

    points = []
    for i, angle in enumerate(angles):
        # Alternate between inner and outer radius for star effect
        r = radius * (0.4 + 0.6 * rng.random()) if i % 2 == 0 else radius * 0.3
        x = int(cx + r * math.cos(angle))
        y = int(cy + r * math.sin(angle))
        points.append([x, y])

    # Close the ring
    points.append(points[0].copy())
    return points


def generate_simple_polygon(
    num_vertices: int,
    bbox: Tuple[int, int, int, int],
    rng: random.Random,
    convex: bool = True
) -> List[List[int]]:
    """Generate a simple polygon within a bounding box."""
    min_x, min_y, max_x, max_y = bbox
    cx = (min_x + max_x) // 2
    cy = (min_y + max_y) // 2
    radius = int(min(max_x - min_x, max_y - min_y) / 2 * 0.9)

    if convex:
        return generate_convex_polygon(num_vertices, (cx, cy), radius, rng)
    else:
        return generate_concave_polygon(num_vertices, (cx, cy), radius, rng)


def generate_polygon_with_hole(
    outer_vertices: int,
    hole_vertices: int,
    bbox: Tuple[int, int, int, int],
    rng: random.Random
) -> List[List[List[int]]]:
    """Generate a polygon with a hole."""
    min_x, min_y, max_x, max_y = bbox
    cx = (min_x + max_x) // 2
    cy = (min_y + max_y) // 2
    outer_radius = int(min(max_x - min_x, max_y - min_y) / 2 * 0.9)
    hole_radius = int(outer_radius * rng.uniform(0.2, 0.5))

    exterior = generate_convex_polygon(outer_vertices, (cx, cy), outer_radius, rng)
    hole = generate_convex_polygon(hole_vertices, (cx, cy), hole_radius, rng)
    # Reverse hole for correct winding
    hole = hole[::-1]

    return [exterior, hole]


def generate_multi_polygon(
    num_polygons: int,
    vertices_per_polygon: int,
    coord_range: Tuple[int, int],
    rng: random.Random
) -> List[List[List[List[int]]]]:
    """Generate a multi-polygon (multiple separate polygons)."""
    min_coord, max_coord = coord_range
    spread = max_coord - min_coord

    polygons = []
    for i in range(num_polygons):
        # Spread polygons across the coordinate space
        offset_x = min_coord + (i % 3) * spread // 3
        offset_y = min_coord + (i // 3) * spread // 3
        bbox_size = spread // 4

        bbox = (offset_x, offset_y, offset_x + bbox_size, offset_y + bbox_size)
        convex = rng.choice([True, False])
        ring = generate_simple_polygon(vertices_per_polygon, bbox, rng, convex)
        polygons.append([ring])

    return polygons


def generate_random_polygon_pair(rng: random.Random) -> dict:
    """Generate a random subject/clip polygon pair."""
    coord_range = rng.choice(COORD_RANGES)
    min_coord, max_coord = coord_range

    # Choose polygon types for subject and clip
    subject_type = rng.choice(POLYGON_TYPES)
    clip_type = rng.choice(POLYGON_TYPES)

    def make_polygon(poly_type: str) -> List:
        if poly_type == "convex":
            num_verts = rng.randint(3, 20)
            bbox = (min_coord, min_coord, max_coord, max_coord)
            ring = generate_simple_polygon(num_verts, bbox, rng, convex=True)
            return [[ring]]

        elif poly_type == "concave":
            num_verts = rng.randint(5, 50)
            bbox = (min_coord, min_coord, max_coord, max_coord)
            ring = generate_simple_polygon(num_verts, bbox, rng, convex=False)
            return [[ring]]

        elif poly_type == "with_holes":
            outer_verts = rng.randint(4, 12)
            hole_verts = rng.randint(3, 8)
            bbox = (min_coord, min_coord, max_coord, max_coord)
            rings = generate_polygon_with_hole(outer_verts, hole_verts, bbox, rng)
            return [rings]

        else:  # multi
            num_polys = rng.randint(2, 4)
            verts = rng.randint(3, 8)
            return generate_multi_polygon(num_polys, verts, coord_range, rng)

    return {
        "subject": make_polygon(subject_type),
        "clip": make_polygon(clip_type),
        "_meta": {
            "subject_type": subject_type,
            "clip_type": clip_type,
            "coord_range": coord_range,
        }
    }


@dataclass
class TestResult:
    """Result of a single fuzz test."""
    test_id: int
    operation: str
    fill_type: str
    passed: bool
    error: bool
    input_data: dict
    error_message: Optional[str] = None


def run_single_test(
    test_id: int,
    input_data: dict,
    operation: str,
    fill_type: str,
    script_dir: Path
) -> TestResult:
    """Run a single oracle comparison test."""
    # Remove metadata before writing JSON
    data_to_write = {k: v for k, v in input_data.items() if not k.startswith("_")}

    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(data_to_write, f)
        temp_file = f.name

    try:
        compare_script = script_dir / "compare.sh"
        result = subprocess.run(
            [str(compare_script), temp_file, operation, fill_type],
            capture_output=True,
            text=True,
            timeout=30
        )

        passed = result.returncode == 0
        error = result.returncode == 2
        error_msg = result.stderr if not passed else None

        return TestResult(
            test_id=test_id,
            operation=operation,
            fill_type=fill_type,
            passed=passed,
            error=error,
            input_data=input_data,
            error_message=error_msg
        )
    except subprocess.TimeoutExpired:
        return TestResult(
            test_id=test_id,
            operation=operation,
            fill_type=fill_type,
            passed=False,
            error=True,
            input_data=input_data,
            error_message="Timeout after 30 seconds"
        )
    except Exception as e:
        return TestResult(
            test_id=test_id,
            operation=operation,
            fill_type=fill_type,
            passed=False,
            error=True,
            input_data=input_data,
            error_message=str(e)
        )
    finally:
        os.unlink(temp_file)


def run_test_case(args: tuple) -> List[TestResult]:
    """Run all operations for a single test case (for parallel execution)."""
    test_id, seed, script_dir = args
    rng = random.Random(seed)
    input_data = generate_random_polygon_pair(rng)

    results = []
    for operation in OPERATIONS:
        fill_type = rng.choice(FILL_TYPES)
        result = run_single_test(test_id, input_data, operation, fill_type, script_dir)
        results.append(result)

    return results


def save_failure(result: TestResult, output_dir: Path):
    """Save a failing test case for reproduction."""
    filename = f"failure_{result.test_id}_{result.operation}_{result.fill_type}.json"
    filepath = output_dir / filename

    # Include metadata about the failure
    output = {
        **{k: v for k, v in result.input_data.items() if not k.startswith("_")},
        "_failure_info": {
            "operation": result.operation,
            "fill_type": result.fill_type,
            "error": result.error,
            "error_message": result.error_message,
            "meta": result.input_data.get("_meta", {}),
        }
    }

    with open(filepath, 'w') as f:
        json.dump(output, f, indent=2)

    return filepath


def main():
    parser = argparse.ArgumentParser(description="Oracle fuzz tester for wagyu-rs")
    parser.add_argument("--count", type=int, default=500, help="Number of test cases")
    parser.add_argument("--parallel", type=int, default=None, help="Number of parallel workers")
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    parser.add_argument("--save-all", action="store_true", help="Save all test cases")
    args = parser.parse_args()

    script_dir = Path(__file__).parent.resolve()
    output_dir = script_dir / "fuzz_failures"
    output_dir.mkdir(exist_ok=True)

    # Determine seed
    seed = args.seed if args.seed is not None else random.randint(0, 2**32 - 1)
    print(f"Fuzz seed: {seed}")
    print(f"Use --seed {seed} to reproduce this run")
    print()

    # Check prerequisites
    compare_script = script_dir / "compare.sh"
    if not compare_script.exists():
        print(f"Error: compare.sh not found at {compare_script}", file=sys.stderr)
        sys.exit(1)

    # Generate test arguments
    base_rng = random.Random(seed)
    test_args = [
        (i, base_rng.randint(0, 2**32 - 1), script_dir)
        for i in range(args.count)
    ]

    # Run tests
    total_tests = args.count * len(OPERATIONS)
    passed = 0
    failed = 0
    errors = 0
    failures: List[TestResult] = []

    print(f"Running {args.count} test cases ({total_tests} total comparisons)...")
    print()

    workers = args.parallel or os.cpu_count() or 4

    with ProcessPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(run_test_case, arg): arg[0] for arg in test_args}

        completed = 0
        for future in as_completed(futures):
            test_id = futures[future]
            completed += 1

            try:
                results = future.result()
                for result in results:
                    if result.passed:
                        passed += 1
                    elif result.error:
                        errors += 1
                        failures.append(result)
                    else:
                        failed += 1
                        failures.append(result)
            except Exception as e:
                errors += 4  # All 4 operations failed
                print(f"Test {test_id}: Exception - {e}")

            # Progress update
            if completed % 50 == 0 or completed == args.count:
                pct = completed * 100 // args.count
                print(f"Progress: {completed}/{args.count} cases ({pct}%) - "
                      f"Pass: {passed}, Fail: {failed}, Error: {errors}")

    print()
    print("=" * 60)
    print("FUZZ TEST RESULTS")
    print("=" * 60)
    print(f"Total comparisons: {total_tests}")
    print(f"Passed: {passed} ({passed * 100 / total_tests:.1f}%)")
    print(f"Failed: {failed} ({failed * 100 / total_tests:.1f}%)")
    print(f"Errors: {errors} ({errors * 100 / total_tests:.1f}%)")
    print()

    # Save failures
    if failures:
        print(f"Saving {len(failures)} failures to {output_dir}/")
        for result in failures:
            filepath = save_failure(result, output_dir)
            print(f"  - {filepath.name}")
        print()
        print("To reproduce a failure:")
        print(f"  ./compare.sh {output_dir}/<file>.json <operation> <fill_type>")
    else:
        print("No failures!")

    print()
    print(f"Seed for reproduction: --seed {seed}")

    # Exit with appropriate code
    if failed > 0 or errors > 0:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
