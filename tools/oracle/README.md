# C++ Oracle Testing Tools

This directory contains tools for comparing wagyu-rs output against the original C++ wagyu implementation.

## Prerequisites

You must have a local clone of the C++ wagyu repository at `../wagyu` relative to this repo root:

```bash
cd /path/to/wagyu-rs/..
git clone https://github.com/mapbox/wagyu.git
```

## Building the C++ Oracle

The oracle binary needs to be built from the C++ wagyu source:

```bash
./tools/oracle/build_oracle.sh
```

This creates `../wagyu/build/wagyu-oracle` - a CLI tool that:
- Takes a JSON file with subject/clip polygons
- Runs the operation through C++ wagyu
- Outputs the result as JSON

## Usage

### Single Comparison

```bash
# Compare Rust vs C++ for a specific input
./tools/oracle/compare.sh tests/fixtures/simple_union.json union evenodd
```

### Batch Comparison

```bash
# Compare all golden test inputs
./tools/oracle/compare_all.sh
```

### Generate Expected Output

```bash
# Generate C++ expected output for a test case
./tools/oracle/run_cpp.sh input.json union evenodd > expected.json
```

## Input Format

The oracle accepts JSON files in this format:

```json
{
  "subject": [
    [[[x, y], [x, y], ...]]
  ],
  "clip": [
    [[[x, y], [x, y], ...]]
  ]
}
```

Where:
- `subject`: Array of polygons (each polygon is array of rings, each ring is array of [x,y] points)
- `clip`: Optional array of clip polygons (same structure)

## Operations

- `union` - Combine polygons
- `intersection` - Common area
- `difference` - Subject minus clip
- `xor` - Symmetric difference

## Fill Types

- `evenodd` - Even-odd fill rule
- `nonzero` - Non-zero winding fill rule
- `positive` - Positive winding only
- `negative` - Negative winding only

## Debug Logging

Both implementations support structured debug logging to help identify divergence points.

### Enabling Debug Mode

```bash
# With compare.sh
./tools/oracle/compare.sh input.json xor evenodd --debug

# Directly with Rust oracle
WAGYU_DEBUG=1 ./target/release/wagyu-oracle input.json xor evenodd

# Directly with C++ oracle
../wagyu/build/wagyu-oracle input.json xor evenodd --debug
```

### Debug Log Format

Both implementations output structured logs to stderr:

```
[VATTI_START] minima=2 scanbeam=1
[SCANBEAM] y=10
[LOCAL_MIN] y=10 left=0 right=1 type=Subject
[WINDING] idx=0 wc=1 wc2=0 delta=0
[RING_NEW] id=0 pt=(5,10)
[INTERSECT] b1=1 b2=2 pt=(7,7)
[RING_POINT] id=0 pt=(0,0) front=true
[RING_MERGE] from=2 to=1
[RING_CLOSE] id=0 points=5
[VATTI_END] rings=2
```

### Comparing Debug Logs

When `--debug` is passed to compare.sh, both logs are captured and diffed to help identify where the algorithms diverge.

**Note:** The C++ oracle can only log input/output (not internal algorithm state) because the wagyu library is header-only and would require source modification for deeper instrumentation. The Rust implementation has full internal logging.

## Fuzz Testing

The `fuzz.py` script generates random polygon pairs and validates Rust output matches C++ oracle output.

### Usage

```bash
# Run 500 test cases (default)
./tools/oracle/fuzz.py

# Run custom number of cases with reproducible seed
./tools/oracle/fuzz.py --count 1000 --seed 12345

# Control parallelism
./tools/oracle/fuzz.py --parallel 4
```

### What It Generates

The fuzzer generates four types of polygon pairs:
- **Convex polygons**: 3-20 vertices placed on a circle
- **Concave polygons**: 5-50 vertices with varying radii (star-like)
- **Polygons with holes**: Convex exterior with convex hole
- **Multi-polygons**: 2-4 separate simple polygons

Coordinate ranges vary: small (-10 to 10), medium (-100 to 100), and large (-1000 to 1000) integers.

### Output

- Progress updates during execution
- Summary report with pass/fail/error counts
- Failure files saved to `tools/oracle/fuzz_failures/` for reproduction

### Reproducing Failures

```bash
# Each failure is saved with operation and fill type in filename
./tools/oracle/compare.sh tools/oracle/fuzz_failures/failure_42_union_evenodd.json union evenodd

# With debug output
./tools/oracle/compare.sh tools/oracle/fuzz_failures/failure_42_union_evenodd.json union evenodd --debug
```

### Current Status

As of the initial fuzz run (seed 12345):
- **500 test cases × 4 operations = 2000 comparisons**
- **114 passed (5.7%)**
- **1886 failed (94.3%)**

The golden test suite (148 tests) represents the subset where both implementations agree. Random inputs expose edge cases in topology correction that are known gaps in the Rust port.
