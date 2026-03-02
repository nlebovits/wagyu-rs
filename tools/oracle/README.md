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
