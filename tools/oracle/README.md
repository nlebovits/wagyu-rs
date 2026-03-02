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
