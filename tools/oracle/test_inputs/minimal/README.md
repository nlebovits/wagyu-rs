# Minimal Failing Test Cases

This directory contains minimal reproducing cases for each open wagyu-rs bug. Each case demonstrates a specific divergence between the Rust and C++ implementations.

## How to Use

Run any case with the oracle comparison tool:

```bash
# Run a specific test
./tools/oracle/compare.sh tools/oracle/test_inputs/minimal/issue_25_xor_winding.json xor evenodd

# Run all minimal tests
for f in tools/oracle/test_inputs/minimal/issue_*.json; do
  op=$(jq -r '._operation' "$f")
  fill=$(jq -r '._fill' "$f")
  echo "=== Testing $f ($op, $fill) ==="
  ./tools/oracle/compare.sh "$f" "$op" "$fill"
  echo
done
```

## Test Cases

### Issue #25: XOR Winding Count

**File:** `issue_25_xor_winding.json`
**Operation:** `xor`
**Description:** Two overlapping triangles

The `is_contributing` logic for XOR operations incorrectly determines which edges should contribute to the output. This is the root cause blocking several other fixes.

**Symptom:** XOR produces wrong polygon structures - rings are merged incorrectly or have wrong winding.

**Expected (C++):** Two separate triangular regions (butterfly/bowtie shape)
**Actual (Rust):** Incorrectly merged polygons with wrong vertex counts

---

### Issue #28: Complex Hole Topology

**File:** `issue_28_hole_topology.json`
**Operation:** `union`
**Description:** Square with two overlapping holes

Parent/child ring relationships are incorrectly assigned when holes intersect. The topology correction only handles basic two-ring merges, not multi-ring chains.

**Symptom:** Output contains many empty ring arrays `[]`, indicating broken ring assignments.

**Expected (C++):** Single polygon with properly merged hole boundary
**Actual (Rust):** Multiple empty rings and incorrect hole geometry

---

### Issue #29: XOR Nested Multi-Polygon

**File:** `issue_29_xor_nested.json`
**Operation:** `xor`
**Description:** Outer square with inner square (containment scenario)

Combination of XOR winding bug (#25) and nested ring handling issues. Blocked by #25.

**Symptom:** Minor ring rotation differences in output (may be acceptable after #25 is fixed).

**Expected (C++):** Donut polygon starting at `[20, 0]`
**Actual (Rust):** Same polygon but starting at `[20, 20]`

---

### Issue #30: No Interior Edge Case

**File:** `issue_30_no_interior.json`
**Operation:** `difference`
**Description:** Large subject completely covers smaller clip

Complete containment scenarios not handled correctly. The algorithm doesn't properly identify when one polygon entirely contains another.

**Symptom:** Similar ring rotation differences (linked to containment logic).

**Expected (C++):** Donut polygon (outer minus inner)
**Actual (Rust):** Same topology but different starting vertex

---

### Issue #36: Hot Pixel Insertion

**File:** `issue_36_hot_pixel.json`
**Operation:** `union`
**Description:** Two rectangles with shared horizontal edge

Hot pixels (snap-rounding intersection points) are not inserted during horizontal edge processing. This causes precision inconsistencies.

**Symptom:** Missing vertex `[10, 10]` at critical horizontal edge intersection.

**Expected (C++):** 9-vertex polygon with `[10, 10]` present
**Actual (Rust):** 8-vertex polygon missing the shared edge point

---

### Issue #37: Chained Merges

**File:** `issue_37_chained_merge.json`
**Operation:** `union`
**Description:** Three squares sharing boundary points in an L-shape

The `process_merge_i_list` function only handles basic two-ring merges. When 3+ rings share boundary points forming a chain, only the first pair is merged.

**Symptom:** Duplicate vertices `[10, 10]` appearing three times, missing `[0, 10]` vertex.

**Expected (C++):** Clean 9-vertex L-shaped polygon
**Actual (Rust):** 10-vertex polygon with duplicate points

---

## Test Results Summary

All 6 cases demonstrate divergences as of 2026-03-02:

| Issue | Status | Severity |
|-------|--------|----------|
| #25 XOR Winding | DIVERGENT | High (blocks others) |
| #28 Hole Topology | DIVERGENT | High (empty rings) |
| #29 XOR Nested | DIVERGENT | Medium (rotation only) |
| #30 No Interior | DIVERGENT | Medium (rotation only) |
| #36 Hot Pixel | DIVERGENT | Medium (missing vertex) |
| #37 Chained Merge | DIVERGENT | High (duplicate vertices) |

## Design Principles

These minimal cases follow the principles:

1. **Smallest possible input** - Each case uses simple geometric primitives
2. **Single bug focus** - Each case isolates one specific failure mode
3. **Easy visualization** - Coordinates are small integers for easy plotting
4. **Reproducible** - JSON format works with both C++ and Rust oracles
