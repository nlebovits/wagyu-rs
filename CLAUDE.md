# wagyu-rs

<!-- freshness: 2026-03-09 -->

Rust port of [Mapbox wagyu](https://github.com/mapbox/wagyu) C++ polygon clipping library.

## Critical Constraints

### TDD is Mandatory

Write tests BEFORE implementation. No exceptions.

```
RED    → Write failing test first
GREEN  → Write minimum code to pass
REFACTOR → Clean up while tests stay green
```

Run pre-commit hooks: `pre-commit install`

### Reference Implementation

C++ source at `../wagyu` (local clone). All algorithms must match C++ behavior.

When porting, add header comments:
```rust
// PORT FROM: wagyu/include/mapbox/geometry/wagyu/local_minimum.hpp
```

When deviating from C++:
```rust
// DIVERGENCE FROM WAGYU: [reason]
// C++ does X (see file.hpp:L45), Rust does Y because [ownership/etc.]
```

Document divergences in `context/ARCHITECTURE.md`.

### Ownership Strategy

Use `Vec` + `usize` indices for graph structures. No `Rc<RefCell<>>`, no arena allocators, no unsafe.

### OGC Validity

Output must be valid per OGC: no self-intersections, correct ring orientations, proper hole containment.

## Tools

### Oracle Harness (Working Setup)

The oracle harness compares Rust output against the C++ reference implementation.

**Prerequisites:**
- C++ wagyu cloned at `../wagyu` (sibling directory)
- CMake and C++ compiler installed

**Build:**
```bash
cd ../wagyu && mkdir -p build && cd build && cmake .. && make
cd tools/oracle && ./build_oracle.sh
```

**Usage:**
```bash
# Compare single test case
./tools/oracle/compare.sh tools/oracle/test_inputs/minimal/test.json union evenodd

# With debug output (shows ring operations)
./tools/oracle/compare.sh test.json union evenodd --debug

# Compare against golden test fixtures
./tools/oracle/compare.sh crates/core/tests/fixtures/polygon.json xor even_odd
```

**Minimal test cases:** `tools/oracle/test_inputs/minimal/` contains small reproducible cases for debugging specific issues.

### Debug Logging

<!-- BEGIN AUTO-GENERATED: debug-flags -->
Enable with `WAGYU_DEBUG=1`:

- `[AEL_ADD]` - / Log an active edge list addition.
- `[AEL_REMOVE]` - / Log an active edge list removal.
- `[APPEND_RING_END]` - In Rust, multiple bounds can share the same ring index.
- `[APPEND_RING_START]` - DEBUG: Log all bounds' ring assignments before merge
- `[BOUND_UPDATE]`
- `[CONTRIBUTING]` - / Log contributing edge decision.
- `[HORIZONTAL]` - / Log horizontal edge processing.
- `[HORIZ_INTERSECT]` - BUGFIX: Capture and handle IntersectResult to update other bounds
- `[HORIZ_MERGE_SEARCH]` - Update other active bounds that reference the removed ring
- `[HORIZ_MERGE_SEARCH_L]` - Update other active bounds that reference the removed ring
- `[INTERSECT]` - / Log an intersection detection.
- `[INTERSECT_RESULT]` - / Log intersection handling result.
- `[LOCAL_MIN]` - / Log local minimum insertion.
- `[MERGE_RINGS]` - DEBUG: Log which bounds/rings are being merged
- `[MERGE_SEARCH]` - PORT FROM: wagyu/include/mapbox/geometry/wagyu/ring_util.hpp - append_ring (lines 597-606)
- `[RING_CLOSE]` - / Log a ring close operation.
- `[RING_MERGE]` - / Log a ring merge operation.
- `[RING_NEW]` - / Log a new ring creation.
- `[RING_POINT]` - / Log a point added to a ring.
- `[SCANBEAM]` - / Log a scanbeam event.
- `[SET_PARENT]`
- `[TOPOLOGY]` - reverse the ring to fix the orientation
- `[TOPOLOGY_CHAIN]`
- `[TOPOLOGY_COLLINEAR]` - The C++ recalculates stats and updates point ownership here;
- `[TOPOLOGY_COLLINEAR_MERGE]`
- `[TOPOLOGY_COLLINEAR_SPLIT]`
- `[TOPOLOGY_RINGS]`
- `[TOPOLOGY_TREE]`
- `[TOPOLOGY_VTXINS]`
- `[VATTI_END]` - / Log the end of vatti algorithm.
- `[VATTI_START]` - / Log the start of vatti algorithm.
- `[WARNING]` - DEBUG: Check if ring actually exists
- `[WINDING]` - / Log winding count calculation.
<!-- END AUTO-GENERATED: debug-flags -->

## Debugging Patterns

### Infinite Loop Bugs

Topology correction convergence loops are prone to infinite loops. Pattern:

1. **Spawn parallel agents**: reproducer (confirm + gather debug output) + comparator (C++ vs Rust line-by-line)
2. **Check return value semantics**: C++ may return "was visited" while Rust returns "did something"
3. **Check data structure operations**: C++ linked-list pointer swaps → Rust Vec splits (not concatenations)

Loop guards exist in `crates/core/src/vatti.rs` and `crates/core/src/intersect_util.rs` (panic after 100k iterations).

### C++ Linked-List → Rust Vec Translation

| C++ Pattern | Rust Pattern |
|-------------|--------------|
| `ptr->next = other->next` (next-swap) | Split into two fragments, create new ring |
| `ptr->prev`, `ptr->next` traversal | Index arithmetic with modulo wrapping |
| Pointer comparison | Index comparison |

### Known Bug Areas

- `merge_rings_at_intersection` (Vatti): Merged rings tracked via `RingManager.mark_as_merged()`. NOTE: Junction point deduplication was attempted but reverted - duplicate points are needed for `correct_self_intersections` to properly split corner-touching polygons.
- `correct_collinear_edges` (Topology): Corrupts rings when merged rings have stale points - fix blocked on `clear_merged_rings()` bug
- `correct_ring_self_intersections`: Return `true` for visited rings, not just when splits occur
- Parent/child ring assignment after topology operations

### Merged Ring Tracking

Infrastructure exists in `RingManager` to track and clear merged rings:
- `mark_as_merged(ring_idx)` - Called during Vatti merge
- `clear_merged_rings()` - Should be called at topology correction start (currently disabled, see TODO in `correct_topology`)

## Open TODOs

<!-- BEGIN AUTO-GENERATED: todos -->
**TODOs:**
- `crates/core/src/build_local_minima_list.rs:383` - (#94): Enable once process_horizontal_right_to_left is fixed
- `crates/core/src/build_local_minima_list.rs:388` - (#94): Enable once process_horizontal_right_to_left is fixed
- `crates/core/src/topology_correction.rs:5457` - This test is failing because merge_rings_at_intersection produces
<!-- END AUTO-GENERATED: todos -->

## Current Status

Main gaps: topology correction for complex hole arrangements. Run `cargo test --test golden` for current count.
