# wagyu-rs

Rust port of [Mapbox wagyu](https://github.com/mapbox/wagyu) C++ polygon clipping library.

## Critical Constraints

### TDD is Mandatory

Write tests BEFORE implementation. No exceptions.

```
RED    → Write failing test first
GREEN  → Write minimum code to pass
REFACTOR → Clean up while tests stay green
```

Run pre-commit hooks: `git config core.hooksPath .githooks`

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

### Oracle Harness

Compare Rust output against C++ oracle (PR #41):

```bash
# Build C++ oracle
cd tools/oracle && ./build_oracle.sh

# Compare single test
./compare.sh tests/fixtures/polygon.json xor even_odd

# Compare all failing tests
./compare_all.sh
```

### Debug Logging

Enable with `WAGYU_DEBUG=1`:
```bash
WAGYU_DEBUG=1 cargo test test_name -- --nocapture
```

## Debugging Patterns

### Infinite Loop Bugs

Topology correction convergence loops are prone to infinite loops. Pattern:

1. **Spawn parallel agents**: reproducer (confirm + gather debug output) + comparator (C++ vs Rust line-by-line)
2. **Check return value semantics**: C++ may return "was visited" while Rust returns "did something"
3. **Check data structure operations**: C++ linked-list pointer swaps → Rust Vec splits (not concatenations)

Loop guards exist in `vatti.rs` and `intersect_util.rs` (panic after 100k iterations).

### C++ Linked-List → Rust Vec Translation

| C++ Pattern | Rust Pattern |
|-------------|--------------|
| `ptr->next = other->next` (next-swap) | Split into two fragments, create new ring |
| `ptr->prev`, `ptr->next` traversal | Index arithmetic with modulo wrapping |
| Pointer comparison | Index comparison |

### Known Bug Areas

- `merge_rings_at_intersection`: Must SPLIT rings, not concatenate (#37 tracks `i_list` chain handling)
- `correct_ring_self_intersections`: Return `true` for visited rings, not just when splits occur
- Parent/child ring assignment after topology operations

## Current Status

~39/148 golden tests passing. Main gaps: topology correction for complex hole arrangements.
