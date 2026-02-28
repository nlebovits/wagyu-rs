# Research: `correct_collinear_edges` - C++ Wagyu Analysis

Source file: `../wagyu/include/mapbox/geometry/wagyu/topology_correction.hpp`

---

## 1. What Is a Collinear Edge?

Two ring points `pt_a` and `pt_b` occupy the **same coordinate** (duplicate spatial position). A *collinear edge* exists when one ring's edge leaving `pt_a` overlays the other ring's edge arriving at `pt_b` (or vice versa). Concretely:

```cpp
// topology_correction.hpp lines 1028-1031
bool has_collinear_edge(point_ptr<T> pt_a, point_ptr<T> pt_b) {
    // pt_a and pt_b are at the same location
    return (*pt_a->next == *pt_b->prev || *pt_b->next == *pt_a->prev);
}
```

The two edges go in **opposite directions** and perfectly overlap — a degenerate "spike" shape. This can happen within a single ring (a self-touching spike) or between two different rings that share a boundary segment.

---

## 2. Data Structures

### `point` (from `point.hpp`, lines 32-55)

```cpp
struct point {
    ring_ptr<T> ring;   // which ring owns this point (nullable; null = deleted)
    T x, y;            // coordinate
    point_ptr<T> next; // next point in ring (circular linked list)
    point_ptr<T> prev; // previous point in ring (circular linked list)
};
```

Points are stored in a **circular doubly-linked list**. The `ring->points` pointer is an arbitrary entry point into the cycle. Traversal forward: `pt->next`. Traversal backward: `pt->prev`. "Deleted" points have `ring = nullptr`.

### `ring_manager::all_points` (from `ring.hpp`, lines 157-181)

```cpp
struct ring_manager {
    point_vector<T> all_points;  // flat Vec of raw point_ptrs
    std::deque<point<T>> points; // heap storage for points
    std::deque<ring<T>> rings;   // heap storage for rings
    ...
};
```

`all_points` is a `std::vector<point_ptr<T>>` — a flat list of raw pointers to *every* point ever created across all rings. Every call to `create_new_point` appends to `all_points`. This vector persists for the entire topology correction phase.

**Critically**: in `correct_topology` (line 1325), `all_points` is **sorted once** with `point_ptr_cmp` before any collinear/chained correction begins, and the same sorted order is reused by both `correct_collinear_edges` and `correct_chained_rings`.

### `point_ptr_cmp` (lines 148-160)

```cpp
struct point_ptr_cmp {
    bool operator()(point_ptr<T> op1, point_ptr<T> op2) {
        if (op1->y != op2->y) return (op1->y > op2->y);       // larger Y first
        if (op1->x != op2->x) return (op1->x < op2->x);       // smaller X first
        // tiebreaker: deeper ring (more nested) first
        return ring_depth(op1->ring) > ring_depth(op2->ring);
    }
};
```

This sorts all points so that spatially co-located points appear **adjacent** in the sorted vector. The ring-depth tiebreaker ensures a deterministic order for points at the exact same (x, y).

**Rust note**: The Rust port's `compare_points` (topology_correction.rs line 518) implements the y-desc / x-asc ordering but **does not include the ring-depth tiebreaker**. This is acceptable for grouping (it doesn't affect which points land in the same group), but may affect which specific pair is processed first.

### `collinear_path<T>` (lines 831-840)

```cpp
struct collinear_path {
    point_ptr<T> start_1; // first collinear point on path A (ring-forward direction)
    point_ptr<T> end_1;   // last collinear point on path A
    point_ptr<T> start_2; // first collinear point on path B (ring-forward direction)
    point_ptr<T> end_2;   // last collinear point on path B
};
```

The two paths are on **opposite winding directions** at the same physical location: `start_1 == end_2` position and `start_2 == end_1` position. "Forward" always means `->next`.

### `collinear_result<T>` (lines 843-846)

```cpp
struct collinear_result {
    point_ptr<T> pt1; // nullptr means ring was deleted entirely
    point_ptr<T> pt2; // nullptr means single-ring result (no split)
};
```

---

## 3. Function Call Graph

```
correct_topology (line 1321)
└── correct_collinear_edges (line 1229)          [MISSING from Rust port]
    └── correct_collinear_repeats (line 1204)
        └── process_collinear_edges (line 1177)
            ├── remove_duplicate_points (line 1091)   [first: strips exact dupes]
            ├── has_collinear_edge (line 1028)         [detects collinear edge]
            ├── correct_self_intersection (line 200)   [same-ring, no collinear]
            ├── process_collinear_edges_same_ring (line 1034)
            │   ├── find_start_and_end_of_collinear_edges (line 936)
            │   └── fix_collinear_path (line 849)
            └── process_collinear_edges_different_rings (line 1064)
                ├── find_start_and_end_of_collinear_edges (line 936)
                └── fix_collinear_path (line 849)
```

---

## 4. Algorithm: `correct_collinear_edges` (lines 1229-1259)

```cpp
void correct_collinear_edges(ring_manager<T>& manager) {
    if (manager.all_points.size() < 2) return;

    std::size_t count = 0;
    auto prev_itr = manager.all_points.begin();
    auto itr = std::next(prev_itr);

    while (itr != manager.all_points.end()) {
        if (*(*prev_itr) == *(*(itr))) {  // same (x, y)?
            ++count;
            ++prev_itr; ++itr;
            if (itr != manager.all_points.end()) continue;
            else ++prev_itr;              // flush at end of vector
        } else {
            ++prev_itr; ++itr;
        }
        if (count == 0) continue;
        auto first = prev_itr;
        std::advance(first, -(static_cast<int>(count) + 1));
        correct_collinear_repeats(manager, first, prev_itr);
        count = 0;
    }
}
```

**Step-by-step**:

1. Walk the sorted `all_points` with two adjacent iterators.
2. When consecutive points have equal (x, y), accumulate a run length in `count`.
3. When the run ends (different coord, or end of vector), compute `[first, prev_itr)` — a slice of all points at that coordinate.
4. Call `correct_collinear_repeats` on that slice.
5. Reset `count = 0` and continue.

The "flush at end of vector" case (`++prev_itr` when `itr == end`) shifts `prev_itr` past the last element so the `first` calculation remains correct.

**Key invariant**: `all_points` is pre-sorted by `point_ptr_cmp`, so all points at the same coordinate are adjacent. The algorithm groups them and hands each group to `correct_collinear_repeats`.

---

## 5. Algorithm: `correct_collinear_repeats` (lines 1204-1226)

```cpp
void correct_collinear_repeats(ring_manager<T>& manager,
                               point_vector_itr<T> const& begin,
                               point_vector_itr<T> const& end) {
    for (auto itr1 = begin; itr1 != end; ++itr1) {
        if ((*itr1)->ring == nullptr) continue;         // skip deleted

        for (auto itr2 = begin; itr2 != end;) {        // NOTE: itr2 starts at begin, not next(itr1)
            if ((*itr1)->ring == nullptr) break;        // itr1 may have been deleted

            if ((*itr2)->ring == nullptr || *itr2 == *itr1) {
                ++itr2;
                continue;
            }
            if (process_collinear_edges(*itr1, *itr2, manager)) {
                itr2 = begin;                           // RESTART itr2 from begin
            } else {
                ++itr2;
            }
        }
    }
}
```

**Key observations**:

1. The **outer loop** (`itr1`) walks forward through the group.
2. The **inner loop** (`itr2`) starts at `begin` (not `next(itr1)`) — so it examines all pairs including `itr2 == itr1`. The guard `*itr2 == *itr1` (pointer equality, not value equality) skips self-pairs.
3. When `process_collinear_edges` returns `true`, `itr2` is **reset to `begin`** — the whole group is re-scanned from the start because the ring topology changed and previously skipped points (e.g., those with `ring == nullptr`) might now have active rings, or the collinear structure has changed.
4. Deletions set `ring = nullptr`, which the skipping guards handle.

**Rust translation note**: The restart-to-begin behavior is critical for correctness. A simple nested `for` loop won't work — this must use explicit index/iterator manipulation that supports reset.

---

## 6. Algorithm: `process_collinear_edges` (lines 1177-1201)

This is the dispatch function. It handles three cases in order:

```cpp
bool process_collinear_edges(point_ptr<T> pt_a, point_ptr<T> pt_b, ring_manager<T>& manager) {
    // 1. Either point deleted → nothing to do
    if (!pt_a->ring || !pt_b->ring) return false;

    // 2. Strip adjacent duplicate points from both points first
    if (remove_duplicate_points(pt_a, pt_b, manager)) return true;

    // 3. Check for actual collinear edge
    if (!has_collinear_edge(pt_a, pt_b)) {
        // Same-ring duplicates that aren't collinear → self-intersection
        if (pt_a->ring == pt_b->ring) {
            correct_self_intersection(pt_a, pt_b, manager);
            return true;
        }
        return false;  // different rings, no collinear edge → not our problem
    }

    // 4. Dispatch based on whether they share a ring
    if (pt_a->ring == pt_b->ring) {
        process_collinear_edges_same_ring(pt_a, pt_b, manager);
    } else {
        process_collinear_edges_different_rings(pt_a, pt_b, manager);
    }
    return true;
}
```

**Returns `true`** whenever topology was modified (ring deleted, spike removed, or rings merged/split). The caller uses this to restart its iteration.

---

## 7. Algorithm: `remove_duplicate_points` (lines 1091-1174)

Handles the case where `pt_a` and `pt_b` are at the same coordinate but there is no collinear *edge* (or before checking for one, clean up adjacent duplicate chains).

Cases handled:
- If same ring and `pt_a->next == pt_b`: remove `pt_b` from the linked list, update `ring->points` if needed.
- If same ring and `pt_b->next == pt_a`: remove `pt_b` similarly.
- Otherwise: walk `pt_a->next` and `pt_a->prev` stripping any chain of duplicate-coordinate points adjacent to `pt_a`. Then do the same for `pt_b`.
- If after stripping, the ring collapses to a single point (`pt_a->next == pt_a`), remove the ring entirely.

The function sets `ring = nullptr` on removed points. It returns `true` if it did any work.

---

## 8. Algorithm: `find_start_and_end_of_collinear_edges` (lines 936-1025)

This function extends the collinear path outward from the two seed points `pt_a` and `pt_b`.

**Conceptual model**: At the shared coordinate, two ring paths diverge. They may share not just one point but a whole run of identical coordinates (a "collinear stretch"). This function finds the full extent of that stretch in both directions.

### Phase 1 — Search backward on A, forward on B:

```
Start: back = pt_a, forward = pt_b
Loop:
  - Extend back backward along A (while back->prev has same coords)
  - Extend forward forward along B (while forward->next has same coords)
  - Step: back = back->prev, forward = forward->next
  - Stop when *back != *forward (diverged) or guards trigger
Result: start_a = back->next, end_b = forward->prev
```

The `!same_ring` guards after each phase advance through leading repeated points at the diverge point to find the "first non-repeat" start/end.

### Phase 2 — Search backward on B, forward on A:

```
Start: back = pt_b, forward = pt_a
Same loop structure, but stops earlier if it would overlap with Phase 1 results
Result: start_b = back->next, end_a = forward->prev
```

### Returns:

```cpp
return { start_a, end_a, start_b, end_b };
// path.start_1 == start_a: first point of A in forward (->next) direction
// path.end_1   == end_a:   last  point of A in forward direction
// path.start_2 == start_b: first point of B in forward direction
// path.end_2   == end_b:   last  point of B in forward direction
```

**The invariant**: `start_a` and `end_b` are at the same position (one end of the collinear stretch). `start_b` and `end_a` are at the same position (the other end).

**Rust translation note**: The linked-list pointer comparisons (`back == forward` comparing *pointers*, not values) are identity checks that don't have a direct Vec-index equivalent. In Rust with index-based storage, these checks should compare ring-index + point-index pairs (structural identity), not just coordinate values.

---

## 9. Algorithm: `fix_collinear_path` (lines 849-933)

Takes the `collinear_path` struct and performs the actual pointer surgery.

### Spike detection:

```cpp
bool spike_left  = (path.start_1 == path.end_2); // pointer identity
bool spike_right = (path.start_2 == path.end_1); // pointer identity
```

"Spike left" means the path A enters and immediately exits from the same point (zero-length path A). "Spike right" similarly.

### Case 1: Both spikes (lines 863-873)

The entire stretch collapses to a single point. Walk from `start_1` and null out all points. Return `{nullptr, nullptr}`.

### Case 2: Spike left only (lines 874-885)

Path A is a single point, path B has extent. Remove path B's internal points. Reconnect: `prev(start_2)->next = end_1`. Return `{end_1, nullptr}`.

### Case 3: Spike right only (lines 886-897)

Mirror of case 2 for path A. Remove path A's internal points. Reconnect: `prev(start_1)->next = end_2`. Return `{end_2, nullptr}`.

### Case 4: General case (lines 899-931)

Both paths have extent. Remove all collinear points from both paths:

```
prev_1 = start_1->prev
prev_2 = start_2->prev
Null out: start_1 ... (up to but not including end_1)
Null out: start_2 ... (up to but not including end_2)
```

Then reconnect based on degenerate sub-cases:
- If `start_1 == end_1` AND `start_2 == end_2`: `{nullptr, nullptr}` (both degenerate)
- If `start_1 == end_1` only: `prev_2->next = end_2`, `end_2->prev = prev_2` → `{end_2, nullptr}`
- If `start_2 == end_2` only: `prev_1->next = end_1`, `end_1->prev = prev_1` → `{end_1, nullptr}`
- Normal case: cross-stitch:
  ```
  prev_1->next = end_2;  end_2->prev = prev_1;
  prev_2->next = end_1;  end_1->prev = prev_2;
  return {end_1, end_2};
  ```

**The cross-stitch reconnection** is what merges/splits rings: the two paths are swapped out, and the remaining tails are reconnected to the opposite prevs.

---

## 10. Algorithm: `process_collinear_edges_same_ring` (lines 1034-1061)

```cpp
void process_collinear_edges_same_ring(point_ptr<T> pt_a, point_ptr<T> pt_b, ring_manager<T>& manager) {
    ring_ptr<T> original_ring = pt_a->ring;
    auto path = find_start_and_end_of_collinear_edges(pt_a, pt_b);
    auto results = fix_collinear_path(path);

    if (results.pt1 == nullptr) {
        // Ring was completely removed
        remove_ring(original_ring, manager, false);
    } else if (results.pt2 == nullptr) {
        // Spike removed; ring survives as single piece
        original_ring->points = results.pt1;
        original_ring->recalculate_stats();
    } else {
        // Ring split into two rings
        ring_ptr<T> ring_new = create_new_ring(manager);
        ring_new->points = results.pt2;
        ring_new->recalculate_stats();
        update_points_ring(ring_new);   // set ->ring pointer on each new point
        original_ring->points = results.pt1;
        original_ring->recalculate_stats();
        // Note: parent/child NOT re-assigned here — done later by correct_tree
    }
}
```

When a single ring has a collinear edge back on itself, removing the overlapping segment either:
- Destroys the ring entirely (degenerate), or
- Removes a spike (ring continues as one piece), or
- Splits it into two separate rings.

**No parent/child fixup is done here.** The comment in `correct_topology` says "We should only have to fix collinear edges once" and `correct_tree` runs after to re-establish the hierarchy.

---

## 11. Algorithm: `process_collinear_edges_different_rings` (lines 1064-1088)

```cpp
void process_collinear_edges_different_rings(point_ptr<T> pt_a, point_ptr<T> pt_b, ring_manager<T>& manager) {
    ring_ptr<T> ring_a = pt_a->ring;
    ring_ptr<T> ring_b = pt_b->ring;
    bool ring_a_larger = std::fabs(ring_a->area()) > std::fabs(ring_b->area());
    auto path = find_start_and_end_of_collinear_edges(pt_a, pt_b);
    auto results = fix_collinear_path(path);

    if (results.pt1 == nullptr) {
        // Both rings completely removed
        remove_ring(ring_a, manager, false);
        remove_ring(ring_b, manager, false);
        return;
    }
    // Two rings sharing an edge merge into one ring
    ring_ptr<T> merged_ring  = ring_a_larger ? ring_a : ring_b;
    ring_ptr<T> deleted_ring = ring_a_larger ? ring_b : ring_a;

    merged_ring->points = results.pt1;
    update_points_ring(merged_ring);
    merged_ring->recalculate_stats();
    if (merged_ring->size() < 3) {
        remove_ring_and_points(merged_ring, manager, false);
    }
    remove_ring(deleted_ring, manager, false);
    // Note: results.pt2 is not used — only one merged ring survives
}
```

When two different rings share a collinear edge, removing it **merges** them. The larger ring absorbs the smaller one. The merged ring takes the concatenated boundary minus the shared edge. No parent/child fixup here either.

---

## 12. Position of `correct_collinear_edges` in the Pipeline

The C++ `correct_topology` (line 1321) calls these in order:

```cpp
void correct_topology(ring_manager<T>& manager) {
    // Pre-sort all_points ONCE — used by both collinear and chained corrections
    std::stable_sort(manager.all_points.begin(), manager.all_points.end(), point_ptr_cmp<T>());

    correct_orientations(manager);          // 1. Fix winding directions
    correct_collinear_edges(manager);       // 2. Remove collinear (overlapping) edges  <-- MISSING IN RUST
    correct_self_intersections(manager, false);  // 3. Split self-intersecting rings
    correct_tree(manager);                  // 4. Rebuild parent/child hierarchy
    bool fixed = true;
    while (fixed) {
        correct_chained_rings(manager);          // 5. Fix rings that share boundary points
        fixed = correct_self_intersections(manager, true);  // 6. Repeat until stable
    }
}
```

**The current Rust `correct_topology` (line 1366) is missing step 2** — `correct_collinear_edges` is not called. The function needs to be implemented and inserted between `correct_orientations` and `correct_self_intersections`.

---

## 13. Rust Translation Notes

### 13.1 Pointer Identity vs. Value Equality

In C++, `spike_left = (path.start_1 == path.end_2)` compares **raw pointers** (identity: "same memory address"). In the Rust Vec-based model, the equivalent is comparing **(ring_idx, point_idx) pairs** — two points are "the same" if and only if they have the same ring index and the same index within that ring's points Vec.

Never use coordinate equality (`coord == coord`) as a substitute for pointer identity in the spike checks.

### 13.2 The Circular Linked List → Vec

C++ operations like `pt->prev`, `pt->next`, `pt->ring` map to:

| C++ | Rust equivalent |
|-----|-----------------|
| `pt->next` | `ring.points[(idx + 1) % ring.points.len()]` |
| `pt->prev` | `ring.points[(idx + ring.points.len() - 1) % ring.points.len()]` |
| `pt->ring == nullptr` | point has been logically deleted (ring removed or points cleared) |
| `pt->ring` | `manager.get(ring_idx)` |

### 13.3 Deletion Marking

C++ marks deleted points with `ring = nullptr`. In Rust with Vec storage, there's no individual point deletion — instead, entire rings are cleared. After `remove_ring`, the ring's `points` Vec is empty. The equivalent of the `ring == nullptr` check is:

```rust
manager.get(ring_idx).map_or(true, |r| r.points().is_empty())
```

However the `all_points` snapshot was taken before removal. The Rust port needs to verify that a ring still exists and has points before processing its entry in the snapshot.

### 13.4 `find_start_and_end_of_collinear_edges` Traversal

This function traverses the circular linked list backwards (`->prev`) and forwards (`->next`) from seed points. In the Rust Vec model this becomes:

```
back_idx = (seed_a_idx + N - 1) % N  // prev
forward_idx = (seed_b_idx + 1) % N    // next
```

The termination condition `back == forward` (pointer identity) becomes `back_idx == forward_idx && back_ring_idx == forward_ring_idx`.

The "same_ring" case (`pt_a->ring == pt_b->ring`) changes the duplicate-stripping behavior at diverge points: when both points are on the same ring, we don't try to skip leading repeats.

### 13.5 `fix_collinear_path` Pointer Surgery

The cross-stitch reconnection changes the `next`/`prev` links across two paths. In the Rust Vec model this would literally reorder elements within the Vec(s) or swap segment slices, depending on the representation chosen.

One practical approach:
1. Represent the collinear path as `(ring_idx, start_idx, end_idx)` on each side.
2. Extract both path segments as `Vec<Coord>`.
3. Null them out from their source rings.
4. Reconnect: insert the remaining tails into each other's rings at the splice points.

This is complex but mechanical. The key is that after `fix_collinear_path`, the correct ring topology (which points belong to which ring, in what order) is what matters, not the specific linked-list structure.

### 13.6 `remove_duplicate_points` Translation

The adjacent-duplicate-stripping loops:

```cpp
while (*pt_a->next == *pt_a && pt_a->next != pt_a) { ... }
while (*pt_a->prev == *pt_a && pt_a->prev != pt_a) { ... }
```

translate to: walk forward/backward from the index, removing consecutive points with the same coordinate. Stop if you wrap all the way around (i.e., `next_idx == seed_idx`). Remove points by splicing them out of the Vec.

### 13.7 The `correct_collinear_repeats` Restart Logic

The inner loop restarts to `begin` after any successful `process_collinear_edges`. In Rust:

```rust
'outer: for i in 0..group.len() {
    if ring_is_deleted(group[i]) { continue; }
    let mut j = 0;
    while j < group.len() {
        if ring_is_deleted(group[i]) { continue 'outer; }
        if ring_is_deleted(group[j]) || j == i {
            j += 1;
            continue;
        }
        if process_collinear_edges(manager, group[i], group[j]) {
            j = 0;  // restart inner loop
        } else {
            j += 1;
        }
    }
}
```

### 13.8 The `all_points` Snapshot Problem

The C++ uses live pointers: when a point's ring is set to `nullptr`, iterating `all_points` sees that change immediately. In Rust, `all_points` is a snapshot of `(ring_idx, point_idx)` pairs taken at the start of the function. After ring mutation (removal, merge, split), the snapshot may contain stale entries.

**Solution**: Before processing any `(ring_idx, point_idx)` from the snapshot, verify the ring still exists and has points. Also verify the `point_idx` is still in bounds. The `is_deleted` check must go through the live `manager.get(ring_idx)`.

### 13.9 `update_points_ring` Equivalent

```cpp
void update_points_ring(ring_ptr<T> ring) {
    point_ptr<T> op = ring->points;
    do { op->ring = ring; op = op->prev; } while (op != ring->points);
}
```

In Rust, this is a no-op for Vec-based storage because each `Ring` owns its points directly — no `point->ring` back-pointer exists. However, when rings are merged/split and points move from one ring's Vec to another, we do need to ensure those points now conceptually belong to the new ring. This is automatic since the Vec is owned by the ring.

---

## 14. Edge Cases

### 14.1 Empty / Tiny Groups

- `correct_collinear_edges` early-exits if `all_points.size() < 2`.
- `correct_collinear_repeats` is only called with groups of 2 or more same-coordinate points.
- Groups of exactly 1 are impossible given the grouping logic.

### 14.2 Ring Degeneracy After Repair

After `fix_collinear_path` returns `{pt1, pt2}`, `process_collinear_edges_different_rings` checks `merged_ring->size() < 3` and removes it if degenerate. This prevents zero-area rings leaking into subsequent steps.

### 14.3 Both Spikes: Complete Deletion

If both `spike_left` and `spike_right` are true, the entire collinear stretch forms a closed loop that should be entirely deleted. All points in the loop have their `ring` set to `nullptr` and their links severed.

### 14.4 Same-Ring Collinear Edges

When both points are on the same ring, the fix creates up to two rings from one:
- Zero output rings: ring was degenerate, delete it.
- One output ring: spike removed, same ring persists.
- Two output rings: ring splits at the collinear stretch.

The **second ring** from the split is created with `create_new_ring` but its parent/child is not set — `correct_tree` will assign it later.

### 14.5 Pointer Guard in `find_start_and_end_of_collinear_edges`

The loop uses `back == pt_a` and `forward == pt_b` as sentinel stops to prevent infinite traversal if a ring is all-duplicate-coordinates. These pointer-identity sentinels prevent going around the circle more than once.

### 14.6 Interaction with `all_points` After Mutations

`correct_collinear_edges` walks the already-sorted `all_points` vector and calls `correct_collinear_repeats` for each group. Within that call, ring topology changes. The outer walk then continues to the next group. This means later groups may encounter points from rings that were already modified or deleted in earlier groups — the `ring == nullptr` guard handles this.

---

## 15. Summary Table

| Function | Lines | Purpose |
|---|---|---|
| `correct_collinear_edges` | 1229-1259 | Entry: groups all_points by coord, dispatches groups |
| `correct_collinear_repeats` | 1204-1226 | Processes each group: pair-wise with restart-on-change |
| `process_collinear_edges` | 1177-1201 | Dispatch: duplicate-strip → collinear-check → same/diff ring |
| `remove_duplicate_points` | 1091-1174 | Strip adjacent exact duplicates from both points' neighborhoods |
| `has_collinear_edge` | 1028-1031 | Detect overlapping opposite-direction edges at same coord |
| `process_collinear_edges_same_ring` | 1034-1061 | Fix same-ring collinear: delete, spike-remove, or split |
| `process_collinear_edges_different_rings` | 1064-1088 | Fix cross-ring collinear: delete both or merge into one |
| `find_start_and_end_of_collinear_edges` | 936-1025 | Extend seed points to full collinear stretch extents |
| `fix_collinear_path` | 849-933 | Perform pointer surgery: spike/cross-stitch reconnection |
| `point_ptr_cmp` | 148-160 | Comparator: y-desc, x-asc, depth-desc for sorting all_points |
