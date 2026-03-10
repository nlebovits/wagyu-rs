# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## v0.2.1 (2026-03-10)

### Fix

- **vatti**: handle horizontal edges in create_bound_towards_maximum

## v0.2.0 (2026-03-09)

### Feat

- **ci**: add comprehensive CI tooling and docs validation
- **oracle**: add fuzz testing with random polygon generation
- **local_min**: port move_horizontals_on_left_to_right from C++
- **topology**: add hot pixel insertion for edge-edge intersections
- **core**: add merged ring tracking infrastructure
- **topology**: implement chained merge with linked-list simulation (#37)
- **debug**: add debug logging parity between C++ and Rust
- **tools**: add C++ oracle testing infrastructure
- **topology**: port correct_collinear_edges main loop from C++ wagyu
- **core**: implement correct_chained_rings for topology correction
- **topology**: port correct_self_intersections from C++ wagyu
- **topology**: port correct_collinear_edges from C++ wagyu
- **process_horizontal**: add complete ring and intersection handling
- integrate ring building and winding count logic from parallel agents
- **sweep**: wire up ring operations in sweep algorithm
- **winding**: port set_winding_count and is_contributing from C++ wagyu
- **ring_util**: port ring creation functions from C++ wagyu
- integrate orchestration layer implementations from parallel agents
- complete algorithm port with orchestration layer
- add Phase 3 utility modules for Vatti algorithm
- add Phase 2 algorithm infrastructure

### Fix

- **ci**: remove deprecated cargo-deny keys (unlicensed, copyleft)
- **ci**: remove stale reference to deleted context/ARCHITECTURE.md
- **ci**: update cargo-deny config for current API
- **ci**: add 'abl' (Active Bound List) to codespell ignore list
- **oracle**: add geometric normalization for accurate comparison
- **topology**: implement ring-splitting for self-intersecting polygons
- **topology**: remove collinear points after ring merge (#80)
- **topology**: detect stale indices in correct_chained_rings (#72)
- **topology**: include junction points in collinear edge merge (#68)
- **merge**: deduplicate points at ring merge join to preserve hot pixels (#36)
- **topology**: handle wrap-around spikes in collinear edge correction
- **horizontal**: use round() instead of truncation for intersection coordinates
- **topology**: use depth-based hole detection for XOR orientation (#25)
- **topology**: filter degenerate rings in correct_tree (#64)
- **topology**: add debug logging for unplaceable rings + fix clear_parent (#59)
- **topology**: port missing C++ swap logic for hole-origin to exterior (#58)
- **ring**: make assign_as_child safe for re-assignment (#57)
- **topology**: implement ring1_replaces_ring2 and child-steal loop (#48)
- **intersect**: sort by winding_count2_sum for tie-breaking (#55)
- **intersect**: prevent spurious ring creation after merge (#54)
- **merge**: update ALL bounds after ring merge, not just first (#53)
- **intersect**: add missing is_horizontal check and debug logging (#51)
- **topology**: resolve infinite loop in correct_topology Step 5
- **core**: implement proper geometry comparison for golden tests
- **core**: implement horizontal local minima insertion for shared edge handling
- **topology**: prevent false positive collinear edge detection for OGC closing pairs
- **core**: add missing last_point updates and hole state tracking
- **core**: correct winding_delta, intersect_bounds, and boundary containment
- resolve clippy warnings
- **intersect_util**: add missing ring merge condition for subject/clip intersections
- **build_result**: correct hole winding direction in output
- **topology_correction**: calculate is_hole from area sign
- **core**: align coordinate system with C++ wagyu
- **intersect_util**: add missing C++ edge case in get_current_x
- **test**: correct do_maxima return value assertion
- **maxima**: check BOTH bounds before calling do_maxima
- resolve clippy warnings in golden tests
- resolve all clippy warnings
- address code review feedback from adversarial review
- remove unnecessary clone on Copy type
- sync .cz.toml version and improve pre-commit hook
- add contents:write permission for GitHub releases

### Refactor

- **topology**: introduce options structs for multi-argument functions (#86)
- **topology**: introduce options structs for multi-argument functions
- **topology**: replace complex tuple with IntersectionContext struct (#85)
- **topology**: replace complex tuple with IntersectionContext struct

## v0.1.1 (2026-02-26)

### Feat

- add core data structures (Point, Ring, Bound, Edge)
- **config**: add clip_type and fill_type enums (TDD green)

### Fix

- update release workflow with new crate names
- bundle golden test fixtures in repository
- remove duplicate FillType enum from golden.rs
