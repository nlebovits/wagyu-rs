# wagyu-rs Architecture

Design decisions and divergences from wagyu C++.

## Design Principles

1. **1:1 Port**: Match wagyu C++ algorithms as closely as possible
2. **OGC Validity**: All output geometry must be valid per OGC standards
3. **Test Coverage**: Port all wagyu test cases to Rust
4. **Idiomatic Rust**: Use Rust idioms where they don't compromise correctness

## Reference Implementation

The original wagyu is a C++ header-only library:
- https://github.com/mapbox/wagyu
- Derived from Angus Johnson's Clipper library
- Uses Vatti's polygon clipping algorithm

### Key C++ Files to Port

```
wagyu/include/mapbox/geometry/wagyu/
├── wagyu.hpp              # Main entry point
├── vatti.hpp              # Vatti algorithm implementation
├── local_minimum.hpp      # Local minima handling
├── build_edges.hpp        # Edge construction
├── build_result.hpp       # Result polygon construction
├── process_horizontal.hpp # Horizontal edge processing
├── intersect.hpp          # Edge intersections
├── ring.hpp               # Ring data structures
├── bound.hpp              # Bound data structures
├── edge.hpp               # Edge data structures
├── point.hpp              # Point data structures
├── config.hpp             # Configuration types
└── almost_equal.hpp       # Floating point comparison
```

## Known Divergences from wagyu C++

| Area | Rust Approach | C++ Approach | Reason |
|------|---------------|--------------|--------|
| Memory | Owned structures | Raw pointers | Rust ownership |
| Templates | Generics | C++ templates | Language difference |
| - | - | - | (Add as porting progresses) |

## Module Structure (Planned)

```
crates/core/src/
├── lib.rs           # Public API
├── error.rs         # Error types
├── operation.rs     # Boolean operation types
├── vatti.rs         # Vatti algorithm (main clipper)
├── local_minimum.rs # Local minima detection
├── edge.rs          # Edge structures
├── bound.rs         # Bound structures
├── ring.rs          # Ring structures
├── intersect.rs     # Intersection handling
├── horizontal.rs    # Horizontal edge processing
└── almost_equal.rs  # Float comparison utilities
```

## Algorithm Overview

### Vatti Polygon Clipping

The core algorithm (Vatti 1992) works by:

1. **Build edges** from input polygons
2. **Find local minima** (lowest points of each polygon)
3. **Sweep from bottom to top**, processing:
   - Intersections between active edges
   - Horizontal edges
   - Local minima and maxima
4. **Build output rings** from processed edges

### Data Flow

```
Input Polygons
      │
      ▼
  Build Edges
      │
      ▼
Find Local Minima
      │
      ▼
  Vatti Sweep ─────┐
      │            │
      ▼            │
Process Intersections
      │            │
      ▼            │
Process Horizontals
      │            │
      ▼            │
      └────────────┘
      │
      ▼
  Build Result
      │
      ▼
Output Polygons (OGC Valid)
```

## Testing Strategy

1. **Port wagyu test cases** directly from C++ tests
2. **Golden tests** comparing output with wagyu C++
3. **Property-based tests** for validity invariants
4. **Fuzz testing** for robustness

## Performance Considerations

(To be documented as implementation progresses)
