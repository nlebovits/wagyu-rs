# wagyu-rs - Claude Code Instructions

## Project Overview

Rust port of [Mapbox wagyu](https://github.com/mapbox/wagyu), a C++ geometry boolean operations library.

**Goal:** 1:1 port of wagyu C++ algorithms to idiomatic Rust, maintaining correctness and OGC validity guarantees.

## Critical Constraints

### 1. Test-Driven Development (TDD) is MANDATORY

**YOU MUST WRITE TESTS BEFORE IMPLEMENTATION. NO EXCEPTIONS.**

This is not optional. This is not "when convenient." Every single piece of functionality follows:

```
RED    → Write a failing test first
GREEN  → Write minimum code to pass
REFACTOR → Clean up while tests stay green
```

#### TDD Workflow (REQUIRED)

```bash
# Step 1: Write test, verify it FAILS (RED)
cargo test --package wagyu-core <test_name> -- --nocapture
# Must see: "test <test_name> ... FAILED"

# Step 2: Implement ONLY enough to pass (GREEN)
cargo test --package wagyu-core <test_name> -- --nocapture
# Must see: "test <test_name> ... ok"

# Step 3: Refactor if needed, tests must stay green
cargo test --package wagyu-core --lib

# Step 4: Commit with TDD marker
git commit -m "feat: implement X (TDD green)"
```

#### WHY THIS MATTERS FOR PORTING

- wagyu C++ has **148 golden test fixtures** - use them!
- Tests prove your port matches C++ behavior
- Without tests, you're just guessing if the port is correct

#### RED FLAGS (DO NOT DO THESE)

- Writing implementation code before any test exists
- "I'll add tests later" - NO, add them NOW
- Porting a whole file then writing tests - port ONE function with tests first
- Skipping tests for "simple" code - simple code has bugs too

### 2. Reference Implementation: wagyu C++

**All algorithms MUST match the original wagyu behavior.**

#### Local Reference

A local clone exists at `../wagyu` (relative to this repo root). Use this for:
- Reading C++ source files directly
- Checking test cases
- Understanding algorithm implementations

```bash
# Example: read the main entry point
cat ../wagyu/include/mapbox/geometry/wagyu/wagyu.hpp
```

#### Remote Reference (if no local clone)

If `../wagyu` doesn't exist, use **gitingest + distill** to fetch from GitHub:

```bash
# Generate digest from GitHub
gitingest https://github.com/mapbox/wagyu

# Then compress with distill for token efficiency
mcp__distill__auto_optimize(content, hint="code")
```

**GitHub remote:** https://github.com/mapbox/wagyu

#### When Porting

```rust
// PORT FROM: wagyu/include/mapbox/geometry/wagyu/local_minimum.hpp
// Original C++ comment preserved here...
```

#### When Deviating

```rust
// DIVERGENCE FROM WAGYU: [reason]
// C++ does X (see local_minimum.hpp:L45)
// Rust does Y because [ownership / performance / etc.]
```

Document all divergences in `context/ARCHITECTURE.md`.

### 3. OGC Validity

Output geometry MUST be valid and simple per [OGC standards](http://postgis.net/docs/using_postgis_dbmanagement.html#OGC_Validity):
- No self-intersections
- Correct ring orientations
- Proper hole containment

## Architecture

```
crates/
├── core/     # ALL clipping logic lives here
└── cli/      # Thin CLI wrapper (placeholder for now)
```

**Library-first:** CLI is a thin consumer. Never put logic in CLI that belongs in core.

## Operations

| Operation | Description |
|-----------|-------------|
| Union | Combine two polygons |
| Intersection | Common area |
| Difference | A minus B |
| Xor | Symmetric difference |

## Commands

```bash
cargo build                   # Build
cargo test                    # Run all tests
cargo bench                   # Run benchmarks
cargo fmt --all               # Format (required before commit)
cargo clippy                  # Lint
```

## Git Workflow

### Branch Protection

**The `main` branch is protected.** All changes must go through pull requests:

```bash
git checkout -b feat/my-feature
git push -u origin feat/my-feature
gh pr create --title "feat: description" --body "..."
```

### DO NOT

- Push directly to `main`
- Force push to shared branches
- Merge without CI passing

## Commit Convention

We use [Conventional Commits](https://www.conventionalcommits.org/). See `CONTRIBUTING.md`.

```bash
feat: add local minima detection
fix: correct ring orientation for holes
port(core): translate vatti_clip from C++
test: add golden tests for union operation
```

## Key Documents

| Document | Purpose |
|----------|---------|
| `context/ARCHITECTURE.md` | Design decisions, wagyu divergences |
| `CONTRIBUTING.md` | How to contribute, commit conventions |

## Setup

```bash
git config core.hooksPath .githooks  # Enable pre-commit hooks
```

## Porting Guide

**REMEMBER: TDD IS MANDATORY. WRITE TESTS BEFORE CODE.**

When porting from wagyu C++:

1. **Check for local clone** at `../wagyu`
   - If missing, use `gitingest https://github.com/mapbox/wagyu` + distill

2. **Find the corresponding C++ file** in `include/mapbox/geometry/wagyu/`

3. **Find/write tests FIRST** (before any implementation!)
   - Check `../wagyu/tests/unit/` for existing C++ tests
   - Check `../wagyu/tests/fixtures/` and `../wagyu/tests/expected/` for golden tests
   - Translate test cases to Rust `#[test]` functions
   - Run tests - they MUST FAIL (red)

4. **Implement minimum code to pass tests** (green)
   - Port ONE function at a time
   - Run tests after each function
   - Stop when tests pass

5. **Refactor** while keeping tests green

6. **Preserve comments** - include original C++ documentation

7. **Document divergences** - Rust ownership model may require changes

### Porting Workflow Example

```bash
# 1. Read C++ test
cat ../wagyu/tests/unit/edge.cpp

# 2. Write Rust test (MUST FAIL)
# In crates/core/src/edge.rs:
#[cfg(test)]
mod tests {
    #[test]
    fn test_edge_is_horizontal() {
        // Translated from edge.cpp
        todo!("implement after writing test")
    }
}

# 3. Verify RED
cargo test --package wagyu-core edge::tests::test_edge_is_horizontal
# Expected: FAILED or compile error

# 4. Implement, verify GREEN
cargo test --package wagyu-core edge::tests::test_edge_is_horizontal
# Expected: ok

# 5. Commit
git commit -m "feat(edge): add is_horizontal check (TDD green)"
```

### Key C++ Files

```
../wagyu/include/mapbox/geometry/wagyu/
├── wagyu.hpp              # Main entry point
├── vatti.hpp              # Vatti algorithm (core clipper)
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
└── almost_equal.hpp       # Floating point comparison (Google license)
```
