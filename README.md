# wagyu-rs

> This library is a port of Wagyu into Rust from the original C++. Although 100% of golden tests from the original library pass, code has still been generated completely with Claude. Please take it with a grain of salt until it stabilizes. --Nissim

[![CI](https://github.com/nlebovits/wagyu-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/nlebovits/wagyu-rs/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/nlebovits/wagyu-rs/branch/main/graph/badge.svg)](https://codecov.io/gh/nlebovits/wagyu-rs)
[![Crates.io](https://img.shields.io/crates/v/wagyu-rs?color=blue)](https://crates.io/crates/wagyu-rs)

Geometry boolean operations in Rust - port of [Mapbox wagyu](https://github.com/mapbox/wagyu).

## Operations

- **Union** - Combine two polygons
- **Intersection** - Common area of two polygons
- **Difference** - Subtract one polygon from another
- **Xor** - Symmetric difference

All output is guaranteed valid per [OGC standards](http://postgis.net/docs/using_postgis_dbmanagement.html#OGC_Validity). All 148 golden tests from the original wagyu pass.

For algorithm details, see the [original wagyu documentation](https://github.com/mapbox/wagyu#documentation).

## Install

```bash
cargo add wagyu-rs
```

## Usage

```rust,ignore
use wagyu_rs::{
    config::{FillType, PolygonType},
    point::Point,
    wagyu::Wagyu,
    Operation,
};

// Create wagyu instance
let mut wagyu: Wagyu<i64> = Wagyu::new();

// Add subject polygon (ring as slice of Points)
let subject: Vec<Point<i64>> = vec![
    Point::new(0, 0), Point::new(100, 0),
    Point::new(100, 100), Point::new(0, 100)
];
wagyu.add_ring(&subject.into(), PolygonType::Subject);

// Add clip polygon
let clip: Vec<Point<i64>> = vec![
    Point::new(50, 50), Point::new(150, 50),
    Point::new(150, 150), Point::new(50, 150)
];
wagyu.add_ring(&clip.into(), PolygonType::Clip);

// Execute operation - returns rings as result
let result = wagyu.execute(
    Operation::Intersection,
    FillType::EvenOdd,
    FillType::EvenOdd
);
```

See `crates/cli/src/oracle.rs` for a complete working example.

## Reporting Issues

If you find a case where wagyu-rs produces incorrect output:

1. **Save the input polygons** as JSON (see `tools/oracle/test_inputs/` for format)
2. **Run the oracle comparison**: `./tools/oracle/compare.sh input.json union evenodd`
3. **File an issue** with the JSON file attached

The oracle harness compares Rust vs C++ output to identify divergences.

## Development

```bash
git clone https://github.com/nlebovits/wagyu-rs.git && cd wagyu-rs
git config core.hooksPath .githooks
cargo build && cargo test
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## License

BSL-1.0 (Boost Software License) - same as original wagyu.
