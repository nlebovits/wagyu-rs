# wagyu-rs

> **⚠️ Work in Progress**: This is a WIP port of Wagyu to Rust. Code is generated with Claude; please take it with a grain of salt as I figure it out. --Nissim

[![CI](https://github.com/nlebovits/wagyu-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/nlebovits/wagyu-rs/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/nlebovits/wagyu-rs/branch/main/graph/badge.svg)](https://codecov.io/gh/nlebovits/wagyu-rs)
[![Crates.io](https://img.shields.io/crates/v/wagyu-rs?color=blue)](https://crates.io/crates/wagyu-rs)

Geometry boolean operations in Rust - port of [Mapbox wagyu](https://github.com/mapbox/wagyu).

## Operations

- **Union** - Combine two polygons
- **Intersection** - Common area of two polygons
- **Difference** - Subtract one polygon from another
- **Xor** - Symmetric difference

All output is guaranteed valid per [OGC standards](http://postgis.net/docs/using_postgis_dbmanagement.html#OGC_Validity).

## Install

```bash
cargo add wagyu-core
```

## Usage

```rust,ignore
use wagyu_rs::{Operation, /* ... */};

// API under development - see context/ARCHITECTURE.md for porting progress
```

## Development

```bash
git clone https://github.com/nlebovits/wagyu-rs.git && cd wagyu-rs
git config core.hooksPath .githooks
cargo build && cargo test
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## License

BSL-1.0 (Boost Software License) - same as original wagyu.
