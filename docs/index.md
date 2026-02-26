# wagyu-rs

Rust port of [Mapbox wagyu](https://github.com/mapbox/wagyu) - geometry boolean operations.

## Operations

- **Union** - Combine two polygons
- **Intersection** - Common area
- **Difference** - A minus B
- **Xor** - Symmetric difference

All output is guaranteed valid per [OGC standards](http://postgis.net/docs/using_postgis_dbmanagement.html#OGC_Validity).

## Install

```bash
cargo add wagyu-core
```

## Usage

```rust,ignore
use wagyu_core::{Operation, /* ... */};

// Coming soon - API under development
```

## Development

```bash
git clone https://github.com/nlebovits/wagyu-rs.git && cd wagyu-rs
git config core.hooksPath .githooks
cargo build && cargo test
```

See [Contributing](contributing.md) for details.

## License

BSL-1.0 (Boost Software License)
