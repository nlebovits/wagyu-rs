#!/bin/bash
# Compare Rust wagyu output against C++ oracle
#
# Usage: ./compare.sh <input.json> <operation> [fill_type]
#
# Exit codes:
#   0 - Outputs match
#   1 - Outputs differ
#   2 - Error running one of the implementations

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WAGYU_RS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ $# -lt 2 ]; then
    echo "Usage: $0 <input.json> <operation> [fill_type]" >&2
    exit 2
fi

INPUT_FILE="$1"
OPERATION="$2"
FILL_TYPE="${3:-evenodd}"

# Create temp files for outputs
CPP_OUT=$(mktemp)
RUST_OUT=$(mktemp)
trap "rm -f $CPP_OUT $RUST_OUT" EXIT

# Run C++ oracle
echo "Running C++ wagyu..." >&2
CPP_ORACLE="$WAGYU_RS_ROOT/../wagyu/build/wagyu-oracle"
if [ ! -f "$CPP_ORACLE" ]; then
    echo "Error: C++ oracle not found. Run: ./tools/oracle/build_oracle.sh" >&2
    exit 2
fi
if ! "$CPP_ORACLE" "$INPUT_FILE" "$OPERATION" "$FILL_TYPE" > "$CPP_OUT" 2>/dev/null; then
    echo "Error: C++ oracle failed" >&2
    exit 2
fi

# Run Rust wagyu
echo "Running Rust wagyu..." >&2
RUST_ORACLE="$WAGYU_RS_ROOT/target/release/wagyu-oracle"
if [ ! -f "$RUST_ORACLE" ]; then
    echo "Building Rust oracle..." >&2
    cargo build --release --bin wagyu-oracle 2>/dev/null
fi
if ! "$RUST_ORACLE" "$INPUT_FILE" "$OPERATION" "$FILL_TYPE" > "$RUST_OUT" 2>/dev/null; then
    echo "Error: Rust oracle failed" >&2
    exit 2
fi

# Compare outputs (normalized JSON)
echo "Comparing outputs..." >&2

# Use jq to normalize JSON (sort arrays, consistent formatting)
CPP_NORMALIZED=$(jq -S '.' "$CPP_OUT" 2>/dev/null || cat "$CPP_OUT")
RUST_NORMALIZED=$(jq -S '.' "$RUST_OUT" 2>/dev/null || cat "$RUST_OUT")

if [ "$CPP_NORMALIZED" = "$RUST_NORMALIZED" ]; then
    echo "MATCH" >&2
    exit 0
else
    echo "DIVERGENCE" >&2
    echo "" >&2
    echo "=== C++ output ===" >&2
    echo "$CPP_NORMALIZED" >&2
    echo "" >&2
    echo "=== Rust output ===" >&2
    echo "$RUST_NORMALIZED" >&2
    echo "" >&2
    echo "=== Diff ===" >&2
    diff <(echo "$CPP_NORMALIZED") <(echo "$RUST_NORMALIZED") >&2 || true
    exit 1
fi
