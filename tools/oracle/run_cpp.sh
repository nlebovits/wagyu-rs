#!/bin/bash
# Run C++ wagyu oracle on an input file
#
# Usage: ./run_cpp.sh <input.json> <operation> [fill_type]
#
# Output: JSON result to stdout

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WAGYU_RS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ORACLE_BIN="$WAGYU_RS_ROOT/../wagyu/build/wagyu-oracle"

if [ ! -f "$ORACLE_BIN" ]; then
    echo "Error: wagyu-oracle not found at $ORACLE_BIN" >&2
    echo "Run: $SCRIPT_DIR/build_oracle.sh" >&2
    exit 1
fi

if [ $# -lt 2 ]; then
    echo "Usage: $0 <input.json> <operation> [fill_type]" >&2
    echo "" >&2
    echo "Operations: union, intersection, difference, xor" >&2
    echo "Fill types: evenodd (default), nonzero, positive, negative" >&2
    exit 1
fi

INPUT_FILE="$1"
OPERATION="$2"
FILL_TYPE="${3:-evenodd}"

"$ORACLE_BIN" "$INPUT_FILE" "$OPERATION" "$FILL_TYPE"
