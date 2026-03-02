#!/bin/bash
# Compare Rust wagyu output against C++ oracle
#
# Usage: ./compare.sh <input.json> <operation> [fill_type] [--debug]
#
# Exit codes:
#   0 - Outputs match
#   1 - Outputs differ
#   2 - Error running one of the implementations
#
# Flags:
#   --debug  Enable debug logging and compare logs (outputs to stderr)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WAGYU_RS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

DEBUG_MODE=false

# Parse arguments
POSITIONAL_ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            DEBUG_MODE=true
            shift
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

# Restore positional parameters
set -- "${POSITIONAL_ARGS[@]}"

if [ $# -lt 2 ]; then
    echo "Usage: $0 <input.json> <operation> [fill_type] [--debug]" >&2
    exit 2
fi

INPUT_FILE="$1"
OPERATION="$2"
FILL_TYPE="${3:-evenodd}"

# Create temp files for outputs
CPP_OUT=$(mktemp)
RUST_OUT=$(mktemp)
CPP_LOG=$(mktemp)
RUST_LOG=$(mktemp)
trap "rm -f $CPP_OUT $RUST_OUT $CPP_LOG $RUST_LOG" EXIT

# Set debug flags
CPP_DEBUG_FLAG=""
RUST_ENV=""
if [ "$DEBUG_MODE" = true ]; then
    CPP_DEBUG_FLAG="--debug"
    RUST_ENV="WAGYU_DEBUG=1"
fi

# Run C++ oracle
echo "Running C++ wagyu..." >&2
CPP_ORACLE="$WAGYU_RS_ROOT/../wagyu/build/wagyu-oracle"
if [ ! -f "$CPP_ORACLE" ]; then
    echo "Error: C++ oracle not found. Run: ./tools/oracle/build_oracle.sh" >&2
    exit 2
fi
if [ "$DEBUG_MODE" = true ]; then
    if ! "$CPP_ORACLE" "$INPUT_FILE" "$OPERATION" "$FILL_TYPE" $CPP_DEBUG_FLAG > "$CPP_OUT" 2>"$CPP_LOG"; then
        echo "Error: C++ oracle failed" >&2
        exit 2
    fi
else
    if ! "$CPP_ORACLE" "$INPUT_FILE" "$OPERATION" "$FILL_TYPE" > "$CPP_OUT" 2>/dev/null; then
        echo "Error: C++ oracle failed" >&2
        exit 2
    fi
fi

# Run Rust wagyu
echo "Running Rust wagyu..." >&2
RUST_ORACLE="$WAGYU_RS_ROOT/target/release/wagyu-oracle"
if [ ! -f "$RUST_ORACLE" ]; then
    echo "Building Rust oracle..." >&2
    cargo build --release --bin wagyu-oracle 2>/dev/null
fi
if [ "$DEBUG_MODE" = true ]; then
    if ! env $RUST_ENV "$RUST_ORACLE" "$INPUT_FILE" "$OPERATION" "$FILL_TYPE" > "$RUST_OUT" 2>"$RUST_LOG"; then
        echo "Error: Rust oracle failed" >&2
        exit 2
    fi
else
    if ! "$RUST_ORACLE" "$INPUT_FILE" "$OPERATION" "$FILL_TYPE" > "$RUST_OUT" 2>/dev/null; then
        echo "Error: Rust oracle failed" >&2
        exit 2
    fi
fi

# Compare outputs (normalized JSON)
echo "Comparing outputs..." >&2

# Use jq to normalize JSON (sort arrays, consistent formatting)
CPP_NORMALIZED=$(jq -S '.' "$CPP_OUT" 2>/dev/null || cat "$CPP_OUT")
RUST_NORMALIZED=$(jq -S '.' "$RUST_OUT" 2>/dev/null || cat "$RUST_OUT")

if [ "$CPP_NORMALIZED" = "$RUST_NORMALIZED" ]; then
    echo "MATCH" >&2

    # If debug mode, still show logs for inspection
    if [ "$DEBUG_MODE" = true ]; then
        echo "" >&2
        echo "=== C++ Debug Log ===" >&2
        cat "$CPP_LOG" >&2
        echo "" >&2
        echo "=== Rust Debug Log ===" >&2
        cat "$RUST_LOG" >&2
    fi

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
    echo "=== Output Diff ===" >&2
    diff <(echo "$CPP_NORMALIZED") <(echo "$RUST_NORMALIZED") >&2 || true

    # If debug mode, show log diff
    if [ "$DEBUG_MODE" = true ]; then
        echo "" >&2
        echo "=== C++ Debug Log ===" >&2
        cat "$CPP_LOG" >&2
        echo "" >&2
        echo "=== Rust Debug Log ===" >&2
        cat "$RUST_LOG" >&2
        echo "" >&2
        echo "=== Log Diff ===" >&2
        diff "$CPP_LOG" "$RUST_LOG" >&2 || true
    fi

    exit 1
fi
