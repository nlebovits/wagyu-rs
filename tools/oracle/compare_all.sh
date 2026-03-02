#!/bin/bash
# Run oracle comparison on all test inputs
#
# Usage: ./compare_all.sh [operation]
#
# If operation is specified, runs only that operation.
# Otherwise, runs union, intersection, difference, and xor.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUT_DIR="$SCRIPT_DIR/test_inputs"

if [ ! -d "$INPUT_DIR" ]; then
    echo "Error: test_inputs directory not found at $INPUT_DIR"
    exit 1
fi

OPERATIONS="${1:-union intersection difference xor}"

PASS=0
FAIL=0
ERROR=0

echo "=== Oracle Comparison Report ==="
echo ""

for input in "$INPUT_DIR"/*.json; do
    filename=$(basename "$input")

    for op in $OPERATIONS; do
        echo -n "Testing $filename ($op)... "

        "$SCRIPT_DIR/compare.sh" "$input" "$op" evenodd >/dev/null 2>&1
        exit_code=$?

        if [ $exit_code -eq 0 ]; then
            echo "PASS"
            PASS=$((PASS + 1))
        elif [ $exit_code -eq 1 ]; then
            echo "FAIL (divergence)"
            FAIL=$((FAIL + 1))
        else
            echo "ERROR"
            ERROR=$((ERROR + 1))
        fi
    done
done

echo ""
echo "=== Summary ==="
echo "Pass: $PASS"
echo "Fail: $FAIL"
echo "Error: $ERROR"

if [ $FAIL -gt 0 ] || [ $ERROR -gt 0 ]; then
    exit 1
fi
