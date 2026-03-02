#!/bin/bash
# Build the C++ wagyu oracle binary
#
# Prerequisites:
# - C++ wagyu cloned at ../wagyu relative to wagyu-rs root
# - CMake and a C++ compiler installed
#
# This script:
# 1. Copies oracle.cpp into the wagyu build tree
# 2. Adds it to CMakeLists.txt if needed
# 3. Builds the oracle binary

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WAGYU_RS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WAGYU_CPP_ROOT="$(cd "$WAGYU_RS_ROOT/../wagyu" 2>/dev/null && pwd)" || {
    echo "Error: C++ wagyu not found at ../wagyu"
    echo ""
    echo "Please clone it first:"
    echo "  cd $WAGYU_RS_ROOT/.."
    echo "  git clone https://github.com/mapbox/wagyu.git"
    exit 1
}

echo "wagyu-rs root: $WAGYU_RS_ROOT"
echo "C++ wagyu root: $WAGYU_CPP_ROOT"

# Copy oracle.cpp to wagyu tools directory
ORACLE_SRC="$SCRIPT_DIR/oracle.cpp"
ORACLE_DEST="$WAGYU_CPP_ROOT/tools"
mkdir -p "$ORACLE_DEST"
cp "$ORACLE_SRC" "$ORACLE_DEST/"

echo "Copied oracle.cpp to $ORACLE_DEST/"

# Check if CMakeLists.txt already has the oracle target
CMAKE_FILE="$WAGYU_CPP_ROOT/CMakeLists.txt"
if ! grep -q "wagyu-oracle" "$CMAKE_FILE"; then
    echo "Adding wagyu-oracle target to CMakeLists.txt..."

    # Append oracle target to CMakeLists.txt
    cat >> "$CMAKE_FILE" << 'CMAKEEOF'

# wagyu-oracle: CLI tool for oracle testing
add_executable(wagyu-oracle tools/oracle.cpp)
CMAKEEOF

    echo "Added wagyu-oracle target"
else
    echo "wagyu-oracle target already in CMakeLists.txt"
fi

# Build
echo ""
echo "Building wagyu-oracle..."
cd "$WAGYU_CPP_ROOT"

# Create build directory if needed
mkdir -p build
cd build

# Always re-run cmake to pick up changes
echo "Running cmake..."
cmake .. -DCMAKE_BUILD_TYPE=Release -DWERROR=OFF

# Build just the oracle
make wagyu-oracle

# Verify it was built
if [ -f wagyu-oracle ]; then
    echo ""
    echo "Success! Oracle built at:"
    echo "  $WAGYU_CPP_ROOT/build/wagyu-oracle"
    echo ""
    echo "Test it with:"
    echo "  $WAGYU_CPP_ROOT/build/wagyu-oracle --help"
else
    echo "Error: wagyu-oracle not found after build"
    exit 1
fi
