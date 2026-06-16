#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PRIVATE="$SCRIPT_DIR/vinfast_private"
VINFAST_CAR="$SCRIPT_DIR/opendbc_repo/opendbc/car/vinfast"
DBC_DIR="$SCRIPT_DIR/opendbc_repo/opendbc/dbc"

if [ ! -d "$PRIVATE" ]; then
  echo "Error: vinfast_private submodule not found."
  echo "Run: git submodule update --init vinfast_private"
  exit 1
fi

# Deploy DBC files (needed at runtime for CAN parsing)
cp "$PRIVATE/dbc/"*.dbc "$DBC_DIR/"
echo "Deployed VinFast DBC files."

# Deploy source .py files (needed for Cython compilation)
cp "$PRIVATE/car/"*.py "$VINFAST_CAR/"
echo "Deployed VinFast Python source files."

# Compile to .so (requires Cython + C compiler)
echo "Compiling VinFast modules with Cython..."
python "$VINFAST_CAR/build_vinfast.py"
echo "Compilation complete."

# Remove .py source files after compilation (keep only .so)
rm -f "$VINFAST_CAR/vinfastcan.py"
rm -f "$VINFAST_CAR/carcontroller.py"
rm -f "$VINFAST_CAR/carstate.py"
rm -f "$VINFAST_CAR/radar_interface.py"
# Also remove Cython intermediate .c files
rm -f "$VINFAST_CAR/vinfastcan.c"
rm -f "$VINFAST_CAR/carcontroller.c"
rm -f "$VINFAST_CAR/carstate.c"
rm -f "$VINFAST_CAR/radar_interface.c"
# Clean up build directory
rm -rf "$SCRIPT_DIR/build"
echo "Cleaned up source and intermediate files."

# Package DBCs as runtime caches and remove plaintext from disk
echo "Packaging VinFast DBC caches..."
bash "$SCRIPT_DIR/opendbc_repo/scripts/package_vinfast_dbc.sh" --strip-dbc

echo "VinFast setup complete. Runtime uses .so modules and .dbc.cache files."
