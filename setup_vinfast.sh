#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PRIVATE="$SCRIPT_DIR/vinfast_private"
VINFAST_CAR="$SCRIPT_DIR/opendbc_repo/opendbc/car/vinfast"
DBC_DIR="$SCRIPT_DIR/opendbc_repo/opendbc/dbc"

if [ -d "$PRIVATE" ]; then
  cp "$PRIVATE/dbc/"*.dbc "$DBC_DIR/"
  echo "Deployed VinFast DBC files from vinfast_private."

  cp "$PRIVATE/car/"*.py "$VINFAST_CAR/"
  echo "Deployed VinFast Python source files from vinfast_private."
else
  echo "vinfast_private not found — packaging sources already in tree."
fi

echo "Packaging VinFast modules as bytecode shims..."
bash "$SCRIPT_DIR/opendbc_repo/scripts/package_vinfast_bytecode.sh"

echo "Packaging VinFast DBC caches..."
bash "$SCRIPT_DIR/opendbc_repo/scripts/package_vinfast_dbc.sh" --strip-dbc

echo "VinFast setup complete. Runtime uses bytecode shims and .dbc.cache files."
