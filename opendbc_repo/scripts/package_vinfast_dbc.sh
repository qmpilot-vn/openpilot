#!/usr/bin/env bash
# Build pickled VinFast DBC runtime caches and optionally remove plaintext .dbc files.
#
# Public releases ship only *.dbc.cache (loaded by opendbc.can.dbc.DBC at runtime).
# Keep plaintext .dbc files in a private repo / dev tree; run this before pushing
# to qmpilot-vn.
#
# Usage:
#   ./scripts/package_vinfast_dbc.sh            # write caches, keep .dbc
#   ./scripts/package_vinfast_dbc.sh --strip-dbc  # write caches, delete .dbc

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DBC_DIR="${PACKAGE_VINFAST_DBC_DIR:-${SCRIPT_DIR}/../opendbc/dbc}"
STRIP_DBC=0

pick_python() {
  local candidate
  for candidate in python3.12 python3.11 python3; do
    if command -v "${candidate}" >/dev/null 2>&1 && \
       "${candidate}" -c 'import enum; assert hasattr(enum, "ReprEnum")' >/dev/null 2>&1; then
      echo "${candidate}"
      return
    fi
  done
  for candidate in "${HOME}/.pyenv/versions/"*/bin/python; do
    if [[ -x "${candidate}" ]] && \
       "${candidate}" -c 'import enum; assert hasattr(enum, "ReprEnum")' >/dev/null 2>&1; then
      echo "${candidate}"
      return
    fi
  done
  echo "need Python 3.11+ (enum.ReprEnum) to package VinFast DBC caches" >&2
  exit 1
}

PYTHON="$(pick_python)"

for arg in "$@"; do
  case "${arg}" in
    --strip-dbc) STRIP_DBC=1 ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *)
      echo "unknown argument: ${arg}" >&2
      exit 1
      ;;
  esac
done

VINFAST_DBCS=(
  vinfast_vf6_chassis_can
  vinfast_vf6_info_can
  vinfast_vf8_body_can
  vinfast_vf8_chassis_can
  vinfast_vf8_info_can
  vinfast_vf8_mrr_scam
)

export DBC_DIR
export VINFAST_DBCS_JSON
VINFAST_DBCS_JSON="$(printf '%s\n' "${VINFAST_DBCS[@]}" | "${PYTHON}" -c 'import json,sys; print(json.dumps([l.strip() for l in sys.stdin if l.strip()]))')"

"${PYTHON}" <<'PY'
from __future__ import annotations

import json
import os
import pickle
import sys
from pathlib import Path

repo_root = Path(os.environ.get("PACKAGE_VINFAST_OPENDBC_ROOT", Path(os.environ["DBC_DIR"]).resolve().parents[1]))
sys.path.insert(0, str(repo_root))

from opendbc.can.dbc import DBC

dbc_dir = Path(os.environ["DBC_DIR"]).resolve()
names = json.loads(os.environ["VINFAST_DBCS_JSON"])

for name in names:
  plain = dbc_dir / f"{name}.dbc"
  cache = dbc_dir / f"{name}.dbc.cache"
  if not plain.is_file():
    if cache.is_file():
      print(f"skip {name} — no plaintext .dbc (cache already present)")
      continue
    sys.exit(f"Missing plaintext DBC: {plain}")

  parsed = DBC(str(plain))
  cache.write_bytes(DBC.cache_blob(parsed))
  print(f"wrote {cache.name} ({cache.stat().st_size} bytes, {len(parsed.msgs)} msgs)")

print("done — runtime will load *.dbc.cache when plaintext .dbc is absent")
PY

if [[ "${STRIP_DBC}" -eq 1 ]]; then
  for name in "${VINFAST_DBCS[@]}"; do
    if [[ -f "${DBC_DIR}/${name}.dbc" ]]; then
      rm -f "${DBC_DIR}/${name}.dbc"
      echo "removed ${name}.dbc"
    fi
  done
  echo "plaintext VinFast DBC files removed — ship only *.dbc.cache"
fi
