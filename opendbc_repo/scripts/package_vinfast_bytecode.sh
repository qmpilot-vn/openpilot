#!/usr/bin/env bash
# Compile VinFast modules to bytecode under distinct *_impl.*.pyc names, then replace
# each .py with a tiny loader (so __pycache__/modname.*.pyc from the shim cannot clobber impl).
#
# Does not keep a local plaintext copy of originals (no backup tree). To recover readable
# .py sources, use scripts/restore_vinfast_sources.sh with a GitHub PAT (private repo).
#
# Requires the SAME Python minor version at runtime as used when running this script.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PACKAGE_VINFAST_VF="${PACKAGE_VINFAST_VF:-${SCRIPT_DIR}/../opendbc/car/vinfast}"

pick_python() {
  local candidate
  for candidate in python3.12 python3.11 python3; do
    if command -v "${candidate}" >/dev/null 2>&1 && \
       "${candidate}" -c 'import sys; assert sys.version_info >= (3, 11)' >/dev/null 2>&1; then
      echo "${candidate}"
      return
    fi
  done
  for candidate in "${HOME}/.pyenv/versions/"*/bin/python; do
    if [[ -x "${candidate}" ]] && \
       "${candidate}" -c 'import sys; assert sys.version_info >= (3, 11)' >/dev/null 2>&1; then
      echo "${candidate}"
      return
    fi
  done
  echo "need Python 3.11+ to package VinFast bytecode" >&2
  exit 1
}

PYTHON="${PACKAGE_VINFAST_PYTHON:-$(pick_python)}"

"${PYTHON}" <<'PY'
from __future__ import annotations

import os
import py_compile
import sys
from pathlib import Path

vf = Path(os.environ["PACKAGE_VINFAST_VF"]).resolve()
modules = (
  "carstate",
  "vinfastcan",
  "carcontroller",
  "radar_interface",
  "info_radar_interface",
  "values",
  "fingerprints",
  "interface",
)
tag = sys.implementation.cache_tag


def shim_template(impl_stem: str) -> str:
  return f'''\
"""_BYTECODE_SHIM — logic is loaded from __pycache__/{impl_stem}.*.pyc (same Python minor as build)."""
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "{impl_stem}"
_pyc = Path(__file__).resolve().parent / "__pycache__" / f"{{_IMPL_STEM}}.{{sys.implementation.cache_tag}}.pyc"
if not _pyc.is_file():
  raise ImportError(
    "Missing bytecode %s — rebuild with opendbc_repo/scripts/package_vinfast_bytecode.sh using Python %s"
    % (_pyc, ".".join(map(str, sys.version_info[:3])))
  )
_spec = importlib.util.spec_from_file_location(__name__, _pyc)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
sys.modules[__name__] = _mod
'''

# Compile implementations to *_impl.<tag>.pyc
for name in modules:
  py = vf / f"{name}.py"
  if not py.exists():
    sys.exit(f"Missing {py}")

  head = py.read_text(encoding="utf-8")[:800]
  if "_BYTECODE_SHIM" in head:
    print(f"skip compile {name}.py (already a shim — restore from GitHub PAT first if updating bytecode)")
    continue

  impl_stem = f"{name}_impl"
  (vf / "__pycache__").mkdir(exist_ok=True)
  for p in vf.glob(f"__pycache__/{impl_stem}.*.pyc"):
    p.unlink()

  out = vf / "__pycache__" / f"{impl_stem}.{tag}.pyc"
  py_compile.compile(str(py), cfile=str(out), dfile=name, optimize=0)
  print(f"compiled implementation -> {out.name}")

# Install shims
for name in modules:
  py = vf / f"{name}.py"
  head = py.read_text(encoding="utf-8")[:800]
  if "_BYTECODE_SHIM" in head:
    continue
  py.write_text(shim_template(f"{name}_impl"), encoding="utf-8")
  print(f"wrote shim {py.name}")

print(
  "done — implementation bytecode:",
  ", ".join(f"{n}_impl.{tag}.pyc" for n in modules),
)
PY
