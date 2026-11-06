#!/usr/bin/env bash
# Build pandad from qmpilot origin/c4 (not sunnypilot_0_10).
set -euo pipefail
C4_ROOT="${C4_ROOT:-/data/qmpilot_c4_build}"
CAPNP_ROOT="${CAPNP_ROOT:-/usr/local/venv/lib/python3.12/site-packages/capnproto/install}"
OP_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

if [[ ! -f "$C4_ROOT/SConstruct" ]]; then
  echo "Missing $C4_ROOT — create with:"
  echo "  cd /data/openpilot_qmpilot && git worktree add $C4_ROOT origin/c4"
  exit 1
fi

rsync -a --delete "$OP_ROOT/selfdrive/pandad/" "$C4_ROOT/selfdrive/pandad/" \
  --exclude='pandad' --exclude='*.o' --exclude='__pycache__' \
  --exclude='libpanda.a' --exclude='*.so' --exclude='build_pandad_c4.sh'

cd "$C4_ROOT"
scons --ccflags="-I${CAPNP_ROOT}/include" -j"$(nproc)" \
  selfdrive/pandad/pandad selfdrive/pandad/pandad_api_impl.so

cp -f "$C4_ROOT/selfdrive/pandad/pandad" "$OP_ROOT/selfdrive/pandad/pandad"
cp -f "$C4_ROOT/selfdrive/pandad/pandad_api_impl.so" "$OP_ROOT/selfdrive/pandad/pandad_api_impl.so"
rsync -a "$C4_ROOT/selfdrive/pandad/" "$OP_ROOT/selfdrive/pandad/" \
  --exclude='pandad' --exclude='*.o' --exclude='__pycache__' \
  --exclude='libpanda.a' --exclude='*.so' --exclude='build_pandad_c4.sh'

echo "Installed: $OP_ROOT/selfdrive/pandad/pandad"
ls -la "$OP_ROOT/selfdrive/pandad/pandad" "$OP_ROOT/selfdrive/pandad/pandad_api_impl.so"
