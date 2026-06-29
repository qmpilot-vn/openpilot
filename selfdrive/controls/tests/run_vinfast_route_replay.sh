#!/usr/bin/env bash
# Automated VinFast route radar replay — download rlog + re-run radard.get_lead().
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
ROUTE="${1:-5928b56aca881917/0000011b--989a00c832}"
SEGMENTS="${2:-all}"
export PYENV_VERSION="${PYENV_VERSION:-3.11.4}"
export PYTHONPATH="$ROOT"
cd "$ROOT"
exec python3 selfdrive/controls/tests/test_vinfast_route_replay.py "$ROUTE" --segments "$SEGMENTS"
