#!/usr/bin/env bash
# Default entrypoint — exec launch_chffrplus (no 1.sh bootstrap).
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd "$DIR"

# MR-One cloud (optional; override in launch_env.sh if needed)
export ATHENA_HOST="${ATHENA_HOST:-ws://athena.mr-one.cn}"
export API_HOST="${API_HOST:-http://res.mr-one.cn}"

exec ./launch_chffrplus.sh
