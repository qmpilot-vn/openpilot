#!/usr/bin/env bash
# Default entrypoint — exec launch_chffrplus (no 1.sh bootstrap).
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd "$DIR"

# qmpilot-server (upload URL + callback). Athena still optional via ATHENA_HOST.
# API key: set QMPILOT_API_KEY or put it in /data/qmpilot/QmpilotApiKey (do not commit secrets)
export API_HOST="${API_HOST:-https://qmpilot-connect.com}"

exec ./launch_chffrplus.sh
