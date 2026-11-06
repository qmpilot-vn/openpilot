#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="18.4"
fi

export STAGING_ROOT="/data/safe_staging"

# Settings → Device → backend. 1 is qmpilot connect. Anything else is comma connect.
# The key stays in the param file. Manager logs the environment, so it is not exported.
_read_saved() {
  local name="$1" f val
  for f in "/data/params/d/$name" "/data/qmpilot/$name" "/persist/qmpilot/$name"; do
    if [ -f "$f" ]; then
      val=$(tr -d '\r\n' < "$f")
      if [ -n "$val" ]; then
        printf '%s' "$val"
        return 0
      fi
    fi
  done
}

_backend=$(_read_saved UploadBackend)
_host=$(_read_saved ApiHost)
_host=${_host%/}
_key=$(_read_saved QmpilotApiKey)
if [ "$_backend" = "1" ] && [ -n "$_host" ] && [ "$_host" != "https://api.commadotai.com" ] && [ -n "$_key" ]; then
  export API_HOST="$_host"
else
  export API_HOST="https://api.commadotai.com"
fi
unset QMPILOT_API_KEY
unset _backend _host _key
unset -f _read_saved

# Normal C3X: use real panda (pandad enabled). Manager skips pandad if NOBOARD is set
# to any value — it must be unset, not 0. Bench mode overrides via bench_env.sh below.
unset NOBOARD

