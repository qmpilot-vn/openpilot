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

# qmpilot-server upload target (force — do not keep stale mr-one API_HOST)
# API key: set QMPILOT_API_KEY or put it in /data/qmpilot/QmpilotApiKey (do not commit secrets)
export API_HOST="https://qmpilot-connect.com"

# Normal C3X: use real panda (pandad enabled). Manager skips pandad if NOBOARD is set
# to any value — it must be unset, not 0. Bench mode overrides via bench_env.sh below.
unset NOBOARD

