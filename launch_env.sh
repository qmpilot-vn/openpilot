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

# Force comma connect (not qmpilot-connect) — overrides stale ApiHost / env on device
export API_HOST="https://api.commadotai.com"
unset QMPILOT_API_KEY

# Normal C3X: use real panda (pandad enabled). Manager skips pandad if NOBOARD is set
# to any value — it must be unset, not 0. Bench mode overrides via bench_env.sh below.
unset NOBOARD

