#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export PXR_WORK_THREAD_LIMIT="${PXR_WORK_THREAD_LIMIT:-8}"
export PATH="$root/outputs/envs/dynamicbench61/bin:$PATH"
python -c 'from importlib.metadata import version; assert version("isaacsim") == "6.1.0.0", "Use the pinned 6.1 environment"'
exec bash "$root/scripts/dynamicbench/run_isaac.sh" "$@"
