#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"
export LD_LIBRARY_PATH="$root/outputs/deps/root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OMNI_KIT_ACCEPT_EULA=YES
entry="${1:-run_mvp}"
shift || true
case "$entry" in run_mvp|collect_expert|validate_mvp) ;; *) echo 'Use run_mvp, collect_expert or validate_mvp' >&2; exit 2;; esac
exec python -u "scripts/dynamicbench/$entry.py" "$@"
