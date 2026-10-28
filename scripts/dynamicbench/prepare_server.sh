#!/usr/bin/env bash
# Use an existing, compatible Isaac Sim environment; no system-wide changes.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"
python -m pip install -r scripts/dynamicbench/requirements.txt
python - <<'PY'
from importlib.metadata import version
for name in ('isaacsim','isaaclab','torch','numpy','scipy'):
    print(name,version(name))
PY
# libGLU is needed by MDL even for headless RGB. Prefer installed libraries;
# otherwise extract distro packages into this repository's ignored output area.
if ! ldconfig -p 2>/dev/null | grep -q 'libGLU.so.1'; then
  mkdir -p outputs/deps
  (
    cd outputs/deps
    apt-get download libglu1-mesa libopengl0
    for package in ./*.deb; do dpkg-deb -x "$package" root; done
  )
fi
if [[ ! -f Assets/Robots/x5/ARX.usd || ! -f Assets/Robots/x5/X5A.urdf ]]; then
  echo 'Assets missing. Run bash scripts/init_assets.sh or link an existing complete Assets directory.' >&2
  exit 2
fi
