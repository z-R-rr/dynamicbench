#!/usr/bin/env bash
# Independent pinned installation. Never modifies RoboDojo's Conda environment.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
lab="$root/outputs/deps/IsaacLab61"
bootstrap="$root/outputs/envs/isaacsim61"
sha=ae37b028ea415c91ea2bc32609efcd759ed2b974
mkdir -p "$root/outputs/deps" "$root/outputs/envs"
if [[ ! -e "$lab" ]]; then
    git clone --depth 1 --branch v3.0.0-EA https://github.com/isaac-sim/IsaacLab.git "$lab"
fi
[[ "$(git -C "$lab" rev-parse HEAD)" == "$sha" ]] || { echo 'Unexpected Isaac Lab checkout' >&2; exit 1; }
[[ -z "$(git -C "$lab" status --porcelain)" ]] || { echo 'Isaac Lab checkout has local edits' >&2; exit 1; }
if [[ ! -x "$bootstrap/bin/python" ]]; then
    conda create -y -p "$bootstrap" --override-channels -c conda-forge python=3.12.15 pip
fi
"$bootstrap/bin/python" -m pip install uv==0.13.5
export UV_PROJECT_ENVIRONMENT="$root/outputs/envs/dynamicbench61"
export UV_CACHE_DIR="$root/outputs/cache/uv61"
export UV_HTTP_TIMEOUT=180
"$bootstrap/bin/uv" sync --project "$lab" --extra isaacsim --frozen --python "$bootstrap/bin/python" --no-dev
"$bootstrap/bin/uv" pip install --python "$UV_PROJECT_ENVIRONMENT/bin/python" omegaconf==2.4.0.dev13 imageio==2.37.2 imageio-ffmpeg==0.6.0 pytest==9.1.1 pyyaml==6.0.3
"$UV_PROJECT_ENVIRONMENT/bin/python" -c 'from importlib.metadata import version; assert version("isaacsim") == "6.1.0.0"; print("Isaac Sim", version("isaacsim"), "Isaac Lab", version("isaaclab"))'
