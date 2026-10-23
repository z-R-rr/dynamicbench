#!/usr/bin/env python3
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import json

from dynamicbench.data.dataset_reader import export_video, read_episode

p = argparse.ArgumentParser()
p.add_argument("path", type=Path)
p.add_argument("--video", type=Path)
a = p.parse_args()
paths = [a.path] if (a.path / "trajectory.npz").exists() else sorted(a.path.glob("episode_*"))
if not paths:
    p.error("no complete episode found")
for path in paths:
    arrays, metadata, summary = read_episode(path)
    print(json.dumps(dict(path=str(path), **summary, seed=metadata["seed"], source_commit=metadata["source_commit"])))
if a.video:
    if len(paths) != 1:
        p.error("--video needs one episode path")
    export_video(arrays, a.video)
