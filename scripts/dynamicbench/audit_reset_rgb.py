#!/usr/bin/env python3
"""Audit the first RGB in complete conveyor episodes; no simulator required."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import json

from dynamicbench.data.dataset_reader import read_episode
from dynamicbench.data.reset_image_audit import audit_reset_image

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("path", type=Path)
p.add_argument("--output", type=Path)
args = p.parse_args()
paths = [args.path] if (args.path / "trajectory.npz").exists() else sorted(args.path.glob("*/trajectory.npz"))
paths = [path.parent if path.is_file() else path for path in paths]
if not paths:
    p.error("no complete episode found")
results = []
for path in paths:
    arrays, metadata, _ = read_episode(path)
    result = audit_reset_image(
        arrays["observation.images.head"][0],
        arrays["oracle_debug.cube_pose"][0, :3],
        metadata["settings"]["camera"],
    )
    results.append(dict(episode=path.name, source_commit=metadata["source_commit"], **result))
report = dict(passed=all(row["passed"] for row in results), episodes=results)
if args.output:
    args.output.write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
raise SystemExit(0 if report["passed"] else 2)
