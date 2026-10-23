"""Bootstrap shared by real simulation CLIs; --help works without Isaac Sim."""

import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def parser(collect=False):
    p = argparse.ArgumentParser()
    p.add_argument("--task", default="pick_cube_from_conveyor", choices=["pick_cube_from_conveyor"])
    p.add_argument("--belt-speed", type=float, default=0.03)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--ticks", type=int, default=600)
    p.add_argument("--output", type=Path, default=ROOT / ("datasets/conveyor_cube_v0" if collect else "outputs/mvp"))
    p.add_argument("--config", type=Path, default=ROOT / "dynamicbench/configs/pick_cube_from_conveyor.yaml")
    p.add_argument("--latency-mode", choices=["no_latency", "sync_hold", "rtc_async"], default="no_latency")
    p.add_argument("--latency", type=float, default=0.25)
    p.add_argument("--no-video", action="store_true")
    if collect:
        p.add_argument("--num-success", type=int, default=3)
        p.add_argument("--max-attempts", type=int, default=10)
        p.add_argument("--save-failures", action="store_true")
    else:
        p.add_argument("--test", choices=["belt", "expert", "schedule"], default="expert")
    # Do not require simulator packages for help or dataset inspection.
    import sys

    if "--help" in sys.argv or "-h" in sys.argv:
        p.add_argument("--headless", action="store_true")
        p.print_help()
        raise SystemExit(0)
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(p)
    return p


def launch(p):
    from isaaclab.app import AppLauncher

    from env.camera_manager.capture.render_sync import add_zero_delay_kit_args

    args = p.parse_args()
    if args.ticks < 1:
        p.error("--ticks must be positive")
    if args.latency_mode != "no_latency" and getattr(args, "test", "expert") == "expert":
        p.error("expert collection currently supports no_latency; use run_mvp --test schedule for latency evaluation")
    args.enable_cameras = True
    add_zero_delay_kit_args(args)
    args.output.mkdir(parents=True, exist_ok=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    return args, AppLauncher(args), commit
