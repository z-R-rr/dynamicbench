"""Bootstrap shared by real simulation CLIs; --help works without Isaac Sim."""

import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def parser(collect=False, compare_lift=False):
    p = argparse.ArgumentParser()
    p.add_argument("--task", default="pick_cube_from_conveyor", choices=["pick_cube_from_conveyor"])
    p.add_argument("--belt-speed", type=float, default=0.03)
    p.add_argument("--belt-source", choices=["primitive", "robodojo"], default=None)
    p.add_argument("--scene-source", choices=["minimal", "robodojo"], default=None)
    p.add_argument("--lift-xy-mode", choices=["fixed", "track_cube", "contact_anchor"], default=None)
    p.add_argument("--lift-tcp-frame", choices=["nominal", "actual_pinch"], default=None)
    if compare_lift:
        p.add_argument("--compare-lift", action="store_true")
    p.add_argument(
        "--render-gpu", type=int, default=None, help="physical Vulkan GPU index when CUDA indices are remapped"
    )
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
    # Lab 3 removed this CLI flag; keep the public DynamicBench command stable.
    if "--headless" not in p._option_string_actions:
        p.add_argument("--headless", action="store_true")
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
    # Warp is imported while Kit extensions start, before SimulationContext.
    # Select the requested CUDA context now, otherwise it can allocate on GPU 0
    # even when AppLauncher renders on a different physical GPU.
    if args.device.startswith("cuda"):
        import torch

        torch.cuda.set_device(args.device)
        print(f"[DynamicBench] initial CUDA context: {torch.cuda.current_device()}", flush=True)
    # Native Isaac Lab Camera render products can deadlock at their first warmup
    # with both idle waits enabled in the older server build. Keep rendering
    # asynchronous and fence two delivered frames at the same frozen physics
    # state in wait_for_latest_cameras instead. Never advance physics to render.
    kit_defaults = {
        "/renderer/multiGpu/enabled": "false",
        "/app/renderer/waitIdle": "false",
        "/app/hydraEngine/waitIdle": "false",
    }
    for key, value in kit_defaults.items():
        if f"--{key}=" not in (args.kit_args or ""):
            args.kit_args = ((args.kit_args or "") + f" --{key}={value}").strip()
    add_zero_delay_kit_args(args)
    args.output.mkdir(parents=True, exist_ok=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    # SimulationApp's multi_gpu launch config otherwise defaults to True and
    # can override the renderer setting while enumerating unrelated busy GPUs.
    class SingleGpuLauncher(AppLauncher):
        def _create_app(self):
            # Isaac Lab derives this from the CUDA ordinal. Under an explicit
            # CUDA mask the Vulkan ordinal must instead name the physical GPU.
            if args.render_gpu is not None:
                self._sim_app_config["active_gpu"] = args.render_gpu
            super()._create_app()

    return args, SingleGpuLauncher(args, multi_gpu=False), commit


def load_settings(args):
    import yaml

    settings = yaml.safe_load(args.config.read_text())
    if "layout" in settings:
        from dynamicbench.layout import resolve_robodojo_layout

        settings = resolve_robodojo_layout(settings, ROOT)
    if args.belt_source is not None:
        settings["belt"]["source"] = args.belt_source
    if args.lift_xy_mode is not None:
        settings["expert"]["lift_xy_mode"] = args.lift_xy_mode
    if args.lift_tcp_frame is not None:
        settings["expert"]["lift_tcp_frame"] = args.lift_tcp_frame
    if args.scene_source is not None:
        settings.setdefault("scene", {})["source"] = args.scene_source
    if settings.get("scene", {}).get("source") == "robodojo":
        from dynamicbench.scene import resolve_scene

        settings = resolve_scene(settings, ROOT)
    return settings


def finish(args, status):
    """Persist status and give Kit the same return code before its shutdown.

    Some Kit builds terminate from close() before Python's SystemExit runs.
    A failed physical acceptance must still report a nonzero exit code.
    """
    import json

    import omni.kit.app

    temp = args.output / ".exit_status.tmp"
    temp.write_text(json.dumps({"exit_code": status}))
    temp.replace(args.output / "exit_status.json")
    omni.kit.app.get_app().post_quit(status)
    return status


def exception_details(exc):
    import builtins
    import traceback

    callback = getattr(builtins, "ISAACLAB_CALLBACK_EXCEPTION", None)
    return dict(
        status="ERROR",
        error=repr(exc),
        traceback=traceback.format_exc(),
        isaac_callback_error=repr(callback) if callback else None,
        isaac_callback_traceback="".join(traceback.format_exception(callback)) if callback else None,
    )
