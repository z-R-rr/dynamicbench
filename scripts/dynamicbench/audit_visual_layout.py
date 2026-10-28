#!/usr/bin/env python3
"""Render-only layout ablations; never use these variants as grasp datasets."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    import json
    import traceback

    from dynamicbench.cli import exception_details, finish, launch, load_settings, parser

    p = parser()
    p.add_argument("--visual-variant", choices=["current", "official_belt", "official_belt_home"], default="current")
    p.add_argument("--reference-layout", type=Path)
    args, launcher, commit = launch(p)
    env = None
    try:
        import carb.settings
        import imageio.v2 as imageio
        from pxr import Usd, UsdGeom

        from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask
        from dynamicbench.envs.fixtures61 import transform
        from dynamicbench.envs.runtime import ISAAC61, runtime_metadata, tensor

        if not ISAAC61:
            raise RuntimeError("This isolated visual diagnostic requires the pinned 6.1 launcher environment")
        reference = args.reference_layout or Path(
            "Assets/Eval_Layout/RoboDojo/arx_x5/0/pick_from_conveyor_by_image_0.json"
        )
        original = json.loads(reference.read_text())["Dynamic"]["conveyor"][0]

        class VisualTask(ConveyorCubeTask):
            def _spawn_original_belt(self):
                belt = super()._spawn_original_belt()
                if args.visual_variant != "current":
                    root = self.stage.GetPrimAtPath("/World/envs/env_0/OriginalConveyor")
                    transform(root, original["default_pos"], original["default_ori"], original["scale"])
                return belt

        settings = load_settings(args)
        settings["belt"]["source"] = "robodojo"
        if original["category_idx"] != int(settings["belt"].get("asset_index", "00001")):
            raise ValueError("Reference and task must use the same original conveyor asset")
        if args.visual_variant == "official_belt_home":
            settings["robot"]["initial_q"] = [0.0] * 6
        env = VisualTask(settings, launcher.app, args.seed)
        # Capture at the settled initial state; no expert/controller is executed.
        before = env.physics_step
        for _ in range(3):
            observation = env.observe()
        assert env.physics_step == before
        imageio.imwrite(args.output / "head.png", observation["head"])
        imageio.imwrite(args.output / "wrist.png", observation["wrist"])
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

        def prim_details(path, bounds=True):
            prim = env.stage.GetPrimAtPath(path)
            xf = UsdGeom.Xformable(prim)
            matrix = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            record = dict(
                path=path,
                world_transform=[list(row) for row in matrix],
                local_xform_ops={op.GetOpName(): str(op.Get()) for op in xf.GetOrderedXformOps()},
            )
            if bounds:
                box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                record.update(bounds_min=list(box.GetMin()), bounds_max=list(box.GetMax()), size=list(box.GetSize()))
            return record

        camera_prim = env.stage.GetPrimAtPath("/World/envs/env_0/Head")
        lens = {
            name: camera_prim.GetAttribute(name).Get()
            for name in ("focalLength", "horizontalAperture", "verticalAperture", "projection", "clippingRange")
        }
        lens = {k: str(v) if k == "clippingRange" else v for k, v in lens.items()}
        render_settings = carb.settings.get_settings()
        report = dict(
            source_commit=commit,
            runtime=runtime_metadata(),
            visual_variant=args.visual_variant,
            reference_layout=str(reference),
            reference_belt=original,
            settings=settings,
            robot_count=1,
            actual_qpos=observation["qpos"].tolist(),
            physics_steps_before_capture=before,
            physics_steps_after_capture=env.physics_step,
            head=prim_details("/World/envs/env_0/Head", bounds=False),
            lens=lens,
            intrinsic_matrix=tensor(env.head.data.intrinsic_matrices)[0].cpu().numpy().tolist(),
            conveyor=prim_details("/World/envs/env_0/OriginalConveyor"),
            belt=prim_details("/World/envs/env_0/OriginalConveyor/ConveyorTrack/Belt"),
            ground_visibility=UsdGeom.Imageable(env.stage.GetPrimAtPath("/World/Ground")).ComputeVisibility(),
            render_settings={
                key: render_settings.get(key)
                for key in (
                    "/app/renderer/waitIdle",
                    "/app/hydraEngine/waitIdle",
                    "/rtx/rendermode",
                    "/rtx/post/aa/op",
                    "/rtx/post/tonemap/op",
                    "/rtx/post/histogram/enabled",
                )
            },
            asset_audit=env.asset_audit,
            scope=("Single-arm render ablation; same fixtures/camera; no grasp outcome or full official replay"),
        )
        (args.output / "audit.json").write_text(json.dumps(report, indent=2))
        return finish(args, 0)
    except Exception as exc:
        (args.output / "error.json").write_text(json.dumps(exception_details(exc), indent=2))
        traceback.print_exc()
        return finish(args, 1)
    finally:
        if env is not None:
            env.close()
        launcher.app.close()


if __name__ == "__main__":
    raise SystemExit(main())
