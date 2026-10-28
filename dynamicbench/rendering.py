"""Read RoboDojo render defaults and adapt its pinned Lab 2 quality preset."""

import ast
import hashlib
import tomllib


def resolve_render_settings(root):
    source = root / "env/environment/base_env.py"
    tree = ast.parse(source.read_text())
    definitions = [
        node.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "DEFAULT_RENDER_CONFIG" for target in node.targets)
    ]
    if len(definitions) != 1:
        raise ValueError("Cannot resolve the official DEFAULT_RENDER_CONFIG")
    defaults = ast.literal_eval(definitions[0])
    if defaults["rendering_mode"] != "quality":
        raise ValueError("The pinned conveyor render adapter expects RoboDojo quality mode")
    preset = root / "dynamicbench/configs/robodojo_quality.kit"

    def flatten(mapping, prefix=""):
        result = {}
        for key, value in mapping.items():
            path = prefix + "/" + key
            if isinstance(value, dict):
                result.update(flatten(value, path))
            else:
                result[path] = value
        return result

    return dict(
        defaults=defaults,
        quality_settings=flatten(tomllib.loads(preset.read_text())),
        sources={
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in (source, preset)
        },
    )


def isaac61_global_settings(render):
    from isaaclab_physx.renderers import IsaacRtxRendererGlobalSettingsCfg

    names = (
        "enable_translucency",
        "enable_reflections",
        "enable_global_illumination",
        "antialiasing_mode",
        "enable_dlssg",
        "enable_dl_denoiser",
        "dlss_mode",
        "enable_direct_lighting",
        "samples_per_pixel",
        "enable_shadows",
        "enable_ambient_occlusion",
    )
    return IsaacRtxRendererGlobalSettingsCfg(
        **{key: render["defaults"][key] for key in names if render["defaults"][key] is not None},
        carb_settings=render["quality_settings"],
    )


def audit_render_settings(render):
    import carb.settings

    settings = carb.settings.get_settings()
    checks = {key: dict(expected=value, actual=settings.get(key)) for key, value in render["quality_settings"].items()}
    # DLAA is set through Replicator and corresponds to enum value 4.
    checks["/rtx/post/aa/op"] = dict(expected=4, actual=settings.get("/rtx/post/aa/op"))
    return dict(
        requested_mode=render["defaults"]["rendering_mode"],
        backend_mode=settings.get("/rtx/rendermode"),
        checks=checks,
        passed=all(row["expected"] == row["actual"] for row in checks.values()),
        sources=render["sources"],
        note="Lab 2 quality settings mapped to Lab 3 RTX; equal pixels across engines are not claimed",
    )
