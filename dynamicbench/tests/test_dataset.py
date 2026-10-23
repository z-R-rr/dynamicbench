import numpy as np
import pytest

from dynamicbench.data.dataset_reader import read_episode, validate
from dynamicbench.data.recorder import EpisodeRecorder
from dynamicbench.envs.poses import encode_eef10


def fixture(tmp_path):
    recorder = EpisodeRecorder()

    def obs(k):
        return dict(
            eef10=encode_eef10([0, 0, 1], np.eye(3), 1),
            qpos=np.zeros(8),
            qvel=np.zeros(8),
            head=np.full((8, 12, 3), k, dtype=np.uint8),
            sim_time=k / 30,
            control_tick=k,
            physics_step=k * 10,
            oracle_debug=dict(
                cube_pose=np.r_[0, 0, 0.8, 1, 0, 0, 0], cube_velocity=np.zeros(3), finger_contact_forces=np.zeros(2)
            ),
        )

    for k in range(3):
        recorder.append(
            dict(observation=obs(k), next_observation=obs(k + 1), action=obs(k)["eef10"], joint_target=np.ones(8)),
            {"k": k},
        )
    meta = dict(
        success=False, failure_reason="synthetic_unit_fixture", seed=1, belt_speed=0.03, source_commit="unit-test"
    )
    recorder.save(tmp_path, "episode_test", meta, video=False)
    return tmp_path / "episode_test"


def test_full_round_trip_alignment_and_atomic_directory(tmp_path):
    path = fixture(tmp_path)
    a, m, s = read_episode(path)
    assert s["actions"] == 3 and s["observations"] == 4
    assert a["observation.images.head"][-1, 0, 0, 0] == 3
    assert m["wrist_status"] == "not_available"
    assert "cube_pose" not in a
    assert not list(tmp_path.glob(".episode-*"))
    assert (tmp_path / "manifest.jsonl").is_file()


@pytest.mark.parametrize("damage", ["time", "shape", "terminal", "missing_image", "future_shift", "joint", "nan"])
def test_reject_corrupt_data(tmp_path, damage):
    a, m, _ = read_episode(fixture(tmp_path))
    if damage == "time":
        a["timestamp.sim"][1] += 0.01
    if damage == "shape":
        a["action.eef10"] = np.zeros((3, 9))
    if damage == "terminal":
        a["done"][0] = True
    if damage == "missing_image":
        del a["observation.images.head"]
    if damage == "future_shift":
        a["physics_step"] += 10
    if damage == "joint":
        a["action.joint_target"] = np.zeros((3, 7))
    if damage == "nan":
        a["observation.state.qpos"][1, 0] = np.nan
    with pytest.raises((ValueError, KeyError)):
        validate(a, m)


def test_success_requires_physical_evidence(tmp_path):
    a, m, _ = read_episode(fixture(tmp_path))
    m.update(
        success=True, failure_reason=None, success_hold_ticks=2, cube_size=0.035, belt_top=0.75, success_height=0.05
    )
    with pytest.raises(ValueError, match="lift evidence"):
        validate(a, m)
