#!/usr/bin/env python3
"""Differential queue test against an actual pinned LIBERO-Dynamic checkout."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dynamicbench.envs.poses import encode_eef10
from dynamicbench.runtime.libero_scheduler import REFERENCE_COMMIT, LiberoScheduler


def main():
    p = argparse.ArgumentParser()
    p.add_argument("reference", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=args.reference, text=True).strip()
    if sha != REFERENCE_COMMIT:
        raise ValueError(f"Reference must be pinned to {REFERENCE_COMMIT}, got {sha}")
    sys.path.insert(0, str(args.reference / "scripts"))
    from realtime_eval.control import ActionRuntime, LatencySampler, ObservationPacket, build_policy_controller

    class Policy:
        def actions(self, k, horizon):
            out = np.tile(encode_eef10([0, 0, 1], np.eye(3), 1), (horizon, 1))
            out[:, 0] = k / 1000 + np.arange(horizon) / 10000
            return out

        def predict(self, obs, k, horizon):
            return self.actions(k, horizon)

        def get_action_chunk(self, obs, language):
            return self.actions(int(obs["robot0_eef_pos"][0]), 32)

    reports = []
    for mode in ("sync_hold", "async_queue", "rtc"):
        for latency in (0, 0.25, 1.2):
            a = ActionRuntime(10, empty_action="hold")
            reference = build_policy_controller(
                policy=Policy(),
                language="test",
                control_freq=30,
                mode="rtc_queue" if mode == "rtc" else mode,
                prefetch_threshold=16,
                action_chunk_size=32,
                chunk_size_threshold=None,
                latency_sampler=LatencySampler(source="fixed", fixed_seconds=latency),
            )
            actual = LiberoScheduler(Policy(), mode=mode, latency=latency)
            try:
                for k in range(120):
                    obs = dict(eef10=encode_eef10([0, 0, 1], np.eye(3), 1), sim_time=k / 30)
                    packet = ObservationPacket(
                        "unit",
                        k,
                        k / 30,
                        "test",
                        {"robot0_eef_pos": np.array([k, 0, 0])},
                        a.queue_len,
                        a.last_action.copy(),
                    )
                    ready = reference.tick(packet, a)
                    expected, _, _, underflow, _, _, remaining = a.next_action()
                    result, info = actual.tick(k, obs)
                    assert underflow == info["underflow"], (mode, latency, k, "underflow")
                    assert remaining == info["queue_after"], (mode, latency, k, "queue")
                    if not underflow:
                        np.testing.assert_allclose(result, expected, atol=1e-7)
                    if mode == "sync_hold":
                        assert info["sync_latency_seconds"] == sum(x.metadata["sim_latency_s"] for x in ready)
                reports.append(dict(mode=mode, latency=latency, ticks=120, passed=True))
            finally:
                actual.close()
                reference.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(reference_commit=sha, cases=reports, passed=True), indent=2))
    print(f"PASS: {len(reports)} cases / 1080 control decisions against {sha}")


if __name__ == "__main__":
    main()
