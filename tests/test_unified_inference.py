import json
from pathlib import Path

import numpy as np
import pytest

from vlai_l1_runtime.collection.ft import FTSample
from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.contracts import NamedJointVector, SampleMetadata
from vlai_l1_runtime.pi0_inference import Pi0PredictionClient, Pi0Spec
from vlai_l1_runtime.task_definition import FEATURES, load_task
from vlai_l1_runtime.unified_inference import EpisodeHistory, UnifiedFeatures

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")


def features():
    return UnifiedFeatures(ROOT / "configs/inference/unified")


@pytest.mark.parametrize(
    "case", json.loads((ROOT / "tests/fixtures/unified_features.json").read_text())
)
def test_features_match_training_observations(case):
    encoder = features()
    previous, actual = None, []
    for row in case["short_state"]:
        value, previous = encoder.encode(row[:8], row[8:], previous)
        actual.append(value)
    np.testing.assert_allclose(actual[1:], case["expected_features"], atol=5e-5, rtol=1e-4)


def sample(i, stamp=None, force_stamp=None):
    stamp = 1_000_000_000 + round(i * 1e9 / 30) if stamp is None else stamp
    state = NamedJointVector(dict.fromkeys(FEATURES, 0.0), SampleMetadata(i, stamp), FEATURES)
    force = FTSample((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), force_stamp or stamp, i)
    return state, force


def test_history_padding_causality_and_reset():
    history = EpisodeHistory(features())
    for i in range(12):
        state, force = sample(i)
        history.append(state, force, now_ns=state.metadata.monotonic_ns)
    state, payload, _ = history.snapshot(sample(5)[0].metadata.monotonic_ns)
    assert state.metadata.source_sequence == 5
    assert payload["interaction_mask"].sum() == 6
    assert not payload["interaction_history"][6:].any()
    assert not payload["interaction_dt"][6:].any()
    assert payload["short_state"].shape == (10, 14)
    np.testing.assert_array_equal(payload["short_state"][0], payload["short_state"][4])
    np.testing.assert_array_equal(payload["force_history_mask"], np.ones(10, bool))
    np.testing.assert_array_equal(payload["interaction_history"][0, 16:], np.zeros(6))
    assert EpisodeHistory(features()).rows == []


@pytest.mark.parametrize(
    "fault", ["stale_joint", "stale_force", "skew", "regression", "gap", "capacity"]
)
def test_bad_history_is_rejected(fault):
    history = EpisodeHistory(features())
    state, force = sample(0)
    history.append(state, force, now_ns=1_000_000_000)
    state, force = sample(1)
    now = state.metadata.monotonic_ns
    if fault == "stale_joint":
        now += 100_000_001
    elif fault == "stale_force":
        force = FTSample(force.values, now - 100_000_001, 1)
    elif fault == "skew":
        force = FTSample(force.values, now - 60_000_000, 1)
    elif fault == "regression":
        state, force = sample(0)
    elif fault == "gap":
        state, force = sample(1, stamp=1_200_000_000)
        now = state.metadata.monotonic_ns
    elif fault == "capacity":
        history.rows *= 2048
    with pytest.raises(ValueError, match=r"fresh|skew|regressed|capacity"):
        history.append(state, force, now_ns=now)


def test_unified_manifest_reuses_saved_start_and_requires_matching_spec():
    pi0 = load_task(CONFIG, ROOT / "configs/tasks/push-distance.json")
    unified = load_task(CONFIG, ROOT / "configs/tasks/push-distance-unified.json")
    assert pi0.start_pose == unified.start_pose
    assert pi0.pose == unified.pose
    assert unified.expected_checkpoint["kind"] == "unified"
    assert Pi0Spec.load(unified.inference_spec, kind="unified").endpoint.endswith(":8809")
    with pytest.raises(ValueError, match="unsupported inference"):
        Pi0Spec.load(unified.inference_spec)


def test_unified_client_rejects_pi0_and_accepts_unified(monkeypatch):
    from test_pi0_inference import METADATA, SPEC, connect_fake

    connection = connect_fake(monkeypatch, METADATA)
    with (
        pytest.raises(ValueError, match="pinned native unified"),
        Pi0PredictionClient(SPEC, expected_kind="unified"),
    ):
        pytest.fail("pi0 accepted as Unified")
    assert connection.closed
    assert not connection.sent
    connection = connect_fake(monkeypatch, METADATA | {"kind": "unified"})
    with Pi0PredictionClient(SPEC, expected_kind="unified"):
        pass
    assert connection.closed


def test_live_history_worker_uses_force_source_and_closes(monkeypatch):
    import time

    from vlai_l1_runtime import unified_inference as unified

    closed = []

    class Source:
        def __init__(self, config):
            pass

        def __enter__(self):
            return self

        def wait_first(self, **kwargs):
            pass

        def closest(self, *, target_monotonic_ns, timeout_s):
            return FTSample((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), target_monotonic_ns - 1_000_000, 1)

        def __exit__(self, *args):
            closed.append(True)

    monkeypatch.setattr(unified, "HKVLCollectionSource", Source)
    history = unified.UnifiedHistory(CONFIG, lambda stamp: sample(stamp, stamp=stamp)[0]).start()
    try:
        state, payload, force_stamp = history.snapshot(time.monotonic_ns())
        assert payload["interaction_mask"].sum() >= 2
        assert force_stamp < state.metadata.monotonic_ns
        np.testing.assert_array_equal(payload["short_state"][-1, 8:], [1, 2, 3, 4, 5, 6])
    finally:
        history.close()
    assert closed == [True]
    assert not history._thread.is_alive()


def test_predictor_unified_payload_and_episode_reset(monkeypatch):
    from types import SimpleNamespace

    from vlai_l1_runtime import camera_ipc, task_runtime, unified_inference
    from vlai_l1_runtime.pi0_inference import Pi0Observation

    task = load_task(CONFIG, ROOT / "configs/tasks/push-distance-unified.json")
    histories, sent, stopped = [], [], []

    class History:
        def __init__(self, *_):
            histories.append(self)

        def start(self):
            return self

        def snapshot(self, stamp):
            return None, {"short_state": np.ones((10, 14))}, 99

        def stop(self):
            stopped.append(self)

        def close(self):
            self.stop()

    class Client:
        def __init__(self, spec, *, expected_kind):
            assert expected_kind == "unified"
            self.server_metadata = {
                "checkpoint": task.expected_checkpoint,
                "history_delivery": "full_causal_prefix",
            }

        def __enter__(self):
            return self

        def predict(self, observation):
            sent.append(observation)
            return np.zeros((32, 8)), b""

        def close(self):
            pass

    class Cameras:
        def __init__(self, *_):
            pass

        def __enter__(self):
            return self

        def capture(self, **_):
            return {
                name: SimpleNamespace(metadata=SimpleNamespace(monotonic_ns=100))
                for name in ("agent", "wrist_right")
            }

        def __exit__(self, *_):
            pass

    monkeypatch.setattr(unified_inference, "UnifiedHistory", History)
    monkeypatch.setattr(task_runtime, "Pi0PredictionClient", Client)
    monkeypatch.setattr(camera_ipc, "RawCameraBridgeClient", Cameras)
    monkeypatch.setattr(
        task_runtime,
        "Pi0ObservationBuilder",
        lambda *a, **k: SimpleNamespace(
            build=lambda *a, **k: Pi0Observation({"target_distance_cm": 10}, 100, 1)
        ),
    )
    predictor = task_runtime.LivePredictor(CONFIG, task, lambda _: None)
    try:
        predictor.begin_episode()
        _, stamp = predictor()
        assert stamp == 99
        assert sent[-1].payload["short_state"].shape == (10, 14)
        predictor()
        assert len(histories) == 1
        predictor.end_episode()
        assert histories[0] in stopped
        predictor.begin_episode()
        predictor()
        assert len(histories) == 2
    finally:
        predictor.close()
