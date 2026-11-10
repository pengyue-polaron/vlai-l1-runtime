from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from vlai_l1_runtime.cameras import CameraFrameMetadata
from vlai_l1_runtime.collection.schema import CameraSample
from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.contracts import NamedJointVector, SampleMetadata
from vlai_l1_runtime.pi0_inference import (
    FEATURES,
    Pi0Observation,
    Pi0ObservationBuilder,
    Pi0PredictionClient,
    Pi0Spec,
    pack_message,
    unpack_message,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")
SPEC = Pi0Spec.load(ROOT / "configs/inference/pi0_right.json")
NS = 1_000_000_000


def state(*, sequence=0, source=NS):
    return NamedJointVector(
        dict.fromkeys(FEATURES, 0.0), SampleMetadata(sequence, source), FEATURES
    )


def images(*, sequence=0, source=NS):
    result = {}
    for camera in CONFIG.cameras.streams:
        if camera.enabled:
            result[camera.role] = CameraSample(
                CameraFrameMetadata(camera.role, camera.device_id, "test", sequence, source),
                np.full((camera.height, camera.width, 3), (10, 20, 30), dtype=np.uint8),
            )
    return result


def builder():
    pytest.importorskip("PIL")
    return Pi0ObservationBuilder(CONFIG, SPEC, task="将方块向前推动20厘米", distance_cm=20)


def test_real_contract_preserves_degrees_rgb_and_capture_time():
    result = builder().build(state(), images(source=NS - 1_000_000), now_ns=NS + 10_000_000)
    assert result.oldest_source_ns == NS - 1_000_000
    assert result.payload["position_unit"] == "degree"
    assert result.payload["joint_names"] == list(FEATURES)
    assert result.payload["state"].dtype == np.float32
    assert result.payload["image"].shape == (224, 224, 3)
    assert tuple(result.payload["image"][112, 112]) == (10, 20, 30)
    assert tuple(result.payload["image"][0, 0]) == (0, 0, 0)
    assert result.payload["wrist_image"].shape == (224, 224, 3)
    assert result.payload["prompt"] == "将方块向前推动20厘米"
    assert result.payload["target_distance_cm"] == 20


@pytest.mark.parametrize("delta", [-1, round(CONFIG.teleoperation.state_timeout_s * 1e9) + 1])
def test_stale_or_future_state_is_rejected(delta):
    with pytest.raises(ValueError, match="future or stale"):
        builder().build(state(source=NS - delta), images(), now_ns=NS)


def test_source_skew_is_not_hidden_by_request_timestamp():
    with pytest.raises(ValueError, match="skew"):
        builder().build(state(), images(source=NS - 60_000_000), now_ns=NS)


def test_camera_physical_identity_must_match():
    camera = images()
    sample = camera["agent"]
    camera["agent"] = CameraSample(replace(sample.metadata, device_id="other"), sample.image)
    with pytest.raises(ValueError, match="physical identity"):
        builder().build(state(), camera, now_ns=NS)


def test_camera_restart_cannot_reuse_old_history():
    b = builder()
    b.build(state(), images(), now_ns=NS)
    camera = images(sequence=1, source=NS + 1)
    sample = camera["agent"]
    camera["agent"] = CameraSample(replace(sample.metadata, stream_epoch="restarted"), sample.image)
    with pytest.raises(ValueError, match="restarted"):
        b.build(state(sequence=1, source=NS + 1), camera, now_ns=NS + 1)


def test_replayed_joint_sample_is_rejected():
    b = builder()
    b.build(state(), images(), now_ns=NS)
    with pytest.raises(ValueError, match="did not increase"):
        b.build(state(), images(sequence=1, source=NS + 1), now_ns=NS + 1)


def test_failed_frame_validation_does_not_consume_joint_sequence():
    b = builder()
    with pytest.raises(ValueError, match="missing camera"):
        b.build(state(), {}, now_ns=NS)
    b.build(state(), images(), now_ns=NS)


def test_position_outside_bounds_is_not_clamped():
    q = dict(state().values)
    q[FEATURES[1]] = 181
    bad = NamedJointVector(q, SampleMetadata(0, NS), FEATURES)
    with pytest.raises(ValueError, match="outside tracked bounds"):
        builder().build(bad, images(), now_ns=NS)


@pytest.mark.parametrize(
    "endpoint",
    ["ws://10.0.0.1:8808", "wss://127.0.0.1:8808", "ws://user@127.0.0.1:8808", "ws://127.0.0.1"],
)
def test_only_explicit_loopback_endpoint_is_accepted(endpoint):
    with pytest.raises(ValueError, match="loopback"):
        replace(SPEC, endpoint=endpoint)


@pytest.mark.parametrize("distance", [True, 0, float("nan"), -1])
def test_invalid_distance_is_rejected(distance):
    with pytest.raises(ValueError, match="distance"):
        Pi0ObservationBuilder(CONFIG, SPEC, task="push", distance_cm=distance)


def test_wire_format_matches_native_openpi_numpy_schema():
    msgpack = pytest.importorskip("msgpack")
    x = np.arange(256, dtype=np.float32).reshape(32, 8)
    raw = pack_message({"actions": x, "scalar": np.float32(1.5)})
    native = msgpack.unpackb(raw, strict_map_key=False)
    assert native["actions"][b"__ndarray__"] is True
    assert native["actions"][b"dtype"] == x.dtype.str
    assert native["actions"][b"data"] == x.tobytes()
    assert np.array_equal(unpack_message(raw)["actions"], x)
    assert unpack_message(raw)["scalar"] == 1.5


def test_object_array_and_malformed_buffer_are_rejected():
    msgpack = pytest.importorskip("msgpack")
    with pytest.raises(ValueError, match="numeric"):
        pack_message(np.array([object()], dtype=object))
    bad = msgpack.packb({b"__ndarray__": True, b"dtype": "<f4", b"data": b"x", b"shape": [32, 8]})
    with pytest.raises(ValueError, match="buffer size"):
        unpack_message(bad)


METADATA = {
    "kind": "pi0",
    "native_joint_degrees": True,
    "prediction_only": True,
    "fps": 30,
    "prediction_horizon": 32,
}


class FakeSocket:
    def __init__(self, metadata, responses):
        self.incoming = [pack_message(metadata), *responses]
        self.sent = []
        self.closed = False

    def recv(self, **kwargs):
        item = self.incoming.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def send(self, item):
        self.sent.append(item)

    def close(self):
        self.closed = True


def connect_fake(monkeypatch, metadata=METADATA, responses=()):
    pytest.importorskip("msgpack")
    sync_client = pytest.importorskip("websockets.sync.client")
    connection = FakeSocket(metadata, list(responses))
    monkeypatch.setattr(sync_client, "connect", lambda *a, **k: connection)
    return connection


@pytest.mark.parametrize(
    "override",
    [
        {"kind": "unified"},
        {"transport_only": True},
        {"native_joint_degrees": False},
        {"fps": True},
        {"prediction_horizon": 16},
    ],
)
def test_incompatible_server_is_closed_before_any_observation(monkeypatch, override):
    connection = connect_fake(monkeypatch, METADATA | override)
    with pytest.raises(ValueError, match="pinned native pi0"), Pi0PredictionClient(SPEC):
        pytest.fail("incompatible handshake accepted")
    assert connection.closed
    assert not connection.sent


def test_two_predictions_reuse_one_connection(monkeypatch):
    chunk = np.zeros((32, 8), dtype=np.float32)
    raw = pack_message({"actions": chunk})
    connection = connect_fake(monkeypatch, responses=[raw, raw])
    obs = Pi0Observation({"state": np.zeros(8, dtype=np.float32)}, NS, 0)
    with Pi0PredictionClient(SPEC) as client:
        assert np.array_equal(client.predict(obs)[0], chunk)
        assert np.array_equal(client.predict(obs)[0], chunk)
    assert connection.closed
    assert len(connection.sent) == 2


@pytest.mark.parametrize(
    "response",
    [
        TimeoutError("late"),
        "server error",
        {"actions": [[0.0] * 8] * 31},
        {"actions": np.full((32, 8), np.nan)},
    ],
)
def test_ambiguous_or_invalid_response_closes_session(monkeypatch, response):
    if isinstance(response, dict):
        response = pack_message(response)
    connection = connect_fake(monkeypatch, responses=[response])
    with Pi0PredictionClient(SPEC) as client:
        with pytest.raises((ValueError, TimeoutError)):
            client.predict(Pi0Observation({}, NS, 0))
        with pytest.raises(RuntimeError, match="not open"):
            client.predict(Pi0Observation({}, NS, 0))
    assert connection.closed


def test_extra_configuration_keys_are_rejected(tmp_path):
    config = json.loads((ROOT / "configs/inference/pi0_right.json").read_text())
    config["allow_motor_commands"] = True
    path = tmp_path / "inference.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="exact"):
        Pi0Spec.load(path)
