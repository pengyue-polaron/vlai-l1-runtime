from __future__ import annotations

import threading
import time
from dataclasses import replace
from pathlib import Path

import pytest

from vlai_l1_runtime import (
    CameraContractError,
    CameraFrameMetadata,
    CameraSetValidator,
    load_system_config,
)
from vlai_l1_runtime.camera_bridge import (
    CameraCapture,
    V4L2CameraSet,
    check_v4l2_cameras,
)
from vlai_l1_runtime.configuration import CameraConfig, CamerasConfig, ConfigError

ROOT = Path(__file__).resolve().parents[1]


class _Image:
    shape = (480, 640, 3)
    dtype = "uint8"


class _Reader:
    def __init__(self, sequence: int, *, capture_barrier: threading.Barrier | None = None) -> None:
        self.sequence = sequence
        self.capture_count = 0
        self.closed = False
        self.capture_barrier = capture_barrier
        self._latest: CameraCapture | None = None

    def capture(self, *, timeout_s: float) -> CameraCapture:
        assert timeout_s > 0
        self.capture_count += 1
        if self.capture_barrier is not None and self.capture_count > 1:
            self.capture_barrier.wait(timeout_s)
        self.sequence += 1
        self._latest = CameraCapture(self.sequence, time.monotonic_ns(), _Image())
        return self._latest

    def latest(self) -> CameraCapture | None:
        return self._latest

    def close(self) -> None:
        self.closed = True


class _Backend:
    def __init__(
        self,
        *,
        fail_role: str | None = None,
        capture_barrier: threading.Barrier | None = None,
    ) -> None:
        self.fail_role = fail_role
        self.capture_barrier = capture_barrier
        self.readers: list[_Reader] = []

    def open(self, stream: CameraConfig) -> _Reader:
        if stream.role == self.fail_role:
            raise RuntimeError("camera open failed")
        reader = _Reader(
            len(self.readers) + 1,
            capture_barrier=self.capture_barrier,
        )
        self.readers.append(reader)
        return reader


class _ScriptedReader:
    def __init__(self, timestamps_ns: list[int]) -> None:
        self._timestamps_ns = iter(timestamps_ns)
        self.capture_count = 0
        self.closed = False
        self._latest: CameraCapture | None = None

    def capture(self, *, timeout_s: float) -> CameraCapture:
        assert timeout_s > 0
        self.capture_count += 1
        self._latest = CameraCapture(
            self.capture_count,
            next(self._timestamps_ns),
            _Image(),
        )
        return self._latest

    def latest(self) -> CameraCapture | None:
        return self._latest

    def close(self) -> None:
        self.closed = True


class _ScriptedBackend:
    def __init__(self, timestamps_by_role: dict[str, list[int]]) -> None:
        self._timestamps_by_role = timestamps_by_role
        self.readers: dict[str, _ScriptedReader] = {}

    def open(self, stream: CameraConfig) -> _ScriptedReader:
        reader = _ScriptedReader(self._timestamps_by_role[stream.role])
        self.readers[stream.role] = reader
        return reader


def _commissioned() -> CamerasConfig:
    return CamerasConfig(
        max_age_s=0.5,
        max_pair_skew_s=0.05,
        startup_timeout_s=2.0,
        streams=(
            CameraConfig("wrist_left", True, True, 640, 480, 30, "v4l2", "left-by-id", 4),
            CameraConfig("wrist_right", True, True, 640, 480, 30, "v4l2", "right-by-id", 4),
            CameraConfig("agent", False, False, 640, 480, 30, "unassigned", None),
        ),
    )


def _frames(
    *,
    epoch: str = "boot-a",
    left_sequence: int = 10,
    right_sequence: int = 20,
    left_ns: int = 1_000_000_000,
    right_ns: int = 1_010_000_000,
):
    return {
        "wrist_left": CameraFrameMetadata(
            "wrist_left", "left-by-id", epoch, left_sequence, left_ns
        ),
        "wrist_right": CameraFrameMetadata(
            "wrist_right", "right-by-id", epoch, right_sequence, right_ns
        ),
    }


def test_tracked_camera_mapping_is_commissioned() -> None:
    system = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    CameraSetValidator(system.cameras)
    assert {
        stream.role: (stream.device_id, stream.video_index)
        for stream in system.cameras.streams
        if stream.enabled
    } == {
        "wrist_left": ("255323074436", 4),
        "wrist_right": ("255323074499", 4),
        "agent": ("251643060089", 0),
    }


def test_camera_set_validates_freshness_skew_and_identity() -> None:
    validator = CameraSetValidator(_commissioned())
    current = validator.validate(_frames(), now_ns=1_020_000_000)
    assert current["wrist_left"].source_sequence == 10
    assert current["wrist_right"].source_sequence == 20

    with pytest.raises(CameraContractError, match="stale"):
        CameraSetValidator(_commissioned()).validate(_frames(), now_ns=1_600_000_000)
    with pytest.raises(CameraContractError, match="skew"):
        CameraSetValidator(_commissioned()).validate(
            _frames(right_ns=1_060_000_000),
            now_ns=1_070_000_000,
        )
    wrong_device = _frames()
    wrong_device["wrist_left"] = CameraFrameMetadata(
        "wrist_left", "other-device", "boot-a", 10, 1_000_000_000
    )
    with pytest.raises(CameraContractError, match="does not match stream"):
        CameraSetValidator(_commissioned()).validate(
            wrong_device,
            now_ns=1_020_000_000,
        )


def test_camera_continuity_is_stateful_and_reset_is_explicit() -> None:
    validator = CameraSetValidator(_commissioned())
    validator.validate(_frames(), now_ns=1_020_000_000)

    with pytest.raises(CameraContractError, match="sequence did not increase"):
        validator.validate(_frames(), now_ns=1_020_000_000)
    with pytest.raises(CameraContractError, match="timestamp did not increase"):
        validator.validate(
            _frames(
                left_sequence=11,
                right_sequence=21,
                left_ns=999_000_000,
                right_ns=1_009_000_000,
            ),
            now_ns=1_020_000_000,
        )
    with pytest.raises(CameraContractError, match="without explicit reset"):
        validator.validate(
            _frames(
                epoch="boot-b",
                left_sequence=0,
                right_sequence=0,
                left_ns=1_020_000_000,
                right_ns=1_020_000_000,
            ),
            now_ns=1_020_000_000,
        )

    with pytest.raises(CameraContractError, match="must differ from the active epoch"):
        validator.reset(restarted_epochs={"wrist_left": "boot-a"})

    validator.reset(restarted_epochs={"wrist_left": "boot-b", "wrist_right": "boot-b"})
    with pytest.raises(CameraContractError, match="declared restart epoch"):
        validator.validate(
            _frames(
                epoch="boot-a",
                left_sequence=0,
                right_sequence=0,
                left_ns=1_020_000_000,
                right_ns=1_020_000_000,
            ),
            now_ns=1_020_000_000,
        )
    reset = validator.validate(
        _frames(
            epoch="boot-b",
            left_sequence=0,
            right_sequence=0,
            left_ns=1_020_000_000,
            right_ns=1_020_000_000,
        ),
        now_ns=1_020_000_000,
    )
    assert reset["wrist_left"].stream_epoch == "boot-b"


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("missing", None),
        ("source_sequence", True),
        ("source_sequence", float("nan")),
        ("monotonic_ns", float("inf")),
    ],
)
def test_camera_validator_revalidates_external_metadata(field: str, value: object) -> None:
    validator = CameraSetValidator(_commissioned())
    validator.validate(_frames(), now_ns=1_020_000_000)
    candidate = CameraFrameMetadata("wrist_left", "left-by-id", "boot-a", 11, 1_020_000_000)
    if field == "missing":
        object.__delattr__(candidate, "monotonic_ns")
    else:
        object.__setattr__(candidate, field, value)
    frames = _frames(
        left_sequence=11,
        right_sequence=21,
        left_ns=1_020_000_000,
        right_ns=1_020_000_000,
    )
    frames["wrist_left"] = candidate

    with pytest.raises(CameraContractError):
        validator.validate(frames, now_ns=1_020_000_000)

    accepted = validator.validate(
        _frames(
            left_sequence=11,
            right_sequence=21,
            left_ns=1_020_000_000,
            right_ns=1_020_000_000,
        ),
        now_ns=1_020_000_000,
    )
    assert accepted["wrist_left"].source_sequence == 11


def test_camera_set_requires_every_role_exactly_once() -> None:
    with pytest.raises(CameraContractError, match="every enabled role"):
        CameraSetValidator(_commissioned()).validate(
            {"wrist_left": CameraFrameMetadata("wrist_left", "left-by-id", "boot-a", 1, 1)},
            now_ns=1,
        )


def test_commissioned_camera_devices_must_be_distinct() -> None:
    with pytest.raises(ConfigError, match="unique device identities"):
        CamerasConfig(
            max_age_s=0.5,
            max_pair_skew_s=0.05,
            startup_timeout_s=2.0,
            streams=(
                CameraConfig("wrist_left", True, True, 640, 480, 30, "v4l2", "same-device", 4),
                CameraConfig("wrist_right", True, True, 640, 480, 30, "v4l2", "same-device", 4),
                CameraConfig("agent", False, False, 640, 480, 30, "unassigned", None),
            ),
        )


def test_v4l2_bridge_warms_and_owns_enabled_cameras_and_unwinds_partial_startup() -> None:
    system = replace(
        load_system_config(ROOT / "configs/system/vlai_l1.toml"),
        cameras=_commissioned(),
    )
    backend = _Backend()
    with V4L2CameraSet(system, backend=backend) as cameras:
        samples = cameras.capture(timeout_s=0.1)
        assert tuple(samples) == ("wrist_left", "wrist_right")
        assert samples["wrist_left"].metadata.device_id == "left-by-id"
        assert all(reader.capture_count == 2 for reader in backend.readers)
        latest = cameras.latest()
        assert latest["wrist_left"].metadata.source_sequence == (
            samples["wrist_left"].metadata.source_sequence
        )
        assert all(reader.capture_count == 2 for reader in backend.readers)
    assert all(reader.closed for reader in backend.readers)

    failing = _Backend(fail_role="wrist_right")
    with (
        pytest.raises(RuntimeError, match="camera open failed"),
        V4L2CameraSet(system, backend=failing),
    ):
        pass
    assert failing.readers[0].closed is True


def test_v4l2_bridge_requests_each_camera_frame_concurrently() -> None:
    system = replace(
        load_system_config(ROOT / "configs/system/vlai_l1.toml"),
        cameras=_commissioned(),
    )
    backend = _Backend(capture_barrier=threading.Barrier(2))

    with V4L2CameraSet(system, backend=backend) as cameras:
        assert tuple(cameras.capture(timeout_s=0.1)) == (
            "wrist_left",
            "wrist_right",
        )


def test_v4l2_bridge_advances_lagging_frames_to_form_a_synchronized_set() -> None:
    cameras = CamerasConfig(
        max_age_s=0.5,
        max_pair_skew_s=0.05,
        startup_timeout_s=2.0,
        streams=(
            CameraConfig("wrist_left", True, True, 640, 480, 30, "v4l2", "left-by-id", 4),
            CameraConfig("wrist_right", True, True, 640, 480, 30, "v4l2", "right-by-id", 4),
            CameraConfig("agent", False, True, 640, 480, 30, "v4l2", "agent-by-id", 0),
        ),
    )
    system = replace(
        load_system_config(ROOT / "configs/system/vlai_l1.toml"),
        cameras=cameras,
    )
    backend = _ScriptedBackend(
        {
            "wrist_left": [1, 1_000_000_000, 1_066_000_000],
            "wrist_right": [2, 1_010_000_000, 1_070_000_000],
            "agent": [3, 1_060_291_000],
        }
    )

    with V4L2CameraSet(system, backend=backend) as camera_set:
        samples = camera_set.capture(timeout_s=0.1)

    timestamps = [sample.metadata.monotonic_ns for sample in samples.values()]
    assert max(timestamps) - min(timestamps) == 9_709_000
    assert backend.readers["wrist_left"].capture_count == 3
    assert backend.readers["wrist_right"].capture_count == 3
    assert backend.readers["agent"].capture_count == 2


def test_camera_health_check_validates_a_finite_sample_window() -> None:
    system = replace(
        load_system_config(ROOT / "configs/system/vlai_l1.toml"),
        cameras=_commissioned(),
    )
    backend = _Backend()

    report = check_v4l2_cameras(
        system,
        sample_count=3,
        timeout_s=0.1,
        backend=backend,
    )

    assert report.sample_count == 3
    assert report.elapsed_s > 0
    assert report.effective_fps > 0
    assert report.max_pair_skew_ms < 50
    assert report.streams["wrist_left"].device_id == "left-by-id"
    assert report.streams["wrist_left"].configured_fps == 30
    assert report.streams["wrist_left"].last_sequence > report.streams["wrist_left"].first_sequence
    assert report.streams["wrist_right"].shape == (480, 640, 3)
    assert all(reader.closed for reader in backend.readers)
