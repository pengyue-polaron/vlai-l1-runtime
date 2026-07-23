from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from vlai_l1_runtime import (
    FEATURE_NAMES,
    CommandEnvelope,
    CommandSession,
    NamedJointVector,
    SampleMetadata,
    SessionError,
    SessionMode,
    load_system_config,
)
from vlai_l1_runtime.session import _SessionPolicy

ROOT = Path(__file__).resolve().parents[1]
BASE_CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")


def _pose(value: float = 0.0) -> dict[str, float]:
    return dict.fromkeys(FEATURE_NAMES, value)


def _joint_1_offset() -> dict[str, float]:
    pose = _pose()
    pose["left_joint_1.pos"] = 1.0
    return pose


def _session(*, ready: bool = True) -> CommandSession:
    return CommandSession._for_hardware_free_test(
        _SessionPolicy(
            command_ready=ready,
            liveness_timeout_ns=1_000,
            command_inactivity_timeout_ns=100,
            first_command_hold_tolerance_deg=0.1,
        )
    )


def _connected_session() -> CommandSession:
    session = _session()
    session.connect_read_only(now_ns=0)
    session.acquire_command(
        lease_id="lease-a",
        measured=NamedJointVector(_pose(), SampleMetadata(0, 9)),
        now_ns=10,
    )
    return session


def _first_feedback(value: float = 0.0, *, sequence: int = 1, now_ns: int = 11):
    return NamedJointVector(_pose(value), SampleMetadata(sequence, now_ns))


def test_unready_policy_cannot_acquire_a_command_lease() -> None:
    session = CommandSession(BASE_CONFIG)
    session.connect_read_only(now_ns=0)
    with pytest.raises(SessionError, match="readiness"):
        session.acquire_command(
            lease_id="lease-a",
            measured=NamedJointVector(_pose(), SampleMetadata(0, 0)),
            now_ns=0,
        )
    assert session.mode is SessionMode.READ_ONLY


def test_modified_system_config_cannot_construct_a_session() -> None:
    with pytest.raises(ValueError, match="modified or fabricated"):
        CommandSession(replace(BASE_CONFIG, robot_id="forged"))


def test_command_acquisition_requires_fresh_measured_feedback() -> None:
    session = _session()
    session.connect_read_only(now_ns=0)
    with pytest.raises(SessionError, match="too old"):
        session.acquire_command(
            lease_id="lease-a",
            measured=NamedJointVector(_pose(), SampleMetadata(0, 0)),
            now_ns=101,
        )
    assert session.mode is SessionMode.READ_ONLY


def test_heartbeat_does_not_refresh_command_activity() -> None:
    session = _connected_session()
    session.accept(
        CommandEnvelope("lease-a", 0, 11, _pose()),
        now_ns=11,
        feedback=_first_feedback(),
    )
    session.heartbeat(lease_id="lease-a", now_ns=100)

    assert session.check_timeouts(now_ns=112) is True
    assert session.mode is SessionMode.READ_ONLY
    assert session.release_reason == "command_inactivity_timeout"


def test_liveness_timeout_releases_the_command_lease() -> None:
    session = _connected_session()
    session.accept(
        CommandEnvelope("lease-a", 0, 11, _pose()),
        now_ns=11,
        feedback=_first_feedback(),
    )

    assert session.check_timeouts(now_ns=1_011) is True
    assert session.release_reason == "liveness_timeout"
    assert session.lease_id is None


def test_hold_then_contiguous_command_and_requested_release() -> None:
    session = _connected_session()
    session.accept(
        CommandEnvelope("lease-a", 0, 11, _pose()),
        now_ns=11,
        feedback=_first_feedback(),
    )
    session.accept(
        CommandEnvelope("lease-a", 1, 12, _pose()),
        now_ns=12,
        feedback=_first_feedback(sequence=2, now_ns=12),
    )
    assert session.mode is SessionMode.COMMAND

    session.release(lease_id="lease-a")
    assert session.mode is SessionMode.READ_ONLY
    assert session.release_reason == "requested"


@pytest.mark.parametrize(
    ("command", "match"),
    [
        (CommandEnvelope("lease-a", 0, 11, _joint_1_offset()), "hold measured"),
        (CommandEnvelope("lease-a", 1, 11, _pose()), "contiguous"),
    ],
)
def test_invalid_first_command_fails_closed(command: CommandEnvelope, match: str) -> None:
    session = _connected_session()
    with pytest.raises(SessionError, match=match):
        session.accept(command, now_ns=11, feedback=_first_feedback())
    assert session.mode is SessionMode.READ_ONLY
    assert session.lease_id is None


def test_lease_mismatch_fails_closed() -> None:
    session = _connected_session()
    with pytest.raises(SessionError, match="does not match"):
        session.release(lease_id="lease-b")
    assert session.mode is SessionMode.READ_ONLY
    assert session.release_reason == "lease_mismatch"
    assert session.lease_id is None


def test_stale_first_command_fails_closed() -> None:
    session = _connected_session()
    with pytest.raises(SessionError, match="predates"):
        session.accept(
            CommandEnvelope("lease-a", 0, 10, _pose()),
            now_ns=11,
            feedback=_first_feedback(),
        )
    assert session.mode is SessionMode.READ_ONLY


def test_first_command_requires_new_feedback() -> None:
    session = _connected_session()
    with pytest.raises(SessionError, match="NamedJointVector"):
        session.accept(CommandEnvelope("lease-a", 0, 11, _pose()), now_ns=11)
    assert session.mode is SessionMode.READ_ONLY

    session = _connected_session()
    with pytest.raises(SessionError, match="sequence did not increase"):
        session.accept(
            CommandEnvelope("lease-a", 0, 11, _pose()),
            now_ns=11,
            feedback=_first_feedback(sequence=0),
        )
    assert session.mode is SessionMode.READ_ONLY


@pytest.mark.parametrize(
    ("input_kind", "field", "value"),
    [
        ("feedback", "missing", None),
        ("feedback", "source_sequence", True),
        ("feedback", "source_sequence", float("nan")),
        ("feedback", "monotonic_ns", float("inf")),
        ("command", "sequence", True),
        ("command", "monotonic_ns", float("nan")),
    ],
)
def test_corrupted_live_inputs_fail_closed(input_kind: str, field: str, value: object) -> None:
    session = _connected_session()
    command = CommandEnvelope("lease-a", 0, 11, _pose())
    feedback = _first_feedback()
    if input_kind == "command":
        object.__setattr__(command, field, value)
    else:
        metadata = object.__new__(SampleMetadata) if field == "missing" else SampleMetadata(1, 11)
        if field != "missing":
            object.__setattr__(metadata, field, value)
        object.__setattr__(feedback, "metadata", metadata)

    with pytest.raises(SessionError):
        session.accept(command, now_ns=11, feedback=feedback)
    assert session.mode is SessionMode.READ_ONLY
    assert session.lease_id is None


def test_first_feedback_must_postdate_lease_acquisition() -> None:
    session = _connected_session()
    with pytest.raises(SessionError, match="postdate lease acquisition"):
        session.accept(
            CommandEnvelope("lease-a", 0, 11, _pose()),
            now_ns=11,
            feedback=_first_feedback(now_ns=10),
        )
    assert session.mode is SessionMode.READ_ONLY


@pytest.mark.parametrize(
    ("method_name", "kwargs"),
    [("heartbeat", {"lease_id": "lease-a"}), ("check_timeouts", {})],
)
def test_invalid_live_clock_fails_closed(method_name: str, kwargs: dict[str, str]) -> None:
    session = _connected_session()
    method = getattr(session, method_name)
    with pytest.raises(SessionError, match="non-negative integer"):
        method(now_ns=-1, **kwargs)
    assert session.mode is SessionMode.READ_ONLY
    assert session.release_reason == "invalid_session_time"
