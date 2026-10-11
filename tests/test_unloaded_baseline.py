from __future__ import annotations

import csv
import io
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from test_static_wrench import estimator as estimator
from test_static_wrench import forbid_hardware as forbid_hardware
from test_static_wrench import snapshot

from vlai_l1_runtime.configuration import ConfigError, _read_local_regular_file
from vlai_l1_runtime.feedback_decode import decode_follower_feedback
from vlai_l1_runtime.unloaded_baseline import (
    FIELDS,
    analyze,
    digest,
    evaluate_baseline,
    load_policy,
    parse_capture,
    read_artifact,
)

ROOT = Path(__file__).resolve().parents[1]


def policy():
    value = json.loads((ROOT / "configs/diagnostics/right_unloaded.json").read_text())
    value.update(
        fit_seconds=[0.2, 0.6],
        validation_seconds=[0.8, 1.2],
        minimum_fit_samples=3,
        minimum_validation_samples=3,
    )
    return value


def batches(estimator, *, move_at=None, force_after=False):
    items = []
    for i in range(14):
        kwargs = {"q": 0.31} if i == move_at else {}
        if force_after and i >= 8:
            kwargs["force"] = (2.4, -0.8, 3.3, 0.11, 0.18, -0.09)
        raw = snapshot(estimator, i, **kwargs)
        items.append(decode_follower_feedback(estimator.config, "right", raw).feedback)
    return items


def test_unloaded_and_injected_delta_use_disjoint_windows(estimator):
    report, rows, _ = evaluate_baseline(estimator, batches(estimator, force_after=True), policy())
    assert report["status"] == "passed"
    assert report["ft_validated"] is False
    assert report["fit_samples"] == 4
    assert report["validation_samples"] == 5
    for row in rows:
        assert row["seconds"] >= 0.8
        expected = np.array([0.4, 0.2, 0.3, 0.01, -0.02, 0.01])
        assert np.all(
            np.abs(np.array(row["delta_wrench"]) - expected)
            <= np.array(row["propagated_error_bound"]) + 1e-12
        )


def test_motion_between_windows_latches_even_after_return_to_pose(estimator):
    report, rows, _ = evaluate_baseline(estimator, batches(estimator, move_at=7), policy())
    assert report["status"] == "rejected"
    assert report["validation_candidates"] == 0
    assert "pose envelope" in report["baseline_invalidated_reason"]
    assert all(row["delta_wrench"] is None for row in rows)


def test_unchanged_unloaded_signal_has_zero_delta(estimator):
    report, rows, _ = evaluate_baseline(estimator, batches(estimator), policy())
    assert report["status"] == "passed"
    assert np.max(np.abs(report["delta_wrench_mean"])) < 1e-12
    assert all(row["ft_validated"] is False for row in rows)


def test_existing_output_is_refused_before_reading_any_input(tmp_path):
    marker = tmp_path / "keep.txt"
    marker.write_text("preserve me")
    missing = tmp_path / "does_not_exist"
    with pytest.raises(FileExistsError, match="already exists"):
        analyze(missing, missing, missing, missing, tmp_path)
    assert marker.read_text() == "preserve me"


def test_declared_loaded_capture_cannot_be_tared_as_unloaded(tmp_path):
    (tmp_path / "raw_feedback.csv").write_text("not needed for this check")
    (tmp_path / "report.json").write_text(json.dumps({"external_load_present": True}))
    missing = tmp_path / "missing"
    with pytest.raises(ValueError, match="cannot establish an unloaded baseline"):
        analyze(
            ROOT / "configs/system/vlai_l1.toml", tmp_path, missing, missing, tmp_path / "result"
        )
    assert not (tmp_path / "result").exists()


def test_gripper_motion_invalidates_tare(estimator):
    samples = batches(estimator)
    samples[7] = replace(samples[7], position_rad=(*samples[7].position_rad[:7], 0.4))
    report, rows, _ = evaluate_baseline(estimator, samples, policy())
    assert report["validation_candidates"] == 0
    assert all(row["delta_wrench"] is None for row in rows)


def test_moving_fit_and_truncated_validation_fail(estimator):
    with pytest.raises(ValueError, match="fully stationary"):
        evaluate_baseline(estimator, batches(estimator, move_at=4), policy())


def test_truncated_validation_fails(estimator):
    with pytest.raises(ValueError, match="coverage"):
        evaluate_baseline(estimator, batches(estimator)[:10], policy())


def test_policy_rejects_overlap_and_cannot_override_runtime_freshness(estimator):
    value = policy()
    value["validation_seconds"] = [0.5, 1.2]
    with pytest.raises(ValueError, match="disjoint"):
        load_policy(value, estimator.config)
    value = policy()
    value["static"]["maximum_pair_age_ns"] = 999999999
    with pytest.raises(ValueError, match="Runtime"):
        load_policy(value, estimator.config)
    resolved = load_policy(policy(), estimator.config)
    assert resolved.maximum_pair_age_ns == int(estimator.config.teleoperation.state_timeout_s * 1e9)


def raw_capture(estimator, *, corrupt=None):
    buffer = io.StringIO()
    endpoint = next(
        e for e in estimator.config.can.endpoints if e.side == "right" and e.role == "follower"
    )
    buffer.write(
        f"# interface={endpoint.interface}\n# status=passed\n# diagnostic_only=true\n# error=\n"
    )
    writer = csv.DictWriter(buffer, fieldnames=FIELDS)
    writer.writeheader()
    for i, frame in enumerate(snapshot(estimator, 0).raw_frames):
        ns = frame.host_receive_monotonic_ns
        row = dict(
            zip(
                FIELDS,
                (i, ns, ns + 1000, ns, frame.response_id, 1, frame.payload_hex, 0),
                strict=True,
            )
        )
        if i == 2 and corrupt:
            row.update(corrupt)
        writer.writerow(row)
    data = buffer.getvalue().encode()
    capture = {
        "status": "passed",
        "side": "right",
        "role": "follower",
        "interface": endpoint.interface,
        "parentdev": endpoint.parentdev,
        "raw_sha256": digest(data),
        "config_sha256": digest(read_artifact(estimator.config.path)),
    }
    return capture, data


@pytest.mark.parametrize(
    "corrupt",
    [
        {"can_id": 999},
        {"sequence": 0},
        {"rx_overflow_count": 1},
        {"kernel_receive_realtime_ns": 0},
        {"can_fd": 0},
        {"payload_hex": "ff" * 8},
    ],
)
def test_raw_capture_integrity_is_fail_closed(estimator, corrupt):
    capture, data = raw_capture(estimator, corrupt=corrupt)
    with pytest.raises(ValueError, match=r"invalid raw|raw slave"):
        parse_capture(estimator.config, capture, data)


def test_capture_digest_and_identity_are_bound_to_config(estimator):
    capture, data = raw_capture(estimator)
    samples, metadata = parse_capture(estimator.config, capture, data)
    assert len(samples) == 1
    assert metadata["raw_frames"] == 8
    for key in ("raw_sha256", "config_sha256", "parentdev", "side"):
        with pytest.raises(ValueError, match="mismatch"):
            parse_capture(estimator.config, capture | {key: "wrong"}, data)


def test_latest_capture_declares_eviction_and_keeps_raw_integrity_checks(estimator):
    capture, data = raw_capture(estimator)
    data = data.replace(b"# error=\n", b"# error=\n# retained_latest_evicted=123\n")
    capture.update(raw_sha256=digest(data), feedback_retention="latest", evicted_raw_frames=123)
    samples, metadata = parse_capture(estimator.config, capture, data)
    assert len(samples) == 1
    assert metadata["retention"] == "latest"
    assert metadata["evicted_raw_frames"] == 123
    with pytest.raises(ValueError, match="mismatch"):
        parse_capture(estimator.config, capture | {"evicted_raw_frames": 122}, data)
    with pytest.raises(ValueError, match="CSV fields"):
        parse_capture(estimator.config, capture | {"feedback_retention": "full"}, data)
    bad_capture, bad = raw_capture(estimator, corrupt={"rx_overflow_count": 1})
    bad = bad.replace(b"# error=\n", b"# error=\n# retained_latest_evicted=123\n")
    bad_capture.update(raw_sha256=digest(bad), feedback_retention="latest", evicted_raw_frames=123)
    with pytest.raises(ValueError, match="overflow"):
        parse_capture(estimator.config, bad_capture, bad)


@pytest.mark.parametrize("value", ["-1", "nan", "", "١٢٣"])
def test_latest_capture_rejects_invalid_eviction_metadata(estimator, value):
    capture, data = raw_capture(estimator)
    data = data.replace(b"# error=\n", f"# error=\n# retained_latest_evicted={value}\n".encode())
    capture.update(raw_sha256=digest(data), feedback_retention="latest", evicted_raw_frames=0)
    with pytest.raises(ValueError, match="eviction count"):
        parse_capture(estimator.config, capture, data)


def test_large_artifact_reader_preserves_config_limit_and_refuses_links(tmp_path):
    path = tmp_path / "raw.csv"
    path.write_bytes(b"a" * 1_100_000)
    with pytest.raises(ConfigError, match="exceeds"):
        _read_local_regular_file(path)
    assert len(read_artifact(path)) == 1_100_000
    link = tmp_path / "link"
    link.symlink_to(path)
    with pytest.raises(ConfigError, match="symbolic"):
        read_artifact(link)
