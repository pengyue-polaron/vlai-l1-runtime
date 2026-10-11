from __future__ import annotations

import csv
import ctypes
import hashlib
import json
import socket
import subprocess
from dataclasses import asdict, replace
from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import MOTOR_NAMES, ConfigError, load_system_config
from vlai_l1_runtime.feedback import MotorFeedback, RawMotorFrame
from vlai_l1_runtime.feedback_decode import decode_follower_feedback, redecode_capture

CONFIG = Path(__file__).resolve().parents[1] / "configs/system/vlai_l1.toml"


@pytest.fixture(autouse=True)
def forbid_hardware(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("offline decode attempted hardware/process access")

    libc_loader = ctypes.CDLL

    def only_process_libc(name, *args, **kwargs):
        if name is not None:
            forbidden()
        return libc_loader(name, *args, **kwargs)

    monkeypatch.setattr(ctypes, "CDLL", only_process_libc)
    for module, name in ((socket, "socket"), (subprocess, "Popen")):
        monkeypatch.setattr(module, name, forbidden)


def snapshot():
    # Recorded right J3 payload from 2026-10-09; other frames use midpoint fields.
    frames = tuple(
        RawMotorFrame(
            name,
            17 + i,
            100 + i,
            "03800d7fe7fd1c1a" if i == 2 else f"{i + 1:02x}80008008001e1f",
            True,
        )
        for i, name in enumerate(MOTOR_NAMES)
    )
    return MotorFeedback(MOTOR_NAMES, 120, (0.0,) * 8, (0.0,) * 8, (0.0,) * 8, frames)


def test_reported_range_decoding_preserves_raw_and_sdk_and_exposes_status():
    config = load_system_config(CONFIG)
    sdk = snapshot()
    decoded = decode_follower_feedback(config, "right", sdk)
    assert decoded.feedback.raw_frames is sdk.raw_frames
    assert sdk.velocity_rad_s == (0.0,) * 8  # SDK data never used as a decoder input.
    assert decoded.feedback.position_rad[2] == 0.005149920005351305
    assert decoded.feedback.velocity_rad_s[2] == -0.007326007355004549
    assert decoded.feedback.torque_nm[2] == -0.03418803587555885
    assert decoded.status_nibbles == (0,) * 8
    assert decoded.at_protocol_endpoint == (False,) * 8
    assert config.follower_feedback.for_side("left")[3].velocity_max_rad_s == 10
    assert decoded.feedback.velocity_rad_s[3] == pytest.approx(10 / 4095)
    assert decoded.feedback.velocity_rad_s[4] == pytest.approx(30 / 4095)


def test_fault_status_and_endpoint_are_preserved_not_treated_as_healthy():
    sdk = snapshot()
    raw = replace(sdk.raw_frames[0], payload_hex="8100000000001e1f")
    sdk = replace(sdk, raw_frames=(raw, *sdk.raw_frames[1:]))
    decoded = decode_follower_feedback(load_system_config(CONFIG), "left", sdk)
    assert decoded.status_nibbles[0] == 8
    assert decoded.at_protocol_endpoint[0] is True
    assert decoded.feedback.position_rad[0] == -12.5
    assert decoded.feedback.velocity_rad_s[0] == -45
    assert decoded.feedback.torque_nm[0] == -54


@pytest.mark.parametrize(
    "change",
    [
        {"response_id": 27},
        {"payload_hex": "0280008008001e1f"},
        {"can_fd": False},
    ],
)
def test_raw_identity_must_match_config(change):
    sdk = snapshot()
    sdk = replace(sdk, raw_frames=(replace(sdk.raw_frames[0], **change), *sdk.raw_frames[1:]))
    with pytest.raises(ValueError, match="identity/format"):
        decode_follower_feedback(load_system_config(CONFIG), "right", sdk)


@pytest.mark.parametrize(
    ("old", "new", "match"),
    [
        ("velocity_max_rad_s = 10.0", "velocity_max_rad_s = nan", "finite"),
        ("torque_max_nm = 54.0", "torque_max_nm = 0.0", "positive"),
        ("position_max_rad = 12.5", "position_max_rad = 1e100", "float32"),
        ("hardware_version = 0", "hardware_version = -1", "allowed range"),
        (
            '[[follower_feedback.left]]\nname = "joint_1"',
            '[[follower_feedback.left]]\nname = "joint_2"',
            "eight-motor order",
        ),
        ('parameter_sha256 = "', 'parameter_sha256 = "bad', "SHA256"),
    ],
)
def test_invalid_range_evidence_is_rejected(tmp_path, old, new, match):
    path = tmp_path / "system.toml"
    text = CONFIG.read_text()
    assert old in text
    path.write_text(text.replace(old, new, 1))
    with pytest.raises(ConfigError, match=match):
        load_system_config(path)


def test_optional_evidence_has_no_decoder_fallback_and_is_fingerprinted(tmp_path):
    path = tmp_path / "system.toml"
    path.write_text(CONFIG.read_text().split("[follower_feedback]")[0])
    config = load_system_config(path)
    assert config.follower_feedback is None
    with pytest.raises(ValueError, match="range evidence"):
        decode_follower_feedback(config, "right", snapshot())
    config = load_system_config(CONFIG)
    ranges = config.follower_feedback
    altered = replace(ranges.right[2], velocity_max_rad_s=8)
    config = replace(
        config,
        follower_feedback=replace(ranges, right=(*ranges.right[:2], altered, *ranges.right[3:])),
    )
    with pytest.raises(ValueError, match="validated config"):
        decode_follower_feedback(config, "right", snapshot())


def saved_capture(tmp_path):
    config = load_system_config(CONFIG)
    source = tmp_path / "source"
    source.mkdir()
    report = {
        "schema_version": 3,
        "status": "passed",
        "config_sha256": config.follower_feedback.source_config_sha256,
        "endpoints": [
            {
                "side": "right",
                "interface": "can2",
                "parentdev": "1-3.1.2:1.0",
                "completed_rounds": 1,
                "bound_violations": [],
            }
        ],
    }
    record = {
        "side": "right",
        "role": "follower",
        "interface": "can2",
        "round": 1,
        "feedback": asdict(snapshot()),
    }
    (source / "report.json").write_text(json.dumps(report))
    (source / "raw_feedback.jsonl").write_text(json.dumps(record) + "\n")
    return config, source


def test_replay_writes_separate_comparison_and_provenance(tmp_path, monkeypatch):
    config, source = saved_capture(tmp_path)
    raw_before = (source / "raw_feedback.jsonl").read_bytes()
    output = tmp_path / "decoded"
    monkeypatch.chdir(tmp_path)
    report_path = redecode_capture(config, Path("source"), output)
    report = json.loads(report_path.read_text())
    assert report["status"] == "passed"
    assert report["ft_validated"] is False
    assert report["source_sha256"]["raw_feedback.jsonl"] == hashlib.sha256(raw_before).hexdigest()
    assert (source / "raw_feedback.jsonl").read_bytes() == raw_before
    with (output / "decoded.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 8
    assert float(rows[2]["reported_velocity_rad_s"]) == -0.007326007355004549
    assert float(rows[2]["sdk_velocity_rad_s"]) == 0
    with pytest.raises(FileExistsError):
        redecode_capture(config, source, output)


@pytest.mark.parametrize("damage", ["truncated", "wrong_config", "wrong_parent", "repeated_time"])
def test_bad_capture_rejected_before_output_creation(tmp_path, damage):
    config, source = saved_capture(tmp_path)
    path = source / "report.json"
    report = json.loads(path.read_text())
    if damage == "truncated":
        report["endpoints"][0]["completed_rounds"] = 2
    elif damage == "wrong_config":
        report["config_sha256"] = "f" * 64
    elif damage == "wrong_parent":
        report["endpoints"][0]["parentdev"] = "unverified"
    else:
        raw_path = source / "raw_feedback.jsonl"
        first = raw_path.read_text()
        second = json.loads(first)
        second["round"] = 2
        raw_path.write_text(first + json.dumps(second) + "\n")
        report["endpoints"][0]["completed_rounds"] = 2
    path.write_text(json.dumps(report))
    output = tmp_path / "decoded"
    with pytest.raises(ValueError, match=r"incomplete|configuration|identity|non-monotonic"):
        redecode_capture(config, source, output)
    assert not output.exists()
