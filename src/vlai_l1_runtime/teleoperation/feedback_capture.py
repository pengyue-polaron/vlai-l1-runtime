"""Explicit, bounded Follower feedback diagnostic; never enables motors."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import signal
import subprocess
import time
from dataclasses import asdict
from functools import partial
from pathlib import Path
from typing import Any

from ..configuration import MOTOR_NAMES, SIDES, SystemConfig
from ..feedback import MotorFeedback
from . import lifecycle
from .can_probe import probe_motor_feedback
from .xair import verify_xair_dependency


def _link(interface: str) -> dict[str, Any]:
    result = subprocess.run(
        ("ip", "-j", "-details", "-statistics", "link", "show", "dev", interface),
        check=True,
        capture_output=True,
        text=True,
    )
    entries = json.loads(result.stdout)
    if len(entries) != 1 or entries[0].get("ifname") != interface:
        raise RuntimeError(f"cannot inspect {interface}")
    return entries[0]


def _require_healthy(interface: str, baseline: dict[str, Any]) -> None:
    current = _link(interface)
    health = lifecycle._decode_can_health(interface, json.dumps([current]))
    if not health.healthy:
        raise RuntimeError(health.detail())
    if current["parentdev"] != baseline["parentdev"]:
        raise RuntimeError(f"{interface}: physical identity changed")
    if current["linkinfo"]["info_xstats"] != baseline["linkinfo"]["info_xstats"]:
        raise RuntimeError(f"{interface}: cumulative CAN health counters changed")
    for direction in ("rx", "tx"):
        for counter in ("errors", "dropped"):
            if current["stats64"][direction][counter] != baseline["stats64"][direction][counter]:
                raise RuntimeError(f"{interface}: {direction} {counter} changed")
    if lifecycle._qdisc_drops(interface) != 0:
        raise RuntimeError(f"{interface}: nonzero qdisc drops")


def _down(interface: str) -> None:
    subprocess.run(("ip", "link", "set", interface, "down"), check=True)
    if lifecycle._read_link_is_up(interface):
        raise RuntimeError(f"{interface}: did not return DOWN")


def _require_services_inactive(config: SystemConfig) -> None:
    for service in (config.lifecycle.left_teleop_service, config.lifecycle.right_teleop_service):
        result = subprocess.run(("systemctl", "is-active", service), capture_output=True, text=True)
        if result.returncode not in (3, 4) or result.stdout.strip() not in (
            "inactive",
            "failed",
            "unknown",
        ):
            raise RuntimeError(f"cannot establish inactive controller service: {service}")


def capture_follower_feedback(
    config: SystemConfig,
    *,
    sides: tuple[str, ...],
    output: Path,
    parameter_motor: str | None = None,
) -> Path:
    """One configured probe window per selected Follower, then close owned links.

    This is a live query operation. It must only be called after operator
    authorization. Existing output directories and active controllers are refused.
    No retry, enable, disable, reset or teleoperation constructor is used.
    The optional single-motor parameter stage uses only allowlisted read queries
    after successful state feedback and SDK-handle release. It needs separate
    operator authorization from a previous state-only diagnostic.
    """

    if not sides or len(set(sides)) != len(sides) or any(side not in SIDES for side in sides):
        raise ValueError("select distinct left/right Follower sides")
    if parameter_motor is not None and (parameter_motor not in MOTOR_NAMES or len(sides) != 1):
        raise ValueError("parameter reads require exactly one Follower side and one named motor")
    if os.geteuid() != 0:
        raise RuntimeError("Follower feedback capture requires root")
    dependency = verify_xair_dependency(config)
    endpoints = tuple(
        next(e for e in config.can.endpoints if e.side == side and e.role == "follower")
        for side in sides
    )
    output.mkdir(parents=True, exist_ok=False)
    report: dict[str, Any] = {
        "schema_version": 3 if parameter_motor is not None else 2,
        "status": "failed",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "config_sha256": hashlib.sha256(config.path.read_bytes()).hexdigest(),
        "sdk_revision": dependency.revision,
        "can_library_sha256": dependency.can_library_sha256,
        "timestamp_kind": "host_sdk_cache_read_completion_monotonic_ns",
        "raw_timestamp_kind": "host_socket_dequeue_monotonic_ns_not_hardware_time",
        "coordinate_kind": "raw_motor_coordinates_unverified",
        "torque_kind": "sdk_estimate_nm_unvalidated",
        "ft_validated": False,
        "motion_authorized": False,
        "endpoints": [],
    }
    owned: list[str] = []
    lock = None
    handlers = {}

    def interrupted(signum: int, _frame: object) -> None:
        raise InterruptedError(f"Follower feedback capture interrupted by signal {signum}")

    try:
        lock = lifecycle._acquire_lifecycle_lock(config, exclusive=True)
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, interrupted)
        interfaces = tuple(endpoint.interface for endpoint in config.can.endpoints)
        lifecycle._require_exclusive_control(interfaces)
        _require_services_inactive(config)
        lifecycle._require_interfaces_down(interfaces)
        for endpoint in config.can.endpoints:
            lifecycle._require_can_identity(config, endpoint.interface)
        with (
            (output / "feedback.csv").open("x", newline="", encoding="utf-8") as stream,
            (output / "raw_feedback.jsonl").open("x", encoding="utf-8") as raw_stream,
        ):
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "side",
                    "role",
                    "interface",
                    "round",
                    "host_monotonic_ns",
                    "motor",
                    "position_rad",
                    "velocity_rad_s",
                    "torque_nm",
                )
            )
            for endpoint in endpoints:
                interface = endpoint.interface
                baseline = _link(interface)
                health = lifecycle._decode_can_health(interface, json.dumps([baseline]))
                if health.state != "STOPPED" or health.txerr or health.rxerr:
                    raise RuntimeError(health.detail())
                entry: dict[str, Any] = {
                    "side": endpoint.side,
                    "interface": interface,
                    "parentdev": endpoint.parentdev,
                    "before": baseline,
                    "completed_rounds": 0,
                    "bound_violations": [],
                }
                report["endpoints"].append(entry)
                owned.append(interface)  # Also clean up partially failed configuration.
                lifecycle._configure_can(config, interface)
                _require_healthy(interface, baseline)

                def on_feedback(
                    round_index: int,
                    feedback: MotorFeedback,
                    *,
                    endpoint=endpoint,
                    interface=interface,
                    entry=entry,
                ) -> None:
                    if feedback.raw_frames is None:
                        raise ValueError(
                            "capture requires complete raw feedback paired with SDK data"
                        )
                    if tuple(frame.response_id for frame in feedback.raw_frames) != tuple(
                        motor.receive_id for motor in config.motors
                    ):
                        raise ValueError("raw feedback IDs differ from configured motors")
                    limits = config.teleoperation.joint_safety.for_side(endpoint.side)
                    for index, name in enumerate(feedback.names):
                        writer.writerow(
                            (
                                endpoint.side,
                                endpoint.role,
                                interface,
                                round_index,
                                feedback.monotonic_ns,
                                name,
                                feedback.position_rad[index],
                                feedback.velocity_rad_s[index],
                                feedback.torque_nm[index],
                            )
                        )
                        if index < len(limits.min_deg):
                            degrees = math.degrees(feedback.position_rad[index])
                            if (
                                not limits.min_deg[index] <= degrees <= limits.max_deg[index]
                                and name not in entry["bound_violations"]
                            ):
                                entry["bound_violations"].append(name)
                    raw_stream.write(
                        json.dumps(
                            {
                                "side": endpoint.side,
                                "role": endpoint.role,
                                "interface": interface,
                                "round": round_index,
                                "feedback": asdict(feedback),
                            }
                        )
                        + "\n"
                    )
                    raw_stream.flush()
                    entry["completed_rounds"] = round_index
                    stream.flush()

                result = probe_motor_feedback(
                    config,
                    interface,
                    dependency.can_library,
                    on_feedback=on_feedback,
                    check_health=partial(_require_healthy, interface, baseline),
                )
                entry["probe"] = asdict(result)
                if parameter_motor is not None:
                    from .parameter_probe import read_motor_parameters

                    motor = next(m for m in config.motors if m.name == parameter_motor)
                    entry["parameter_reads"] = []
                    entry["parameter_probe_budget_s"] = config.teleoperation.motor_probe_duration_s
                    entry["parameter_reply_window_s"] = 1 / config.teleoperation.motor_probe_rate_hz
                    with (output / "parameters.jsonl").open("x", encoding="utf-8") as params:

                        def on_reading(reading: dict[str, Any], *, entry=entry) -> None:
                            params.write(json.dumps(reading) + "\n")
                            params.flush()
                            entry["parameter_reads"].append(reading)

                        read_motor_parameters(
                            config,
                            interface,
                            motor,
                            check_health=partial(_require_healthy, interface, baseline),
                            on_reading=on_reading,
                        )
                _require_healthy(interface, baseline)
                entry["after"] = _link(interface)
                entry["qdisc_drops"] = lifecycle._qdisc_drops(interface)
                _down(interface)
                owned.remove(interface)
        report["status"] = "passed"
    except BaseException as exc:
        report["error"] = str(exc)
        raise
    finally:
        cleanup_errors = []
        # One interrupt is enough. Keep cleanup deterministic on a second signal.
        for signum in handlers:
            signal.signal(signum, signal.SIG_IGN)
        for interface in owned:
            try:
                _down(interface)
            except Exception as exc:
                cleanup_errors.append(f"{interface}: {exc}")
        if cleanup_errors:
            report["status"] = "failed"
            report["cleanup_errors"] = cleanup_errors
        try:
            report["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            (output / "report.json").write_text(
                json.dumps(report, indent=2) + "\n", encoding="utf-8"
            )
        finally:
            if lock is not None:
                lock.close()
            for signum, handler in handlers.items():
                signal.signal(signum, handler)
        if cleanup_errors:
            raise RuntimeError("Follower feedback cleanup failed: " + "; ".join(cleanup_errors))
    return output / "report.json"
