"""Operator-authorized isolated right-Follower experiment, separate from policy commands.

Validation/rendering is hardware-free. run_automatic() is explicitly motion
capable; the native child alone owns motor handles and sends position commands.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from ..configuration import MOTOR_TYPE_CODES, SystemConfig, _is_loader_validated
from ..trajectory import SweepSpec


def validate_policy(config: SystemConfig, policy: dict) -> None:
    if not isinstance(config, SystemConfig) or not _is_loader_validated(config):
        raise ValueError("loader-validated unmodified system config required")
    expected = {
        "kind",
        "side",
        "role",
        "active_motor",
        "amplitude_deg",
        "transition_seconds",
        "hold_seconds",
        "cycles",
        "hold_test_seconds",
        "maximum_amplitude_deg",
        "maximum_reference_speed_deg_s",
        "maximum_reference_acceleration_deg_s2",
        "maximum_observed_speed_deg_s",
        "maximum_tracking_error_deg",
        "tracking_persistence_seconds",
        "maximum_motor_torque_nm",
        "maximum_temperature_c",
        "maximum_relative_excursion_deg",
        "enabled_raw_status",
        "disabled_raw_status",
        "raw_log_capacity",
        "shutdown_confirmation_seconds",
        "description",
    }
    if type(policy) is not dict or set(policy) != expected:
        raise ValueError("exact automatic policy fields required")
    if (policy["kind"], policy["side"], policy["role"], policy["active_motor"]) != (
        "isolated_right_follower_diagnostic_v1",
        "right",
        "follower",
        "joint_7",
    ):
        raise ValueError("automatic experiment is right Follower J7 only")
    for name in (
        "amplitude_deg",
        "transition_seconds",
        "hold_seconds",
        "hold_test_seconds",
        "maximum_amplitude_deg",
        "maximum_reference_speed_deg_s",
        "maximum_reference_acceleration_deg_s2",
        "tracking_persistence_seconds",
        "shutdown_confirmation_seconds",
    ):
        value = policy[name]
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"positive finite {name} required")
    for name in (
        "maximum_observed_speed_deg_s",
        "maximum_tracking_error_deg",
        "maximum_motor_torque_nm",
        "maximum_relative_excursion_deg",
    ):
        v = policy[name]
        if (
            type(v) is not list
            or len(v) != 8
            or any(type(x) not in (float, int) or not math.isfinite(x) or x <= 0 for x in v)
        ):
            raise ValueError(f"eight positive finite {name} values required")
    for name in ("enabled_raw_status", "disabled_raw_status"):
        if type(policy[name]) is not int or not 0 <= policy[name] <= 15:
            raise ValueError("invalid status nibble")
    if policy["enabled_raw_status"] == policy["disabled_raw_status"]:
        raise ValueError("enable and disable status must differ")
    if (
        type(policy["maximum_temperature_c"]) is not int
        or not 1 <= policy["maximum_temperature_c"] <= 255
    ):
        raise ValueError("invalid temperature bound")
    if (
        type(policy["raw_log_capacity"]) is not int
        or not 8 <= policy["raw_log_capacity"] <= 2_000_000
    ):
        raise ValueError("invalid log capacity")
    if policy["amplitude_deg"] > min(
        policy["maximum_amplitude_deg"], policy["maximum_relative_excursion_deg"][6]
    ):
        raise ValueError("amplitude exceeds experimental envelope")
    SweepSpec(
        policy["amplitude_deg"],
        round(policy["transition_seconds"] * 1e9),
        round(policy["hold_seconds"] * 1e9),
        policy["cycles"],
        policy["maximum_reference_speed_deg_s"],
        policy["maximum_reference_acceleration_deg_s2"],
    )
    duration = max(
        policy["hold_test_seconds"],
        policy["hold_seconds"]
        + 4 * policy["cycles"] * (policy["transition_seconds"] + policy["hold_seconds"]),
    )
    if (duration + 10) * config.teleoperation.publish_hz * 8 > policy["raw_log_capacity"]:
        raise ValueError("insufficient raw log capacity")
    if config.follower_feedback is None or not config.can.fd:
        raise ValueError("tracked FD motor feedback profiles required")
    for limit, motor in zip(
        policy["maximum_motor_torque_nm"], config.follower_feedback.right, strict=True
    ):
        if limit > motor.torque_max_nm:
            raise ValueError("torque budget exceeds firmware range")


def render_native_spec(config: SystemConfig, policy: dict) -> str:
    """Only pure rendering; all identities/gains/ranges/bounds come from config."""
    validate_policy(config, policy)
    endpoint = next(e for e in config.can.endpoints if e.side == "right" and e.role == "follower")
    values = [
        policy["amplitude_deg"],
        policy["transition_seconds"],
        policy["hold_seconds"],
        policy["hold_test_seconds"],
        policy["maximum_amplitude_deg"],
        policy["maximum_reference_speed_deg_s"],
        policy["maximum_reference_acceleration_deg_s2"],
        policy["cycles"],
        config.teleoperation.publish_hz,
        policy["raw_log_capacity"],
        policy["enabled_raw_status"],
        policy["disabled_raw_status"],
        policy["maximum_temperature_c"],
        round(config.teleoperation.state_timeout_s * 1e9),
        round(policy["tracking_persistence_seconds"] * 1e9),
        round(policy["shutdown_confirmation_seconds"] * 1e9),
        round(config.teleoperation.can_health_poll_s * 1000),
        config.teleoperation.rt_priority,
    ]
    lines = ["VLAI_RIGHT_FOLLOWER_DIAGNOSTIC_V1", endpoint.interface, " ".join(map(str, values))]
    bounds = config.teleoperation.joint_safety.for_side("right")
    gains = config.control["follower"]
    for i, (motor, profile) in enumerate(
        zip(config.motors, config.follower_feedback.right, strict=True)
    ):
        lo, hi = (
            (bounds.min_deg[i], bounds.max_deg[i])
            if i < 7
            else (-math.degrees(profile.position_max_rad), math.degrees(profile.position_max_rad))
        )
        lines.append(
            " ".join(
                map(
                    str,
                    (
                        motor.name,
                        motor.send_id,
                        motor.receive_id,
                        motor.send_id,
                        MOTOR_TYPE_CODES[motor.motor_type],
                        profile.position_max_rad,
                        profile.velocity_max_rad_s,
                        profile.torque_max_nm,
                        gains.kp[i],
                        gains.kd[i],
                        lo,
                        hi,
                        policy["maximum_tracking_error_deg"][i],
                        policy["maximum_observed_speed_deg_s"][i],
                        policy["maximum_motor_torque_nm"][i],
                        policy["maximum_relative_excursion_deg"][i],
                    ),
                )
            )
        )
    return "\n".join(lines) + "\n"


def run_automatic(config: SystemConfig, policy_path: Path, output: Path, *, mode: str) -> dict:
    """LIVE MOTOR ENABLE: one bounded hold/sweep, then verified disable/down.

    Never retries enable or trajectory after failure. A killed/unconfirmed child
    may be followed only by a sequential disable-only cleanup owner.
    """
    import os
    import selectors
    import signal
    import subprocess
    import time

    from ..unloaded_baseline import digest, read_artifact
    from . import lifecycle
    from .can_probe import probe_motor_feedback
    from .feedback_capture import _down, _link, _require_healthy, _require_services_inactive
    from .xair import verify_xair_dependency

    policy_bytes = read_artifact(policy_path)
    policy = json.loads(policy_bytes)
    rendered = render_native_spec(config, policy)
    if mode not in ("hold", "sweep"):
        raise ValueError("select hold or sweep")
    if os.geteuid() != 0:
        raise RuntimeError("automatic experiment requires root")
    endpoint = next(e for e in config.can.endpoints if e.side == "right" and e.role == "follower")
    interfaces = tuple(e.interface for e in config.can.endpoints)
    dependency = verify_xair_dependency(config)
    owner_uid, owner_gid = lifecycle._invoking_identity()
    binary = config.path.parents[2] / "build/xair-sidecar/vlai_l1_automatic_follower"
    binary_hash = digest(read_artifact(binary))
    output.mkdir(parents=True, exist_ok=False)
    (output / "native_spec.txt").write_text(rendered)
    report = {
        "kind": "isolated_right_follower_automatic_diagnostic",
        "status": "failed",
        "mode": mode,
        "side": "right",
        "role": "follower",
        "interface": endpoint.interface,
        "parentdev": endpoint.parentdev,
        "config_sha256": digest(read_artifact(config.path)),
        "policy_sha256": digest(policy_bytes),
        "native_sha256": binary_hash,
        "sdk_revision": dependency.revision,
        "ft_validated": False,
        "coordinate_kind": "raw_motor_coordinates",
        "external_load_present": None,
        "feedback_retention": "full",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "disabled_verified": False,
        "events": [],
        "policy": policy,
    }
    lock = None
    configured = False
    stop = False
    old_handlers = {}

    def request_stop(_sig, _frame):
        nonlocal stop
        stop = True

    def supervise(which, raw_name, samples_name, health_baseline):
        command = [
            "chrt",
            "-f",
            str(config.teleoperation.rt_priority),
            str(binary),
            str(output / "native_spec.txt"),
            which,
            str(output / raw_name),
            str(output / samples_name),
            str(os.getpid()),
        ]
        child = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, pass_fds=(lock.fileno(),)
        )
        last_line = time.monotonic()
        startup = True
        termination_at = None
        disabled = False
        failure = None
        buffer = b""
        next_health = time.monotonic()
        try:
            with (
                selectors.DefaultSelector() as selector,
                (output / f"{which}_console.log").open("xb") as console,
            ):
                selector.register(child.stdout, selectors.EVENT_READ)
                while child.poll() is None:
                    now = time.monotonic()
                    timeout = (
                        config.teleoperation.startup_timeout_s
                        if startup
                        else config.safety.session_liveness_timeout_s
                    )
                    if which != "disable" and stop and termination_at is None:
                        failure = "operator_stop"
                    if now - last_line > timeout and termination_at is None:
                        failure = "native_heartbeat_timeout"
                    if now >= next_health and failure is None:
                        try:
                            _require_healthy(endpoint.interface, health_baseline)
                            lifecycle._require_interfaces_down(
                                tuple(i for i in interfaces if i != endpoint.interface)
                            )
                        except BaseException as exc:
                            failure = f"health:{exc}"
                        next_health = now + config.teleoperation.can_health_poll_s
                    if failure and termination_at is None:
                        child.terminate()
                        termination_at = now
                    if (
                        termination_at is not None
                        and now - termination_at > config.teleoperation.shutdown_timeout_s
                    ):
                        child.kill()
                        failure = f"{failure};forced_kill"
                        child.wait()
                        break
                    for key, _mask in selector.select(timeout=0.02):
                        chunk = os.read(key.fd, 65536)
                        if not chunk:
                            selector.unregister(key.fileobj)
                            continue
                        console.write(chunk)
                        console.flush()
                        buffer += chunk
                        while b"\n" in buffer:
                            line, buffer = buffer.split(b"\n", 1)
                            try:
                                event = json.loads(line)
                            except (ValueError, UnicodeDecodeError):
                                continue
                            if not isinstance(event, dict) or "state" not in event:
                                continue
                            last_line = time.monotonic()
                            startup = False
                            if event["state"] == "disabled":
                                disabled = True
                            report["events"].append(event)
                            temporary = output / "status.tmp"
                            temporary.write_text(json.dumps(event))
                            temporary.replace(output / "status.json")
                            if event["state"] != "running":
                                print(json.dumps(event), flush=True)
                        if len(buffer) > 65536:
                            raise RuntimeError("native output line too long")
                # Native disable marker is emitted before final file output and exit.
                remaining = child.stdout.read()
                console.write(remaining)
                for line in (buffer + remaining).splitlines():
                    try:
                        event = json.loads(line)
                    except (ValueError, UnicodeDecodeError):
                        continue
                    if isinstance(event, dict) and "state" in event:
                        report["events"].append(event)
                        if event["state"] == "disabled":
                            disabled = True
            return child.wait(), disabled, failure
        finally:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=config.teleoperation.shutdown_timeout_s)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            child.stdout.close()

    try:
        lock = lifecycle._acquire_lifecycle_lock(config, exclusive=True)
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, request_stop)
        lifecycle._require_exclusive_control(interfaces)
        _require_services_inactive(config)
        lifecycle._require_interfaces_down(interfaces)
        for e in config.can.endpoints:
            lifecycle._require_can_identity(config, e.interface)
        baseline = _link(endpoint.interface)
        info = baseline["linkinfo"]
        if (
            info["info_data"]["state"] != "STOPPED"
            or any(info["info_data"]["berr_counter"].values())
            or any(info["info_xstats"].values())
        ):
            raise RuntimeError("startup CAN fault; no reset/retry")
        report["can_before"] = baseline
        configured = True
        lifecycle._configure_can(config, endpoint.interface)
        _require_healthy(endpoint.interface, baseline)
        probe = probe_motor_feedback(
            config,
            endpoint.interface,
            dependency.can_library,
            check_health=lambda: _require_healthy(endpoint.interface, baseline),
        )
        report["preflight_rounds"] = probe.rounds
        print(
            f"PASS {endpoint.interface}: {probe.rounds} motion-free rounds, every motor responded",
            flush=True,
        )
        if stop:
            raise InterruptedError("stopped before enable")
        native_started = False
        try:
            native_started = True
            code, disabled, failure = supervise(mode, "raw_feedback.csv", "samples.csv", baseline)
            report["native_returncode"] = code
            report["disabled_verified"] = disabled
            if code != 0 or failure or not disabled:
                raise RuntimeError(
                    f"native diagnostic failed: exit={code}, disabled={disabled}, reason={failure}"
                )
            _require_healthy(endpoint.interface, baseline)
            if stop:
                raise InterruptedError("stopped at stage completion")
            report["status"] = "passed"
        finally:
            if native_started and not report["disabled_verified"]:
                # The former process has exited/reaped before this sole cleanup writer starts.
                code, disabled, failure = supervise(
                    "disable", "cleanup_raw.csv", "cleanup_samples.csv", baseline
                )
                report["fallback_cleanup"] = {
                    "returncode": code,
                    "disabled_verified": disabled,
                    "failure": failure,
                }
                report["disabled_verified"] = disabled
                if not disabled:
                    raise RuntimeError(
                        "disable could not be verified; operator intervention required"
                    )
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["status"] = "failed"
    finally:
        try:
            if configured:
                report["can_before_down"] = _link(endpoint.interface)
                _down(endpoint.interface)
            report["can_after"] = _link(endpoint.interface)
        except BaseException as exc:
            report["status"] = "failed"
            report["cleanup_error"] = str(exc)
        finally:
            if lock is not None:
                lock.close()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
        report["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        for name in ("raw_feedback.csv", "samples.csv"):
            if (output / name).exists():
                report["raw_sha256" if name == "raw_feedback.csv" else "samples_sha256"] = digest(
                    read_artifact(output / name)
                )
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        if report["status"] == "passed":
            try:
                from ..automatic_review import review_automatic

                analysis = review_automatic(config, output)
                (output / "analysis.json").write_text(json.dumps(analysis, indent=2) + "\n")
                report["analysis_status"] = "passed"
            except BaseException as exc:
                report["status"] = "failed"
                report["analysis_status"] = "failed"
                report["error"] = f"offline review: {type(exc).__name__}: {exc}"
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        (output / "status.json").write_text(
            json.dumps(
                {
                    "state": "stopped",
                    "status": report["status"],
                    "disabled_verified": report["disabled_verified"],
                }
            )
        )
        for artifact in output.iterdir():
            os.chown(artifact, owner_uid, owner_gid, follow_symlinks=False)
        os.chown(output, owner_uid, owner_gid, follow_symlinks=False)
    print(
        json.dumps(
            {k: report.get(k) for k in ("status", "disabled_verified", "error", "cleanup_error")}
        ),
        flush=True,
    )
    return report


def run_automatic_sequence(config: SystemConfig, policy_path: Path, output: Path) -> dict:
    """LIVE: hold, verify saved evidence, sweep, verify; no failed-stage retry."""
    # Validate before creating an output or acquiring hardware resources.
    validate_policy(config, json.loads(policy_path.read_text()))
    output.mkdir(parents=True, exist_ok=False)
    result = {"kind": "automatic_right_sequence_v1", "status": "failed", "stages": []}
    try:
        for mode in ("hold", "sweep"):
            report = run_automatic(config, policy_path, output / mode, mode=mode)
            result["stages"].append(
                {
                    "mode": mode,
                    "status": report["status"],
                    "disabled_verified": report["disabled_verified"],
                    "analysis_status": report.get("analysis_status"),
                    "error": report.get("error"),
                }
            )
            if (
                report["status"] != "passed"
                or report["disabled_verified"] is not True
                or report.get("analysis_status") != "passed"
            ):
                break
        else:
            result["status"] = "passed"
    except BaseException as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        (output / "sequence.json").write_text(json.dumps(result, indent=2) + "\n")
    return result
