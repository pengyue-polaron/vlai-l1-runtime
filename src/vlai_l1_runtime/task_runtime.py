"""Resident right-Follower task orchestration; never opens CAN or owns motors.

The original automatic owner handles the real-time loop. This module schedules
bounded targets, consumes its feedback and uses the existing prediction client.
Normal completion returns to the saved task pose and keeps that owner alive.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
from threading import Lock

from .contracts import NamedJointVector, SampleMetadata, feature_names_for_sides
from .pi0_inference import Pi0ObservationBuilder, Pi0PredictionClient, Pi0Spec

FEATURES = feature_names_for_sides(("right",))


def check_target(config, target):
    """Check the actual motor/joint contract; no diagnostic excursion gate."""
    if len(target) != 8:
        raise ValueError("eight joint targets required")
    bounds = config.teleoperation.joint_safety.for_side("right")
    for i, q in enumerate(target):
        lo, hi = (
            (bounds.min_deg[i], bounds.max_deg[i])
            if i < 7
            else (
                -math.degrees(config.follower_feedback.right[i].position_max_rad),
                math.degrees(config.follower_feedback.right[i].position_max_rad),
            )
        )
        if not math.isfinite(q) or not lo <= q <= hi:
            raise ValueError(f"task target outside bounds: {FEATURES[i]}")


def transit_duration(config, policy, origin, target):
    """Use the needed original smooth-move duration, not a fixed 240-second wait."""
    check_target(config, target)
    delta = max(abs(a - b) for a, b in zip(origin, target, strict=True))
    seconds = max(
        1 / config.teleoperation.publish_hz,
        1.875 * delta / policy["maximum_reference_speed_deg_s"],
        math.sqrt(
            (10 * math.sqrt(3) / 3) * delta / policy["maximum_reference_acceleration_deg_s2"]
        ),
    )
    # Round upward to a control tick, keeping native derivative checks identical.
    return (
        math.ceil(seconds * config.teleoperation.publish_hz) + 1
    ) / config.teleoperation.publish_hz


def render_task_spec(config, policy, task):
    from .teleoperation.automatic import render_native_spec

    # Retain motor identities, PD gains and physical feedback protection.
    # Diagnostic waypoint/excursion fields do not constrain the policy stream.
    base = dict(policy)
    base.pop("waypoints_deg", None)
    base["kind"] = "isolated_right_follower_diagnostic_v1"
    base["active_motor"] = "joint_4"
    text = render_native_spec(config, base)
    lines = text.splitlines()
    lines[0] = "VLAI_RIGHT_FOLLOWER_TASK_V2"
    s = task.settings
    lines.append(
        " ".join(
            map(
                str,
                (
                    round(config.teleoperation.state_timeout_s * 1e9),
                    round(s["observation_max_age_s"] * 1e9),
                    round(s["producer_timeout_s"] * 1e9),
                ),
            )
        )
    )
    return "\n".join(lines) + "\n"


class LivePredictor:
    """One worker owns model/camera IPC; it cannot write any motor command."""

    def __init__(self, config, task, history):
        self.config, self.task, self.history = config, task, history
        spec = Pi0Spec.load(task.inference_spec)
        self.spec = replace(
            spec,
            response_timeout_s=min(spec.response_timeout_s, task.settings["observation_max_age_s"]),
        )
        self.client = None
        self.cameras = None
        self.builder = Pi0ObservationBuilder(
            config, spec, task=task.prompt, distance_cm=task.distance_cm
        )

    def __call__(self):
        try:
            return self._predict_once()
        except Exception:
            # No automatic retry. A later explicit policy request may open a
            # fresh session instead of getting stuck on a closed connection.
            self.close()
            raise

    def _predict_once(self):
        from .camera_ipc import RawCameraBridgeClient

        if self.client is None:
            self.client = Pi0PredictionClient(self.spec).__enter__()
            expected = self.task.expected_checkpoint
            actual = self.client.server_metadata.get("checkpoint", {})
            if actual != expected:
                self.client.close()
                self.client = None
                raise ValueError("prediction server checkpoint does not match this task")
            self.cameras = RawCameraBridgeClient(self.config).__enter__()
        images = self.cameras.capture(timeout_s=self.config.cameras.startup_timeout_s)
        stamps = [images[r].metadata.monotonic_ns for r in ("agent", "wrist_right")]
        state = self.history((min(stamps) + max(stamps)) // 2)
        observation = self.builder.build(state, images, now_ns=time.monotonic_ns())
        actions, _raw = self.client.predict(observation)
        return actions.tolist(), observation.oldest_source_ns

    def close(self):
        client, cameras = self.client, self.cameras
        self.client = self.cameras = None
        try:
            if client is not None:
                client.close()
        finally:
            if cameras is not None:
                cameras.__exit__(None, None, None)


class TaskController:
    """Single supervisor thread drives stages; model work runs separately.

    A rejected prediction clears only pending targets and holds after the current
    already-validated segment. Hardware faults still use the native disable path.
    No normal task state, action budget or finish request exits the motor owner.
    """

    def __init__(self, config, task, policy, output: Path, *, predictor=None):
        self.config, self.task, self.policy, self.output = config, task, policy, output
        self.phase = "starting"
        self.error = None
        self.state = None
        self.initial = None
        self.destination = None
        self.sequence = 0
        self.last_source = 0
        self.last_send = 0
        self.last_motion = 0
        self.awaiting_accept = 0
        self.start_sequence = None
        self.return_sequence = None
        self.hold_needed = False
        self.finish_requested = False
        self.shutdown_requested = False
        self.actions_remaining = 0
        self.actions_sent = 0
        self.chunks_received = 0
        self.targets = deque()
        self._history = deque(maxlen=512)
        self._lock = Lock()
        self._future = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="task-prediction")
        self.predictor = predictor or LivePredictor(config, task, self.closest_state)

    def closest_state(self, stamp):
        with self._lock:
            if not self._history:
                raise RuntimeError("no Follower state available")
            return min(self._history, key=lambda s: abs(s.metadata.monotonic_ns - stamp))

    def on_state(self, event):
        if event.get("state") != "task_state":
            return
        if self.state is not None and (
            event["owner_pid"] != self.state["owner_pid"]
            or event["monotonic_ns"] <= self.state["monotonic_ns"]
        ):
            raise ValueError("task owner changed or state timestamp regressed")
        q = event["q_deg"]
        if len(q) != 8 or any(not math.isfinite(v) for v in q):
            raise ValueError("malformed task feedback")
        self.state = event
        sample = NamedJointVector(
            dict(zip(FEATURES, q, strict=True)),
            SampleMetadata(event["monotonic_ns"], event["earliest_motor_response_ns"]),
            FEATURES,
        )
        with self._lock:
            if (
                not self._history
                or sample.metadata.monotonic_ns > self._history[-1].metadata.monotonic_ns
            ):
                self._history.append(sample)
        if self.initial is None:
            self.initial = list(q)
            self.destination = list(event["reference_deg"])
        if self.phase in ("going_to_start", "returning"):
            expected = (
                self.start_sequence if self.phase == "going_to_start" else self.return_sequence
            )
            if event["completed_sequence"] >= expected and event["phase"] == "holding":
                target = self.task.pose["follower_deg"]
                close = all(
                    abs(q[i] - target[name])
                    <= self.task.settings.get(
                        "arrival_tolerance_deg", self.policy["maximum_tracking_error_deg"][i]
                    )
                    for i, name in enumerate(FEATURES)
                )
                if close:
                    self.phase = "ready" if self.phase == "going_to_start" else "returned_hold"

    def request(self, operation, *, actions=None):
        if operation == "shutdown":
            self.shutdown_requested = True
        elif operation == "policy":
            if self.phase not in ("ready", "returned_hold"):
                raise ValueError("policy requires arrival at the task start")
            if actions is not None and (type(actions) is not int or actions < 1):
                raise ValueError("invalid requested policy action count")
            if self._future is not None:
                raise ValueError("previous prediction is still finishing")
            self.actions_remaining, self.actions_sent = actions, 0
            self.finish_requested, self.error = False, None
            self.phase = "policy"
        elif operation == "finish":
            if self.start_sequence is None:
                raise ValueError("no saved start has been accepted by this owner")
            if self.phase not in ("policy", "policy_rejected_hold", "ready", "returned_hold"):
                raise ValueError("finish unavailable in current task stage")
            self.targets.clear()
            self.finish_requested = True
            self.hold_needed = True
            self.phase = "finishing"
        else:
            raise ValueError("operation must be policy, finish or shutdown")

    def reject(self, reason):
        self.error = str(reason)
        self.targets.clear()
        self.hold_needed = True
        self.phase = "policy_rejected_hold"

    def _command(self, op, now_ns, *, duration=0.0, target=None, observation_ns=None):
        self.sequence += 1
        source = max(now_ns, self.last_source + 1)
        self.last_source, self.last_send = source, now_ns
        q = target if target is not None else [0.0] * 8
        observation_ns = observation_ns or self.state["earliest_motor_response_ns"]
        return (
            " ".join(
                map(str, ("VLAI_TASK_V1", self.sequence, source, observation_ns, op, duration, *q))
            )
            + "\n"
        )

    def _motion(self, op, target, duration, now_ns, observation_ns=None):
        check_target(self.config, target)
        line = self._command(
            op,
            now_ns,
            duration=duration,
            target=None if op == "return" else target,
            observation_ns=observation_ns,
        )
        self.last_motion = self.awaiting_accept = self.sequence
        self.destination = list(target)
        return line

    def tick(self, now_ns):
        """Return at most one bounded command; never block on model/camera IPC."""
        if self.state is None:
            return None
        if now_ns - self.state["earliest_motor_response_ns"] > round(
            self.config.teleoperation.state_timeout_s * 1e9
        ):
            raise TimeoutError("task feedback stale")
        if self._future is not None and self._future.done():
            future, self._future = self._future, None
            try:
                chunk, stamp = future.result()
                if self.phase == "policy":
                    if len(chunk) != 32 or any(len(row) != 8 for row in chunk):
                        raise ValueError("policy response must contain 32x8 native joint degrees")
                    if stamp > now_ns or now_ns - stamp > round(
                        self.task.settings["observation_max_age_s"] * 1e9
                    ):
                        raise ValueError("model observation expired")
                    needed = (
                        len(chunk)
                        if self.actions_remaining is None
                        else max(0, self.actions_remaining - len(self.targets))
                    )
                    selected = chunk[:needed]
                    for row in selected:
                        check_target(self.config, row)
                    self.targets.extend((list(row), stamp) for row in selected)
                    self.chunks_received += 1
            except Exception as exc:
                if self.phase == "policy":
                    self.reject(exc)
        if self.phase == "starting":
            target = [self.task.pose["follower_deg"][name] for name in FEATURES]
            try:
                command = self._motion(
                    "start",
                    target,
                    transit_duration(self.config, self.policy, self.destination, target),
                    now_ns,
                )
                self.start_sequence = self.sequence
                self.phase = "going_to_start"
                return command
            except ValueError as exc:
                self.reject(exc)
        if self.hold_needed:
            self.hold_needed = False
            return self._command("hold", now_ns)
        accepted = self.state["accepted_sequence"] >= self.awaiting_accept
        can_queue = accepted and self.state["pending_sequence"] == 0
        if self.phase == "policy":
            if (
                (self.actions_remaining is None or self.actions_remaining > len(self.targets))
                and len(self.targets) <= 16
                and self._future is None
            ):
                self._future = self._executor.submit(self.predictor)
            if can_queue and self.targets:
                row, stamp = self.targets.popleft()
                # Freshness was validated for the complete predicted chunk.
                # Later points are its future plan, not newly captured inputs.
                command = self._motion("target", row, 1 / 30, now_ns, stamp)
                if self.actions_remaining is not None:
                    self.actions_remaining -= 1
                self.actions_sent += 1
                return command
            if self.actions_remaining == 0:
                self.finish_requested = True
                self.phase = "finishing"
        if (
            self.phase == "finishing"
            and accepted
            and self.state["pending_sequence"] == 0
            and self.state["phase"] == "holding"
        ):
            # Return uses the latest known endpoint, never the stale teleop reference.
            self.destination = list(self.state["reference_deg"])
            target = [self.task.pose["follower_deg"][name] for name in FEATURES]
            try:
                command = self._motion(
                    "return",
                    target,
                    transit_duration(self.config, self.policy, self.destination, target),
                    now_ns,
                )
                self.return_sequence = self.sequence
                self.phase = "returning"
                return command
            except ValueError as exc:
                self.reject(exc)
        if now_ns - self.last_send >= min(
            50_000_000, round(self.task.settings["producer_timeout_s"] * 1e9 / 4)
        ):
            return self._command("heartbeat", now_ns)
        return None

    def service_request(self):
        path = self.output / "request.json"
        if not path.exists() and not path.is_symlink():
            return
        request = None
        try:
            if path.is_symlink() or not path.is_file() or path.stat().st_size > 1024:
                raise ValueError("invalid task request file")
            request = json.loads(path.read_text())
            if type(request) is not dict or set(request) - {"operation", "actions"}:
                raise ValueError("invalid task request fields")
            self.request(request["operation"], actions=request.get("actions"))
            result = {"accepted": True, "request": request}
        except (ValueError, KeyError, OSError) as exc:
            result = {"accepted": False, "error": str(exc), "request": request}
        finally:
            if path.is_file() or path.is_symlink():
                path.unlink(missing_ok=True)
        (self.output / "request-result.json").write_text(json.dumps(result) + "\n")

    def status(self):
        return {
            "task": self.task.id,
            "phase": self.phase,
            "error": self.error,
            "owner_pid": None if self.state is None else self.state["owner_pid"],
            "actions_sent": self.actions_sent,
            "actions_remaining": self.actions_remaining,
            "chunks_received": self.chunks_received,
            "start_pose": str(self.task.start_pose),
            "checkpoint": self.task.expected_checkpoint,
            "native": self.state,
        }

    def close(self):
        self._executor.shutdown(wait=False, cancel_futures=True)
        close = getattr(self.predictor, "close", None)
        if close:
            close()
