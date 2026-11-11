"""Unified inputs from measured joints and real sensor-frame wrench history.

History has the training definition: a full episode prefix, never a rolling window.
Sampling and FK run outside the motor supervisor and inference request loop.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from threading import Condition, Event, Thread

import numpy as np

from .collection.ft import HKVLCollectionSource
from .kinematics import SerialChain
from .task_definition import FEATURES


class UnifiedFeatures:
    def __init__(self, directory: Path):
        self.settings = json.loads((directory / "features.json").read_text())
        urdf = directory / "v10_follower.urdf"
        if hashlib.sha256(urdf.read_bytes()).hexdigest() != self.settings["urdf_sha256"]:
            raise ValueError("Unified geometry asset changed")
        self.chain = SerialChain.from_urdf(
            urdf,
            root=self.settings["root"],
            tip=self.settings["tip"],
            expected_joint_names=tuple(f"xarm_right_joint{i}" for i in range(1, 8)),
        )
        self.mean = np.asarray(self.settings["force_mean"], dtype=np.float64)
        self.std = np.asarray(self.settings["force_std"], dtype=np.float64)
        if (
            self.mean.shape != (6,)
            or self.std.shape != (6,)
            or not np.isfinite(np.r_[self.mean, self.std]).all()
            or np.any(self.std <= 0)
        ):
            raise ValueError("invalid training force normalization")
        if self.settings["frame_hz"] != 30 or self.settings["capacity"] != 2048:
            raise ValueError("Unified requires 30Hz history and capacity 2048")
        for key in ("maximum_sample_gap_s", "maximum_force_skew_s"):
            if not 0 < self.settings[key] <= 0.1:
                raise ValueError("invalid Unified history freshness limit")

    def encode(self, joints, force, previous=None):
        joints, force = np.asarray(joints, dtype=np.float64), np.asarray(force, dtype=np.float64)
        if (
            joints.shape != (8,)
            or force.shape != (6,)
            or not np.isfinite(np.r_[joints, force]).all()
        ):
            raise ValueError("finite measured joint8 and wrench6 required")
        pose = self.chain.evaluate(
            dict(zip(self.chain.joint_names, np.deg2rad(joints[:7]), strict=True))
        )
        position, rotation = np.asarray(pose.position_m), np.asarray(pose.rotation)
        dp, dr = np.zeros(3), np.zeros(3)
        if previous is not None:
            dp = position - previous[0]
            relative = rotation @ previous[1].T
            sine_vector = (
                np.array(
                    [
                        relative[2, 1] - relative[1, 2],
                        relative[0, 2] - relative[2, 0],
                        relative[1, 0] - relative[0, 1],
                    ]
                )
                / 2
            )
            sine = np.linalg.norm(sine_vector)
            angle = math.atan2(sine, np.clip((np.trace(relative) - 1) / 2, -1, 1))
            if angle > math.pi - 1e-4:
                raise ValueError("discontinuous Unified orientation history")
            dr = sine_vector * (angle / sine if sine > 1e-12 else 1)
        feature = np.concatenate(
            [
                (force - self.mean) / (self.std + 1e-6),
                position / 0.5,
                rotation[:, :2].T.reshape(6),
                joints[7:8] / 90,
                dp / 0.01,
                dr / 0.1,
            ]
        ).astype(np.float32)
        if not np.isfinite(feature).all():
            raise ValueError("nonfinite Unified feature")
        return feature, (position, rotation)


class EpisodeHistory:
    """Pure measured prefix construction, shared by live capture and regression tests."""

    def __init__(self, features):
        self.features = features
        self.rows = []
        self.previous_geometry = None

    def append(self, state, force, *, now_ns):
        settings = self.features.settings
        stamp = state.metadata.monotonic_ns
        if tuple(state.feature_names) != FEATURES or not 0 <= now_ns - stamp < 100_000_000:
            raise ValueError("Unified requires fresh ordered right Follower feedback")
        if not 0 <= now_ns - force.monotonic_ns < 100_000_000:
            raise ValueError("Unified requires fresh real force feedback")
        if abs(force.monotonic_ns - stamp) > settings["maximum_force_skew_s"] * 1e9:
            raise ValueError("Unified force/joint timestamp skew")
        if len(self.rows) >= settings["capacity"]:
            raise ValueError("Unified episode capacity reached; finish and start a new episode")
        dt = 1 / 30 if not self.rows else (stamp - self.rows[-1][0].metadata.monotonic_ns) / 1e9
        if not 0 < dt <= settings["maximum_sample_gap_s"]:
            raise ValueError("Unified joint history regressed or has a gap")
        feature, geometry = self.features.encode(
            [state.values[name] for name in FEATURES], force.values, self.previous_geometry
        )
        self.rows.append((state, force, feature, dt))
        self.previous_geometry = geometry

    def snapshot(self, stamp):
        rows = [r for r in self.rows if r[0].metadata.monotonic_ns <= stamp]
        if not rows or stamp - rows[-1][0].metadata.monotonic_ns > 50_000_000:
            raise ValueError("no causal Unified history aligned with camera frame")
        short = np.asarray(
            [[*(row[0].values[name] for name in FEATURES), *row[1].values] for row in rows[-10:]],
            dtype=np.float32,
        )
        short = np.pad(short, ((10 - len(short), 0), (0, 0)), mode="edge")
        history, dt = np.zeros((2048, 22), np.float32), np.zeros(2048, np.float32)
        history[: len(rows)] = [row[2] for row in rows]
        dt[: len(rows)] = [row[3] for row in rows]
        mask = np.arange(2048) < len(rows)
        payload = {
            "short_state": short,
            "force_history_mask": np.ones(10, bool),
            "interaction_history": history,
            "interaction_dt": dt,
            "interaction_mask": mask,
            "interaction_force_mask": mask.copy(),
        }
        return rows[-1][0], payload, rows[-1][1].monotonic_ns


class UnifiedHistory:
    def __init__(self, config, closest_state):
        self.config, self.closest_state = config, closest_state
        self.episode = EpisodeHistory(
            UnifiedFeatures(config.path.parents[2] / "configs/inference/unified")
        )
        self.source = HKVLCollectionSource(config.ft_sensor)
        self._condition = Condition()
        self._stop = Event()
        self._error = None
        self._thread = None

    def start(self):
        self.source.__enter__()
        try:
            self.source.wait_first(timeout_s=self.config.ft_sensor.startup_timeout_s, max_age_s=0.1)
            self._thread = Thread(target=self._sample, name="unified-history", daemon=True)
            self._thread.start()
            deadline = time.monotonic() + 1
            with self._condition:
                while len(self.episode.rows) < 2 and self._error is None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("no Unified joint/force history")
                    self._condition.wait(remaining)
                if self._error is not None:
                    raise RuntimeError("Unified history failed") from self._error
            return self
        except BaseException:
            self.close()
            raise

    def _sample(self):
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                state = self.closest_state(time.monotonic_ns())
                with self._condition:
                    duplicate = bool(self.episode.rows) and (
                        state.metadata.monotonic_ns
                        == self.episode.rows[-1][0].metadata.monotonic_ns
                    )
                if duplicate:
                    if time.monotonic_ns() - state.metadata.monotonic_ns >= 100_000_000:
                        raise ValueError("Unified joint feedback stopped updating")
                    deadline = max(deadline + 1 / 30, time.monotonic())
                    self._stop.wait(max(0, deadline - time.monotonic()))
                    continue
                force = self.source.closest(
                    target_monotonic_ns=state.metadata.monotonic_ns, timeout_s=0.02
                )
                with self._condition:
                    self.episode.append(state, force, now_ns=time.monotonic_ns())
                    self._condition.notify_all()
                deadline += 1 / 30
                now = time.monotonic()
                if deadline < now:
                    deadline = now + 1 / 30
                self._stop.wait(deadline - now)
        except Exception as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()

    def snapshot(self, stamp):
        with self._condition:
            if self._error is not None:
                raise RuntimeError(f"Unified history failed: {self._error}") from self._error
            return self.episode.snapshot(stamp)

    def stop(self):
        self._stop.set()

    def close(self):
        self.stop()
        if self._thread is not None:
            self._thread.join(timeout=1)
        self.source.__exit__(None, None, None)
