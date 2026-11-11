"""Native pi0 observation assembly and persistent prediction transport.

No motor command transport is imported by this module. Live observations are
consumed from the existing camera and teleoperation owners via IPC. A prediction
is never evidence that a controller accepted or physically executed an action.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlparse

from .configuration import SystemConfig, load_system_config
from .contracts import NamedJointVector, feature_names_for_sides

FEATURES = feature_names_for_sides(("right",))


@dataclass(frozen=True)
class Pi0Spec:
    endpoint: str
    image_height: int
    image_width: int
    action_horizon: int
    action_hz: int
    open_timeout_s: float
    response_timeout_s: float
    close_timeout_s: float
    maximum_message_bytes: int

    def __post_init__(self):
        endpoint = urlparse(self.endpoint)
        if (
            endpoint.scheme != "ws"
            or endpoint.hostname != "127.0.0.1"
            or endpoint.username is not None
            or endpoint.password is not None
            or endpoint.path not in ("", "/")
            or endpoint.query
            or endpoint.fragment
            or endpoint.port is None
        ):
            raise ValueError("pi0 endpoint must be an explicit loopback WebSocket port")
        for name in ("image_height", "image_width", "action_horizon", "action_hz"):
            expected = {
                "image_height": 224,
                "image_width": 224,
                "action_horizon": 32,
                "action_hz": 30,
            }[name]
            if type(getattr(self, name)) is not int or getattr(self, name) != expected:
                raise ValueError(f"native pi0 {name} must be {expected}")
        for name in ("open_timeout_s", "response_timeout_s", "close_timeout_s"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if (
            type(self.maximum_message_bytes) is not int
            or not 1 <= self.maximum_message_bytes <= 2**26
        ):
            raise ValueError("maximum_message_bytes must be in [1, 64 MiB]")

    @classmethod
    def load(cls, path: Path, *, kind="pi0"):
        document = json.loads(path.read_text())
        if not isinstance(document, dict) or set(document) != {"kind", *cls.__dataclass_fields__}:
            raise ValueError("exact pi0 inference configuration fields required")
        if (
            kind not in ("pi0", "unified")
            or document.pop("kind") != f"vlai_native_{kind}_prediction_v1"
        ):
            raise ValueError("unsupported inference configuration")
        return cls(**document)


@dataclass(frozen=True)
class Pi0Observation:
    payload: dict
    oldest_source_ns: int
    state_sequence: int


class Pi0ObservationBuilder:
    """Keep capture timestamps and enforce exact right-arm/camera identity."""

    def __init__(self, config: SystemConfig, spec: Pi0Spec, *, task: str, distance_cm: float):
        if not isinstance(task, str) or not task.strip() or task != task.strip():
            raise ValueError("a nonempty normalized task is required")
        if (
            type(distance_cm) not in (int, float)
            or not math.isfinite(distance_cm)
            or distance_cm <= 0
        ):
            raise ValueError("distance_cm must be finite and positive")
        self.config, self.spec, self.task, self.distance_cm = config, spec, task, distance_cm
        self._camera_specs = {s.role: s for s in config.cameras.streams if s.enabled}
        if not {"agent", "wrist_right"} <= set(self._camera_specs):
            raise ValueError("pi0 requires configured agent and right wrist cameras")
        self._last_state = None
        self._last_cameras = {}

    def build(self, state: NamedJointVector, cameras: dict, *, now_ns: int) -> Pi0Observation:
        import numpy as np

        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a nonnegative monotonic timestamp")
        if not isinstance(state, NamedJointVector) or state.feature_names != FEATURES:
            raise ValueError("exact named right-arm degree state required")
        state = NamedJointVector(state.values, state.metadata, FEATURES)
        metadata = state.metadata
        current = (metadata.source_sequence, metadata.monotonic_ns)
        if self._last_state is not None and any(
            a <= b for a, b in zip(current, self._last_state, strict=True)
        ):
            raise ValueError("joint source sequence/timestamp did not increase")
        self._fresh(metadata.monotonic_ns, now_ns, self.config.teleoperation.state_timeout_s)
        limits = self.config.teleoperation.joint_safety.for_side("right")
        for name, low, high in zip(FEATURES[:7], limits.min_deg, limits.max_deg, strict=True):
            if not low <= state.values[name] <= high:
                raise ValueError(f"observed joint outside tracked bounds: {name}")
        images, sources, candidate_cameras = {}, [metadata.monotonic_ns], {}
        for role, key in (("agent", "image"), ("wrist_right", "wrist_image")):
            if role not in cameras:
                raise ValueError(f"missing camera: {role}")
            sample, camera_spec = cameras[role], self._camera_specs[role]
            frame = sample.metadata
            if frame.role != role or frame.device_id != camera_spec.device_id:
                raise ValueError(f"camera physical identity mismatch: {role}")
            self._fresh(frame.monotonic_ns, now_ns, self.config.cameras.max_age_s)
            prior = self._last_cameras.get(role)
            candidate = (frame.stream_epoch, frame.source_sequence, frame.monotonic_ns)
            if prior is not None and (
                candidate[0] != prior[0] or candidate[1] <= prior[1] or candidate[2] <= prior[2]
            ):
                raise ValueError(f"camera restarted or source did not increase: {role}")
            image = np.asarray(sample.image)
            if image.dtype != np.uint8 or image.shape != (camera_spec.height, camera_spec.width, 3):
                raise ValueError(f"configured RGB uint8 camera shape required: {role}")
            images[key] = self._resize(image)
            sources.append(frame.monotonic_ns)
            candidate_cameras[role] = candidate
        if max(sources) - min(sources) > round(self.config.cameras.max_pair_skew_s * 1e9):
            raise ValueError("image/joint source timestamp skew exceeds tracked camera skew")
        values = np.asarray([state.values[name] for name in FEATURES], dtype=np.float32)
        if not np.isfinite(values).all():
            raise ValueError("joint state cannot be represented as finite float32")
        payload = {
            "state": values,
            **images,
            "prompt": self.task,
            "joint_names": list(FEATURES),
            "position_unit": "degree",
            "target_distance_cm": self.distance_cm,
        }
        self._last_state, self._last_cameras = current, candidate_cameras
        return Pi0Observation(payload, min(sources), metadata.source_sequence)

    @staticmethod
    def _fresh(source_ns, now_ns, max_age_s):
        if source_ns > now_ns or now_ns - source_ns > round(max_age_s * 1e9):
            raise ValueError("observation source timestamp is future or stale")

    def _resize(self, image):
        # Match openpi_client.image_tools.resize_with_pad: PIL bilinear, aspect
        # ratio preserved, floor resized dimensions and centered black padding.
        import numpy as np
        from PIL import Image

        height, width = self.spec.image_height, self.spec.image_width
        if image.shape[:2] == (height, width):
            return image.copy()
        ratio = max(image.shape[1] / width, image.shape[0] / height)
        size = (int(image.shape[1] / ratio), int(image.shape[0] / ratio))
        resized = Image.fromarray(image).resize(size, resample=Image.Resampling.BILINEAR)
        result = Image.new("RGB", (width, height), 0)
        result.paste(resized, ((width - size[0]) // 2, (height - size[1]) // 2))
        return np.asarray(result).copy()


class Pi0PredictionClient:
    """One persistent, bounded WebSocket; no automatic connection retry."""

    def __init__(self, spec: Pi0Spec, *, expected_kind="pi0"):
        self.spec = spec
        if expected_kind not in ("pi0", "unified"):
            raise ValueError("unsupported native prediction kind")
        self.expected_kind = expected_kind
        self._socket = None
        self.server_metadata = {}

    def __enter__(self):
        from websockets.sync.client import connect

        self._socket = connect(
            self.spec.endpoint,
            compression=None,
            max_size=self.spec.maximum_message_bytes,
            open_timeout=self.spec.open_timeout_s,
            close_timeout=self.spec.close_timeout_s,
            ping_interval=None,
        )
        try:
            metadata = unpack_message(self._socket.recv(timeout=self.spec.open_timeout_s))
            if (
                not isinstance(metadata, dict)
                or metadata.get("kind") != self.expected_kind
                or metadata.get("native_joint_degrees") is not True
                or metadata.get("prediction_only") is not True
                or metadata.get("transport_only", False) is not False
                or type(metadata.get("fps")) is not int
                or metadata["fps"] != self.spec.action_hz
                or type(metadata.get("prediction_horizon")) is not int
                or metadata["prediction_horizon"] != self.spec.action_horizon
            ):
                raise ValueError(
                    f"server is not the pinned native {self.expected_kind} prediction server"
                )
            self.server_metadata = metadata
        except BaseException:
            self._socket.close()
            self._socket = None
            raise
        return self

    def predict(self, observation: Pi0Observation):
        import numpy as np

        connection = self._socket
        if connection is None:
            raise RuntimeError("prediction connection is not open")
        try:
            raw = pack_message(observation.payload)
            if len(raw) > self.spec.maximum_message_bytes:
                raise ValueError("observation exceeds configured message bound")
            connection.send(raw)
            response_raw = connection.recv(timeout=self.spec.response_timeout_s)
            response = unpack_message(response_raw)
            if not isinstance(response, dict) or "actions" not in response:
                raise ValueError("server did not return actions")
            actions = np.asarray(response["actions"])
            if (
                actions.shape != (self.spec.action_horizon, len(FEATURES))
                or actions.dtype.kind != "f"
                or not np.isfinite(actions).all()
            ):
                raise ValueError("expected finite floating-point native 32x8 actions")
            return actions.copy(), response_raw
        except BaseException:
            # An unanswered request makes response ordering ambiguous. Never
            # reuse the connection after a timeout, decode or contract failure.
            connection.close()
            if self._socket is connection:
                self._socket = None
            raise

    def close(self):
        connection, self._socket = self._socket, None
        if connection is not None:
            connection.close()

    def __exit__(self, *args):
        self.close()


def pack_message(value):
    import msgpack
    import numpy as np

    def encode(item):
        if isinstance(item, np.ndarray):
            if item.dtype.kind not in "biuf":
                raise ValueError("only numeric NumPy arrays are allowed")
            return {
                b"__ndarray__": True,
                b"data": item.tobytes(),
                b"dtype": item.dtype.str,
                b"shape": item.shape,
            }
        if isinstance(item, np.generic) and item.dtype.kind in "biuf":
            return {b"__npgeneric__": True, b"data": item.item(), b"dtype": item.dtype.str}
        raise TypeError(f"unsupported prediction payload type: {type(item).__name__}")

    return msgpack.packb(value, default=encode)


def unpack_message(raw):
    import msgpack
    import numpy as np

    if not isinstance(raw, bytes):
        raise ValueError("prediction server returned a nonbinary message")

    def decode(item):
        if b"__ndarray__" in item or b"__npgeneric__" in item:
            dtype = np.dtype(item[b"dtype"])
            if dtype.kind not in "biuf":
                raise ValueError("non-numeric array dtype rejected")
            if b"__ndarray__" in item:
                shape = item[b"shape"]
                if (
                    not isinstance(shape, (list, tuple))
                    or len(shape) > 4
                    or any(type(n) is not int or n < 0 for n in shape)
                ):
                    raise ValueError("invalid prediction array shape")
                data = item[b"data"]
                if not isinstance(data, bytes) or math.prod(shape) * dtype.itemsize != len(data):
                    raise ValueError("prediction array buffer size mismatch")
                return np.frombuffer(data, dtype=dtype).reshape(shape)
            return dtype.type(item[b"data"])
        return item

    return msgpack.unpackb(raw, object_hook=decode, strict_map_key=False)


def run_predictions(config, spec, *, task, distance_cm, samples, output):
    from .camera_ipc import RawCameraBridgeClient
    from .teleoperation.xair import XAirStateReceiver

    builder = Pi0ObservationBuilder(config, spec, task=task, distance_cm=distance_cm)
    if type(samples) is not int or samples <= 0:
        raise ValueError("samples must be a positive integer")
    # Create-only artifacts; never overwrite a previous experiment.
    output.mkdir(parents=True, exist_ok=False)
    with ExitStack() as stack:
        client = stack.enter_context(Pi0PredictionClient(spec))
        cameras = stack.enter_context(RawCameraBridgeClient(config))
        states = stack.enter_context(XAirStateReceiver(config, sides=("right",)))
        for index in range(samples):
            images = cameras.capture(timeout_s=config.cameras.startup_timeout_s)
            times = [images[role].metadata.monotonic_ns for role in ("agent", "wrist_right")]
            robot = states.receive_closest(
                target_monotonic_ns=(min(times) + max(times)) // 2,
                timeout_s=config.teleoperation.state_timeout_s,
            )
            if robot is None:
                raise TimeoutError("no fresh right-arm state from the existing teleop owner")
            observation = builder.build(robot[0], images, now_ns=time.monotonic_ns())
            started = time.monotonic_ns()
            actions, response_raw = client.predict(observation)
            received = time.monotonic_ns()
            outside = []
            limits = config.teleoperation.joint_safety.for_side("right")
            for j, name in enumerate(FEATURES[:7]):
                if (actions[:, j] < limits.min_deg[j]).any() or (
                    actions[:, j] > limits.max_deg[j]
                ).any():
                    outside.append(name)
            report = {
                "model": "pi0",
                "source": "live_ipc",
                "motor_commands_sent": False,
                "task": task,
                "target_distance_cm": distance_cm,
                "state_sequence": observation.state_sequence,
                "oldest_source_monotonic_ns": observation.oldest_source_ns,
                "request_monotonic_ns": started,
                "response_monotonic_ns": received,
                "roundtrip_ms": (received - started) / 1e6,
                "source_age_ms": (received - observation.oldest_source_ns) / 1e6,
                "within_command_age": received - observation.oldest_source_ns
                <= round(config.safety.command_inactivity_timeout_s * 1e9),
                "actions_outside_joint_bounds": outside,
                "shape": list(actions.shape),
                "command_blockers": list(config.command_blockers),
            }
            with (output / f"{index:04d}-request.msgpack").open("xb") as handle:
                handle.write(pack_message(observation.payload))
            with (output / f"{index:04d}-response.msgpack").open("xb") as handle:
                handle.write(response_raw)
            with (output / f"{index:04d}.json").open("x") as handle:
                json.dump(report, handle, indent=2, allow_nan=False)
            print(json.dumps(report, allow_nan=False), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--inference", type=Path, required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--distance-cm", type=float, required=True)
    parser.add_argument("--samples", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        run_predictions(
            load_system_config(args.config),
            Pi0Spec.load(args.inference),
            task=args.task,
            distance_cm=args.distance_cm,
            samples=args.samples,
            output=args.output,
        )
    except Exception as exc:
        parser.exit(1, f"pi0 prediction failed; no motor commands sent: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
