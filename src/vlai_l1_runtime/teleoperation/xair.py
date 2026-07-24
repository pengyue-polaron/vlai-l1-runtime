"""Pure contracts and static integration checks for the pinned x_air SDK."""

from __future__ import annotations

import hashlib
import json
import math
import platform
import re
import select
import shutil
import socket
import stat
import struct
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

from ..configuration import MOTOR_NAMES, ROLES, SIDES, ControlProfile, SystemConfig
from ..contracts import NamedJointVector, SampleMetadata

_PACKET = struct.Struct("<4sHBBQQ16d")
_MAGIC = b"VL1S"
_PROTOCOL_VERSION = 1
_SIDE_BY_CODE = {0: "left", 1: "right"}
_REQUIRED_SYMBOLS = (
    "xarm_teleop_create_unilateral",
    "xarm_teleop_destroy",
    "xarm_teleop_get_last_error",
    "xarm_teleop_go_home",
    "xarm_teleop_is_running",
    "xarm_teleop_set_full_state_callback",
    "xarm_teleop_start",
    "xarm_teleop_stop",
    "xarm_teleop_version",
)
_ELF_MACHINE = {"aarch64": 183, "x86_64": 62}
_PROFILE_FIELDS = {
    "Kp": "kp",
    "Kd": "kd",
    "Fc": "fc",
    "k": "friction_k",
    "Fv": "fv",
    "Fo": "fo",
}
_YAML_VECTOR = re.compile(r"^\s*(Kp|Kd|Fc|k|Fv|Fo):\s*\[(.*)]\s*$")
_PACKAGE_VERSION = re.compile(r"<version>\s*([^<\s]+)\s*</version>")
_ADJUST_POSITION_REQUEST = b"ADJUST_POSITION\n"
_CONTROL_RESPONSE_LIMIT = 4096


@dataclass(frozen=True)
class XAirDependencyReport:
    revision: str
    architecture: str
    sdk_version: str
    teleop_library: Path
    teleop_library_sha256: str
    can_library: Path
    can_library_sha256: str


@dataclass(frozen=True)
class XAirStatePacket:
    side: str
    source_sequence: int
    monotonic_ns: int
    leader_radians: tuple[float, ...]
    follower_radians: tuple[float, ...]

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError(f"unknown x_air side: {self.side!r}")
        for label, value in (
            ("source_sequence", self.source_sequence),
            ("monotonic_ns", self.monotonic_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"x_air {label} must be a non-negative integer")
        for label, vector in (
            ("leader", self.leader_radians),
            ("follower", self.follower_radians),
        ):
            if len(vector) != len(MOTOR_NAMES):
                raise ValueError(f"x_air {label} state must contain exactly 8 positions")
            if any(not math.isfinite(value) for value in vector):
                raise ValueError(f"x_air {label} state must be finite")

    @classmethod
    def decode(cls, payload: bytes) -> XAirStatePacket:
        if not isinstance(payload, bytes) or len(payload) != _PACKET.size:
            raise ValueError(f"x_air packet must contain exactly {_PACKET.size} bytes")
        magic, version, side_code, reserved, sequence, timestamp, *positions = _PACKET.unpack(
            payload
        )
        if magic != _MAGIC:
            raise ValueError("x_air packet magic is invalid")
        if version != _PROTOCOL_VERSION:
            raise ValueError(f"unsupported x_air packet version: {version}")
        if reserved != 0:
            raise ValueError("x_air packet reserved byte must be zero")
        try:
            side = _SIDE_BY_CODE[side_code]
        except KeyError as exc:
            raise ValueError(f"unknown x_air packet side code: {side_code}") from exc
        return cls(side, sequence, timestamp, tuple(positions[:8]), tuple(positions[8:]))


class XAirBimanualAssembler:
    """Combine fresh left/right SDK packets into named degree-valued vectors."""

    def __init__(self, *, max_side_skew_s: float) -> None:
        if (
            isinstance(max_side_skew_s, bool)
            or not isinstance(max_side_skew_s, (int, float))
            or not math.isfinite(max_side_skew_s)
            or max_side_skew_s <= 0
        ):
            raise ValueError("max_side_skew_s must be finite and positive")
        self._max_side_skew_ns = int(max_side_skew_s * 1_000_000_000)
        self._latest: dict[str, XAirStatePacket] = {}
        self._last_received: dict[str, int] = {}
        self._last_emitted: dict[str, int] = {}
        self._output_sequence = 0

    def accept(self, packet: XAirStatePacket) -> tuple[NamedJointVector, NamedJointVector] | None:
        if not isinstance(packet, XAirStatePacket):
            raise TypeError("packet must be an XAirStatePacket")
        previous = self._last_received.get(packet.side)
        if previous is not None and packet.source_sequence <= previous:
            raise ValueError(f"x_air {packet.side} source sequence did not increase")
        self._last_received[packet.side] = packet.source_sequence
        self._latest[packet.side] = packet
        if set(self._latest) != set(SIDES):
            return None
        if any(
            self._latest[side].source_sequence <= self._last_emitted.get(side, -1) for side in SIDES
        ):
            return None

        timestamps = [self._latest[side].monotonic_ns for side in SIDES]
        if max(timestamps) - min(timestamps) > self._max_side_skew_ns:
            raise ValueError("x_air left/right state skew exceeds the tracked limit")

        action_values: dict[str, float] = {}
        observation_values: dict[str, float] = {}
        for side in SIDES:
            current = self._latest[side]
            for motor, leader, follower in zip(
                MOTOR_NAMES,
                current.leader_radians,
                current.follower_radians,
                strict=True,
            ):
                action_values[f"{side}_{motor}.pos"] = math.degrees(leader)
                observation_values[f"{side}_{motor}.pos"] = math.degrees(follower)
            self._last_emitted[side] = current.source_sequence

        metadata = SampleMetadata(self._output_sequence, max(timestamps))
        self._output_sequence += 1
        return NamedJointVector(observation_values, metadata), NamedJointVector(
            action_values, metadata
        )


class XAirStateReceiver:
    """Own the configured Unix datagram endpoint and emit paired named vectors."""

    def __init__(self, config: SystemConfig) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("XAirStateReceiver requires SystemConfig")
        self._path = config.teleoperation.state_socket_path
        self._max_side_skew_s = config.teleoperation.max_side_skew_s
        self._assembler = XAirBimanualAssembler(max_side_skew_s=self._max_side_skew_s)
        self._socket: socket.socket | None = None
        self._socket_inode: int | None = None

    def __enter__(self) -> XAirStateReceiver:
        if self._socket is not None:
            raise RuntimeError("x_air state receiver is already open")
        if not self._path.parent.is_dir():
            raise FileNotFoundError(f"x_air state socket directory is missing: {self._path.parent}")
        if self._path.exists() or self._path.is_symlink():
            raise FileExistsError(f"x_air state socket path is already in use: {self._path}")
        receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)
        try:
            receiver.bind(str(self._path))
            identity = self._path.lstat()
            if not stat.S_ISSOCK(identity.st_mode):
                raise RuntimeError("bound x_air state endpoint is not a socket")
        except BaseException:
            receiver.close()
            raise
        self._socket = receiver
        self._socket_inode = identity.st_ino
        return self

    def receive(self, *, timeout_s: float) -> tuple[NamedJointVector, NamedJointVector] | None:
        _require_timeout(timeout_s)
        deadline = _monotonic_seconds() + timeout_s
        while True:
            remaining = max(0.0, deadline - _monotonic_seconds())
            packet = self.receive_packet(timeout_s=remaining)
            if packet is None:
                return None
            result = self._assembler.accept(packet)
            if result is not None:
                return result
            if _monotonic_seconds() >= deadline:
                return None

    def receive_packet(self, *, timeout_s: float) -> XAirStatePacket | None:
        if self._socket is None:
            raise RuntimeError("x_air state receiver is not open")
        _require_timeout(timeout_s)
        readable, _, _ = select.select((self._socket,), (), (), timeout_s)
        if not readable:
            return None
        return XAirStatePacket.decode(self._socket.recv(_PACKET.size + 1))

    def receive_closest(
        self,
        *,
        target_monotonic_ns: int,
        timeout_s: float,
    ) -> tuple[NamedJointVector, NamedJointVector] | None:
        """Return the queued complete pair closest to the requested sample time."""

        if (
            isinstance(target_monotonic_ns, bool)
            or not isinstance(target_monotonic_ns, int)
            or target_monotonic_ns < 0
        ):
            raise ValueError("target_monotonic_ns must be a non-negative integer")
        _require_timeout(timeout_s)
        closest: tuple[NamedJointVector, NamedJointVector] | None = None
        closest_key: tuple[int, int] | None = None
        while True:
            packet = self.receive_packet(timeout_s=0)
            if packet is None:
                break
            candidate = self._assembler.accept(packet)
            if candidate is not None:
                timestamp_ns = candidate[0].metadata.monotonic_ns
                key = (abs(timestamp_ns - target_monotonic_ns), -timestamp_ns)
                if closest_key is None or key < closest_key:
                    closest = candidate
                    closest_key = key
        if closest is not None:
            return closest
        return self.receive(timeout_s=timeout_s)

    def reset_pairing(self) -> None:
        """Discard queued packets after a deliberate SDK control-loop restart."""

        if self._socket is None:
            raise RuntimeError("x_air state receiver is not open")
        while True:
            readable, _, _ = select.select((self._socket,), (), (), 0)
            if not readable:
                break
            self._socket.recv(_PACKET.size + 1)
        self._assembler = XAirBimanualAssembler(max_side_skew_s=self._max_side_skew_s)

    def __exit__(self, exc_type, exc, traceback) -> None:
        receiver, self._socket = self._socket, None
        if receiver is not None:
            receiver.close()
        try:
            identity = self._path.lstat()
        except FileNotFoundError:
            return
        if stat.S_ISSOCK(identity.st_mode) and identity.st_ino == self._socket_inode:
            self._path.unlink()
        self._socket_inode = None


def xair_control_socket_path(config: SystemConfig, side: str) -> Path:
    """Derive one sidecar control endpoint from the tracked state endpoint."""

    if not isinstance(config, SystemConfig):
        raise TypeError("xair_control_socket_path requires SystemConfig")
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}")
    state_path = config.teleoperation.state_socket_path
    return state_path.with_name(f"{state_path.stem}-{side}-control.sock")


def request_xair_adjust_position(config: SystemConfig, side: str) -> None:
    """Run the SDK AdjustPosition routine through the owning sidecar."""

    path = xair_control_socket_path(config, side)
    timeout_s = config.teleoperation.startup_timeout_s
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout_s)
    try:
        client.connect(str(path))
        client.sendall(_ADJUST_POSITION_REQUEST)
        client.shutdown(socket.SHUT_WR)
        response = bytearray()
        while len(response) <= _CONTROL_RESPONSE_LIMIT:
            chunk = client.recv(_CONTROL_RESPONSE_LIMIT + 1 - len(response))
            if not chunk:
                break
            response.extend(chunk)
    except TimeoutError as exc:
        raise TimeoutError(f"{side} x_air AdjustPosition exceeded {timeout_s:.1f}s") from exc
    except OSError as exc:
        raise RuntimeError(f"cannot request {side} x_air AdjustPosition: {exc}") from exc
    finally:
        client.close()
    if len(response) > _CONTROL_RESPONSE_LIMIT:
        raise RuntimeError(f"{side} x_air control response is too large")
    if response == b"OK\n":
        return
    try:
        detail = bytes(response).decode("utf-8").strip()
    except UnicodeDecodeError as exc:
        raise RuntimeError(f"{side} x_air control response is not UTF-8") from exc
    if not detail:
        detail = "empty control response"
    raise RuntimeError(f"{side} x_air AdjustPosition failed: {detail}")


def describe_xair_side(config: SystemConfig, side: str) -> dict[str, object]:
    """Return the explicit launch contract without opening CAN or loading the SDK."""

    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}")
    endpoints = {(endpoint.side, endpoint.role): endpoint for endpoint in config.can.endpoints}
    leader = endpoints[(side, "leader")]
    follower = endpoints[(side, "follower")]
    teleop = config.teleoperation
    if teleop.state_protocol_version != _PROTOCOL_VERSION:
        raise ValueError("tracked x_air state protocol version is unsupported")
    return {
        "provider": teleop.provider,
        "mode": teleop.mode,
        "arm_type": teleop.arm_type,
        "sdk_version": teleop.sdk_version,
        "source_revision": teleop.source_revision,
        "state_protocol_version": teleop.state_protocol_version,
        "side": side,
        "arm_side": f"{side}_arm",
        "leader_can": leader.interface,
        "follower_can": follower.interface,
        "can_fd": config.can.fd,
        "can_nominal_bitrate": config.can.nominal_bitrate,
        "can_data_bitrate": config.can.data_bitrate,
        "can_restart_ms": config.can.restart_ms,
        "can_tx_queue_length": config.can.tx_queue_length,
        "state_socket_path": str(teleop.state_socket_path),
        "control_socket_path": str(xair_control_socket_path(config, side)),
        "publish_hz": teleop.publish_hz,
        "state_timeout_ms": round(teleop.state_timeout_s * 1000),
        "rt_priority": teleop.rt_priority,
        "can_health_poll_ms": round(teleop.can_health_poll_s * 1000),
        "commissioned": teleop.commissioned,
    }


def render_xair_control_config(config: SystemConfig, destination: Path) -> tuple[Path, Path]:
    """Render both vendor YAML files from the authoritative System config."""

    destination = Path(destination)
    if not destination.is_dir():
        raise ValueError("x_air config destination must be an existing directory")
    outputs = tuple(destination / f"{role}.yaml" for role in ROLES)
    if any(path.exists() for path in outputs):
        raise FileExistsError("x_air control config destination is not empty")
    for role, output in zip(ROLES, outputs, strict=True):
        table = "LeaderArmParam" if role == "leader" else "FollowerArmParam"
        output.write_text(_render_profile(table, config.control[role]), encoding="utf-8")
    return outputs


def prepare_xair_assets(config: SystemConfig, destination: Path) -> Path:
    """Atomically render checked control YAML, URDF, and launch provenance."""

    report = verify_xair_dependency(config)
    destination = Path(destination).absolute()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"x_air asset destination already exists: {destination}")
    if not destination.parent.is_dir() or destination.parent.is_symlink():
        raise ValueError("x_air asset destination parent must be a real directory")
    staging = Path(tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent))
    try:
        control_dir = staging / "config"
        control_dir.mkdir()
        render_xair_control_config(config, control_dir)
        description_root = config.teleoperation.source_root / "publish/modules/src/xarm_description"
        xacro = description_root / "urdf/robot" / f"{config.teleoperation.arm_type}.urdf.xacro"
        leader_urdf = staging / f"{config.teleoperation.arm_type}_leader.urdf"
        follower_urdf = staging / f"{config.teleoperation.arm_type}_follower.urdf"
        try:
            subprocess.run(
                ("xacro", str(xacro), "bimanual:=true", "-o", str(leader_urdf)),
                check=True,
                capture_output=True,
                text=True,
            )
        except OSError as exc:
            raise RuntimeError(f"cannot execute xacro: {exc}") from exc
        except subprocess.CalledProcessError as exc:
            detail = (exc.stderr or exc.stdout or str(exc)).strip()
            raise RuntimeError(f"cannot render x_air URDF: {detail}") from exc
        if not leader_urdf.is_file() or leader_urdf.stat().st_size == 0:
            raise RuntimeError("x_air produced an empty leader URDF")
        shutil.copyfile(leader_urdf, follower_urdf)
        manifest = {
            "schema_version": 1,
            "dependency": {
                "revision": report.revision,
                "sdk_version": report.sdk_version,
                "architecture": report.architecture,
                "teleop_library_sha256": report.teleop_library_sha256,
                "can_library_sha256": report.can_library_sha256,
            },
            "state_protocol_version": config.teleoperation.state_protocol_version,
            "sides": {side: describe_xair_side(config, side) for side in SIDES},
            "assets": {
                "leader_urdf": leader_urdf.name,
                "follower_urdf": follower_urdf.name,
                "config_dir": control_dir.name,
            },
        }
        manifest_path = staging / "manifest.json"
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        staging.rename(destination)
        return destination / manifest_path.name
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def verify_xair_dependency(config: SystemConfig) -> XAirDependencyReport:
    """Verify the pinned checkout and ABI surface without initializing hardware."""

    teleop = config.teleoperation
    if teleop.state_protocol_version != _PROTOCOL_VERSION:
        raise ValueError("tracked x_air state protocol version is unsupported")
    root = teleop.source_root
    if not root.is_dir():
        raise ValueError(f"x_air SDK checkout is missing: {root}")
    revision = _git_output(root, "rev-parse", "HEAD")
    if revision != teleop.source_revision:
        raise ValueError(
            f"x_air SDK revision mismatch: expected {teleop.source_revision}, got {revision}"
        )
    if _git_output(root, "status", "--porcelain", "--untracked-files=no"):
        raise ValueError("x_air SDK checkout has tracked modifications")

    architecture = platform.machine()
    if architecture not in _ELF_MACHINE:
        raise ValueError(f"unsupported x_air host architecture: {architecture}")
    teleop_root = root / "publish/modules/src/xarm_teleop"
    teleop_library = (
        teleop_root / "prebuilt/xarm_teleop/lib" / architecture / "libxarm_teleop_lib.so"
    )
    can_library = root / "publish/xarm_can/package/lib" / architecture / "libxarm_can_sdk.so"
    header = teleop_root / "include/xarm_teleop_sdk.h"
    package_manifest = teleop_root / "package.xml"
    xacro = (
        root / "publish/modules/src/xarm_description/urdf/robot" / f"{teleop.arm_type}.urdf.xacro"
    )
    for label, path in (
        ("teleoperation library", teleop_library),
        ("CAN library", can_library),
        ("public SDK header", header),
        ("package manifest", package_manifest),
        ("robot xacro", xacro),
    ):
        if not path.is_file():
            raise ValueError(f"x_air {label} is missing: {path}")
    _verify_elf(teleop_library, expected_machine=_ELF_MACHINE[architecture])
    _verify_elf(can_library, expected_machine=_ELF_MACHINE[architecture])
    symbols = {
        line.split()[-1]
        for line in _command_output("nm", "-D", "--defined-only", str(teleop_library)).splitlines()
        if line.split()
    }
    missing_exports = [symbol for symbol in _REQUIRED_SYMBOLS if symbol not in symbols]
    if missing_exports:
        raise ValueError(f"x_air teleoperation library is missing exports: {missing_exports}")
    header_text = header.read_text(encoding="utf-8")
    missing = [symbol for symbol in _REQUIRED_SYMBOLS if symbol not in header_text]
    if missing:
        raise ValueError(f"x_air public header is missing symbols: {missing}")
    version_match = _PACKAGE_VERSION.search(package_manifest.read_text(encoding="utf-8"))
    if version_match is None or version_match.group(1) != teleop.sdk_version:
        actual = None if version_match is None else version_match.group(1)
        raise ValueError(f"x_air SDK version mismatch: expected {teleop.sdk_version}, got {actual}")
    _verify_vendor_profile(teleop_root / "config/leader.yaml", config.control["leader"])
    _verify_vendor_profile(teleop_root / "config/follower.yaml", config.control["follower"])
    return XAirDependencyReport(
        revision=revision,
        architecture=architecture,
        sdk_version=teleop.sdk_version,
        teleop_library=teleop_library,
        teleop_library_sha256=_sha256(teleop_library),
        can_library=can_library,
        can_library_sha256=_sha256(can_library),
    )


def _render_profile(table: str, profile: ControlProfile) -> str:
    lines = [f"{table}:"]
    for yaml_name, attribute in _PROFILE_FIELDS.items():
        values = getattr(profile, attribute)
        rendered = ", ".join(repr(value) for value in values)
        lines.append(f"  {yaml_name}: [{rendered}]")
    return "\n".join(lines) + "\n"


def _verify_vendor_profile(path: Path, expected: ControlProfile) -> None:
    actual: dict[str, tuple[float, ...]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = _YAML_VECTOR.match(line)
        if match is None:
            continue
        actual[_PROFILE_FIELDS[match.group(1)]] = tuple(
            float(value.strip()) for value in match.group(2).split(",")
        )
    expected_fields = set(_PROFILE_FIELDS.values())
    if set(actual) != expected_fields:
        raise ValueError(f"x_air control profile is incomplete: {path}")
    for field in expected_fields:
        if actual[field] != getattr(expected, field):
            raise ValueError(f"x_air {path.name} {field} differs from the System config")


def _git_output(root: Path, *arguments: str) -> str:
    return _command_output("git", "-C", str(root), *arguments).strip()


def _command_output(*arguments: str) -> str:
    try:
        result = subprocess.run(
            arguments,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise ValueError(f"cannot inspect x_air SDK dependency: {exc}") from exc
    return result.stdout


def _verify_elf(path: Path, *, expected_machine: int) -> None:
    header = path.read_bytes()[:20]
    if len(header) != 20 or header[:4] != b"\x7fELF" or header[4:6] != b"\x02\x01":
        raise ValueError(f"x_air library is not a 64-bit little-endian ELF: {path}")
    if struct.unpack_from("<H", header, 18)[0] != expected_machine:
        raise ValueError(f"x_air library architecture does not match this host: {path}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _monotonic_seconds() -> float:
    return time.monotonic()


def _require_timeout(value: object) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
    ):
        raise ValueError("timeout_s must be finite and non-negative")
