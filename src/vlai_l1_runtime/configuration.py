"""Strict, hardware-free loading for the tracked VLAI L1 system contract."""

from __future__ import annotations

import ctypes
import hashlib
import hmac
import math
import os
import stat
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised by the Python 3.10 check
    import tomli as tomllib

MOTOR_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "gripper",
)
MOTOR_TYPE_CODES = MappingProxyType(
    {
        "DM4310": 0,
        "DM4340": 1,
        "DM6006": 2,
        "DM8006": 3,
        "DM8009": 4,
        "DM10010L": 5,
        "DM10010": 6,
        "DM1015": 7,
        "DMH3510": 8,
        "DM_J4310_2EC": 9,
    }
)
SIDES = ("left", "right")
ROLES = ("leader", "follower")
CAMERA_ROLES = ("wrist_left", "wrist_right", "agent")
CAMERA_DRIVERS = ("v4l2", "unassigned")
_LOADER_VALIDATION_TOKEN = object()
_MAX_CONFIG_BYTES = 1_048_576
_LOCAL_CONFIG_FILESYSTEM_TYPES = frozenset(
    {
        0x01021994,  # tmpfs
        0x2FC12FC1,  # zfs
        0x3153464A,  # jfs
        0x52654973,  # reiserfs
        0x58465342,  # xfs
        0x794C7630,  # overlayfs
        0x9123683E,  # btrfs
        0xE0F5E1E2,  # erofs
        0xEF53,  # ext2/3/4
        0xF2F52010,  # f2fs
    }
)


class ConfigError(ValueError):
    """Tracked configuration is incomplete, ambiguous, or unsafe."""


@dataclass(frozen=True)
class SafetyConfig:
    command_ready: bool
    production_source_available: bool
    j2_coordinate_verified: bool
    follower_right_bus_stability_verified: bool
    first_command_hold_tolerance_deg: float
    session_liveness_timeout_s: float
    command_inactivity_timeout_s: float

    @property
    def blockers(self) -> tuple[str, ...]:
        gates = (
            ("production_source_unavailable", self.production_source_available),
            ("j2_coordinate_unverified", self.j2_coordinate_verified),
            (
                "follower_right_bus_stability_unverified",
                self.follower_right_bus_stability_verified,
            ),
        )
        return tuple(name for name, ready in gates if not ready)


@dataclass(frozen=True)
class CanEndpointConfig:
    endpoint_id: str
    interface: str
    parentdev: str
    side: str
    role: str


@dataclass(frozen=True)
class CanConfig:
    fd: bool
    nominal_bitrate: int
    data_bitrate: int
    restart_ms: int
    tx_queue_length: int
    endpoints: tuple[CanEndpointConfig, ...]


@dataclass(frozen=True)
class MotorConfig:
    name: str
    send_id: int
    receive_id: int
    motor_type: str


@dataclass(frozen=True)
class ControlProfile:
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    fc: tuple[float, ...]
    friction_k: tuple[float, ...]
    fv: tuple[float, ...]
    fo: tuple[float, ...]


@dataclass(frozen=True)
class ImageRoi:
    x: int
    y: int
    width: int
    height: int

    @property
    def xywh(self) -> tuple[int, int, int, int]:
        return (self.x, self.y, self.width, self.height)

    def validate(self, *, image_width: int, image_height: int, label: str) -> None:
        if self.x < 0 or self.y < 0:
            raise ConfigError(f"{label} x/y must be non-negative")
        if self.width <= 0 or self.height <= 0:
            raise ConfigError(f"{label} width/height must be positive")
        if self.x + self.width > image_width or self.y + self.height > image_height:
            raise ConfigError(
                f"{label} {self.xywh} exceeds source image {image_width}x{image_height}"
            )


@dataclass(frozen=True)
class CameraConfig:
    role: str
    required_for_collection: bool
    enabled: bool
    width: int
    height: int
    fps: int
    driver: str
    device_id: str | None
    video_index: int | None = None
    crop: ImageRoi | None = None

    def __post_init__(self) -> None:
        if self.role not in CAMERA_ROLES:
            raise ConfigError(f"unknown camera role: {self.role!r}")
        if not isinstance(self.required_for_collection, bool):
            raise ConfigError(f"camera {self.role} required_for_collection must be a boolean")
        if not isinstance(self.enabled, bool):
            raise ConfigError(f"camera {self.role} enabled must be a boolean")
        for label, value in (
            ("width", self.width),
            ("height", self.height),
            ("fps", self.fps),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ConfigError(f"camera {self.role} {label} must be a positive integer")
        if self.driver not in CAMERA_DRIVERS:
            raise ConfigError(f"camera {self.role} has unknown driver: {self.driver!r}")
        if self.device_id is not None and (
            not isinstance(self.device_id, str)
            or not self.device_id
            or self.device_id != self.device_id.strip()
        ):
            raise ConfigError(f"camera {self.role} device_id must be normalized text")
        if self.enabled and not self.device_id:
            raise ConfigError(f"camera {self.role} requires a device_id when enabled")
        if self.enabled and self.driver == "unassigned":
            raise ConfigError(f"camera {self.role} requires an assigned driver when enabled")
        if not self.enabled and self.device_id is not None:
            raise ConfigError(f"camera {self.role} device_id must be absent while disabled")
        if self.video_index is not None and (
            isinstance(self.video_index, bool)
            or not isinstance(self.video_index, int)
            or self.video_index < 0
        ):
            raise ConfigError(f"camera {self.role} video_index must be a non-negative integer")
        if self.enabled and self.driver == "v4l2" and self.video_index is None:
            raise ConfigError(f"camera {self.role} requires a video_index for v4l2")
        if self.driver != "v4l2" and self.video_index is not None:
            raise ConfigError(f"camera {self.role} video_index is only valid for v4l2")
        if self.crop is not None:
            if not isinstance(self.crop, ImageRoi):
                raise ConfigError(f"camera {self.role} crop must be an ImageRoi")
            self.crop.validate(
                image_width=self.width,
                image_height=self.height,
                label=f"camera {self.role} crop",
            )
            if self.role == "agent" and self.crop.width != self.crop.height:
                raise ConfigError(
                    f"camera agent crop must be square, got {self.crop.width}x{self.crop.height}"
                )


@dataclass(frozen=True)
class CamerasConfig:
    max_age_s: float
    max_pair_skew_s: float
    startup_timeout_s: float
    streams: tuple[CameraConfig, ...]

    def __post_init__(self) -> None:
        for label, value in (
            ("max_age_s", self.max_age_s),
            ("max_pair_skew_s", self.max_pair_skew_s),
            ("startup_timeout_s", self.startup_timeout_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ConfigError(f"camera {label} must be finite and positive")
        if self.max_pair_skew_s > self.max_age_s:
            raise ConfigError("camera max_pair_skew_s must not exceed max_age_s")
        if tuple(stream.role for stream in self.streams) != CAMERA_ROLES:
            raise ConfigError("camera streams must define both wrists and agent exactly once")
        if not any(stream.required_for_collection for stream in self.streams):
            raise ConfigError("at least one camera must be required for collection")
        device_ids = [stream.device_id for stream in self.streams if stream.enabled]
        if len(device_ids) != len(set(device_ids)):
            raise ConfigError("enabled camera streams must have unique device identities")

    @property
    def collection_ready(self) -> bool:
        return all(stream.enabled for stream in self.streams if stream.required_for_collection)


@dataclass(frozen=True)
class RuntimeConfig:
    transport: str
    socket_path: Path


@dataclass(frozen=True)
class JointSafetySideConfig:
    min_deg: tuple[float, ...]
    max_deg: tuple[float, ...]
    max_following_error_deg: tuple[float, ...]


@dataclass(frozen=True)
class TeleoperationJointSafetyConfig:
    following_error_timeout_s: float
    following_error_action: str
    sides: Mapping[str, JointSafetySideConfig]

    def for_side(self, side: str) -> JointSafetySideConfig:
        try:
            return self.sides[side]
        except KeyError as exc:
            raise ValueError(f"unknown teleoperation side: {side!r}") from exc


@dataclass(frozen=True)
class TeleoperationConfig:
    provider: str
    mode: str
    arm_type: str
    sdk_version: str
    source_revision: str
    source_root: Path
    state_protocol_version: int
    state_socket_path: Path
    publish_hz: int
    state_timeout_s: float
    max_side_skew_s: float
    rt_priority: int
    can_health_poll_s: float
    startup_timeout_s: float
    shutdown_timeout_s: float
    motor_probe_duration_s: float
    motor_probe_rate_hz: int
    joint_safety: TeleoperationJointSafetyConfig
    commissioned: bool

    @property
    def blockers(self) -> tuple[str, ...]:
        return () if self.commissioned else ("teleoperation_uncommissioned",)


@dataclass(frozen=True)
class OperatorPanelConfig:
    bind: str
    port: int


@dataclass(frozen=True)
class CameraPreviewConfig:
    bind: str
    port: int
    fps: int
    jpeg_quality: int
    max_age_s: float
    bridge_socket_path: Path
    startup_timeout_s: float
    shutdown_timeout_s: float


@dataclass(frozen=True)
class LifecycleConfig:
    startup: str
    can_setup_service: str
    cpu_performance_service: str
    left_teleop_service: str
    right_teleop_service: str


@dataclass(frozen=True)
class SystemConfig:
    path: Path
    schema_version: int
    robot_id: str
    topology_id: str
    position_unit: str
    safety: SafetyConfig
    can: CanConfig
    motors: tuple[MotorConfig, ...]
    control: Mapping[str, ControlProfile]
    cameras: CamerasConfig
    teleoperation: TeleoperationConfig
    runtime: RuntimeConfig
    operator_panel: OperatorPanelConfig
    camera_preview: CameraPreviewConfig
    lifecycle: LifecycleConfig
    _validation_token: object | None = field(default=None, repr=False, compare=False)
    _validation_fingerprint: str = field(default="", repr=False, compare=False)

    @property
    def command_blockers(self) -> tuple[str, ...]:
        return _command_blockers(self.safety, self.runtime)


def load_system_config(path: Path) -> SystemConfig:
    """Load and exhaustively validate one tracked System TOML file."""

    resolved = Path(os.path.abspath(os.fspath(path)))
    try:
        content = _read_local_regular_file(resolved)
        raw = tomllib.loads(content.decode("utf-8"))
    except ConfigError:
        raise
    except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError) as exc:
        raise ConfigError(f"cannot load system config {resolved}: {exc}") from exc

    root = _mapping(raw, "root")
    _exact_keys(
        root,
        {
            "schema_version",
            "robot_id",
            "topology_id",
            "position_unit",
            "safety",
            "can",
            "motors",
            "control",
            "cameras",
            "teleoperation",
            "runtime",
            "operator_panel",
            "camera_preview",
            "lifecycle",
        },
        "root",
    )

    schema_version = _integer(root["schema_version"], "schema_version", minimum=1)
    if schema_version != 4:
        raise ConfigError(f"unsupported schema_version: {schema_version}")
    robot_id = _text(root["robot_id"], "robot_id")
    topology_id = _text(root["topology_id"], "topology_id")
    position_unit = _text(root["position_unit"], "position_unit")
    if position_unit != "degree":
        raise ConfigError("position_unit must remain truthful hardware degrees")

    safety = _parse_safety(root["safety"])
    can = _parse_can(root["can"])
    motors = _parse_motors(root["motors"])
    control = _parse_control(root["control"], len(motors))
    cameras = _parse_cameras(root["cameras"])
    teleoperation = _parse_teleoperation(root["teleoperation"], config_path=resolved)
    runtime = _parse_runtime(root["runtime"])
    operator_panel = _parse_operator_panel(root["operator_panel"])
    camera_preview = _parse_camera_preview(root["camera_preview"])
    if camera_preview.port == operator_panel.port:
        raise ConfigError("camera_preview.port must differ from operator_panel.port")
    lifecycle = _parse_lifecycle(root["lifecycle"])

    blockers = _command_blockers(safety, runtime)
    readiness = not blockers
    if safety.command_ready != readiness:
        raise ConfigError("command_ready must equal all independent readiness gates")
    if runtime.transport != "unimplemented" or safety.command_ready:
        raise ConfigError(
            "system schema version 4 cannot enable the unimplemented command transport"
        )

    config = SystemConfig(
        path=resolved,
        schema_version=schema_version,
        robot_id=robot_id,
        topology_id=topology_id,
        position_unit=position_unit,
        safety=safety,
        can=can,
        motors=motors,
        control=MappingProxyType(dict(control)),
        cameras=cameras,
        teleoperation=teleoperation,
        runtime=runtime,
        operator_panel=operator_panel,
        camera_preview=camera_preview,
        lifecycle=lifecycle,
    )
    object.__setattr__(config, "_validation_token", _LOADER_VALIDATION_TOKEN)
    object.__setattr__(config, "_validation_fingerprint", _system_config_fingerprint(config))
    return config


def _read_local_regular_file(path: Path, *, label: str = "system config") -> bytes:
    """Read one bounded local file while refusing every symbolic-link component."""

    directory_fd = os.open("/", os.O_PATH | os.O_DIRECTORY | os.O_CLOEXEC)
    path_fd: int | None = None
    try:
        for component in path.parts[1:-1]:
            next_fd = os.open(
                component,
                os.O_PATH | os.O_CLOEXEC | os.O_NOFOLLOW,
                dir_fd=directory_fd,
            )
            component_stat = os.fstat(next_fd)
            if stat.S_ISLNK(component_stat.st_mode):
                os.close(next_fd)
                raise ConfigError(f"{label} path contains a symbolic link: {path}")
            if not stat.S_ISDIR(component_stat.st_mode):
                os.close(next_fd)
                raise ConfigError(f"{label} ancestor is not a directory: {path}")
            os.close(directory_fd)
            directory_fd = next_fd

        path_fd = os.open(
            path.parts[-1],
            os.O_PATH | os.O_CLOEXEC | os.O_NOFOLLOW,
            dir_fd=directory_fd,
        )
        before = os.fstat(path_fd)
        if stat.S_ISLNK(before.st_mode):
            raise ConfigError(f"{label} path contains a symbolic link: {path}")
        if not stat.S_ISREG(before.st_mode):
            raise ConfigError(f"{label} is not a regular file: {path}")
        if before.st_size > _MAX_CONFIG_BYTES:
            raise ConfigError(f"{label} exceeds {_MAX_CONFIG_BYTES} bytes: {path}")
        filesystem_type = _filesystem_type(path_fd)
        if filesystem_type not in _LOCAL_CONFIG_FILESYSTEM_TYPES:
            raise ConfigError(f"{label} is not on a trusted local filesystem: {path}")

        read_fd = os.open(f"/proc/self/fd/{path_fd}", os.O_RDONLY | os.O_CLOEXEC)
        with os.fdopen(read_fd, "rb") as handle:
            content = handle.read(_MAX_CONFIG_BYTES + 1)
            after = os.fstat(handle.fileno())
        if len(content) > _MAX_CONFIG_BYTES:
            raise ConfigError(f"{label} exceeds {_MAX_CONFIG_BYTES} bytes: {path}")
        identity_before = (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
            before.st_ctime_ns,
        )
        identity_after = (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
            after.st_ctime_ns,
        )
        if identity_after != identity_before:
            raise ConfigError(f"{label} changed while it was being read: {path}")
        return content
    finally:
        if path_fd is not None:
            os.close(path_fd)
        os.close(directory_fd)


def _filesystem_type(file_descriptor: int) -> int:
    """Return Linux statfs.f_type without opening a second path."""

    buffer = ctypes.create_string_buffer(256)
    libc = ctypes.CDLL(None, use_errno=True)
    fstatfs = libc.fstatfs
    fstatfs.argtypes = (ctypes.c_int, ctypes.c_void_p)
    fstatfs.restype = ctypes.c_int
    if fstatfs(file_descriptor, ctypes.byref(buffer)) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))
    return ctypes.c_long.from_buffer(buffer).value


def _parse_safety(value: Any) -> SafetyConfig:
    raw = _mapping(value, "safety")
    keys = {
        "command_ready",
        "production_source_available",
        "j2_coordinate_verified",
        "follower_right_bus_stability_verified",
        "first_command_hold_tolerance_deg",
        "session_liveness_timeout_s",
        "command_inactivity_timeout_s",
    }
    _exact_keys(raw, keys, "safety")
    return SafetyConfig(
        command_ready=_boolean(raw["command_ready"], "safety.command_ready"),
        production_source_available=_boolean(
            raw["production_source_available"], "safety.production_source_available"
        ),
        j2_coordinate_verified=_boolean(
            raw["j2_coordinate_verified"], "safety.j2_coordinate_verified"
        ),
        follower_right_bus_stability_verified=_boolean(
            raw["follower_right_bus_stability_verified"],
            "safety.follower_right_bus_stability_verified",
        ),
        first_command_hold_tolerance_deg=_positive_number(
            raw["first_command_hold_tolerance_deg"],
            "safety.first_command_hold_tolerance_deg",
        ),
        session_liveness_timeout_s=_positive_number(
            raw["session_liveness_timeout_s"], "safety.session_liveness_timeout_s"
        ),
        command_inactivity_timeout_s=_positive_number(
            raw["command_inactivity_timeout_s"],
            "safety.command_inactivity_timeout_s",
        ),
    )


def _command_blockers(safety: SafetyConfig, runtime: RuntimeConfig) -> tuple[str, ...]:
    transport = () if runtime.transport == "implemented" else ("command_transport_unimplemented",)
    return (*transport, *safety.blockers)


def _parse_can(value: Any) -> CanConfig:
    raw = _mapping(value, "can")
    _exact_keys(
        raw,
        {
            "fd",
            "nominal_bitrate",
            "data_bitrate",
            "restart_ms",
            "tx_queue_length",
            "endpoints",
        },
        "can",
    )
    fd = _boolean(raw["fd"], "can.fd")
    nominal = _integer(raw["nominal_bitrate"], "can.nominal_bitrate", minimum=1)
    data = _integer(raw["data_bitrate"], "can.data_bitrate", minimum=1)
    restart_ms = _integer(raw["restart_ms"], "can.restart_ms", minimum=1)
    tx_queue_length = _integer(raw["tx_queue_length"], "can.tx_queue_length", minimum=1)
    if fd and data < nominal:
        raise ConfigError("CAN-FD data bitrate must not be lower than the nominal bitrate")

    items = _list(raw["endpoints"], "can.endpoints")
    endpoints: list[CanEndpointConfig] = []
    for index, item in enumerate(items):
        label = f"can.endpoints[{index}]"
        table = _mapping(item, label)
        _exact_keys(table, {"id", "interface", "parentdev", "side", "role"}, label)
        endpoint = CanEndpointConfig(
            endpoint_id=_text(table["id"], f"{label}.id"),
            interface=_text(table["interface"], f"{label}.interface"),
            parentdev=_text(table["parentdev"], f"{label}.parentdev"),
            side=_choice(table["side"], SIDES, f"{label}.side"),
            role=_choice(table["role"], ROLES, f"{label}.role"),
        )
        endpoints.append(endpoint)

    if len(endpoints) != 4:
        raise ConfigError("the verified topology requires exactly four CAN endpoints")
    if len({item.interface for item in endpoints}) != 4:
        raise ConfigError("CAN interfaces must be unique")
    if len({item.parentdev for item in endpoints}) != 4:
        raise ConfigError("CAN parentdev identities must be unique")
    if len({item.endpoint_id for item in endpoints}) != 4:
        raise ConfigError("CAN endpoint ids must be unique")
    expected_roles = {(side, role) for side in SIDES for role in ROLES}
    if {(item.side, item.role) for item in endpoints} != expected_roles:
        raise ConfigError("CAN endpoints must cover each side and role exactly once")
    return CanConfig(fd, nominal, data, restart_ms, tx_queue_length, tuple(endpoints))


def _parse_motors(value: Any) -> tuple[MotorConfig, ...]:
    items = _list(value, "motors")
    motors: list[MotorConfig] = []
    for index, item in enumerate(items):
        label = f"motors[{index}]"
        raw = _mapping(item, label)
        _exact_keys(raw, {"name", "send_id", "receive_id", "motor_type"}, label)
        motors.append(
            MotorConfig(
                name=_text(raw["name"], f"{label}.name"),
                send_id=_integer(raw["send_id"], f"{label}.send_id", minimum=1, maximum=0x7FF),
                receive_id=_integer(
                    raw["receive_id"], f"{label}.receive_id", minimum=1, maximum=0x7FF
                ),
                motor_type=_choice(
                    raw["motor_type"], tuple(MOTOR_TYPE_CODES), f"{label}.motor_type"
                ),
            )
        )
    if tuple(item.name for item in motors) != MOTOR_NAMES:
        raise ConfigError("motors must define joint_1..joint_7 and gripper in canonical order")
    if len({motor.send_id for motor in motors}) != len(motors):
        raise ConfigError("motor send ids must be unique")
    if len({motor.receive_id for motor in motors}) != len(motors):
        raise ConfigError("motor receive ids must be unique")
    if {motor.send_id for motor in motors} & {motor.receive_id for motor in motors}:
        raise ConfigError("motor send and receive id spaces must not overlap")
    return tuple(motors)


def _parse_control(value: Any, motor_count: int) -> Mapping[str, ControlProfile]:
    raw = _mapping(value, "control")
    _exact_keys(raw, set(ROLES), "control")
    result: dict[str, ControlProfile] = {}
    for role in ROLES:
        table = _mapping(raw[role], f"control.{role}")
        _exact_keys(table, {"kp", "kd", "fc", "friction_k", "fv", "fo"}, f"control.{role}")
        kp = _number_vector(table["kp"], f"control.{role}.kp", motor_count, positive=True)
        kd = _number_vector(table["kd"], f"control.{role}.kd", motor_count, positive=True)
        fc = _number_vector(table["fc"], f"control.{role}.fc", motor_count, positive=False)
        friction_k = _number_vector(
            table["friction_k"], f"control.{role}.friction_k", motor_count, positive=True
        )
        fv = _number_vector(table["fv"], f"control.{role}.fv", motor_count, positive=False)
        fo = _number_vector(table["fo"], f"control.{role}.fo", motor_count, positive=False)
        result[role] = ControlProfile(kp, kd, fc, friction_k, fv, fo)
    return result


def _parse_cameras(value: Any) -> CamerasConfig:
    raw = _mapping(value, "cameras")
    _exact_keys(
        raw,
        {"max_age_s", "max_pair_skew_s", "startup_timeout_s", *CAMERA_ROLES},
        "cameras",
    )
    max_age = _positive_number(raw["max_age_s"], "cameras.max_age_s")
    max_skew = _positive_number(raw["max_pair_skew_s"], "cameras.max_pair_skew_s")
    startup_timeout = _positive_number(raw["startup_timeout_s"], "cameras.startup_timeout_s")
    streams: list[CameraConfig] = []
    for role in CAMERA_ROLES:
        label = f"cameras.{role}"
        table = _mapping(raw[role], label)
        _allowed_keys(
            table,
            {
                "required_for_collection",
                "enabled",
                "width",
                "height",
                "fps",
                "driver",
                "crop_enabled",
            },
            {
                "device_id",
                "video_index",
                "crop_x",
                "crop_y",
                "crop_width",
                "crop_height",
            },
            label,
        )
        enabled = _boolean(table["enabled"], f"{label}.enabled")
        crop_enabled = _boolean(table["crop_enabled"], f"{label}.crop_enabled")
        crop_keys = {"crop_x", "crop_y", "crop_width", "crop_height"}
        present_crop_keys = crop_keys & set(table)
        if crop_enabled and present_crop_keys != crop_keys:
            missing = sorted(crop_keys - present_crop_keys)
            raise ConfigError(f"{label} is missing crop keys: {missing}")
        if not crop_enabled and present_crop_keys:
            raise ConfigError(f"{label} crop coordinates require crop_enabled = true")
        device = table.get("device_id")
        if enabled and device is None:
            raise ConfigError(f"{label}.device_id is required when enabled")
        if not enabled and device is not None:
            raise ConfigError(f"{label}.device_id must be absent while disabled")
        streams.append(
            CameraConfig(
                role=role,
                required_for_collection=_boolean(
                    table["required_for_collection"], f"{label}.required_for_collection"
                ),
                enabled=enabled,
                width=_integer(table["width"], f"{label}.width", minimum=1),
                height=_integer(table["height"], f"{label}.height", minimum=1),
                fps=_integer(table["fps"], f"{label}.fps", minimum=1),
                driver=_choice(table["driver"], CAMERA_DRIVERS, f"{label}.driver"),
                device_id=None if device is None else _text(device, f"{label}.device_id"),
                video_index=(
                    None
                    if "video_index" not in table
                    else _integer(table["video_index"], f"{label}.video_index", minimum=0)
                ),
                crop=(
                    None
                    if not crop_enabled
                    else ImageRoi(
                        x=_integer(table["crop_x"], f"{label}.crop_x", minimum=0),
                        y=_integer(table["crop_y"], f"{label}.crop_y", minimum=0),
                        width=_integer(table["crop_width"], f"{label}.crop_width", minimum=1),
                        height=_integer(table["crop_height"], f"{label}.crop_height", minimum=1),
                    )
                ),
            )
        )
    return CamerasConfig(max_age, max_skew, startup_timeout, tuple(streams))


def _parse_runtime(value: Any) -> RuntimeConfig:
    raw = _mapping(value, "runtime")
    _exact_keys(raw, {"transport", "socket_path"}, "runtime")
    transport = _choice(raw["transport"], ("unimplemented",), "runtime.transport")
    socket_path = Path(_text(raw["socket_path"], "runtime.socket_path"))
    if not socket_path.is_absolute():
        raise ConfigError("runtime.socket_path must be absolute")
    return RuntimeConfig(transport, socket_path)


def _parse_teleoperation(value: Any, *, config_path: Path) -> TeleoperationConfig:
    raw = _mapping(value, "teleoperation")
    keys = {
        "provider",
        "mode",
        "arm_type",
        "sdk_version",
        "source_revision",
        "source_root",
        "state_protocol_version",
        "state_socket_path",
        "publish_hz",
        "state_timeout_s",
        "max_side_skew_s",
        "rt_priority",
        "can_health_poll_s",
        "startup_timeout_s",
        "shutdown_timeout_s",
        "motor_probe_duration_s",
        "motor_probe_rate_hz",
        "joint_safety",
        "commissioned",
    }
    _exact_keys(raw, keys, "teleoperation")
    revision = _text(raw["source_revision"], "teleoperation.source_revision")
    if len(revision) != 40 or any(character not in "0123456789abcdef" for character in revision):
        raise ConfigError("teleoperation.source_revision must be a lowercase full Git commit")
    source_root = Path(_text(raw["source_root"], "teleoperation.source_root"))
    if source_root.is_absolute():
        raise ConfigError("teleoperation.source_root must be relative to the System config")
    source_root = Path(os.path.abspath(config_path.parent / source_root))
    state_socket_path = Path(_text(raw["state_socket_path"], "teleoperation.state_socket_path"))
    if not state_socket_path.is_absolute():
        raise ConfigError("teleoperation.state_socket_path must be absolute")
    state_protocol_version = _integer(
        raw["state_protocol_version"], "teleoperation.state_protocol_version", minimum=1
    )
    if state_protocol_version != 1:
        raise ConfigError("unsupported teleoperation.state_protocol_version")
    state_timeout_s = _positive_number(raw["state_timeout_s"], "teleoperation.state_timeout_s")
    can_health_poll_s = _positive_number(
        raw["can_health_poll_s"], "teleoperation.can_health_poll_s"
    )
    startup_timeout_s = _positive_number(
        raw["startup_timeout_s"], "teleoperation.startup_timeout_s"
    )
    shutdown_timeout_s = _positive_number(
        raw["shutdown_timeout_s"], "teleoperation.shutdown_timeout_s"
    )
    motor_probe_duration_s = _positive_number(
        raw["motor_probe_duration_s"], "teleoperation.motor_probe_duration_s"
    )
    motor_probe_rate_hz = _integer(
        raw["motor_probe_rate_hz"],
        "teleoperation.motor_probe_rate_hz",
        minimum=1,
        maximum=100,
    )
    if 2 * motor_probe_duration_s >= startup_timeout_s:
        raise ConfigError("two teleoperation motor probes must fit within startup_timeout_s")
    probe_rounds = motor_probe_duration_s * motor_probe_rate_hz
    if not probe_rounds.is_integer():
        raise ConfigError(
            "teleoperation.motor_probe_duration_s and motor_probe_rate_hz "
            "must resolve to a whole number of rounds"
        )
    if can_health_poll_s > state_timeout_s:
        raise ConfigError("teleoperation.can_health_poll_s must not exceed state_timeout_s")
    joint_safety = _parse_teleoperation_joint_safety(raw["joint_safety"])
    if joint_safety.following_error_timeout_s > state_timeout_s:
        raise ConfigError(
            "teleoperation.joint_safety.following_error_timeout_s must not exceed state_timeout_s"
        )
    for label, seconds in (
        ("state_timeout_s", state_timeout_s),
        ("can_health_poll_s", can_health_poll_s),
        (
            "joint_safety.following_error_timeout_s",
            joint_safety.following_error_timeout_s,
        ),
    ):
        if not (seconds * 1000).is_integer():
            raise ConfigError(f"teleoperation.{label} must resolve to whole milliseconds")
    return TeleoperationConfig(
        provider=_choice(raw["provider"], ("x_air_sdk",), "teleoperation.provider"),
        mode=_choice(raw["mode"], ("unilateral",), "teleoperation.mode"),
        arm_type=_choice(raw["arm_type"], ("v10",), "teleoperation.arm_type"),
        sdk_version=_text(raw["sdk_version"], "teleoperation.sdk_version"),
        source_revision=revision,
        source_root=source_root,
        state_protocol_version=state_protocol_version,
        state_socket_path=state_socket_path,
        publish_hz=_integer(raw["publish_hz"], "teleoperation.publish_hz", minimum=1),
        state_timeout_s=state_timeout_s,
        max_side_skew_s=_positive_number(raw["max_side_skew_s"], "teleoperation.max_side_skew_s"),
        rt_priority=_integer(
            raw["rt_priority"], "teleoperation.rt_priority", minimum=1, maximum=99
        ),
        can_health_poll_s=can_health_poll_s,
        startup_timeout_s=startup_timeout_s,
        shutdown_timeout_s=shutdown_timeout_s,
        motor_probe_duration_s=motor_probe_duration_s,
        motor_probe_rate_hz=motor_probe_rate_hz,
        joint_safety=joint_safety,
        commissioned=_boolean(raw["commissioned"], "teleoperation.commissioned"),
    )


def _parse_teleoperation_joint_safety(value: Any) -> TeleoperationJointSafetyConfig:
    label = "teleoperation.joint_safety"
    raw = _mapping(value, label)
    _exact_keys(raw, {"following_error_timeout_s", "following_error_action", *SIDES}, label)
    side_configs: dict[str, JointSafetySideConfig] = {}
    for side in SIDES:
        side_label = f"{label}.{side}"
        side_raw = _mapping(raw[side], side_label)
        _exact_keys(
            side_raw,
            {"min_deg", "max_deg", "max_following_error_deg"},
            side_label,
        )
        minimum = _number_vector(side_raw["min_deg"], f"{side_label}.min_deg", 7, positive=False)
        maximum = _number_vector(side_raw["max_deg"], f"{side_label}.max_deg", 7, positive=False)
        following = _number_vector(
            side_raw["max_following_error_deg"],
            f"{side_label}.max_following_error_deg",
            7,
            positive=True,
        )
        for index, (lower, upper) in enumerate(zip(minimum, maximum, strict=True), start=1):
            if lower >= upper:
                raise ConfigError(f"{side_label} joint_{index} min_deg must be less than max_deg")
        side_configs[side] = JointSafetySideConfig(minimum, maximum, following)
    return TeleoperationJointSafetyConfig(
        following_error_timeout_s=_positive_number(
            raw["following_error_timeout_s"],
            f"{label}.following_error_timeout_s",
        ),
        following_error_action=_choice(
            raw["following_error_action"],
            ("stop", "warn"),
            f"{label}.following_error_action",
        ),
        sides=MappingProxyType(side_configs),
    )


def _parse_operator_panel(value: Any) -> OperatorPanelConfig:
    raw = _mapping(value, "operator_panel")
    _exact_keys(raw, {"bind", "port"}, "operator_panel")
    return OperatorPanelConfig(
        _text(raw["bind"], "operator_panel.bind"),
        _integer(raw["port"], "operator_panel.port", minimum=1, maximum=65_535),
    )


def _parse_camera_preview(value: Any) -> CameraPreviewConfig:
    raw = _mapping(value, "camera_preview")
    _exact_keys(
        raw,
        {
            "bind",
            "port",
            "fps",
            "jpeg_quality",
            "max_age_s",
            "bridge_socket_path",
            "startup_timeout_s",
            "shutdown_timeout_s",
        },
        "camera_preview",
    )
    bridge_socket_path = Path(_text(raw["bridge_socket_path"], "camera_preview.bridge_socket_path"))
    if not bridge_socket_path.is_absolute():
        raise ConfigError("camera_preview.bridge_socket_path must be absolute")
    return CameraPreviewConfig(
        bind=_text(raw["bind"], "camera_preview.bind"),
        port=_integer(raw["port"], "camera_preview.port", minimum=1, maximum=65_535),
        fps=_integer(raw["fps"], "camera_preview.fps", minimum=1, maximum=60),
        jpeg_quality=_integer(
            raw["jpeg_quality"],
            "camera_preview.jpeg_quality",
            minimum=1,
            maximum=100,
        ),
        max_age_s=_positive_number(raw["max_age_s"], "camera_preview.max_age_s"),
        bridge_socket_path=bridge_socket_path,
        startup_timeout_s=_positive_number(
            raw["startup_timeout_s"],
            "camera_preview.startup_timeout_s",
        ),
        shutdown_timeout_s=_positive_number(
            raw["shutdown_timeout_s"],
            "camera_preview.shutdown_timeout_s",
        ),
    )


def _parse_lifecycle(value: Any) -> LifecycleConfig:
    raw = _mapping(value, "lifecycle")
    keys = {
        "startup",
        "can_setup_service",
        "cpu_performance_service",
        "left_teleop_service",
        "right_teleop_service",
    }
    _exact_keys(raw, keys, "lifecycle")
    startup = _choice(raw["startup"], ("manual",), "lifecycle.startup")
    services = [_text(raw[key], f"lifecycle.{key}") for key in keys if key != "startup"]
    if len(set(services)) != len(services):
        raise ConfigError("lifecycle service names must be unique")
    return LifecycleConfig(
        startup=startup,
        can_setup_service=_text(raw["can_setup_service"], "lifecycle.can_setup_service"),
        cpu_performance_service=_text(
            raw["cpu_performance_service"], "lifecycle.cpu_performance_service"
        ),
        left_teleop_service=_text(raw["left_teleop_service"], "lifecycle.left_teleop_service"),
        right_teleop_service=_text(raw["right_teleop_service"], "lifecycle.right_teleop_service"),
    )


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ConfigError(f"{label} must be a table")
    return value


def _list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ConfigError(f"{label} must be an array")
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], label: str) -> None:
    _allowed_keys(value, expected, set(), label)


def _allowed_keys(
    value: Mapping[str, Any], required: set[str], optional: set[str], label: str
) -> None:
    missing = required - set(value)
    unknown = set(value) - required - optional
    if missing:
        raise ConfigError(f"{label} is missing keys: {sorted(missing)}")
    if unknown:
        raise ConfigError(f"{label} has unknown keys: {sorted(unknown)}")


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ConfigError(f"{label} must be non-empty text without surrounding whitespace")
    return value


def _choice(value: Any, choices: tuple[str, ...], label: str) -> str:
    result = _text(value, label)
    if result not in choices:
        raise ConfigError(f"{label} must be one of {choices}")
    return result


def _boolean(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"{label} must be a boolean")
    return value


def _integer(value: Any, label: str, *, minimum: int, maximum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{label} must be an integer")
    if value < minimum or (maximum is not None and value > maximum):
        raise ConfigError(f"{label} is outside the allowed range")
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{label} must be finite")
    return result


def _positive_number(value: Any, label: str) -> float:
    result = _number(value, label)
    if result <= 0:
        raise ConfigError(f"{label} must be positive")
    return result


def _number_vector(value: Any, label: str, length: int, *, positive: bool) -> tuple[float, ...]:
    values = _list(value, label)
    if len(values) != length:
        raise ConfigError(f"{label} must contain exactly {length} values")
    result = tuple(_number(item, f"{label}[{index}]") for index, item in enumerate(values))
    if positive and any(item <= 0 for item in result):
        raise ConfigError(f"{label} values must be positive")
    return result


def _is_loader_validated(config: SystemConfig) -> bool:
    fingerprint = config._validation_fingerprint
    return (
        config._validation_token is _LOADER_VALIDATION_TOKEN
        and bool(fingerprint)
        and hmac.compare_digest(fingerprint, _system_config_fingerprint(config))
    )


def _system_config_fingerprint(config: SystemConfig) -> str:
    payload = (
        str(config.path),
        config.schema_version,
        config.robot_id,
        config.topology_id,
        config.position_unit,
        (
            config.safety.command_ready,
            config.safety.production_source_available,
            config.safety.j2_coordinate_verified,
            config.safety.follower_right_bus_stability_verified,
            config.safety.first_command_hold_tolerance_deg,
            config.safety.session_liveness_timeout_s,
            config.safety.command_inactivity_timeout_s,
        ),
        (
            config.can.fd,
            config.can.nominal_bitrate,
            config.can.data_bitrate,
            config.can.restart_ms,
            config.can.tx_queue_length,
            tuple(
                (
                    endpoint.endpoint_id,
                    endpoint.interface,
                    endpoint.parentdev,
                    endpoint.side,
                    endpoint.role,
                )
                for endpoint in config.can.endpoints
            ),
        ),
        tuple(
            (motor.name, motor.send_id, motor.receive_id, motor.motor_type)
            for motor in config.motors
        ),
        tuple(
            (
                role,
                config.control[role].kp,
                config.control[role].kd,
                config.control[role].fc,
                config.control[role].friction_k,
                config.control[role].fv,
                config.control[role].fo,
            )
            for role in ROLES
        ),
        (
            config.cameras.max_age_s,
            config.cameras.max_pair_skew_s,
            config.cameras.startup_timeout_s,
            tuple(
                (
                    stream.role,
                    stream.required_for_collection,
                    stream.enabled,
                    stream.width,
                    stream.height,
                    stream.fps,
                    stream.driver,
                    stream.device_id,
                    stream.video_index,
                    None if stream.crop is None else stream.crop.xywh,
                )
                for stream in config.cameras.streams
            ),
        ),
        (
            config.teleoperation.provider,
            config.teleoperation.mode,
            config.teleoperation.arm_type,
            config.teleoperation.sdk_version,
            config.teleoperation.source_revision,
            str(config.teleoperation.source_root),
            config.teleoperation.state_protocol_version,
            str(config.teleoperation.state_socket_path),
            config.teleoperation.publish_hz,
            config.teleoperation.state_timeout_s,
            config.teleoperation.max_side_skew_s,
            config.teleoperation.rt_priority,
            config.teleoperation.can_health_poll_s,
            config.teleoperation.startup_timeout_s,
            config.teleoperation.shutdown_timeout_s,
            config.teleoperation.motor_probe_duration_s,
            config.teleoperation.motor_probe_rate_hz,
            config.teleoperation.joint_safety.following_error_timeout_s,
            config.teleoperation.joint_safety.following_error_action,
            tuple(
                (
                    side,
                    config.teleoperation.joint_safety.for_side(side).min_deg,
                    config.teleoperation.joint_safety.for_side(side).max_deg,
                    config.teleoperation.joint_safety.for_side(side).max_following_error_deg,
                )
                for side in SIDES
            ),
            config.teleoperation.commissioned,
        ),
        (config.runtime.transport, str(config.runtime.socket_path)),
        (config.operator_panel.bind, config.operator_panel.port),
        (
            config.camera_preview.bind,
            config.camera_preview.port,
            config.camera_preview.fps,
            config.camera_preview.jpeg_quality,
            config.camera_preview.max_age_s,
            str(config.camera_preview.bridge_socket_path),
            config.camera_preview.startup_timeout_s,
            config.camera_preview.shutdown_timeout_s,
        ),
        (
            config.lifecycle.startup,
            config.lifecycle.can_setup_service,
            config.lifecycle.cpu_performance_service,
            config.lifecycle.left_teleop_service,
            config.lifecycle.right_teleop_service,
        ),
    )
    return hashlib.sha256(repr(payload).encode()).hexdigest()
