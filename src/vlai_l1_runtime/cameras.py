"""Pure camera identity, freshness, and synchronization contracts."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from .configuration import CameraConfig, CamerasConfig, ConfigError


class CameraContractError(ValueError):
    """A camera sample set is incomplete, stale, or incoherent."""


@dataclass(frozen=True)
class CameraFrameMetadata:
    role: str
    device_id: str
    stream_epoch: str
    source_sequence: int
    monotonic_ns: int

    def __post_init__(self) -> None:
        for label, value in (
            ("role", self.role),
            ("device_id", self.device_id),
            ("stream_epoch", self.stream_epoch),
        ):
            if not isinstance(value, str) or not value or value != value.strip():
                raise CameraContractError(f"camera {label} must be normalized non-empty text")
        if (
            isinstance(self.source_sequence, bool)
            or not isinstance(self.source_sequence, int)
            or self.source_sequence < 0
        ):
            raise CameraContractError("camera source_sequence must be non-negative")
        if (
            isinstance(self.monotonic_ns, bool)
            or not isinstance(self.monotonic_ns, int)
            or self.monotonic_ns < 0
        ):
            raise CameraContractError("camera monotonic_ns must be non-negative")


@dataclass(frozen=True)
class _CameraContinuity:
    stream_epoch: str
    source_sequence: int
    monotonic_ns: int


class CameraSetValidator:
    """Own cross-frame continuity for one commissioned camera bridge."""

    def __init__(self, config: CamerasConfig) -> None:
        if not isinstance(config, CamerasConfig):
            raise TypeError("CameraSetValidator requires CamerasConfig")
        try:
            config_snapshot = CamerasConfig(
                config.max_age_s,
                config.max_pair_skew_s,
                config.startup_timeout_s,
                tuple(
                    CameraConfig(
                        stream.role,
                        stream.required_for_collection,
                        stream.enabled,
                        stream.width,
                        stream.height,
                        stream.fps,
                        stream.driver,
                        stream.device_id,
                        stream.video_index,
                        stream.crop,
                    )
                    for stream in config.streams
                ),
            )
        except (AttributeError, ConfigError) as exc:
            raise CameraContractError("camera configuration is malformed") from exc
        if not config_snapshot.collection_ready:
            raise CameraContractError(
                "all tracked camera roles must be commissioned for collection"
            )
        self._config = config_snapshot
        self._enabled = {
            stream.role: stream for stream in config_snapshot.streams if stream.enabled
        }
        self._previous: Mapping[str, _CameraContinuity] | None = None
        self._pending_restart_epochs: Mapping[str, str] | None = None

    def reset(self, *, restarted_epochs: Mapping[str, str]) -> None:
        """Declare the new epochs for streams deliberately restarted by the bridge."""

        if self._previous is None:
            raise CameraContractError("camera streams cannot reset before an accepted sample set")
        if self._pending_restart_epochs is not None:
            raise CameraContractError("camera stream restart is already pending")
        if not isinstance(restarted_epochs, Mapping) or not restarted_epochs:
            raise CameraContractError("restarted_epochs must name at least one camera role")
        if not all(isinstance(role, str) for role in restarted_epochs):
            raise CameraContractError("restarted camera roles must be text")
        unknown = set(restarted_epochs) - set(self._enabled)
        if unknown:
            raise CameraContractError(f"unknown restarted camera roles: {sorted(unknown)}")

        checked: dict[str, str] = {}
        for role, epoch in restarted_epochs.items():
            if not isinstance(epoch, str) or not epoch or epoch != epoch.strip():
                raise CameraContractError(f"{role} restart epoch must be normalized text")
            if epoch == self._previous[role].stream_epoch:
                raise CameraContractError(f"{role} restart epoch must differ from the active epoch")
            checked[role] = epoch
        self._pending_restart_epochs = MappingProxyType(checked)

    def validate(
        self,
        frames: Mapping[str, CameraFrameMetadata],
        *,
        now_ns: int,
    ) -> Mapping[str, CameraFrameMetadata]:
        if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
            raise CameraContractError("now_ns must be a non-negative integer")
        if not isinstance(frames, Mapping) or not all(isinstance(role, str) for role in frames):
            raise CameraContractError("camera frames must be a role-keyed mapping")
        if set(frames) != set(self._enabled):
            raise CameraContractError("camera set must contain every enabled role exactly once")

        timestamps: list[int] = []
        current: dict[str, CameraFrameMetadata] = {}
        max_age_ns = int(self._config.max_age_s * 1_000_000_000)
        for role, candidate in frames.items():
            frame = _snapshot_frame_metadata(candidate, role=role)
            _validate_frame_identity(frame, self._enabled[role])
            if frame.monotonic_ns > now_ns:
                raise CameraContractError(f"{role} frame timestamp is in the future")
            if now_ns - frame.monotonic_ns > max_age_ns:
                raise CameraContractError(f"{role} frame is stale")
            if self._previous is not None:
                prior = self._previous[role]
                restarted_epoch = (
                    None
                    if self._pending_restart_epochs is None
                    else self._pending_restart_epochs.get(role)
                )
                if restarted_epoch is not None:
                    if frame.stream_epoch != restarted_epoch:
                        raise CameraContractError(
                            f"{role} stream epoch does not match its declared restart epoch"
                        )
                else:
                    if frame.stream_epoch != prior.stream_epoch:
                        raise CameraContractError(
                            f"{role} stream epoch changed without explicit reset"
                        )
                    if frame.source_sequence <= prior.source_sequence:
                        raise CameraContractError(f"{role} source sequence did not increase")
                    if frame.monotonic_ns <= prior.monotonic_ns:
                        raise CameraContractError(f"{role} source timestamp did not increase")
            timestamps.append(frame.monotonic_ns)
            current[role] = frame

        max_skew_ns = int(self._config.max_pair_skew_s * 1_000_000_000)
        if max(timestamps) - min(timestamps) > max_skew_ns:
            earliest = min(current.values(), key=lambda frame: frame.monotonic_ns)
            latest = max(current.values(), key=lambda frame: frame.monotonic_ns)
            actual_ms = (latest.monotonic_ns - earliest.monotonic_ns) / 1_000_000
            raise CameraContractError(
                f"camera frame skew {actual_ms:.3f} ms between "
                f"{earliest.role} and {latest.role} exceeds the tracked limit"
            )
        self._previous = MappingProxyType(
            {
                role: _CameraContinuity(
                    frame.stream_epoch,
                    frame.source_sequence,
                    frame.monotonic_ns,
                )
                for role, frame in current.items()
            }
        )
        self._pending_restart_epochs = None
        return MappingProxyType(current)


def _snapshot_frame_metadata(candidate: object, *, role: str) -> CameraFrameMetadata:
    if not isinstance(candidate, CameraFrameMetadata):
        raise CameraContractError(f"{role} frame metadata has the wrong type")
    try:
        return CameraFrameMetadata(
            candidate.role,
            candidate.device_id,
            candidate.stream_epoch,
            candidate.source_sequence,
            candidate.monotonic_ns,
        )
    except AttributeError as exc:
        raise CameraContractError(f"{role} frame metadata is incomplete") from exc


def _validate_frame_identity(frame: CameraFrameMetadata, spec: CameraConfig) -> None:
    if frame.role != spec.role:
        raise CameraContractError(
            f"camera frame role {frame.role!r} does not match stream {spec.role!r}"
        )
    if frame.device_id != spec.device_id:
        raise CameraContractError(
            f"camera frame device {frame.device_id!r} does not match stream {spec.device_id!r}"
        )
