"""Passive inventory for the selected VLAI L1 collection hardware."""

from __future__ import annotations

import json
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path

from . import console
from .collection.configuration import CollectionConfig
from .teleoperation.lifecycle import _read_can_health, _require_can_identity


@dataclass(frozen=True)
class HardwareCheck:
    name: str
    level: str
    detail: str


def inspect_hardware(config: CollectionConfig) -> tuple[HardwareCheck, ...]:
    """Inspect configured devices without opening CAN or camera streams."""

    checks: list[HardwareCheck] = []
    selected_sides = set(config.teleoperation_sides)
    for endpoint in config.system.can.endpoints:
        if endpoint.side not in selected_sides:
            continue
        try:
            parent = _require_can_identity(config.system, endpoint.interface)
            health = _read_can_health(endpoint.interface)
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            checks.append(
                HardwareCheck(
                    f"{endpoint.side}_{endpoint.role}_can",
                    "FAIL",
                    str(exc),
                )
            )
            continue
        counters_ok = health.txerr == 0 and health.rxerr == 0
        level = "PASS" if counters_ok else "FAIL"
        checks.append(
            HardwareCheck(
                f"{endpoint.side}_{endpoint.role}_can",
                level,
                f"{endpoint.interface} parent={parent} state={health.state} "
                f"txerr={health.txerr} rxerr={health.rxerr}",
            )
        )

    by_id = Path("/dev/v4l/by-id")
    entries = tuple(by_id.glob("*")) if by_id.is_dir() else ()
    for stream in config.recording_camera_streams:
        device_id = stream.device_id or ""
        matches = tuple(path for path in entries if device_id and device_id in path.name)
        checks.append(
            HardwareCheck(
                f"camera_{stream.role}",
                "PASS" if matches else "FAIL",
                (
                    f"{device_id} -> {', '.join(str(path) for path in matches)}"
                    if matches
                    else f"configured device {device_id or '<unset>'} is not enumerated"
                ),
            )
        )

    for service in ("xarm-teleop-left.service", "xarm-teleop-right.service"):
        try:
            result = subprocess.run(
                ("systemctl", "is-active", service),
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
            state = result.stdout.strip() or "unknown"
        except (OSError, subprocess.SubprocessError) as exc:
            checks.append(HardwareCheck(service, "WARN", f"cannot inspect service: {exc}"))
            continue
        checks.append(
            HardwareCheck(
                service,
                "PASS" if state == "inactive" else "WARN",
                state,
            )
        )
    return tuple(checks)


def print_hardware_report(checks: tuple[HardwareCheck, ...], *, json_output: bool) -> int:
    if json_output:
        print(json.dumps([asdict(check) for check in checks], indent=2, sort_keys=True))
    else:
        width = max((len(check.name) for check in checks), default=0)
        for check in checks:
            print(f"{console.padded_label(check.level)} {check.name:<{width}}  {check.detail}")
    return 1 if any(check.level == "FAIL" for check in checks) else 0
