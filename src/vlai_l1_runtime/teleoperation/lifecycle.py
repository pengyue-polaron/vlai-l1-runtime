"""Exclusive live lifecycle for one configured x_air teleoperation side."""

from __future__ import annotations

import json
import os
import platform
import signal
import stat
import subprocess
import time
from pathlib import Path
from typing import Any

from ..configuration import SystemConfig
from .xair import describe_xair_side, xair_control_socket_path


def run_xair_side(config: SystemConfig, side: str) -> int:
    """Run one side until interrupted, then disable it and close both CAN links."""

    if os.geteuid() != 0:
        raise RuntimeError("x_air live launch requires root")
    launch = describe_xair_side(config, side)
    owner_uid, owner_gid = _invoking_identity()
    repo = config.path.parents[2]
    assets = repo / "build/xair-assets"
    sidecar = repo / "build/xair-sidecar/vlai_l1_xair_sidecar"
    _verify_manifest(assets / "manifest.json", side, launch)
    for label, path in (
        ("sidecar", sidecar),
        ("leader URDF", assets / f"{config.teleoperation.arm_type}_leader.urdf"),
        ("follower URDF", assets / f"{config.teleoperation.arm_type}_follower.urdf"),
        ("control config", assets / "config"),
    ):
        if not path.exists():
            raise RuntimeError(f"x_air {label} is missing: {path}")

    interfaces = (str(launch["leader_can"]), str(launch["follower_can"]))
    _require_exclusive_control(interfaces)
    child: subprocess.Popen[str] | None = None
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        signum: signal.signal(signum, request_stop) for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        for interface in interfaces:
            _configure_can(config, interface)
        baseline_drops = {interface: _qdisc_drops(interface) for interface in interfaces}
        environment = dict(os.environ)
        architecture = platform.machine()
        library_paths = (
            str(
                config.teleoperation.source_root
                / "publish/modules/src/xarm_teleop/prebuilt/xarm_teleop/lib"
                / architecture
            ),
            str(config.teleoperation.source_root / "publish/xarm_can/package/lib" / architecture),
            "/opt/ros/humble/lib/x86_64-linux-gnu",
            "/opt/ros/humble/lib",
        )
        current = environment.get("LD_LIBRARY_PATH")
        environment["LD_LIBRARY_PATH"] = ":".join(
            (*library_paths, *((current,) if current else ()))
        )
        command = (
            "chrt",
            "-f",
            str(config.teleoperation.rt_priority),
            str(sidecar),
            "--side",
            side,
            "--leader-can",
            interfaces[0],
            "--follower-can",
            interfaces[1],
            "--leader-urdf",
            str(assets / f"{config.teleoperation.arm_type}_leader.urdf"),
            "--follower-urdf",
            str(assets / f"{config.teleoperation.arm_type}_follower.urdf"),
            "--config-dir",
            str(assets / "config"),
            "--state-socket",
            str(config.teleoperation.state_socket_path),
            "--publish-hz",
            str(config.teleoperation.publish_hz),
            "--state-timeout-ms",
            str(round(config.teleoperation.state_timeout_s * 1000)),
            "--rt-priority",
            str(config.teleoperation.rt_priority),
            "--can-health-poll-ms",
            str(round(config.teleoperation.can_health_poll_s * 1000)),
            "--control-socket",
            str(launch["control_socket_path"]),
            "--control-owner-uid",
            str(owner_uid),
            "--control-owner-gid",
            str(owner_gid),
        )
        child = subprocess.Popen(command, env=environment, text=True)
        while child.poll() is None and not stop_requested:
            time.sleep(config.teleoperation.can_health_poll_s)
            for interface in interfaces:
                drops = _qdisc_drops(interface)
                if drops != baseline_drops[interface]:
                    raise RuntimeError(
                        f"{interface} qdisc drops increased {baseline_drops[interface]} -> {drops}"
                    )
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        return child.wait(timeout=5)
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        _disable_and_close(config, interfaces)
        try:
            remove_orphaned_xair_control_socket(config, side)
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)


def _invoking_identity() -> tuple[int, int]:
    try:
        uid = int(os.environ.get("SUDO_UID", os.getuid()))
        gid = int(os.environ.get("SUDO_GID", os.getgid()))
    except ValueError as exc:
        raise RuntimeError("sudo invoking identity is invalid") from exc
    if uid < 0 or gid < 0:
        raise RuntimeError("sudo invoking identity must be non-negative")
    return uid, gid


def remove_orphaned_xair_control_socket(config: SystemConfig, side: str) -> None:
    """Remove one configured x_air control socket after its owner has exited."""

    _remove_orphaned_control_socket(xair_control_socket_path(config, side))


def _remove_orphaned_control_socket(path: Path) -> None:
    """Remove only an inactive repository-owned Unix control endpoint."""

    if not path.is_absolute():
        raise RuntimeError("x_air control socket path must be absolute")
    try:
        identity = path.lstat()
    except FileNotFoundError:
        return
    if not stat.S_ISSOCK(identity.st_mode):
        raise RuntimeError(f"x_air control socket path is not a socket: {path}")
    try:
        entries = Path("/proc/net/unix").read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise RuntimeError("cannot verify x_air control socket ownership") from exc
    active_paths = {fields[7] for line in entries[1:] if len(fields := line.split(maxsplit=7)) == 8}
    if str(path) in active_paths:
        raise RuntimeError(f"x_air control socket is still active: {path}")
    path.unlink()


def _configure_can(config: SystemConfig, interface: str) -> None:
    subprocess.run(("ip", "link", "set", interface, "down"), check=False)
    arguments = [
        "ip",
        "link",
        "set",
        interface,
        "type",
        "can",
        "bitrate",
        str(config.can.nominal_bitrate),
        "dbitrate",
        str(config.can.data_bitrate),
        "fd",
        "on" if config.can.fd else "off",
        "restart-ms",
        str(config.can.restart_ms),
    ]
    subprocess.run(arguments, check=True)
    subprocess.run(
        ("ip", "link", "set", interface, "txqueuelen", str(config.can.tx_queue_length)),
        check=True,
    )
    subprocess.run(("ip", "link", "set", interface, "up"), check=True)


def _qdisc_drops(interface: str) -> int:
    result = subprocess.run(
        ("tc", "-j", "-s", "qdisc", "show", "dev", interface),
        check=True,
        capture_output=True,
        text=True,
        start_new_session=True,
    )
    entries = json.loads(result.stdout)
    if len(entries) != 1 or not isinstance(entries[0].get("drops"), int):
        raise RuntimeError(f"cannot read {interface} qdisc drop counter")
    return int(entries[0]["drops"])


def _disable_and_close(config: SystemConfig, interfaces: tuple[str, str]) -> None:
    for interface in interfaces:
        subprocess.run(("ip", "link", "set", interface, "up"), check=False)
        for motor in config.motors:
            frame = f"{motor.send_id:03X}##1FFFFFFFFFFFFFFFD"
            subprocess.run(
                ("cansend", interface, frame),
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.03)
        subprocess.run(("ip", "link", "set", interface, "down"), check=False)


def _require_exclusive_control(interfaces: tuple[str, str]) -> None:
    competing = _find_competing_control(interfaces, _process_arguments())
    if competing is not None:
        raise RuntimeError(f"competing teleoperation process is running: {competing}")


def _find_competing_control(
    interfaces: tuple[str, str], commands: tuple[tuple[str, ...], ...]
) -> str | None:
    requested = set(interfaces)
    for arguments in commands:
        if not arguments:
            continue
        executable = Path(arguments[0]).name
        if executable == "unilateral_control":
            return " ".join(arguments)
        if executable != "vlai_l1_xair_sidecar":
            continue
        try:
            active = {
                arguments[arguments.index("--leader-can") + 1],
                arguments[arguments.index("--follower-can") + 1],
            }
        except (ValueError, IndexError):
            return " ".join(arguments)
        if requested & active:
            return " ".join(arguments)
    return None


def _process_arguments() -> tuple[tuple[str, ...], ...]:
    commands: list[tuple[str, ...]] = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            payload = path.read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if payload:
            commands.append(
                tuple(
                    argument.decode(errors="replace")
                    for argument in payload.rstrip(b"\0").split(b"\0")
                )
            )
    return tuple(commands)


def _verify_manifest(path: Path, side: str, expected: dict[str, Any]) -> None:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
        actual = manifest["sides"][side]
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load x_air launch manifest {path}: {exc}") from exc
    if actual != expected:
        raise RuntimeError("x_air launch manifest does not match tracked System config")
