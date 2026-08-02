"""Exclusive live lifecycle for one configured x_air teleoperation side."""

from __future__ import annotations

import fcntl
import json
import os
import platform
import select
import signal
import stat
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

from .. import console
from ..configuration import SystemConfig
from .can_probe import MotorFeedbackProbeError, probe_motor_feedback
from .xair import describe_xair_side, verify_xair_dependency, xair_control_socket_path


@dataclass(frozen=True)
class _CanLinkHealth:
    interface: str
    parentdev: str
    state: str
    txerr: int
    rxerr: int

    @property
    def healthy(self) -> bool:
        return self.state == "ERROR-ACTIVE" and self.txerr == 0 and self.rxerr == 0

    def detail(self) -> str:
        return (
            f"{self.interface} parent={self.parentdev} state={self.state} "
            f"txerr={self.txerr} rxerr={self.rxerr}"
        )


class _StartupCanFault(RuntimeError):
    def __init__(self, unhealthy: tuple[_CanLinkHealth, ...]) -> None:
        self.unhealthy = unhealthy
        super().__init__("; ".join(health.detail() for health in unhealthy))


def _sidecar_command(
    config: SystemConfig,
    side: str,
    *,
    sidecar: Path,
    assets: Path,
    launch: dict[str, Any],
    owner_uid: int,
    owner_gid: int,
) -> tuple[str, ...]:
    joint_safety = config.teleoperation.joint_safety.for_side(side)

    def vector(values: tuple[float, ...]) -> str:
        return ",".join(f"{value:g}" for value in values)

    return (
        "chrt",
        "-f",
        str(config.teleoperation.rt_priority),
        str(sidecar),
        "--side",
        side,
        "--leader-can",
        str(launch["leader_can"]),
        "--follower-can",
        str(launch["follower_can"]),
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
        "--joint-min-deg",
        vector(joint_safety.min_deg),
        "--joint-max-deg",
        vector(joint_safety.max_deg),
        "--max-following-error-deg",
        vector(joint_safety.max_following_error_deg),
        "--following-error-timeout-ms",
        str(round(config.teleoperation.joint_safety.following_error_timeout_s * 1000)),
        "--following-error-action",
        config.teleoperation.joint_safety.following_error_action,
        "--control-socket",
        str(launch["control_socket_path"]),
        "--control-owner-uid",
        str(owner_uid),
        "--control-owner-gid",
        str(owner_gid),
    )


def run_xair_side(
    config: SystemConfig,
    side: str,
    *,
    managed_startup_gate: bool = False,
    isolated_side: bool = False,
) -> int:
    """Run one side until interrupted, then disable it and close both CAN links."""

    if os.geteuid() != 0:
        raise RuntimeError("x_air live launch requires root")
    if managed_startup_gate and isolated_side:
        raise RuntimeError("managed bimanual startup and isolated-side mode are mutually exclusive")
    dependency = verify_xair_dependency(config)
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
    lifecycle_lock = _acquire_lifecycle_lock(config, exclusive=isolated_side)
    try:
        _require_exclusive_control(interfaces)
        inactive_interfaces = _require_inactive_side_off(config, side) if isolated_side else ()
        remove_orphaned_xair_control_socket(config, side)
    except BaseException:
        lifecycle_lock.close()
        raise
    child: subprocess.Popen[str] | None = None
    output_pump: threading.Thread | None = None
    links_open = False
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        signum: signal.signal(signum, request_stop) for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
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
        try:
            if inactive_interfaces:
                _require_interfaces_down(inactive_interfaces)
            for interface in interfaces:
                _configure_can(config, interface)
            links_open = True
            unhealthy = tuple(
                health
                for interface in interfaces
                if not (health := _read_can_health(interface)).healthy
            )
            if unhealthy:
                raise _StartupCanFault(unhealthy)
            baseline_drops = {interface: _qdisc_drops(interface) for interface in interfaces}
            for interface in interfaces:
                console.step(
                    f"Probing all configured motors on {interface} without enabling motion"
                )
                try:
                    probe = probe_motor_feedback(
                        config,
                        interface,
                        dependency.can_library,
                    )
                except MotorFeedbackProbeError as exc:
                    unhealthy = tuple(
                        health
                        for candidate in interfaces
                        if not (health := _read_can_health(candidate)).healthy
                    )
                    if unhealthy:
                        raise _StartupCanFault(unhealthy) from exc
                    raise RuntimeError(str(exc)) from exc
                _require_startup_can_health(interfaces, baseline_drops)
                console.success(
                    f"{interface} returned all {len(probe.response_ids)} motor ids "
                    f"for {probe.rounds} motion-free rounds"
                )

            if inactive_interfaces:
                _require_interfaces_down(inactive_interfaces)

            if managed_startup_gate:
                print(f"PASS x_air {side} motion-free preflight ready", flush=True)
                released = _await_managed_startup_release(
                    input_stream=sys.stdin,
                    side=side,
                    timeout_s=config.teleoperation.startup_timeout_s,
                    poll_s=config.teleoperation.can_health_poll_s,
                    require_healthy=lambda: _require_startup_can_health(interfaces, baseline_drops),
                    stop_requested=lambda: stop_requested,
                )
                if not released:
                    return 0
                console.success(f"Bimanual preflight released {side} x_air startup")

            _require_startup_can_health(interfaces, baseline_drops)
            ready = threading.Event()
            command = _sidecar_command(
                config,
                side,
                sidecar=sidecar,
                assets=assets,
                launch=launch,
                owner_uid=owner_uid,
                owner_gid=owner_gid,
            )
            child = subprocess.Popen(
                command,
                env=environment,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
            )
            if child.stdout is None:
                raise RuntimeError("x_air sidecar has no output stream")
            output_pump = threading.Thread(
                target=_relay_sidecar_output,
                args=(child.stdout, ready),
                name=f"vlai-{side}-sidecar-output",
                daemon=True,
            )
            output_pump.start()
            startup_deadline = time.monotonic() + config.teleoperation.startup_timeout_s
            while child.poll() is None and not stop_requested:
                time.sleep(config.teleoperation.can_health_poll_s)
                if inactive_interfaces:
                    _require_interfaces_down(inactive_interfaces)
                for interface in interfaces:
                    drops = _qdisc_drops(interface)
                    if drops != baseline_drops[interface]:
                        raise RuntimeError(
                            f"{interface} qdisc drops increased "
                            f"{baseline_drops[interface]} -> {drops}"
                        )
                if ready.is_set():
                    continue
                unhealthy = tuple(
                    health
                    for interface in interfaces
                    if not (health := _read_can_health(interface)).healthy
                )
                if unhealthy:
                    raise _StartupCanFault(unhealthy)
                if time.monotonic() >= startup_deadline:
                    raise TimeoutError(
                        f"{side} x_air startup exceeded "
                        f"{config.teleoperation.startup_timeout_s:.1f}s"
                    )
            if stop_requested:
                _stop_sidecar(
                    child,
                    output_pump,
                    timeout_s=config.teleoperation.shutdown_timeout_s,
                )
                child = None
                output_pump = None
                return 0
            if not ready.is_set():
                unhealthy = tuple(
                    health
                    for interface in interfaces
                    if not (health := _read_can_health(interface)).healthy
                )
                if unhealthy:
                    raise _StartupCanFault(unhealthy)
                raise RuntimeError(
                    f"{side} x_air sidecar exited before reporting ready "
                    f"with status {child.returncode}"
                )
            return int(child.returncode or 0)
        except _StartupCanFault as exc:
            raise RuntimeError(
                f"{side} startup CAN unhealthy; no reset or retry attempted: {exc}"
            ) from exc
    finally:
        if child is not None:
            _stop_sidecar(
                child,
                output_pump,
                timeout_s=config.teleoperation.shutdown_timeout_s,
            )
        if links_open:
            _disable_and_close(config, interfaces)
        try:
            remove_orphaned_xair_control_socket(config, side)
        finally:
            try:
                for signum, handler in previous_handlers.items():
                    signal.signal(signum, handler)
            finally:
                lifecycle_lock.close()


def _require_startup_can_health(
    interfaces: tuple[str, str],
    baseline_drops: dict[str, int],
) -> None:
    for interface in interfaces:
        drops = _qdisc_drops(interface)
        if drops != baseline_drops[interface]:
            raise RuntimeError(
                f"{interface} qdisc drops increased "
                f"{baseline_drops[interface]} -> {drops} before startup"
            )
    unhealthy = tuple(
        health for interface in interfaces if not (health := _read_can_health(interface)).healthy
    )
    if unhealthy:
        raise _StartupCanFault(unhealthy)


def _await_managed_startup_release(
    *,
    input_stream: TextIO,
    side: str,
    timeout_s: float,
    poll_s: float,
    require_healthy: Callable[[], None],
    stop_requested: Callable[[], bool],
) -> bool:
    """Wait for the bimanual parent without entering the motion-capable SDK."""

    try:
        descriptor = input_stream.fileno()
    except (AttributeError, OSError) as exc:
        raise RuntimeError("managed x_air startup gate requires a pipe on stdin") from exc
    deadline = time.monotonic() + timeout_s
    while True:
        if stop_requested():
            return False
        require_healthy()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                f"{side} motion-free preflight was not released within {timeout_s:.1f}s"
            )
        try:
            readable, _, _ = select.select(
                (descriptor,),
                (),
                (),
                min(poll_s, remaining),
            )
        except InterruptedError:
            continue
        if not readable:
            continue
        command = input_stream.readline()
        if command == "START\n":
            require_healthy()
            return True
        if not command:
            raise RuntimeError("managed x_air startup parent closed the release gate")
        raise RuntimeError(f"managed x_air startup received an invalid release for {side}")


def _relay_sidecar_output(stream: TextIO, ready: threading.Event) -> None:
    try:
        for line in stream:
            print(line, end="", flush=True)
            if line.startswith("PASS x_air ") and " control running " in line:
                ready.set()
    finally:
        stream.close()


def _stop_sidecar(
    child: subprocess.Popen[str],
    output_pump: threading.Thread | None,
    *,
    timeout_s: float,
) -> int:
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            status = child.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            print(
                f"WARN x_air sidecar exceeded {timeout_s:.1f}s shutdown timeout; killing it",
                flush=True,
            )
            child.kill()
            status = child.wait()
    else:
        status = int(child.returncode or 0)
    if output_pump is not None and output_pump.ident is not None:
        output_pump.join(timeout=1)
    return int(status)


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


def _decode_can_health(interface: str, payload: str) -> _CanLinkHealth:
    try:
        entries = json.loads(payload)
        if len(entries) != 1:
            raise ValueError("expected exactly one CAN link")
        entry = entries[0]
        data = entry["linkinfo"]["info_data"]
        errors = data["berr_counter"]
        parentdev = entry["parentdev"]
        state = data["state"]
        txerr = errors["tx"]
        rxerr = errors["rx"]
        if (
            not isinstance(parentdev, str)
            or not isinstance(state, str)
            or not isinstance(txerr, int)
            or not isinstance(rxerr, int)
        ):
            raise TypeError("CAN health fields have invalid types")
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot decode {interface} CAN health") from exc
    return _CanLinkHealth(interface, parentdev, state, txerr, rxerr)


def _read_can_health(interface: str) -> _CanLinkHealth:
    result = subprocess.run(
        ("ip", "-j", "-details", "-statistics", "link", "show", "dev", interface),
        check=True,
        capture_output=True,
        text=True,
        start_new_session=True,
    )
    return _decode_can_health(interface, result.stdout)


def _decode_link_is_up(interface: str, payload: str) -> bool:
    try:
        entries = json.loads(payload)
        if len(entries) != 1:
            raise ValueError("expected exactly one network link")
        flags = entries[0]["flags"]
        if not isinstance(flags, list) or not all(isinstance(flag, str) for flag in flags):
            raise TypeError("network link flags have invalid types")
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot decode {interface} administrative state") from exc
    return "UP" in flags


def _read_link_is_up(interface: str) -> bool:
    result = subprocess.run(
        ("ip", "-j", "link", "show", "dev", interface),
        check=True,
        capture_output=True,
        text=True,
        start_new_session=True,
    )
    return _decode_link_is_up(interface, result.stdout)


def _inactive_can_interfaces(config: SystemConfig, active_side: str) -> tuple[str, str]:
    interfaces = tuple(
        endpoint.interface for endpoint in config.can.endpoints if endpoint.side != active_side
    )
    if len(interfaces) != 2:
        raise RuntimeError(f"{active_side} does not resolve to exactly two inactive CAN endpoints")
    return interfaces


def _require_interfaces_down(interfaces: tuple[str, str]) -> None:
    active = tuple(interface for interface in interfaces if _read_link_is_up(interface))
    if active:
        raise RuntimeError(
            "isolated-side interlock requires inactive CAN interfaces DOWN: " + ", ".join(active)
        )


def _require_inactive_side_off(config: SystemConfig, active_side: str) -> tuple[str, str]:
    interfaces = _inactive_can_interfaces(config, active_side)
    competing = _find_competing_control(interfaces, _process_arguments())
    if competing is not None:
        raise RuntimeError(f"inactive side still has a controller process: {competing}")
    _require_interfaces_down(interfaces)
    return interfaces


def _acquire_lifecycle_lock(config: SystemConfig, *, exclusive: bool) -> TextIO:
    lock_path = config.teleoperation.state_socket_path.with_name("teleop-lifecycle.lock")
    descriptor: int | None = None
    try:
        descriptor = os.open(
            lock_path,
            os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o600,
        )
        identity = os.fstat(descriptor)
        if not stat.S_ISREG(identity.st_mode):
            raise RuntimeError(f"x_air lifecycle lock is not a regular file: {lock_path}")
        handle = os.fdopen(descriptor, "a+", encoding="utf-8")
        descriptor = None
    except OSError as exc:
        raise RuntimeError(f"cannot open x_air lifecycle lock: {lock_path}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
    operation = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
    try:
        fcntl.flock(handle.fileno(), operation | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        mode = "isolated-side" if exclusive else "shared"
        raise RuntimeError(f"cannot acquire {mode} x_air lifecycle ownership") from exc
    return handle


def _configured_parentdev(config: SystemConfig, interface: str) -> str:
    matches = [
        endpoint.parentdev for endpoint in config.can.endpoints if endpoint.interface == interface
    ]
    if len(matches) != 1:
        raise RuntimeError(f"{interface} is not a unique tracked CAN endpoint")
    return matches[0]


def _require_can_identity(
    config: SystemConfig,
    interface: str,
    *,
    sysfs_root: Path = Path("/sys"),
) -> str:
    expected = _configured_parentdev(config, interface)
    device_link = sysfs_root / "class/net" / interface / "device"
    try:
        device = device_link.resolve(strict=True)
        driver = (device / "driver").resolve(strict=True)
    except OSError as exc:
        raise RuntimeError(f"cannot resolve {interface} physical identity") from exc
    if device.name != expected:
        raise RuntimeError(
            f"{interface} USB identity mismatch: expected {expected}, found {device.name}"
        )
    if driver.name != "peak_usb":
        raise RuntimeError(f"{interface} driver mismatch: expected peak_usb, found {driver.name}")
    return expected


def _configure_can(config: SystemConfig, interface: str) -> None:
    _require_can_identity(config, interface)
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
        "listen-only",
        "off",
        "one-shot",
        "off",
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
