"""Standalone two-camera RGB/position recorder; never opens or commands CAN.

Consumes existing sidecar state. This is an experimental CSV/MJPEG format,
not a canonical LeRobot dataset. Camera timestamps are host read-completion
times, not hardware exposure timestamps. No stillness trimming or arm reset.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing as mp
import queue
import signal
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path

from .configuration import MOTOR_NAMES, load_system_config
from .teleoperation.xair import XAirStatePacket, XAirStateReceiver


def parse_camera_ports(output: str) -> dict[str, str]:
    """Map RealSense serial (not ASIC serial) to the enumerated USB parent."""
    devices = {}
    for block in output.split("Device info:")[1:]:
        fields = {}
        for line in block.splitlines():
            key, separator, value = line.strip().partition(":")
            if separator and key.strip() in {"Serial Number", "Physical Port"}:
                fields[key.strip()] = value.strip()
        if "Serial Number" in fields and "Physical Port" in fields:
            devices[fields["Serial Number"]] = fields["Physical Port"].rsplit("-", 1)[0]
    return devices


def validate_packet(
    packet: XAirStatePacket, previous: XAirStatePacket | None, now_ns: int, max_age_s: float
) -> None:
    age = (now_ns - packet.monotonic_ns) / 1e9
    if age < 0 or age > max_age_s:
        raise RuntimeError(f"{packet.side} motor state is stale or from the future: {age:.3f}s")
    if previous and (
        packet.source_sequence <= previous.source_sequence
        or packet.monotonic_ns <= previous.monotonic_ns
    ):
        raise RuntimeError(f"{packet.side} motor sequence/timestamp did not increase")


def motor_header() -> list[str]:
    return ["source_sequence", "source_monotonic_ns", "received_monotonic_ns"] + [
        f"{role}_{name}_deg" for role in ("leader", "follower") for name in MOTOR_NAMES
    ]


def motor_row(packet: XAirStatePacket, received_ns: int) -> list[int | float]:
    return [packet.source_sequence, packet.monotonic_ns, received_ns] + [
        math.degrees(value) for value in (*packet.leader_radians, *packet.follower_radians)
    ]


def verify_devices(spec: dict) -> dict[str, str]:
    if set(spec["cameras"]) != {"left", "right"}:
        raise ValueError("recording config must define exactly left and right cameras")
    result = subprocess.run(
        ["rs-enumerate-devices", "-S"], capture_output=True, text=True, check=True, timeout=15
    )
    ports = parse_camera_ports(result.stdout)
    resolved = {}
    for role, camera in spec["cameras"].items():
        device = Path(camera["device"]).resolve(strict=True)
        if device.parent != Path("/dev") or not device.name.startswith("video"):
            raise ValueError(f"invalid video device: {device}")
        sys_device = (Path("/sys/class/video4linux") / device.name / "device").resolve()
        parent = next(p for p in sys_device.parents if (p / "idVendor").is_file())
        if ports.get(camera["serial"]) != parent.name:
            raise RuntimeError(f"{role} serial/path mismatch; re-identify cameras before recording")
        owner = subprocess.run(["fuser", str(device)], capture_output=True, text=True)
        if owner.returncode != 1 or owner.stdout.strip():
            raise RuntimeError(f"{device} is in use or ownership could not be checked")
        resolved[role] = str(device)
    if len(set(resolved.values())) != 2:
        raise ValueError("left and right must use different video devices")
    return resolved


def camera_worker(role, device, size, fps, output, gate, stop, events) -> None:
    """Separate process bounds cleanup even if a camera read hangs."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    capture = writer = None
    try:
        import cv2

        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open {device}")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        capture.set(cv2.CAP_PROP_FPS, fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual = tuple(
            round(capture.get(p))
            for p in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS)
        )
        if actual != (*size, fps):
            raise RuntimeError(f"{role} camera mode {actual} != {(*size, fps)}")
        writer = cv2.VideoWriter(
            str(Path(output) / f"{role}.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps, tuple(size)
        )
        if not writer.isOpened():
            raise RuntimeError("MJPEG video writer could not open")
        with (Path(output) / f"{role}_frames.csv").open("x", newline="") as handle:
            rows = csv.writer(handle)
            rows.writerow(["frame_index", "read_monotonic_ns"])
            count = 0
            first = last = None
            maximum_gap = 0
            announced = False
            next_heartbeat = 0.0
            while not stop.is_set():
                ok, frame = capture.read()
                stamp = time.monotonic_ns()
                if not ok or frame.shape != (size[1], size[0], 3):
                    raise RuntimeError(f"{role} camera frame failed")
                if not announced:
                    events.put((role, "ready", None))
                    announced = True
                if not gate.is_set():
                    continue
                writer.write(frame)
                rows.writerow([count, stamp])
                if first is None:
                    first = stamp
                if last is not None:
                    maximum_gap = max(maximum_gap, stamp - last)
                last = stamp
                count += 1
                if time.monotonic() >= next_heartbeat:
                    handle.flush()
                    events.put((role, "heartbeat", None))
                    next_heartbeat = time.monotonic() + 0.1
        capture.release()
        capture = None
        writer.release()
        writer = None
        # Decode the completed file to catch encoder failures and frame loss.
        replay = cv2.VideoCapture(str(Path(output) / f"{role}.avi"))
        decoded = 0
        try:
            while True:
                ok, frame = replay.read()
                if not ok:
                    break
                if frame.shape != (size[1], size[0], 3):
                    raise RuntimeError(f"{role} encoded frame shape is invalid")
                decoded += 1
        finally:
            replay.release()
        if count < 2 or decoded != count:
            raise RuntimeError(
                f"{role} video verification failed: captured={count}, decoded={decoded}"
            )
        events.put(
            (
                role,
                "done",
                {
                    "frames": count,
                    "decoded_frames": decoded,
                    "first_monotonic_ns": first,
                    "last_monotonic_ns": last,
                    "effective_fps": (count - 1) * 1e9 / (last - first),
                    "max_gap_ms": maximum_gap / 1e6,
                },
            )
        )
    except BaseException as exc:
        events.put((role, "error", str(exc)))
    finally:
        if capture is not None:
            capture.release()
        if writer is not None:
            writer.release()


def run_recording(
    spec_path: Path, output: Path, duration: float, *, cameras_only: bool = False
) -> dict:
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive and finite")
    spec = json.loads(spec_path.read_text())
    system = load_system_config((spec_path.parent / spec["system_config"]).resolve())
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"refusing to overwrite {output}")
    for field in ("startup_timeout_s", "minimum_video_fps"):
        if not math.isfinite(spec[field]) or spec[field] <= 0:
            raise ValueError(f"{field} must be positive and finite")
    devices = verify_devices(spec)
    if not cameras_only and not system.teleoperation.state_socket_path.parent.is_dir():
        raise RuntimeError("state directory missing; see docs/TEMPORARY_RECORDING.md")
    context = mp.get_context("spawn")
    gate, stop, events = context.Event(), context.Event(), context.Queue()
    workers = []
    report = {
        "format": "vlai-temporary-rgb-position-v1",
        "status": "incomplete",
        "mode": "cameras_only_check" if cameras_only else "two_cameras_all_motors",
        "requested_duration_s": duration,
        "started_unix_ns": time.time_ns(),
        "position_unit": "degree",
        "video_codec": "MJPG",
        "timestamp_clock": "CLOCK_MONOTONIC",
        "camera_timestamp": "host read completion, not exposure time",
        "alignment": "independent streams; use timestamps, not row indices",
        "config": spec,
        "system_config_text": system.path.read_text(),
        "cameras": {},
        "motor_samples": {"left": 0, "right": 0},
    }
    interrupted = False

    def interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    ready, heartbeat, last_packets = set(), {}, {}
    errors = []
    started = None

    def drain_events():
        while True:
            try:
                role, kind, value = events.get_nowait()
            except queue.Empty:
                return
            if kind == "error":
                errors.append(f"{role}: {value}")
            elif kind == "ready":
                ready.add(role)
                print(f"{role} camera ready", flush=True)
            elif kind == "heartbeat":
                heartbeat[role] = time.monotonic()
            elif kind == "done":
                report["cameras"][role] = value

    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    manifest = output / "metadata.json"
    manifest.write_text(json.dumps(report, indent=2) + "\n")
    previous_handlers = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        with ExitStack() as stack:
            receiver = None
            rows = {}
            if not cameras_only:
                receiver = stack.enter_context(XAirStateReceiver(system))
                for role in ("left", "right"):
                    handle = stack.enter_context(
                        (output / f"{role}_motors.csv").open("x", newline="")
                    )
                    rows[role] = csv.writer(handle)
                    rows[role].writerow(motor_header())
            for role, device in devices.items():
                stream = next(c for c in system.cameras.streams if c.role == f"wrist_{role}")
                process = context.Process(
                    target=camera_worker,
                    args=(
                        role,
                        device,
                        (stream.width, stream.height),
                        stream.fps,
                        str(output),
                        gate,
                        stop,
                        events,
                    ),
                    name=f"record-camera-{role}",
                )
                process.start()
                workers.append(process)
            deadline = time.monotonic() + spec["startup_timeout_s"]
            print(
                "Waiting for both cameras"
                + (" and both teleop state streams" if receiver else " (camera-only check)"),
                flush=True,
            )
            while not interrupted:
                packet = receiver.receive_packet(timeout_s=0.01) if receiver else None
                if receiver is None:
                    time.sleep(0.01)
                now = time.monotonic()
                now_ns = time.monotonic_ns()
                if packet is not None:
                    validate_packet(
                        packet,
                        last_packets.get(packet.side),
                        now_ns,
                        system.teleoperation.state_timeout_s,
                    )
                    last_packets[packet.side] = packet
                    if started is not None:
                        rows[packet.side].writerow(motor_row(packet, now_ns))
                        report["motor_samples"][packet.side] += 1
                drain_events()
                if errors:
                    raise RuntimeError("; ".join(errors))
                if any(p.exitcode is not None for p in workers):
                    raise RuntimeError("camera worker exited unexpectedly")
                if started is None:
                    motors_ready = receiver is None or (
                        set(last_packets) == {"left", "right"}
                        and all(
                            (now_ns - p.monotonic_ns) / 1e9 <= system.teleoperation.state_timeout_s
                            for p in last_packets.values()
                        )
                    )
                    if ready == {"left", "right"} and motors_ready:
                        started = now
                        heartbeat.update(left=now, right=now)
                        report["recording_start_monotonic_ns"] = now_ns
                        gate.set()
                        print(
                            f"RECORDING {duration:g}s -> {output} (Ctrl+C saves and stops recorder)",
                            flush=True,
                        )
                    elif now > deadline:
                        missing_cameras = sorted(set(devices) - ready)
                        missing_motors = (
                            sorted({"left", "right"} - set(last_packets)) if receiver else []
                        )
                        raise TimeoutError(
                            f"startup timed out: missing cameras={missing_cameras}, "
                            f"missing motor sides={missing_motors}; no arms were started"
                        )
                else:
                    if receiver and any(
                        (now_ns - last_packets[s].monotonic_ns) / 1e9
                        > system.teleoperation.state_timeout_s
                        for s in ("left", "right")
                    ):
                        raise TimeoutError("motor stream stopped or became stale")
                    if any(now - heartbeat[s] > system.cameras.max_age_s for s in devices):
                        raise TimeoutError("camera stream stopped or became stale")
                    if now - started >= duration:
                        break
    except Exception as exc:
        errors.append(str(exc))
    finally:
        stop.set()
        shutdown_deadline = time.monotonic() + 10
        while any(p.is_alive() for p in workers) and time.monotonic() < shutdown_deadline:
            drain_events()
            for process in workers:
                process.join(timeout=0.02)
        for process in workers:
            if process.is_alive():
                process.terminate()
                process.join(timeout=2)
                if process.is_alive():
                    process.kill()
                    process.join()
                errors.append(f"{process.name} timed out during cleanup")
        drain_events()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if set(report["cameras"]) != {"left", "right"}:
            errors.append("missing completed camera recordings")
        for role, info in report["cameras"].items():
            if info["effective_fps"] < spec["minimum_video_fps"]:
                errors.append(f"{role} video FPS below {spec['minimum_video_fps']}")
        if not cameras_only and not all(report["motor_samples"].values()):
            errors.append("missing motor samples from one or both sides")
        report.update(
            status="failed" if errors else "complete",
            stop_reason="error" if errors else "operator" if interrupted else "duration",
            errors=errors,
            finished_unix_ns=time.time_ns(),
        )
        manifest.write_text(json.dumps(report, indent=2) + "\n")
        events.close()
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/recording/two_wrist.json"))
    parser.add_argument(
        "--output", type=Path, required=True, help="new directory, never overwritten"
    )
    parser.add_argument(
        "--duration", type=float, default=30, help="seconds after all streams ready"
    )
    parser.add_argument(
        "--check-cameras",
        action="store_true",
        help="explicit camera-only recording test; no motor data",
    )
    args = parser.parse_args()
    try:
        report = run_recording(
            args.config.resolve(),
            args.output.absolute(),
            args.duration,
            cameras_only=args.check_cameras,
        )
    except Exception as exc:
        print(f"FAIL {exc}")
        return 1
    print(
        json.dumps(
            {k: report[k] for k in ("status", "mode", "cameras", "motor_samples", "errors")},
            indent=2,
        )
    )
    print("Recorder released its cameras/state socket. Teleoperation was not stopped or reset.")
    return 0 if report["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
