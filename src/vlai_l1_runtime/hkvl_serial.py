"""Read-only HKVL six-axis serial stream and pure frame decoder.

The documented streaming frame is ST + six little-endian float32 values +
CRC16/MODBUS (low byte first). Importing this module never opens a device.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import select
import struct
import sys
import termios
import time
from dataclasses import dataclass
from pathlib import Path

HEADER = b"\x53\x54"
FRAME_SIZE = 28
AXES = ("fx_n", "fy_n", "fz_n", "mx_nm", "my_nm", "mz_nm")


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


@dataclass(frozen=True)
class Sample:
    values: tuple[float, float, float, float, float, float]
    frame: bytes


class FrameDecoder:
    """Reassemble frames across arbitrary serial reads and resynchronize on errors."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_failures = 0
        self.nonfinite_frames = 0
        self.discarded_bytes = 0

    def feed(self, chunk: bytes) -> list[Sample]:
        self._buffer.extend(chunk)
        samples: list[Sample] = []
        while self._buffer:
            start = self._buffer.find(HEADER)
            if start < 0:
                keep = 1 if self._buffer[-1] == HEADER[0] else 0
                self.discarded_bytes += len(self._buffer) - keep
                del self._buffer[: len(self._buffer) - keep]
                break
            if start:
                self.discarded_bytes += start
                del self._buffer[:start]
            if len(self._buffer) < FRAME_SIZE:
                break
            frame = bytes(self._buffer[:FRAME_SIZE])
            expected = int.from_bytes(frame[-2:], "little")
            if crc16_modbus(frame[:-2]) != expected:
                self.crc_failures += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            values = struct.unpack_from("<6f", frame, 2)
            del self._buffer[:FRAME_SIZE]
            if not all(math.isfinite(value) for value in values):
                self.nonfinite_frames += 1
                continue
            samples.append(Sample(values, frame))
        return samples


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read HKVL serial FT data without sending commands"
    )
    parser.add_argument("--port", required=True, help="dedicated USB-RS485 port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", required=True, type=int, help="use the rate shown in toolbox")
    parser.add_argument("--output-prefix", required=True, type=Path, help="creates .csv and .bin")
    parser.add_argument("--duration", type=float, help="stop after this many seconds")
    parser.add_argument("--display-hz", type=float, default=10.0, help="terminal refresh rate")
    args = parser.parse_args(argv)
    if (
        args.baudrate <= 0
        or args.display_hz <= 0
        or (args.duration is not None and args.duration <= 0)
    ):
        parser.error("baudrate, display-hz and duration must be positive")
    return args


def capture(args: argparse.Namespace) -> int:
    csv_path = Path(f"{args.output_prefix}.csv")
    raw_path = Path(f"{args.output_prefix}.bin")
    if csv_path.exists() or raw_path.exists():
        raise FileExistsError("output already exists; choose another --output-prefix")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    decoder = FrameDecoder()
    count = 0
    start_ns = time.monotonic_ns()
    next_display_ns = start_ns
    display_period_ns = int(1e9 / args.display_hz)

    speed = getattr(termios, f"B{args.baudrate}", None)
    if speed is None:
        raise ValueError(f"baudrate {args.baudrate} is not supported by this host")
    # O_RDONLY and the loop below never transmit or query the sensor.
    fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        original = termios.tcgetattr(fd)
    except Exception:
        os.close(fd)
        raise
    try:
        settings = original.copy()
        settings[0] = 0  # no input translations or software flow control
        settings[1] = 0
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[3] = 0
        settings[4] = speed
        settings[5] = speed
        settings[6] = settings[6].copy()
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        with (
            csv_path.open("x", newline="", encoding="utf-8") as csv_file,
            raw_path.open("xb") as raw_file,
        ):
            writer = csv.writer(csv_file)
            writer.writerow(("sequence", "host_unix_ns", "host_monotonic_ns", *AXES, "frame_hex"))
            print(f"reading {args.port} at {args.baudrate} baud (8N1); Ctrl+C to stop")
            while args.duration is None or (time.monotonic_ns() - start_ns) < args.duration * 1e9:
                ready, _, _ = select.select([fd], [], [], 0.1)
                if not ready:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise OSError("serial adapter closed or disconnected")
                received_unix_ns = time.time_ns()
                received_monotonic_ns = time.monotonic_ns()
                raw_file.write(chunk)
                for sample in decoder.feed(chunk):
                    count += 1
                    writer.writerow(
                        (
                            count,
                            received_unix_ns,
                            received_monotonic_ns,
                            *sample.values,
                            sample.frame.hex(),
                        )
                    )
                    if received_monotonic_ns >= next_display_ns:
                        values = "  ".join(
                            f"{name}={value:+.4f}"
                            for name, value in zip(AXES, sample.values, strict=True)
                        )
                        print(f"#{count} {values}  crc_fail={decoder.crc_failures}", flush=True)
                        next_display_ns = received_monotonic_ns + display_period_ns
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            os.close(fd)
    print(
        f"saved {count} samples; crc_failures={decoder.crc_failures}, "
        f"nonfinite_frames={decoder.nonfinite_frames}, discarded_bytes={decoder.discarded_bytes}"
    )
    return 0 if count else 1


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        return capture(args)
    except KeyboardInterrupt:
        print("stopped by user", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError, termios.error) as exc:
        print(f"capture error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
