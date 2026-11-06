"""Read-only RS232 scale stream for collection; no tare or command writes."""

from __future__ import annotations

import fcntl
import math
import os
import re
import select
import termios
import threading
import time
from collections import deque
from dataclasses import dataclass

from ..configuration import ScaleConfig

_LINE = re.compile(rb"^(ST|US),(NT|GS|TR),([+-])\s*(\d{1,12}(?:\.\d{1,6})?)\s*(g|kg)\r?$", re.I)


@dataclass(frozen=True)
class ScaleSample:
    weight_g: float
    stable: bool
    tared: bool
    net: bool
    monotonic_ns: int
    sequence: int


class ScaleLineDecoder:
    """Decode complete observed text lines, discarding malformed and partial lines."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> list[tuple[float, bool, bool, bool]]:
        self._buffer.extend(chunk.replace(b"\x00", b""))
        parsed: list[tuple[float, bool, bool, bool]] = []
        while b"\n" in self._buffer:
            end = self._buffer.index(b"\n")
            line = bytes(self._buffer[:end])
            del self._buffer[: end + 1]
            if len(line) > 96 or (match := _LINE.fullmatch(line)) is None:
                continue
            status, mode, sign, number, unit = (group.upper() for group in match.groups())
            weight_g = float(number) * (1000.0 if unit == b"KG" else 1.0)
            if sign == b"-":
                weight_g = -weight_g
            if math.isfinite(weight_g):
                parsed.append((weight_g, status == b"ST", mode == b"TR", mode == b"NT"))
        if len(self._buffer) > 96:
            self._buffer.clear()
        return parsed


class ScaleCollectionSource:
    """Own the scale serial port and retain its recent host-timestamped readings."""

    def __init__(self, config: ScaleConfig) -> None:
        self._config = config
        self._condition = threading.Condition()
        self._samples: deque[ScaleSample] = deque(maxlen=128)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fd: int | None = None
        self._original: list[object] | None = None
        self._error: BaseException | None = None

    def __enter__(self) -> ScaleCollectionSource:
        if self._fd is not None:
            raise RuntimeError("scale serial source is already open")
        speed = getattr(termios, f"B{self._config.baudrate}", None)
        if speed is None:
            raise ValueError(f"unsupported scale baudrate: {self._config.baudrate}")
        fd = os.open(self._config.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            original = termios.tcgetattr(fd)
            settings = original.copy()
            settings[0] = 0
            settings[1] = 0
            settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            settings[3] = 0
            settings[4] = speed
            settings[5] = speed
            settings[6] = settings[6].copy()
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, settings)
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd
        self._original = original
        with self._condition:
            self._samples.clear()
            self._error = None
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._read_loop, name="scale-collection", daemon=True
        )
        self._thread.start()
        return self

    def _read_loop(self) -> None:
        assert self._fd is not None
        decoder = ScaleLineDecoder()
        sequence = 0
        try:
            while not self._stop.is_set():
                ready, _, _ = select.select([self._fd], [], [], 0.05)
                if not ready:
                    continue
                chunk = os.read(self._fd, 256)
                if not chunk:
                    raise OSError("scale serial adapter closed or disconnected")
                received_ns = time.monotonic_ns()
                readings = decoder.feed(chunk)
                if readings:
                    with self._condition:
                        for weight_g, stable, tared, net in readings:
                            sequence += 1
                            self._samples.append(
                                ScaleSample(weight_g, stable, tared, net, received_ns, sequence)
                            )
                        self._condition.notify_all()
        except BaseException as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()

    def wait_first(self, *, timeout_s: float, max_age_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise RuntimeError("scale serial stream failed") from self._error
                if len(self._samples) >= 2 and (
                    time.monotonic_ns() - self._samples[-1].monotonic_ns
                    <= max_age_s * 1_000_000_000
                ):
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("no fresh scale stream received")
                self._condition.wait(remaining)

    def latest_before(self, *, target_monotonic_ns: int) -> ScaleSample:
        with self._condition:
            if self._error is not None:
                raise RuntimeError("scale serial stream failed") from self._error
            for sample in reversed(self._samples):
                if sample.monotonic_ns <= target_monotonic_ns:
                    return sample
        raise TimeoutError("no scale reading before robot state")

    def __exit__(self, exc_type, exc, traceback) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1)
            self._thread = None
        if self._fd is not None:
            try:
                if self._original is not None:
                    termios.tcsetattr(self._fd, termios.TCSANOW, self._original)
            finally:
                os.close(self._fd)
                self._fd = None
