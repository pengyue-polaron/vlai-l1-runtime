"""Read-only HKVL stream owner for synchronized collection samples."""

from __future__ import annotations

import fcntl
import os
import select
import termios
import threading
import time
from collections import deque
from dataclasses import dataclass

from ..configuration import FTSensorConfig
from ..hkvl_serial import FrameDecoder


@dataclass(frozen=True)
class FTSample:
    values: tuple[float, float, float, float, float, float]
    monotonic_ns: int
    sequence: int


class HKVLCollectionSource:
    """Own one serial endpoint and retain recent decoded frames in host time."""

    def __init__(self, config: FTSensorConfig) -> None:
        self._config = config
        self._condition = threading.Condition()
        self._samples: deque[FTSample] = deque(maxlen=1024)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fd: int | None = None
        self._original: list[object] | None = None
        self._error: BaseException | None = None

    def __enter__(self) -> HKVLCollectionSource:
        if self._fd is not None:
            raise RuntimeError("HKVL serial source is already open")
        speed = getattr(termios, f"B{self._config.baudrate}", None)
        if speed is None:
            raise ValueError(f"unsupported HKVL baudrate: {self._config.baudrate}")
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
        self._thread = threading.Thread(target=self._read_loop, name="hkvl-collection", daemon=True)
        self._thread.start()
        return self

    def _read_loop(self) -> None:
        assert self._fd is not None
        decoder = FrameDecoder()
        sequence = 0
        try:
            while not self._stop.is_set():
                ready, _, _ = select.select([self._fd], [], [], 0.05)
                if not ready:
                    continue
                chunk = os.read(self._fd, 4096)
                if not chunk:
                    raise OSError("HKVL serial adapter closed or disconnected")
                received_ns = time.monotonic_ns()
                frames = decoder.feed(chunk)
                if frames:
                    with self._condition:
                        for frame in frames:
                            sequence += 1
                            self._samples.append(FTSample(frame.values, received_ns, sequence))
                        self._condition.notify_all()
        except BaseException as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()

    def closest(self, *, target_monotonic_ns: int, timeout_s: float) -> FTSample:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise RuntimeError("HKVL serial stream failed") from self._error
                if self._samples and self._samples[-1].monotonic_ns >= target_monotonic_ns:
                    return min(
                        self._samples,
                        key=lambda item: abs(item.monotonic_ns - target_monotonic_ns),
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if self._samples:
                        return min(
                            self._samples,
                            key=lambda item: abs(item.monotonic_ns - target_monotonic_ns),
                        )
                    raise TimeoutError("no valid HKVL frame received")
                self._condition.wait(remaining)

    def wait_first(self, *, timeout_s: float, max_age_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise RuntimeError("HKVL serial stream failed") from self._error
                if len(self._samples) >= 2 and (
                    time.monotonic_ns() - self._samples[-1].monotonic_ns
                    <= max_age_s * 1_000_000_000
                ):
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("no fresh HKVL stream received")
                self._condition.wait(remaining)

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
