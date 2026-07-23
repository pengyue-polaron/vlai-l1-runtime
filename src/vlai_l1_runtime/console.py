"""Dependency-free terminal presentation for operator-facing commands."""

from __future__ import annotations

import os
import sys
import threading
import time
from collections.abc import Callable
from enum import Enum
from typing import TextIO


class Tone(Enum):
    INFO = "1;34"
    STEP = "1;36"
    SUCCESS = "1;32"
    WARNING = "1;33"
    FAILURE = "1;31"


_LEVEL_TONES = {
    "INFO": Tone.INFO,
    "STEP": Tone.STEP,
    "PASS": Tone.SUCCESS,
    "WARN": Tone.WARNING,
    "FAIL": Tone.FAILURE,
}
_OUTPUT_LOCK = threading.RLock()
_ACTIVE_STATUS: LiveStatusLine | None = None


def color_enabled(stream: TextIO = sys.stdout) -> bool:
    return not os.environ.get("NO_COLOR") and stream.isatty()


def label(level: str, *, stream: TextIO = sys.stdout) -> str:
    normalized = level.upper()
    text = f"[{normalized}]"
    if not color_enabled(stream):
        return text
    return f"\033[{_LEVEL_TONES.get(normalized, Tone.INFO).value}m{text}\033[0m"


def emit(level: str, message: str, *, stream: TextIO = sys.stderr) -> None:
    with _OUTPUT_LOCK:
        status = _ACTIVE_STATUS
        if status is not None and status._stream is stream and status._visible_width:
            stream.write("\r\033[2K")
            print(f"{label(level, stream=stream)} {message}", file=stream)
            status._render()
            stream.flush()
            return
        print(f"{label(level, stream=stream)} {message}", file=stream, flush=True)


def info(message: str) -> None:
    emit("INFO", message, stream=sys.stderr)


def step(message: str) -> None:
    emit("STEP", message, stream=sys.stderr)


def success(message: str) -> None:
    emit("PASS", message, stream=sys.stderr)


def warning(message: str) -> None:
    emit("WARN", message, stream=sys.stderr)


def failure(message: str) -> None:
    emit("FAIL", message, stream=sys.stderr)


class LiveStatusLine:
    """Replace one TTY line while throttling redirected progress output."""

    def __init__(
        self,
        *,
        stream: TextIO = sys.stderr,
        redirected_interval_s: float = 5.0,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        global _ACTIVE_STATUS

        if redirected_interval_s <= 0:
            raise ValueError("redirected status interval must be positive")
        self._stream = stream
        self._redirected_interval_s = redirected_interval_s
        self._monotonic = monotonic
        self._tty = stream.isatty()
        self._visible_width = 0
        self._last_redirected_at: float | None = None
        self._message: str | None = None
        if self._tty:
            with _OUTPUT_LOCK:
                if _ACTIVE_STATUS is not None:
                    raise RuntimeError("only one live terminal status may be active")
                _ACTIVE_STATUS = self

    def update(self, message: str, *, force: bool = False) -> None:
        if self._tty:
            with _OUTPUT_LOCK:
                self._message = message
                self._render()
                self._stream.flush()
            return
        now = self._monotonic()
        if (
            force
            or self._last_redirected_at is None
            or now - self._last_redirected_at >= self._redirected_interval_s
        ):
            print(f"[RUN] {message}", file=self._stream, flush=True)
            self._last_redirected_at = now

    def close(self) -> None:
        global _ACTIVE_STATUS

        if self._tty:
            with _OUTPUT_LOCK:
                if self._visible_width:
                    self._stream.write("\n")
                    self._stream.flush()
                    self._visible_width = 0
                if _ACTIVE_STATUS is self:
                    _ACTIVE_STATUS = None

    def _render(self) -> None:
        if self._message is None:
            return
        rendered = f"{label('RUN', stream=self._stream)} {self._message}"
        visible_width = len(self._message) + 6
        padding = " " * max(0, self._visible_width - visible_width)
        self._stream.write(f"\r{rendered}{padding}")
        self._visible_width = visible_width
