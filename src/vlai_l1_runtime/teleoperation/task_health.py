"""Run read-only OS health queries outside the task producer's heartbeat loop."""

from __future__ import annotations

import time
from threading import Event, Lock, Thread


class TaskHealthMonitor:
    """One bounded worker; stale or failed checks still fault the supervisor.

    The native owner independently checks live CAN health and motor feedback.
    This worker supplements it with OS counters and physical interface identity.
    It never sends motor commands or producer heartbeats.
    """

    def __init__(self, check, *, interval_s, timeout_s):
        self._check = check
        self._interval = interval_s
        self._timeout = timeout_s
        self._stop = Event()
        self._lock = Lock()
        self._last_success = time.monotonic()
        self._error = None
        self.maximum_check_s = 0.0
        self._thread = Thread(target=self._run, name="task-os-health", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            began = time.monotonic()
            try:
                self._check()
            except Exception as exc:
                with self._lock:
                    self._error = exc
                return
            finished = time.monotonic()
            with self._lock:
                self.maximum_check_s = max(self.maximum_check_s, finished - began)
                # A delayed result cannot revive an expired health check.
                if finished - self._last_success >= self._timeout:
                    self._error = TimeoutError("task OS health check expired")
                    return
                self._last_success = finished
            self._stop.wait(max(0.0, self._interval - (finished - began)))

    def check(self):
        with self._lock:
            error, last_success = self._error, self._last_success
        if error is not None:
            raise RuntimeError(f"task OS health: {error}") from error
        if time.monotonic() - last_success >= self._timeout:
            raise TimeoutError("task OS health check expired")

    def close(self):
        self._stop.set()
        self._thread.join(timeout=self._timeout)
