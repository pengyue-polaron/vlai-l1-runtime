"""OS queries may be slow, but must not block the task producer or mask faults."""

import time
from threading import Event

import pytest

from vlai_l1_runtime.teleoperation.task_health import TaskHealthMonitor


def test_slow_query_does_not_block_producer():
    entered, release = Event(), Event()

    def slow_check():
        entered.set()
        assert release.wait(2)

    monitor = TaskHealthMonitor(slow_check, interval_s=0.1, timeout_s=1)
    try:
        assert entered.wait(1)
        began = time.monotonic()
        for _ in range(100):
            monitor.check()
        assert time.monotonic() - began < 0.1
    finally:
        release.set()
        monitor.close()


def test_pending_health_check_expires_and_late_result_cannot_revive_it():
    entered, release = Event(), Event()

    def hung_check():
        entered.set()
        assert release.wait(2)

    monitor = TaskHealthMonitor(hung_check, interval_s=0.01, timeout_s=0.1)
    try:
        assert entered.wait(1)
        time.sleep(0.12)
        with pytest.raises(TimeoutError, match="health check expired"):
            monitor.check()
        release.set()
        monitor.close()
        with pytest.raises(RuntimeError, match="health check expired"):
            monitor.check()
    finally:
        release.set()
        monitor.close()


def test_health_fault_is_delivered_to_supervisor():
    failed = Event()

    def unhealthy():
        failed.set()
        raise ValueError("physical identity changed")

    monitor = TaskHealthMonitor(unhealthy, interval_s=0.1, timeout_s=1)
    assert failed.wait(1)
    monitor.close()
    with pytest.raises(RuntimeError, match="physical identity changed"):
        monitor.check()
