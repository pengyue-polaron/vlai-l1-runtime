from __future__ import annotations

import io

from embodied_ops import console as shared_console

from vlai_l1_runtime import console


def test_redirected_status_is_throttled_and_has_no_ansi() -> None:
    stream = io.StringIO()
    times = iter((0.0, 1.0, 5.0))
    status = console.LiveStatusLine(
        stream=stream,
        redirected_interval_s=5.0,
        monotonic=lambda: next(times),
    )

    status.update("one")
    status.update("two")
    status.update("three")
    status.close()

    assert stream.getvalue() == "[RUN] one\n[RUN] three\n"


def test_console_is_the_shared_embodied_ops_implementation() -> None:
    assert console.ArgumentParser is shared_console.ArgumentParser
    assert console.LiveStatusLine is shared_console.LiveStatusLine
    assert console.emit is shared_console.emit
