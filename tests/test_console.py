from __future__ import annotations

import io

from vlai_l1_runtime import console


class _Tty(io.StringIO):
    def isatty(self) -> bool:
        return True


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


def test_log_message_preserves_active_tty_status() -> None:
    stream = _Tty()
    status = console.LiveStatusLine(stream=stream)
    try:
        status.update("Recording frame 4/300")
        console.emit("PASS", "runtime ready", stream=stream)
    finally:
        status.close()

    output = stream.getvalue()
    assert "\033[2K" in output
    assert "[PASS]" in output
    assert output.count("Recording frame 4/300") == 2
