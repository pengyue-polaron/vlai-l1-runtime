"""Status checks must support sudo-owned controllers without accepting stale files."""

import json
from types import SimpleNamespace

import pytest

from vlai_l1_runtime import cli


@pytest.mark.parametrize(
    ("probe_error", "age_ns", "expected_live"),
    [
        (None, 20_000_000, True),
        (PermissionError, 20_000_000, True),
        (PermissionError, 1_000_000_000, False),
        (PermissionError, -1, False),
        (ProcessLookupError, 20_000_000, False),
        (OSError, 20_000_000, False),
    ],
)
def test_status_owner_probe(tmp_path, monkeypatch, capsys, probe_error, age_ns, expected_live):
    now = 10_000_000_000
    (tmp_path / "task-status.json").write_text(
        json.dumps(
            {
                "phase": "ready",
                "owner_pid": 123,
                "native": {"monotonic_ns": now - age_ns},
            }
        )
    )

    def probe(pid, signal):
        assert (pid, signal) == (123, 0)
        if probe_error is not None:
            raise probe_error()

    monkeypatch.setattr("os.kill", probe)
    monkeypatch.setattr(cli.time, "monotonic_ns", lambda: now)
    result = cli._run_task_command(SimpleNamespace(task_command="status", session=tmp_path))
    assert result == (0 if expected_live else 1)
    assert json.loads(capsys.readouterr().out)["owner_observed_live"] is expected_live
