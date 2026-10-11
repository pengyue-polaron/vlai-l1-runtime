from __future__ import annotations

import struct
from pathlib import Path
from types import SimpleNamespace

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.motor_parameters import ParameterRead, ReadRegister
from vlai_l1_runtime.teleoperation import parameter_probe as probe

CONFIG = Path(__file__).resolve().parents[1] / "configs/system/vlai_l1.toml"


@pytest.fixture
def transport(monkeypatch):
    clock = [100]
    sdk = load_system_config(CONFIG)
    motor = sdk.motors[2]
    reply = struct.pack("=IB3x64s", motor.receive_id, 8, bytes.fromhex("0300331500004841"))
    state = SimpleNamespace(
        config=sdk,
        motor=motor,
        clock=clock,
        sent=[],
        replies=[(reply, [], 0, ())],
        queue=[],
        closed=False,
        options=[],
        short_send=False,
    )

    class Socket:
        def setsockopt(self, *args):
            state.options.append(args)

        def setblocking(self, blocking):
            assert blocking is False

        def bind(self, address):
            assert address == ("can2",)

        def send(self, payload):
            state.sent.append(payload)
            state.queue.extend(state.replies)
            return len(payload) - int(state.short_send)

        def recvmsg(self, *args):
            clock[0] += 1
            return state.queue.pop(0)

        def close(self):
            state.closed = True

    def select(read, write, error, timeout):
        if state.queue:
            return read, (), ()
        clock[0] += round(timeout * 1e9)
        return (), (), ()

    monkeypatch.setattr(probe.socket, "socket", lambda *args: Socket())
    monkeypatch.setattr(probe.time, "monotonic_ns", lambda: clock[0])
    monkeypatch.setattr(probe.select, "select", select)
    return state


def query(state):
    with probe._ParameterSocket("can2", state.motor, can_fd=True) as backend:
        return backend.query(
            ParameterRead("joint_3", 3, 19, ReadRegister.POSITION_MAX), deadline_ns=1000
        )


def test_live_backend_sends_exactly_one_read_and_preserves_raw_reply(transport):
    result = query(transport)
    assert len(transport.sent) == 1
    assert transport.sent[0] == struct.pack("=IB3x64s", 0x7FF, 8, bytes.fromhex("0300331500000000"))
    assert result["value"] == 12.5
    assert result["raw_response"]["payload_hex"] == "0300331500004841"
    assert transport.clock[0] == 1000
    assert transport.closed


@pytest.mark.parametrize(
    "fault", ["missing", "duplicate", "error", "short", "overflow", "truncated", "opcode"]
)
def test_bad_transaction_closes_without_retry(transport, fault):
    data, ancillary, flags, address = transport.replies[0]
    if fault == "missing":
        transport.replies.clear()
    elif fault == "duplicate":
        transport.replies *= 2
    elif fault == "short":
        transport.short_send = True
    else:
        if fault == "error":
            data = struct.pack("=I", 0x20000000) + data[4:]
        elif fault == "overflow":
            ancillary = [(probe.socket.SOL_SOCKET, probe._SO_RXQ_OVFL, struct.pack("=I", 1))]
        elif fault == "truncated":
            flags = probe.socket.MSG_CTRUNC
        elif fault == "opcode":
            data = data[:10] + b"\x55" + data[11:]
        transport.replies = [(data, ancillary, flags, address)]
    with pytest.raises((ValueError, RuntimeError, TimeoutError)):
        query(transport)
    assert len(transport.sent) == 1
    assert transport.closed


def test_queued_or_late_frame_prevents_the_next_query(transport):
    transport.queue = list(transport.replies)
    with pytest.raises(RuntimeError, match="before parameter query"):
        query(transport)
    assert not transport.sent
    assert transport.closed


def test_expired_window_sends_nothing(transport):
    transport.clock[0] = 1000
    with pytest.raises(TimeoutError, match="before send"):
        query(transport)
    assert not transport.sent
    assert transport.closed


def test_global_budget_includes_health_check_time_and_prevents_late_send(transport):
    def slow_health():
        transport.clock[0] += int(transport.config.teleoperation.motor_probe_duration_s * 1e9) + 1

    with pytest.raises(TimeoutError, match="before send"):
        probe.read_motor_parameters(
            transport.config,
            "can2",
            transport.motor,
            check_health=slow_health,
            on_reading=lambda value: pytest.fail("accepted late reply"),
        )
    assert not transport.sent
    assert transport.closed


def test_socket_setup_failure_releases_handle(transport, monkeypatch):
    class BrokenSocket:
        def setsockopt(self, *args):
            raise OSError("socket option unavailable")

        def close(self):
            transport.closed = True

    monkeypatch.setattr(probe.socket, "socket", lambda *args: BrokenSocket())
    with pytest.raises(OSError, match="socket option unavailable"):
        probe._ParameterSocket("can2", transport.motor, can_fd=True)
    assert transport.closed
    assert not transport.sent


def test_health_failure_prevents_any_parameter_send(transport):
    def unhealthy():
        raise RuntimeError("CAN health fault")

    with pytest.raises(RuntimeError, match="CAN health fault"):
        probe.read_motor_parameters(
            transport.config,
            "can2",
            transport.motor,
            check_health=unhealthy,
            on_reading=lambda value: pytest.fail("accepted unhealthy reply"),
        )
    assert not transport.sent
    assert transport.closed


def test_reader_uses_configured_budget_and_allowlist_only(transport):
    queries, readings, checks = [], [], []

    class Backend:
        def __init__(self, interface, motor, *, can_fd):
            assert interface == "can2"
            assert motor == transport.motor
            assert can_fd == transport.config.can.fd

        def __enter__(self):
            return self

        def __exit__(self, *args):
            transport.closed = True

        def query(self, request, *, deadline_ns):
            assert deadline_ns == 100 + int(
                1e9 / transport.config.teleoperation.motor_probe_rate_hz
            )
            queries.append(request)
            return {"register": request.register.name}

    probe.read_motor_parameters(
        transport.config,
        "can2",
        transport.motor,
        check_health=lambda: checks.append(1),
        on_reading=readings.append,
        _backend_factory=Backend,
    )
    assert [q.register for q in queries] == list(ReadRegister)
    assert len(readings) == 8
    assert len(checks) == 16
    assert transport.closed


def test_transaction_failure_stops_later_queries(transport):
    queries, readings = [], []

    class Backend:
        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            transport.closed = True

        def query(self, request, *, deadline_ns):
            queries.append(request)
            raise TimeoutError("missing parameter")

    with pytest.raises(TimeoutError, match="missing parameter"):
        probe.read_motor_parameters(
            transport.config,
            "can2",
            transport.motor,
            check_health=lambda: None,
            on_reading=readings.append,
            _backend_factory=Backend,
        )
    assert len(queries) == 1
    assert not readings
    assert transport.closed
