import copy

import pytest

from vlai_l1_runtime.teleoperation import feedback_capture as fc


def baseline():
    return {
        "parentdev": "1-3.1.4:1.0",
        "linkinfo": {
            "info_data": {"state": "STOPPED", "berr_counter": {"tx": 128, "rx": 0}},
            "info_xstats": {"error_warning": 1, "error_passive": 1},
        },
        "stats64": {"rx": {"errors": 0, "dropped": 0}, "tx": {"errors": 0, "dropped": 0}},
    }


def test_historical_errors_preserved_but_current_must_be_healthy(monkeypatch):
    before = baseline()
    current = copy.deepcopy(before)
    current["linkinfo"]["info_data"] = {"state": "ERROR-ACTIVE", "berr_counter": {"tx": 0, "rx": 0}}
    monkeypatch.setattr(fc, "_link", lambda _: current)
    monkeypatch.setattr(fc.lifecycle, "_qdisc_drops", lambda _: 0)
    fc._require_healthy("can0", before)
    assert before == baseline()
    current["linkinfo"]["info_data"]["berr_counter"]["tx"] = 1
    with pytest.raises(RuntimeError):
        fc._require_healthy("can0", before)
    current["linkinfo"]["info_data"]["berr_counter"]["tx"] = 0
    current["linkinfo"]["info_xstats"]["error_warning"] += 1
    with pytest.raises(RuntimeError):
        fc._require_healthy("can0", before)


def test_passive_current_state_rejected_even_with_zero_counters(monkeypatch):
    before = baseline()
    current = copy.deepcopy(before)
    current["linkinfo"]["info_data"] = {
        "state": "ERROR-PASSIVE",
        "berr_counter": {"tx": 0, "rx": 0},
    }
    monkeypatch.setattr(fc, "_link", lambda _: current)
    monkeypatch.setattr(fc.lifecycle, "_qdisc_drops", lambda _: 0)
    with pytest.raises(RuntimeError):
        fc._require_healthy("can0", before)
