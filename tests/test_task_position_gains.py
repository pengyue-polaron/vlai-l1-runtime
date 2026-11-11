import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.task_runtime import render_task_spec
from vlai_l1_runtime.teleoperation.automatic import render_native_spec

ROOT = Path(__file__).resolve().parents[1]


def test_task_gains_change_only_j7_and_preserve_diagnostic_and_guards():
    config = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    policy = json.loads((ROOT / "configs/tasks/right_follower_control.json").read_text())
    original = dict(policy)
    original.pop("position_gains")
    task = SimpleNamespace(settings={"observation_max_age_s": 1.5, "producer_timeout_s": 0.5})
    before = render_native_spec(config, original).splitlines()
    after = render_task_spec(config, policy, task).splitlines()
    for i in range(8):
        a, b = before[3 + i].split(), after[3 + i].split()
        if i == 6:
            assert list(map(float, a[8:10])) == [25, 0.2]
            assert list(map(float, b[8:10])) == [50, 0.4]
            assert a[:8] == b[:8]
            assert a[10:] == b[10:]
        else:
            assert a == b
    assert config.control["follower"].kp[6] == 25
    assert before == render_native_spec(config, original).splitlines()


@pytest.mark.parametrize("bad", [0, -1, float("nan"), 501, True])
def test_invalid_task_gain_is_rejected_before_launch(bad):
    config = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    policy = json.loads((ROOT / "configs/tasks/right_follower_control.json").read_text())
    policy["position_gains"]["kp"][6] = bad
    task = SimpleNamespace(settings={"observation_max_age_s": 1.5, "producer_timeout_s": 0.5})
    with pytest.raises(ValueError, match="MIT range"):
        render_task_spec(config, policy, task)
