"""Warm Unified with a saved real observation; prediction only, no motor access."""

import argparse
import json
import time
from pathlib import Path

from websockets.exceptions import InvalidHandshake

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.pi0_inference import (
    Pi0Observation,
    Pi0PredictionClient,
    Pi0Spec,
    unpack_message,
)
from vlai_l1_runtime.task_definition import load_task
from vlai_l1_runtime.task_runtime import check_target


def ready_client(spec):
    deadline = time.monotonic() + 180
    while True:
        client = Pi0PredictionClient(spec, expected_kind="unified")
        try:
            return client.__enter__()
        except (OSError, TimeoutError, EOFError, InvalidHandshake):
            if time.monotonic() >= deadline:
                raise
            time.sleep(1)


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--request",
        type=Path,
        default=root / "data/inference-validation-20261108/requests/unified-10cm.msgpack",
    )
    args = parser.parse_args()
    config = load_system_config(root / "configs/system/vlai_l1.toml")
    task = load_task(config, root / "configs/tasks/push-distance-unified.json")
    spec = Pi0Spec.load(task.inference_spec, kind="unified")
    payload = unpack_message(args.request.read_bytes())
    if payload["target_distance_cm"] != task.distance_cm or payload["prompt"] != task.prompt:
        raise ValueError("warmup observation must match configured task distance and prompt")
    observation = Pi0Observation(payload, 0, 0)
    client = ready_client(spec)
    try:
        if client.server_metadata.get("checkpoint") != task.expected_checkpoint:
            raise ValueError("Unified checkpoint identity mismatch")
        if client.server_metadata.get("history_delivery") != "full_causal_prefix":
            raise ValueError("Unified requires full causal history delivery")
        for index in range(2):
            started = time.monotonic()
            actions, _ = client.predict(observation)
            for row in actions:
                check_target(config, row)
            print(
                json.dumps(
                    {
                        "model": "unified-29999",
                        "source": str(args.request),
                        "saved_observation": True,
                        "motor_commands_sent": False,
                        "pass": index + 1,
                        "shape": list(actions.shape),
                        "roundtrip_s": time.monotonic() - started,
                    }
                ),
                flush=True,
            )
    finally:
        client.close()


if __name__ == "__main__":
    main()
