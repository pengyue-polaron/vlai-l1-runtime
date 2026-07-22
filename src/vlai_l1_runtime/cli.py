"""Hardware-free command line interface."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from .collection.configuration import load_collection_config
from .collection.dataset import (
    identity_from_config,
    inspect_direct_dataset,
    provenance_from_config,
)
from .collection.schema import DATASET_SCHEMA, canonical_dataset_contract
from .collection.v21 import export_v21_dataset
from .configuration import MOTOR_NAMES, ConfigError, load_system_config
from .contracts import RobotDescription, robot_description
from .teleoperation import (
    XAirStateReceiver,
    describe_xair_side,
    prepare_xair_assets,
    verify_xair_dependency,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="vlai-l1")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command, help_text in (
        ("validate-config", "validate the tracked System contract"),
        ("describe", "print the static robot contract as JSON"),
        ("verify-xair", "verify the pinned x_air SDK without opening hardware"),
    ):
        child = subparsers.add_parser(command, help=help_text)
        child.add_argument("--config", type=Path, required=True)
    xair = subparsers.add_parser(
        "describe-xair", help="print one side's static x_air launch contract"
    )
    xair.add_argument("--config", type=Path, required=True)
    xair.add_argument("--side", choices=("left", "right"), required=True)
    prepare_xair = subparsers.add_parser(
        "prepare-xair", help="render checked x_air assets without opening hardware"
    )
    prepare_xair.add_argument("--config", type=Path, required=True)
    prepare_xair.add_argument("--output", type=Path, required=True)
    observe_xair = subparsers.add_parser(
        "observe-xair", help="read sidecar state without opening robot hardware"
    )
    observe_xair.add_argument("--config", type=Path, required=True)
    observe_xair.add_argument("--side", choices=("left", "right", "bimanual"), required=True)
    observe_xair.add_argument("--samples", type=int, required=True)
    observe_xair.add_argument("--timeout", type=float, default=1.0)
    for command, help_text in (
        ("validate-collection", "validate collection and System contracts"),
        ("describe-collection", "print the canonical dataset contract as JSON"),
        ("collect", "record one commissioned live episode"),
        ("dataset-doctor", "validate one canonical LeRobot v3 dataset"),
        ("export-v21", "export one canonical dataset to LeRobot v2.1"),
        ("panel", "serve the hardware-free VLAI L1 Operator Panel"),
    ):
        child = subparsers.add_parser(command, help=help_text)
        child.add_argument("--config", type=Path, required=True)
        if command in {"collect", "dataset-doctor", "export-v21"}:
            child.add_argument("--experiment", required=True)
        if command == "collect":
            child.add_argument("--task", required=True)
            child.add_argument("--frames", type=int, required=True)
            child.add_argument("--decision", choices=("save", "discard"), required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "observe-xair":
            return _run_xair_observer(args)
        if args.command in {
            "validate-config",
            "describe",
            "verify-xair",
            "describe-xair",
            "prepare-xair",
        }:
            return _run_system_command(
                args.command,
                args.config,
                side=getattr(args, "side", None),
                output=getattr(args, "output", None),
            )
        return _run_collection_command(args)
    except (ConfigError, ValueError, RuntimeError, OSError) as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        return 2


def _run_system_command(
    command: str,
    path: Path,
    *,
    side: str | None = None,
    output: Path | None = None,
) -> int:
    config = load_system_config(path)
    if command == "validate-config":
        print(f"PASS {config.path}")
        return 0
    if command == "verify-xair":
        report = verify_xair_dependency(config)
        print(
            json.dumps(
                {
                    "status": "PASS",
                    "revision": report.revision,
                    "architecture": report.architecture,
                    "sdk_version": report.sdk_version,
                    "teleop_library": str(report.teleop_library),
                    "teleop_library_sha256": report.teleop_library_sha256,
                    "can_library": str(report.can_library),
                    "can_library_sha256": report.can_library_sha256,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0
    if command == "describe-xair":
        if side is None:
            raise ValueError("describe-xair requires a side")
        print(json.dumps(describe_xair_side(config, side), indent=2, sort_keys=True))
        return 0
    if command == "prepare-xair":
        if output is None:
            raise ValueError("prepare-xair requires an output directory")
        print(json.dumps({"status": "PASS", "manifest": str(prepare_xair_assets(config, output))}))
        return 0
    print(json.dumps(_description_json(robot_description(config)), indent=2, sort_keys=True))
    return 0


def _run_collection_command(args: argparse.Namespace) -> int:
    config = load_collection_config(args.config)
    if args.command == "validate-collection":
        print(f"PASS {config.path}")
        return 0
    if args.command == "describe-collection":
        contract = canonical_dataset_contract(config.system)
        print(
            json.dumps(
                {
                    "dataset_schema": DATASET_SCHEMA,
                    "repo_id_prefix": config.repo_id_prefix,
                    "fps": config.fps,
                    "features": contract.features(),
                    "collection_ready": config.collection_ready,
                    "collection_blockers": list(config.collection_blockers),
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0
    if args.command == "panel":
        from embodied_ops.operator_panel import serve_operator_panel

        from .panel import L1OperatorPanelAdapter

        adapter = L1OperatorPanelAdapter(config.repo_root, config.path)
        return serve_operator_panel(
            adapter,
            bind=adapter.panel_bind,
            port=adapter.panel_port,
        )
    if args.command == "collect":
        from embodied_ops import EpisodeDecision

        from .collection.live import collect_live_episode

        result = collect_live_episode(
            config,
            experiment=args.experiment,
            task=args.task,
            frame_count=args.frames,
            decision=EpisodeDecision(args.decision),
        )
        print(
            json.dumps(
                {
                    "decision": result.decision.value,
                    "frames": result.frame_count,
                    "dataset_root": None
                    if result.dataset_root is None
                    else str(result.dataset_root),
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    identity = identity_from_config(config, args.experiment)
    expected_provenance = provenance_from_config(config)
    if args.command == "dataset-doctor":
        state = inspect_direct_dataset(identity, expected_provenance=expected_provenance)
        if state.total_episodes == 0:
            raise ValueError(f"canonical dataset does not exist: {identity.target_root}")
        print(
            json.dumps(
                {
                    "status": "PASS",
                    "root": str(identity.target_root),
                    "repo_id": identity.repo_id,
                    "episodes": state.total_episodes,
                    "frames": state.total_frames,
                    "task": state.task,
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0
    result = export_v21_dataset(
        source=identity,
        target_root=config.v21_root_for(args.experiment),
        repo_id=config.repo_id_for(args.experiment, derivative="v2.1"),
        expected_provenance=expected_provenance,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def _run_xair_observer(args: argparse.Namespace) -> int:
    if args.samples <= 0:
        raise ValueError("samples must be a positive integer")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    config = load_system_config(args.config)
    snapshots: list[dict[str, object]] = []
    with XAirStateReceiver(config) as receiver:
        if args.side == "bimanual":
            for _ in range(args.samples):
                sample = receiver.receive(timeout_s=args.timeout)
                if sample is None:
                    raise TimeoutError("timed out waiting for paired x_air state")
                observation, action = sample
                snapshots.append(
                    {
                        "source_sequence": observation.metadata.source_sequence,
                        "monotonic_ns": observation.metadata.monotonic_ns,
                        "observation_deg": dict(observation.values),
                        "action_deg": dict(action.values),
                    }
                )
            print(
                json.dumps(
                    {"status": "PASS", "mode": "bimanual", "samples": snapshots},
                    indent=2,
                    sort_keys=True,
                )
            )
            return 0
        for _ in range(args.samples):
            deadline = time.monotonic() + args.timeout
            while True:
                remaining = max(0.0, deadline - time.monotonic())
                packet = receiver.receive_packet(timeout_s=remaining)
                if packet is None:
                    raise TimeoutError(f"timed out waiting for x_air {args.side} state")
                if packet.side == args.side:
                    break
            snapshots.append(
                {
                    "side": packet.side,
                    "source_sequence": packet.source_sequence,
                    "monotonic_ns": packet.monotonic_ns,
                    "leader_deg": {
                        motor: math.degrees(value)
                        for motor, value in zip(MOTOR_NAMES, packet.leader_radians, strict=True)
                    },
                    "follower_deg": {
                        motor: math.degrees(value)
                        for motor, value in zip(MOTOR_NAMES, packet.follower_radians, strict=True)
                    },
                }
            )
    print(json.dumps({"status": "PASS", "samples": snapshots}, indent=2, sort_keys=True))
    return 0


def _description_json(description: RobotDescription) -> dict[str, object]:
    return {
        "robot_id": description.robot_id,
        "topology_id": description.topology_id,
        "observation_features": [feature.__dict__ for feature in description.observation_features],
        "action_features": [feature.__dict__ for feature in description.action_features],
        "teleoperation_ready": description.teleoperation_ready,
        "teleoperation_blockers": list(description.teleoperation_blockers),
        "command_ready": description.command_ready,
        "command_blockers": list(description.command_blockers),
        "camera_roles": list(description.camera_roles),
        "collection_ready": description.collection_ready,
    }


if __name__ == "__main__":
    raise SystemExit(main())
