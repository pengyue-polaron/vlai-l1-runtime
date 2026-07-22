"""Hardware-free command line interface."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from .collection.configuration import load_collection_config
from .collection.dataset import (
    identity_from_config,
    inspect_direct_dataset,
    provenance_from_config,
)
from .collection.schema import canonical_dataset_contract
from .collection.v21 import export_v21_dataset
from .configuration import ConfigError, load_system_config
from .contracts import RobotDescription, robot_description


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="vlai-l1")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command, help_text in (
        ("validate-config", "validate the tracked System contract"),
        ("describe", "print the static robot contract as JSON"),
    ):
        child = subparsers.add_parser(command, help=help_text)
        child.add_argument("--config", type=Path, required=True)
    for command, help_text in (
        ("validate-collection", "validate collection and System contracts"),
        ("describe-collection", "print the canonical dataset contract as JSON"),
        ("dataset-doctor", "validate one canonical LeRobot v3 dataset"),
        ("export-v21", "export one canonical dataset to LeRobot v2.1"),
        ("panel", "serve the hardware-free VLAI L1 Operator Panel"),
    ):
        child = subparsers.add_parser(command, help=help_text)
        child.add_argument("--config", type=Path, required=True)
        if command in {"dataset-doctor", "export-v21"}:
            child.add_argument("--experiment", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command in {"validate-config", "describe"}:
            return _run_system_command(args.command, args.config)
        return _run_collection_command(args)
    except (ConfigError, ValueError, RuntimeError, OSError) as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        return 2


def _run_system_command(command: str, path: Path) -> int:
    config = load_system_config(path)
    if command == "validate-config":
        print(f"PASS {config.path}")
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
                    "dataset_schema": "vlai_l1_lerobot_dataset_v3_v1",
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


def _description_json(description: RobotDescription) -> dict[str, object]:
    return {
        "robot_id": description.robot_id,
        "topology_id": description.topology_id,
        "observation_features": [feature.__dict__ for feature in description.observation_features],
        "action_features": [feature.__dict__ for feature in description.action_features],
        "command_ready": description.command_ready,
        "command_blockers": list(description.command_blockers),
        "camera_roles": list(description.camera_roles),
        "collection_ready": description.collection_ready,
    }


if __name__ == "__main__":
    raise SystemExit(main())
