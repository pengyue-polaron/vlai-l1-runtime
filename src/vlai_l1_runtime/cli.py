"""VLAI L1 command line interface."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from embodied_ops import (
    print_dataset_report,
    print_export_report,
    standard_dataset_report,
    standard_export_report,
)

from . import console
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
    run_xair_side,
    verify_xair_dependency,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_COLLECTION_CONFIG = _REPO_ROOT / "configs/collection/default.toml"
_RIGHT_COLLECTION_CONFIG = _REPO_ROOT / "configs/collection/right_only.toml"


def build_parser() -> argparse.ArgumentParser:
    parser = console.ArgumentParser(
        prog="vlai-l1",
        description="Checks, reports, and tracked VLAI L1 operator workflows.",
    )
    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
        metavar="{panel,hardware,dataset,collect,reset}",
    )
    panel = subparsers.add_parser("panel", help="serve the hardware-free VLAI L1 Operator Panel")
    _add_collection_selection(panel)
    hardware = subparsers.add_parser(
        "hardware",
        help="passively inspect selected CAN and camera hardware",
    )
    _add_collection_selection(hardware)
    hardware.add_argument("--json", action="store_true")
    dataset = subparsers.add_parser("dataset", help="inspect or export canonical datasets")
    dataset_commands = dataset.add_subparsers(dest="dataset_command", required=True)
    for command, help_text in (
        ("doctor", "validate one canonical LeRobot v3 dataset"),
        ("export-v21", "export one canonical dataset to LeRobot v2.1"),
    ):
        child = dataset_commands.add_parser(command, help=help_text)
        _add_collection_selection(child)
        child.add_argument("experiment")
        child.add_argument("--json", action="store_true")
    trim_stillness = dataset_commands.add_parser(
        "trim-leading-stillness",
        help="rebuild a canonical dataset without stationary episode prefixes",
    )
    _add_collection_selection(trim_stillness)
    trim_stillness.add_argument("source_experiment")
    trim_stillness.add_argument("target_experiment")
    trim_stillness.add_argument(
        "--dry-run",
        action="store_true",
        help="print the exact per-episode trim plan without creating the target",
    )
    trim_stillness.add_argument("--json", action="store_true")
    collect = subparsers.add_parser(
        "collect",
        help="MOVES HARDWARE: reset, then record commissioned live episodes",
    )
    _add_collection_selection(collect)
    collect.add_argument("experiment", nargs="?")
    collect.add_argument("--experiment", dest="legacy_experiment", help=argparse.SUPPRESS)
    collect.add_argument("--task", required=True)
    reset = subparsers.add_parser(
        "reset",
        help="MOVES HARDWARE: run x_air AdjustPosition on selected sides",
    )
    _add_collection_selection(reset)

    for command in ("validate-config", "describe", "verify-xair"):
        child = subparsers.add_parser(command)
        child.add_argument("--config", type=Path, required=True)
    xair = subparsers.add_parser("describe-xair")
    xair.add_argument("--config", type=Path, required=True)
    xair.add_argument("--side", choices=("left", "right"), required=True)
    prepare_xair = subparsers.add_parser("prepare-xair")
    prepare_xair.add_argument("--config", type=Path, required=True)
    prepare_xair.add_argument("--output", type=Path, required=True)
    observe_xair = subparsers.add_parser("observe-xair")
    observe_xair.add_argument("--config", type=Path, required=True)
    observe_xair.add_argument("--side", choices=("left", "right", "bimanual"), required=True)
    observe_xair.add_argument("--samples", type=int, required=True)
    observe_xair.add_argument("--timeout", type=float, default=1.0)
    run_xair = subparsers.add_parser("run-xair")
    run_xair.add_argument("--config", type=Path, required=True)
    run_xair.add_argument("--side", choices=("left", "right"), required=True)
    run_xair.add_argument(
        "--managed-startup-gate",
        action="store_true",
        help="wait after motion-free preflight for the bimanual parent release",
    )
    run_xair.add_argument(
        "--isolated-side",
        action="store_true",
        help="require the inactive arm pair to remain fully down",
    )
    camera_check = subparsers.add_parser("camera-check")
    camera_check.add_argument("--config", type=Path, required=True)
    camera_check.add_argument("--samples", type=int, required=True)
    camera_check.add_argument("--timeout", type=float, default=1.0)
    camera_service = subparsers.add_parser("camera-service")
    camera_service.add_argument("--config", type=Path, required=True)
    camera_service.add_argument("action", choices=("start", "stop", "status", "logs"))
    camera_service_run = subparsers.add_parser("camera-service-run")
    camera_service_run.add_argument("--config", type=Path, required=True)
    for command in ("validate-collection", "describe-collection"):
        child = subparsers.add_parser(command)
        _add_collection_selection(child)
    for command in ("dataset-doctor", "export-v21"):
        child = subparsers.add_parser(command)
        _add_collection_selection(child)
        child.add_argument("--experiment", required=True)
        child.add_argument("--json", action="store_true")
    return parser


def _add_collection_selection(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", type=Path)
    parser.add_argument(
        "--side",
        choices=("bimanual", "right"),
        default=None,
        help="select the bimanual or commissioned right-only collection contract",
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "observe-xair":
            return _run_xair_observer(args)
        if args.command == "run-xair":
            return run_xair_side(
                load_system_config(args.config),
                args.side,
                managed_startup_gate=args.managed_startup_gate,
                isolated_side=args.isolated_side,
            )
        if args.command == "camera-check":
            return _run_camera_check(args)
        if args.command in {"camera-service", "camera-service-run"}:
            return _run_camera_service_command(args)
        if args.command == "hardware":
            from .hardware_check import inspect_hardware, print_hardware_report

            config = _load_selected_collection_config(args)
            return print_hardware_report(
                inspect_hardware(config),
                json_output=args.json,
            )
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
    except KeyboardInterrupt:
        console.warning("Interrupted by operator")
        return 130
    except (ConfigError, ValueError, RuntimeError, OSError) as exc:
        console.failure(str(exc))
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
    config = _load_selected_collection_config(args)
    command = args.command
    if command == "dataset":
        command = {
            "doctor": "dataset-doctor",
            "export-v21": "export-v21",
            "trim-leading-stillness": "trim-leading-stillness",
        }[args.dataset_command]
    if command == "validate-collection":
        print(f"PASS {config.path}")
        return 0
    if command == "describe-collection":
        contract = canonical_dataset_contract(config)
        print(
            json.dumps(
                {
                    "dataset_schema": DATASET_SCHEMA,
                    "repo_id_prefix": config.repo_id_prefix,
                    "fps": config.fps,
                    "teleoperation_sides": list(config.teleoperation_sides),
                    "record_camera_roles": list(config.record_camera_roles),
                    "features": contract.features(),
                    "collection_ready": config.collection_ready,
                    "collection_blockers": list(config.collection_blockers),
                    "reset": {
                        "before_collection": config.reset_policy.before_collection,
                        "after_save": config.reset_policy.after_save,
                        "after_discard": config.reset_policy.after_discard,
                    },
                    "leading_stillness": {
                        "enabled": config.leading_stillness.enabled,
                        "reference_frames": config.leading_stillness.reference_frames,
                        "motion_frames": config.leading_stillness.motion_frames,
                        "preroll_frames": config.leading_stillness.preroll_frames,
                        "action_thresholds": list(config.leading_stillness.action_thresholds),
                    },
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0
    if command == "panel":
        from embodied_ops.operator_panel import serve_operator_panel

        from .panel import L1OperatorPanelAdapter

        adapter = L1OperatorPanelAdapter(config.repo_root, config.path)
        return serve_operator_panel(
            adapter,
            bind=adapter.panel_bind,
            port=adapter.panel_port,
        )
    if command == "collect":
        from .collection.managed import collect_managed_session

        experiment = args.experiment or args.legacy_experiment
        if experiment is None:
            raise ValueError("collect requires an experiment")
        collect_managed_session(
            config,
            experiment=experiment,
            task=args.task,
        )
        return 0
    if command == "reset":
        from .collection.managed import reset_managed_teleoperation

        reset_managed_teleoperation(config)
        return 0
    if command == "trim-leading-stillness":
        from .collection.migration import trim_leading_stillness_dataset

        result = trim_leading_stillness_dataset(
            config,
            source_experiment=args.source_experiment,
            target_experiment=args.target_experiment,
            dry_run=args.dry_run,
            episode_completed=lambda episode: console.emit(
                "INFO",
                f"Rebuilt episode {episode.episode_index}: "
                f"trimmed {episode.trimmed_frames} of {episode.source_frames} frames",
                stream=sys.stderr if args.json else sys.stdout,
            ),
        )
        if args.json:
            print(json.dumps(result, indent=2, sort_keys=True))
        else:
            console.success(
                "Leading-stillness trim plan ready"
                if args.dry_run
                else "Leading-stillness migration complete"
            )
            console.info(
                f"Dataset · source={result['source_experiment']} "
                f"· target={result['target_experiment']} · episodes={result['episodes']}"
            )
            console.info(
                f"Frames · source={result['source_frames']} "
                f"· trimmed={result['trimmed_frames']} · output={result['output_frames']}"
            )
        return 0

    identity = identity_from_config(config, args.experiment)
    expected_provenance = provenance_from_config(config)
    if command == "dataset-doctor":
        state = inspect_direct_dataset(identity, expected_provenance=expected_provenance)
        if state.total_episodes == 0:
            raise ValueError(f"canonical dataset does not exist: {identity.target_root}")
        report = standard_dataset_report(
            robot="vlai-l1",
            experiment=args.experiment,
            root=str(identity.target_root),
            repo_id=identity.repo_id,
            episodes=state.total_episodes,
            frames=state.total_frames,
            tasks=[state.task],
        )
        print_dataset_report(report, json_output=args.json)
        return 0
    result = export_v21_dataset(
        source=identity,
        target_root=config.v21_root_for(args.experiment),
        repo_id=config.repo_id_for(args.experiment, derivative="v2.1"),
        expected_provenance=expected_provenance,
    )
    report = standard_export_report(
        robot="vlai-l1",
        experiment=args.experiment,
        result=result,
    )
    print_export_report(report, json_output=args.json)
    return 0


def _load_selected_collection_config(args: argparse.Namespace):
    path = args.config
    selected_side = args.side or "bimanual"
    if path is None:
        path = _RIGHT_COLLECTION_CONFIG if selected_side == "right" else _DEFAULT_COLLECTION_CONFIG
    config = load_collection_config(path)
    expected_sides = ("right",) if selected_side == "right" else ("left", "right")
    if args.side is not None and config.teleoperation_sides != expected_sides:
        raise ValueError(
            f"--side {selected_side} does not match collection config sides "
            f"{config.teleoperation_sides}"
        )
    return config


def _run_xair_observer(args: argparse.Namespace) -> int:
    if args.samples <= 0:
        raise ValueError("samples must be a positive integer")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    config = load_system_config(args.config)
    snapshots: list[dict[str, object]] = []
    with XAirStateReceiver(config) as receiver:
        if args.side == "bimanual":
            first_sequence: int | None = None
            first_monotonic_ns: int | None = None
            observation = None
            action = None
            for _ in range(args.samples):
                sample = receiver.receive(timeout_s=args.timeout)
                if sample is None:
                    raise TimeoutError("timed out waiting for paired x_air state")
                observation, action = sample
                if first_sequence is None:
                    first_sequence = observation.metadata.source_sequence
                    first_monotonic_ns = observation.metadata.monotonic_ns
            assert observation is not None
            assert action is not None
            assert first_sequence is not None
            assert first_monotonic_ns is not None
            print(
                json.dumps(
                    {
                        "status": "PASS",
                        "mode": "bimanual",
                        "sample_count": args.samples,
                        "first_source_sequence": first_sequence,
                        "last_source_sequence": observation.metadata.source_sequence,
                        "first_monotonic_ns": first_monotonic_ns,
                        "last_monotonic_ns": observation.metadata.monotonic_ns,
                        "observation_deg": dict(observation.values),
                        "action_deg": dict(action.values),
                    },
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


def _run_camera_check(args: argparse.Namespace) -> int:
    from .camera_bridge import check_camera_source
    from .camera_ipc import RawCameraBridgeClient
    from .camera_service import CameraServiceController

    config = load_system_config(args.config)
    CameraServiceController(config).start()
    with RawCameraBridgeClient(config) as cameras:
        report = check_camera_source(
            config,
            cameras,
            sample_count=args.samples,
            timeout_s=args.timeout,
        )
    print(
        json.dumps(
            {
                "status": "PASS",
                "sample_count": report.sample_count,
                "elapsed_s": report.elapsed_s,
                "effective_fps": report.effective_fps,
                "max_pair_skew_ms": report.max_pair_skew_ms,
                "streams": {
                    role: {
                        "device_id": stream.device_id,
                        "first_sequence": stream.first_sequence,
                        "last_sequence": stream.last_sequence,
                        "shape": stream.shape,
                        "configured_fps": stream.configured_fps,
                    }
                    for role, stream in report.streams.items()
                },
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def _run_camera_service_command(args: argparse.Namespace) -> int:
    from .camera_service import CameraServiceController, run_camera_service

    config = load_system_config(args.config)
    if args.command == "camera-service-run":
        return run_camera_service(config)
    controller = CameraServiceController(config)
    if args.action == "start":
        console.step("Starting or verifying persistent camera service")
        status = controller.start()
        console.success(
            f"Three-camera service ready · preview port {config.camera_preview.port} "
            f"· pid {status.pid}"
        )
    elif args.action == "stop":
        console.step("Stopping persistent camera service")
        status = controller.stop()
        console.success("Persistent camera service stopped")
    elif args.action == "logs":
        print(controller.log_tail())
        return 0
    else:
        status = controller.status()
        (console.success if status.healthy else console.info)(status.detail)
    print(
        json.dumps(
            {
                "status": "PASS" if status.healthy else "STOPPED",
                "running": status.running,
                "healthy": status.healthy,
                "pid": status.pid,
                "preview": (f"http://{config.camera_preview.bind}:{config.camera_preview.port}"),
                "raw_socket": str(config.camera_preview.bridge_socket_path),
                "log": str(status.log_path),
                "detail": status.detail,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if args.action != "status" or status.healthy else 1


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
