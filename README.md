# VLAI L1 Runtime

This repository is the hardware-specific composition root for the VLAI L1
bimanual robot. It defines the robot and camera contracts, command-session
safety model, and the L1 side of collection and dataset operations.

No live driver or command transport exists in this milestone. The tracked
configuration is deliberately not command-ready because the production C++
teleoperation source is unavailable, the J2 coordinate contract is unresolved,
and the previous `can2` bus-off event has not been closed out.

## Boundaries

- This repository owns VLAI L1 devices, physical limits, lifecycle, safety,
  cameras, and robot-specific collection/evaluation composition.
- LeRobot Robot and Teleoperator plugins are separate thin clients. They will
  use this Runtime and must not open CAN devices.
- `embodied-ops` supplies reusable episode, timing, artifact, and Operator Panel
  primitives. L1 feature names, datasets, provenance, and hardware readiness
  remain here.
- The existing `~/xarm_ros2_ws` deployment remains untouched and authoritative
  for current manual teleoperation until a reviewed Runtime replaces it.

## Dataset contract

The canonical dataset is written directly as LeRobot v3.0. Each frame contains:

- `observation.state`: 16 named left/right joint and gripper positions in degrees;
- `action`: the same 16-name, degree-valued convention;
- `observation.images.agent` and `observation.images.wrist`;
- one normalized task string.

Samples are accepted only when joint vectors are exact and finite, timestamps
are fresh, state/action and camera pairs are synchronized, sequences increase,
configured limits hold, and the action step stays within the tracked collection
contract. An episode is finalized in a hidden sibling snapshot and becomes
visible only through an atomic rename. Failed and discarded episodes never
replace the last complete dataset.

The collection config is [default.toml](configs/collection/default.toml). Dataset
layout, provenance, doctor behavior, and v2.1 derivatives are documented in
[Datasets](docs/DATASETS.md).

## Hardware-free operations

```bash
vlai-l1 validate-config --config configs/system/vlai_l1.toml
vlai-l1 describe --config configs/system/vlai_l1.toml
vlai-l1 validate-collection --config configs/collection/default.toml
vlai-l1 describe-collection --config configs/collection/default.toml
vlai-l1 dataset-doctor --config configs/collection/default.toml --experiment <name>
vlai-l1 export-v21 --config configs/collection/default.toml --experiment <name>
vlai-l1 panel --config configs/collection/default.toml
```

Validation and description do not import LeRobot, Torch, camera libraries, or
device APIs. Dataset recording, deep doctor checks, and v2.1 export use the
optional collection dependencies. The Panel exposes only validation, doctor,
and export while live readiness gates remain unresolved.

```bash
git submodule update --init --recursive
uv sync --python 3.12 --extra collection
```

No live collection command is present yet. The production transport and both
camera identities must be commissioned before that workflow can be added.

The repository pins `embodied-ops` as a submodule; initialize it before running
repository commands. Runtime contracts remain importable on Python 3.10; the
LeRobot collection, doctor, and export processes require Python 3.12.

## Development

```bash
uv sync --dev
uv run pytest
uv run ruff check .
uv run ruff format --check .
```

Read [Architecture](docs/ARCHITECTURE.md), [Safety](docs/SAFETY.md),
[Datasets](docs/DATASETS.md), and [Source gaps](SOURCE_GAPS.md) before extending
the Runtime.
