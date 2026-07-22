# VLAI L1 Runtime

This repository is the hardware-specific composition root for the VLAI L1
bimanual robot. It defines the robot and camera contracts, command-session
safety model, and the L1 side of collection and dataset operations. The
teleoperation integration is built against a pinned `x_air_sdk` checkout and
converts its per-side full-state callback into one versioned, named bimanual
observation contract.

The x_air integration is not commissioned yet. The tracked configuration keeps
teleoperation and policy commands unavailable while J2 coordinates, `can2`
stability, joint limits, and camera identities remain unresolved. The existing
onboard services remain the approved live path until staged tests close those
gates.

## Boundaries

- This repository owns VLAI L1 devices, physical limits, lifecycle, safety,
  cameras, and robot-specific collection/evaluation composition.
- The pinned x_air SDK owns each leader/follower CAN pair. The Runtime sidecar
  consumes its public callback; it does not open a second CAN handle or use the
  upstream ROS2 gripper bridge.
- LeRobot Robot and Teleoperator plugins are separate thin clients. They use
  this Runtime and must not open CAN devices.
- `embodied-ops` supplies reusable episode, timing, artifact, and Operator Panel
  primitives. L1 feature names, datasets, provenance, and hardware readiness
  remain here.
- The existing `~/xarm_ros2_ws` deployment remains untouched and authoritative
  for current manual teleoperation until the pinned candidate is commissioned.

## Dataset contract

The canonical dataset is written directly as LeRobot v3.0. Each frame contains:

- `observation.state`: 16 named left/right joint and gripper positions in degrees;
- `action`: the same 16-name, degree-valued convention;
- `observation.images.wrist_left` and `observation.images.wrist_right`;
- `observation.images.agent` when the optional AgentView camera is enabled;
- one normalized task string.

Samples are accepted only when joint vectors are exact and finite, timestamps
are fresh, state/action, left/right arm, camera pairs, and robot/camera samples
are synchronized, sequences increase, configured limits hold, and the action
step stays within the tracked collection contract. An episode is finalized in
a hidden sibling snapshot and becomes visible only through an atomic rename.
Failed and discarded episodes never replace the last complete dataset.

The collection config is [default.toml](configs/collection/default.toml). Dataset
layout, provenance, doctor behavior, and v2.1 derivatives are documented in
[Datasets](docs/DATASETS.md).

## Hardware-free operations

```bash
vlai-l1 validate-config --config configs/system/vlai_l1.toml
vlai-l1 describe --config configs/system/vlai_l1.toml
vlai-l1 verify-xair --config configs/system/vlai_l1.toml
vlai-l1 describe-xair --config configs/system/vlai_l1.toml --side right
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

The x_air dependency and native state sidecar can be built without hardware:

```bash
./scripts/build_xair.sh
```

The sidecar is deliberately not installed as a CLI or system service yet. The
SDK enables motors and performs position alignment while creating a session, so
its first execution belongs to the staged live-commissioning procedure. The
live collection command refuses to open devices until teleoperation and the
required camera identities are commissioned.

After those tracked gates are commissioned, one finite episode can be recorded
with:

```bash
vlai-l1 collect \
  --config configs/collection/default.toml \
  --experiment fruit_placement_v1 \
  --task "place the fruit in the bowl" \
  --frames 300 \
  --decision save
```

The live source owns both RealSense devices, receives the two x_air sidecars on
one Unix datagram endpoint, and writes the existing atomic canonical dataset
transaction. The Operator Panel exposes the same workflow only when every
tracked collection gate is ready.

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
[Datasets](docs/DATASETS.md), [x_air commissioning](docs/COMMISSIONING.md), and
[Source gaps](SOURCE_GAPS.md) before extending the Runtime.
