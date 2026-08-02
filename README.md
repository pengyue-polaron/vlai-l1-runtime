# VLAI L1 Runtime

This repository is the hardware-specific composition root for the VLAI L1
bimanual robot. It defines the robot and camera contracts, command-session
safety model, and the L1 side of collection and dataset operations. The
teleoperation integration is built against a pinned `x_air_sdk` checkout and
converts its per-side full-state callback into versioned, exact named-vector
contracts for either bimanual or isolated single-side collection.

The camera mappings and x_air teleoperation collection path are commissioned.
A complete motor-power restart recovered the right-leader all-node no-ACK
fault, and a protected isolated-right commissioning window passed.
Policy-command transport and policy evaluation remain separate, explicitly
unavailable capabilities.

## Boundaries

- This repository owns VLAI L1 devices, lifecycle, CAN safety,
  cameras, and robot-specific collection/evaluation composition.
- The pinned x_air SDK owns each leader/follower CAN pair. The Runtime sidecar
  consumes its public callback; it does not open a second CAN handle or use the
  upstream ROS2 gripper bridge.
- LeRobot Robot and Teleoperator plugins are separate thin clients. They use
  this Runtime and must not open CAN devices.
- `embodied-ops` supplies the shared CLI presentation, collection interaction,
  timing, artifact, contract-digest, task-registry, and versioned Operator Panel
  contracts. L1 feature names, datasets, Reset mechanics, provenance, and
  hardware readiness remain here.

## Dataset contract

The canonical dataset is written directly as LeRobot v3.0. The bimanual
[`default.toml`](configs/collection/default.toml) contract contains:

- `observation.state`: 16 named left/right joint and gripper positions in degrees;
- `action`: the same 16-name, degree-valued convention;
- `observation.images.wrist_left`, `observation.images.wrist_right`, and
  `observation.images.agent`;
- one normalized task string.

The wrist observations remain 640×480. AgentView uses the tracked centered
480×480 crop from the 640×480 camera source, so canonical v3 and exported v2.1
datasets present the same square model input.

Samples are accepted only when joint vectors are exact and finite, timestamps
are fresh, state/action, left/right arm, camera pairs, and robot/camera samples
are synchronized, and sequences increase. The Runtime does not clamp or
normalize stored joint/gripper values; the live sidecar separately stops on an
arm-envelope fault and reports sustained following error as a nonfatal warning.
An episode is
finalized in a hidden sibling snapshot and becomes visible only through an
atomic rename. Failed and discarded episodes never replace the last complete
dataset.

The isolated [`right_only.toml`](configs/collection/right_only.toml) contract
instead contains the exact eight right-side positions plus only Right Wrist and
AgentView video. It never fabricates left-arm values and never writes a Left
Wrist video. Dataset layout, provenance, doctor behavior, and v2.1 derivatives
are documented in [Datasets](docs/DATASETS.md).

## Hardware-free operations

```bash
just sdk-verify
just sdk-describe right
just panel
just dataset-doctor <experiment>
just export-v21 <experiment>
```

Validation and description do not import LeRobot, Torch, camera libraries, or
device APIs. Camera checks and dataset operations have separate optional
dependencies. The Panel exposes only operations permitted by the tracked
readiness gates.

```bash
git submodule update --init --recursive
just setup-camera
just cameras
just camera-check
just setup-dataset
```

The x_air dependency and native state sidecar can be built without hardware:

```bash
just sdk-build
```

After tracked teleoperation is recommissioned, start a collection session with:

```bash
just collect fruit_placement_v1 "place the fruit in the bowl"
```

With the left arm intentionally off and both of its CAN links down, use the
isolated right-side contract:

```bash
just collect-right blue_block_red_plate_v1 "put the blue block into the red plate"
```

The command starts or verifies the persistent three-camera
owner, authorizes the privileged robot lifecycle, preflights the selected raw
frames before enabling motors, starts exactly the configured x_air runtime(s),
and waits for fresh selected state.
Runtime creation performs the SDK's startup alignment once for the collection
session. Use teleoperation to refine the episode start pose, then press `Enter`
in the terminal or **Start recording** in the Panel. Enter `r` or use **Reset**
before recording to run `AdjustPosition` again. The workflow
rechecks the recorded camera roles after that confirmation before recording the atomic
canonical dataset transaction. During recording, press `Enter` to save, enter
`d` to discard, or enter `q` to discard the current episode and quit. After a
save or discard, recording stops and the same active runtime immediately runs
`AdjustPosition`; encoding and publication then run with the selected arm(s)
still enabled at Reset. The workflow advances to the next episode without
disabling the selected pair(s). Recording has no automatic
time limit and ends only when the operator makes a decision.

The reusable `embodied-ops` Panel is available with:

```bash
just panel
```

It serves the L1 adapter on port 8765 without opening hardware. Use **Start
cameras** in the Panel or `just cameras` to start the persistent read-only
camera owner. Left-wrist, right-wrist, and AgentView previews then remain on
port 8088 across collection runs. The owner opens each device exactly once,
publishes exact raw RGB sets over a local Unix socket for collection, and
encodes lower-rate MJPEG from the same latest frames. Preview never advances or
blocks formal 30 FPS delivery. The Panel reports camera health, shows capture
progress without terminal spam, and provides a create-only prompt registry
whose task text can be activated directly in the Collect form.

Camera lifecycle is explicit:

```bash
just cameras
just cameras status
just cameras logs
just cameras stop
```

Stopping a collection leaves the read-only camera service available. Only
`just cameras stop` releases the three camera handles.

`just reset` performs the bimanual startup alignment without opening cameras or
a dataset transaction. `just reset-right` does the same for only the isolated
right pair. Each standalone command then disables its selected pair(s) and
returns those CAN links to `DOWN`. The Panel's standalone **Reset** action uses
the currently selected collection contract. This is the
x_air SDK alignment routine, not a policy-command or arbitrary-pose interface.
Use the collection workflow when the robot must remain enabled across episodes.

Camera images are written asynchronously so the live loop can maintain its
tracked 30 FPS rate; encoder banners are suppressed while the current save
stage remains visible. A session below the tracked minimum is discarded instead
of publishing time-compressed data. Normal completion, collection failure,
startup failure after either side begins, and `Ctrl+C` all use the same
fail-closed cleanup: selected motors are disabled and their CAN links return to
`DOWN`. The isolated right mode additionally requires `can1/can3` to remain
down. Do not run `just sdk-start` or a separate observer before collection;
those commands are for bounded commissioning and diagnostics.

On the robot, `just camera-list` prints the serials of connected V4L cameras
without opening their video streams. `just camera-check` starts or reuses the
persistent owner and validates a finite raw-frame window through the same local
bridge used by collection.
`just sdk-status` inspects the deployed processes and CAN controllers, and
`just stop` disables both pairs and returns all four links to `DOWN`.

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

Start with the current [workspace handoff](docs/HANDOFF.md), then read
[Architecture](docs/ARCHITECTURE.md), [Safety](docs/SAFETY.md),
[Datasets](docs/DATASETS.md), [x_air commissioning](docs/COMMISSIONING.md), and
[Source gaps](SOURCE_GAPS.md) before extending the Runtime.
