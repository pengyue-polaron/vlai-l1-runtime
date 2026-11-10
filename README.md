# VLAI L1 Runtime

This repository is the hardware-specific composition root for the VLAI L1
bimanual robot. It defines the robot and camera contracts, command-session
safety model, and the L1 side of collection and dataset operations. The
teleoperation integration is built against a pinned `x_air_sdk` checkout and
converts its per-side full-state callback into versioned, exact named-vector
contracts for either bimanual or isolated single-side collection.

The current task workflow is saved Follower start -> automatic positioning ->
continuous policy -> return and hold; commands are documented below.
The experimental task owner is separate from the unavailable generic production
command transport. Build/offline validation is not completed live commissioning.
Teleoperation collection remains a separate supported workflow.

## Automatic task operation

Workflow: saved Follower start -> move -> hold -> continuous policy -> return
and hold, using the original automatic owner. Normal stages retain the same
enabled process. Use direct pengyue SSH and Jetson tmux.

Task definitions are in `configs/tasks/<id>.json`; starts are in
`data/start_poses/<id>.json`. With teleoperation already active and the Follower
positioned, `just capture-start-pose-right push-distance` reads the measured
Follower joint8 in SDK degrees from existing IPC. Capture settings are in
`configs/tasks/pose_capture.json`. The current capture CLI creates new files;
update an existing pose through the authorized update workflow. Git manages
versioned files; this does not imply that ignored datasets/poses are Git-backed.
End teleoperation while parked/supported before starting the separate task owner.

| Command | When to use |
|---|---|
| `just task-check push-distance` | Validate a new or changed task configuration. |
| `just task-start push-distance SESSION` | Start a new owner when none already serves this task; SESSION is a new output directory. |
| `just task-status SESSION` | Inspect the existing owner when current status is needed. |
| `just task-policy SESSION` | Run policy on the existing owner at the saved start; an optional positive action count is supported. |
| `just task-finish SESSION` | Return to the saved start and hold. |
| `just task-shutdown SESSION` | Explicitly disable and exit. |

These are command references, not a checklist to rerun from the beginning.
Reuse an existing healthy owner and completed setup. Policy uses direct 30 Hz
absolute joint targets, existing Kp/Kd, zero desired velocity and zero feedforward
torque. Only start/return use smooth trajectories, with duration computed from
distance and configured reference limits. Normal finish retains torque;
physical/producer faults use the implemented verified-disable path.

Inference runs on the 4090 host (`asus4090` from pengyue, `work-pc` from Jetson)
under `/data/research/pengyue/vlai-distance-inference-20261108`.
Server source: `scripts/inference/serve_native.py`. Current native pi0 inputs are
AgentView/right-wrist RGB, ordered absolute joint8 degrees and separate
`distance_cm`; output is horizon32 at 30 Hz. Unified needs its actual force/FK
history interface. Existing Camera Service and inference workers operate outside
the motor loop. The selected SSH tunnel provides loopback
`ws://127.0.0.1:8808`; the client validates checkpoint identity.
Chunk-arrival freshness and future scheduled plan samples are distinct.

Implementation: `start_pose.py`, `task_definition.py`, `task_runtime.py`,
`pi0_inference.py`, `teleoperation/automatic.py` and
`native/xair_sidecar/automatic_main.cpp`. Hardware values and checks come from
`configs/system/vlai_l1.toml`; model/task contracts come from
`configs/inference/` and `configs/tasks/`.

## Boundaries

- This repository owns VLAI L1 devices, lifecycle, CAN safety,
  cameras, and robot-specific collection/evaluation composition.
- The pinned x_air SDK owns each leader/follower CAN pair. The Runtime sidecar
  consumes its public callback; it does not open a second CAN handle or use the
  upstream ROS2 gripper bridge.
- LeRobot Robot and Teleoperator plugins are a future boundary, not currently
  shipped by this project. When implemented, they must be thin Runtime clients
  and must not open CAN devices.
- `embodied-ops` supplies the shared CLI presentation, collection interaction,
  timing, artifact, contract-digest, task-registry, versioned Operator Panel,
  and shared LeRobot v3 validation/v2.1 format mechanics. L1 feature names,
  task/provenance policy, Reset mechanics, collection composition, and hardware
  readiness remain here.

## Dataset contract

The canonical dataset is written directly as LeRobot v3.0. The historical bimanual
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

The right-only setup keeps the Right Wrist D405 and uses D455 `251643060089`
as AgentView. The operator verified the D455 on its label and by unplugging its
USB cable. Use `just collect-right` after both cameras pass a live check. The
bimanual `just collect` contract requires a distinct Left Wrist camera and is
unavailable in this setup.
The AgentView RGB exposure is tracked in `configs/system/vlai_l1.toml` under
`[cameras.agent]`. The current manual `exposure_absolute = 200` is about 20 ms.
The camera service applies and verifies it when opening the D455. Changing it
requires stopping and restarting the camera service while collection is idle.

## Hardware-free operations

```bash
just sdk-verify
just sdk-describe right
just hardware
just panel-right
just dataset-doctor <experiment>
just dataset-doctor <experiment> --json
just export-v21 <experiment>
just export-v21 <experiment> --json
```

Validation and description do not import LeRobot, Torch, camera libraries, or
device APIs. Camera checks and dataset operations have separate optional
dependencies. Dataset commands print a compact operator summary by default;
`--json` writes one machine-readable object to stdout. The Panel exposes only
operations permitted by the tracked readiness gates.

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

The former bimanual collection command requires a distinct Left Wrist camera:

```bash
just collect fruit_placement_v1 "place the fruit in the bowl"
```

With the left arm intentionally off and both of its CAN links down, use the
isolated right-side contract:

```bash
just collect-right blue_block_red_plate_v1 "put the blue block into the red plate"
```

Once both cameras are connected, the right-only command starts or verifies the
persistent two-camera
owner, authorizes the privileged robot lifecycle, preflights the selected raw
frames before enabling motors, starts exactly the configured x_air runtime(s),
and waits for fresh selected state.
Runtime creation performs the SDK's startup alignment once for the collection
session, and the managed collection contract explicitly runs Reset once more
before presenting the first episode gate. In the pinned aarch64 SDK, the
`Control::AdjustPosition` target is an internal all-zero raw motor-angle vector
for the seven joints and gripper; it is not read from this repository's pose
reference or collection TOML. Use teleoperation to refine the
episode start pose, then press `Enter`
in the terminal or **Start recording** in the Panel. Enter `r` or use **Reset**
before recording to run `AdjustPosition` again. The workflow
rechecks the recorded camera roles after that confirmation before recording the
atomic canonical dataset transaction. Recording buffers the stationary prefix
and starts writing only after tracked action thresholds indicate sustained
motion, while preserving a short preroll. During recording, press `Enter` to save, enter
`d` to discard, or enter `q` to discard the current episode and quit. For
`collect-right`, saving with `Enter` leaves the right Leader and Follower under
active teleoperation at their current pose while encoding and publication run;
the next episode then waits for another `Enter`. Discard still runs
`AdjustPosition`. The bimanual `collect` contract runs `AdjustPosition` after
both save and discard. The selected runtime stays active between episodes.
Recording has no automatic
time limit and ends only when the operator makes a decision.

The reusable `embodied-ops` Panel is available with:

```bash
just panel-right
```

It serves the L1 adapter on port 8766 without opening hardware. Use **Start
cameras** in the Panel or `just cameras` to start the persistent read-only
camera owner. Right Wrist and AgentView previews remain on
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
`just cameras stop` releases the two camera handles.

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

For temporary two-wrist-camera plus all-motor position recording without
AgentView, see [Temporary recording](docs/TEMPORARY_RECORDING.md). That standalone
CSV/MJPEG recorder consumes already-running teleoperation and never enables or
resets arms. It does not replace the canonical LeRobot collection workflow.

```bash
uv sync --dev
uv run pytest
uv run ruff check .
uv run ruff format --check .
```

Default AI instructions are in [AGENTS.md](AGENTS.md) only.
Architecture, collection, datasets, sensors and source-gap documents are optional
feature references. Historical diagnostics and incident records are not routine
context or prerequisites for unrelated tasks.
