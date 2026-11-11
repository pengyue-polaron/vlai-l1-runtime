# VLAI L1 Runtime

Runtime for the VLAI L1 robot: automatic policy execution, teleoperation collection,
cameras and datasets. Jetson source root: /home/nvidia/vlai-l1-runtime.
Default AI instructions are in [AGENTS.md](AGENTS.md); other documents are feature references.

## Inference workflow

Saved Follower start -> move -> hold -> continuous policy -> return and hold.
One automatic motor owner stays enabled throughout these normal stages. Run
persistent control in Jetson tmux through direct pengyue SSH. Reuse healthy
services and an existing healthy owner instead of restarting them.

### Task and start pose

Task definitions live in `configs/tasks/<id>.json`; saved starts live in
`data/start_poses/<id>.json`. The configured `push-distance` task requests 10 cm
and pins native `pi0-29999`, its deployment identity and inference/control specs
in `configs/tasks/push-distance.json`.

A saved start is reusable: teleoperation is only needed when capturing or changing
the pose. With teleoperation already active and the Follower positioned, run:

```bash
just capture-start-pose-right push-distance
```

Capture reads measured Follower J1-J7 plus gripper in SDK degrees from existing
IPC; settings are in `configs/tasks/pose_capture.json`. Before replacing the
teleoperation owner, park/support the arms because ending teleoperation can
remove holding effort. The automatic owner then moves from measured position to
the saved start. Subsequent inference runs do not require teleoperation.

### Services and execution

Inference deployment is on `asus4090` from pengyue (`work-pc` from Jetson), under
`/data/research/pengyue/vlai-distance-inference-20261108`. The ASUS user systemd
service is `vlai-pi0-live.service`; server source is
`scripts/inference/serve_native.py`. Jetson tmux session `vlai-policy-tunnel`
forwards local port 8808 to `work-pc:8808`, exposing `ws://127.0.0.1:8808`.
Use `just cameras status` to inspect the persistent camera service and
`just cameras` to start or reuse it. Check actual service state before restarting.

Before motion, check current motor ownership, the configured Follower CAN identity
and fresh feedback. Hardware changes can change USB/CAN numbering. Inspect only
devices needed by the selected task; old logs do not establish current hardware
health. Use a new output directory for each new owner and retain previous records.

| Command | Purpose |
|---|---|
| `just task-check push-distance` | Validate a new or changed task configuration. |
| `just task-start push-distance SESSION` | Start an owner, move to the saved start and hold; SESSION is a new output directory. |
| `just task-status SESSION` | Inspect owner state and arrival before requesting policy. |
| `just task-policy SESSION` | Run continuous policy; an optional positive action count is supported. |
| `just task-finish SESSION` | Return to the saved start and hold without disabling. |
| `just task-shutdown SESSION` | Explicitly disable and exit. |

These are lifecycle operations, not checks to repeat indiscriminately. A healthy
owner already holding at the start can proceed directly to policy. Normal finish
retains torque; explicit shutdown and fault handling are separate paths.

### Model interface and motor control

Native pi0 consumes AgentView and right-wrist RGB, ordered joint8 degrees and a
separate `distance_cm`; output is horizon32 at 30 Hz. The client validates
checkpoint identity. Unified requires its actual force/FK-history interface and
is not interchangeable with this pi0 configuration. Camera/model work stays
outside the motor loop.

The 100 Hz motor loop continuously conditions the 30 Hz absolute joint targets
with a 0.15-second first-order filter and a 20 deg/s reference slew limit
(`TaskCommandFilter`). It starts from the last commanded holding position without
resetting at chunk boundaries or dropping holding effort. This introduces lag and
changes the sent reference; it does not guarantee physical speed. Measured-speed
and other fault guards remain active. Desired velocity and feedforward torque
remain zero. Start/return use smooth planned trajectories, with duration calculated
from distance and configured reference limits.

Optional `position_gains` in the task control JSON sets right-Follower-only Kp/Kd;
omitted values use system gains. J7 uses 50/0.4. Teleoperation gains are unchanged.
Telemetry separates `desired_deg` (model/planner) from `reference_deg` (sent
command). `applied_sequence` identifies the sample selected for filtering, not
physical arrival at its original target.

### Results and fault diagnosis

Session `task-status.json`, `report.json` and controller logs describe execution
and cleanup. A current-session pointer identifies a directory, not a live owner.
Read the actual owner state and fresh feedback. Preserve the native fault when
diagnosing a stop rather than treating secondary cleanup errors as its cause.
If communication is lost, a disable request may not be verifiable;
`disabled_verified: false` must not be interpreted as disabled or holding.

Arrival and sustained execution have been demonstrated with the current gains
and filtering: `data/task-runs/push-distance-20261110T110947/task-status.json`
records 470 chunks and 15,008 sent targets (about 8 minutes 20 seconds at 30 Hz).
That run ended with a shared USB/CAN disconnect and unverified disable, as recorded
in its `report.json`. These records establish execution, not verified 10 cm block
displacement or task success. Hardware health must be assessed from current state,
not this historical fault. Native task/filter and Python task, position-gain and
supervision tests cover the implementation; rerun affected checks when it changes.

## Teleoperation and collection

| Command | Purpose |
|---|---|
| `just sdk-start-right-only` | Start isolated right-side teleoperation. |
| `just sdk-status` | Inspect runtime processes and CAN state. |
| `just collect-right TASK "instruction"` | Record the configured right-side dataset. |
| `just collect TASK "instruction"` | Record with the configured bimanual setup. |
| `just reset-right` / `just reset` | Run SDK alignment, then disable the selected pair(s). |
| `just stop` | Disable both pairs and lower their CAN links. |
| `just panel-right` | Open the Operator Panel on port 8766. |

Managed collection starts its own configured motor runtime; do not pre-start a
competing teleoperation owner. SDK startup and Reset cause alignment motion.
Use teleoperation to refine the recording start, then press Enter / Start recording.
During recording, Enter saves, `d` discards, `q` discards and quits.
Right-only save keeps teleoperation active at the current pose; discard resets.
Bimanual save/discard resets. Quitting collection disables the selected motors.
An automatic task's normal finish instead returns and holds.

Collection inputs and data formats: [Datasets](docs/DATASETS.md).
Additional tools: [HKVL serial capture](docs/HKVL_SERIAL.md) and
[temporary two-camera recording](docs/TEMPORARY_RECORDING.md).

## Cameras

`just cameras` starts or reuses the persistent owner.
Use `just cameras status`, `just cameras logs`, `just cameras stop`,
`just camera-list` or `just camera-check` as needed.
MJPEG preview is on port 8088. Collection consumes the service's raw frames;
ending collection leaves the camera service available. Panel/preview are
trusted-LAN services without authentication.

Camera identities, roles, image settings, motor identities, gains, bounds and
timeouts live in `configs/system/vlai_l1.toml`. Runtime clients use one device
owner rather than opening competing handles.

## Development

Initialize pinned dependencies with `git submodule update --init --recursive`
when absent. `just sdk-build` builds the SDK/sidecar without starting hardware.
`just sdk-verify`, `just sdk-describe right` and `just hardware` inspect setup.
Runtime contracts support Python 3.10; LeRobot collection/export use Python 3.12.

```bash
uv sync --dev
uv run pytest
uv run ruff check .
uv run ruff format --check .
```

Run only checks relevant to the change and reuse results for unchanged inputs.
Implementation lives in `src/vlai_l1_runtime/` and `native/xair_sidecar/`.
Task/model configuration lives in `configs/tasks/` and `configs/inference/`.
