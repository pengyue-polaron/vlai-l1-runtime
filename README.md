# VLAI L1 Runtime

Runtime for the VLAI L1 robot: automatic policy execution, teleoperation collection,
cameras and datasets. Jetson source root: /home/nvidia/vlai-l1-runtime.
Default AI instructions are in [AGENTS.md](AGENTS.md); other documents are feature references.

## Automatic policy execution

Saved Follower start -> move -> hold -> continuous policy -> return and hold.
The original automatic owner stays enabled across normal stages. Use direct
pengyue SSH and Jetson tmux. Reuse a healthy running owner and completed setup.

Task definitions: `configs/tasks/<id>.json`. Starts: `data/start_poses/<id>.json`.
With teleoperation already active and the Follower positioned:

```bash
just capture-start-pose-right push-distance
```

Capture reads measured Follower J1-J7 plus gripper in SDK degrees from existing
IPC. Settings are in `configs/tasks/pose_capture.json`. End teleoperation while
parked/supported before starting the separate automatic owner.

| Command | Purpose |
|---|---|
| `just task-check push-distance` | Validate a new or changed task configuration. |
| `just task-start push-distance SESSION` | Start a new owner; SESSION is a new output directory. |
| `just task-status SESSION` | Inspect an existing owner when needed. |
| `just task-policy SESSION` | Run continuous policy; an optional positive action count is supported. |
| `just task-finish SESSION` | Return to the saved start and hold. |
| `just task-shutdown SESSION` | Explicitly disable and exit. |

These are command references, not a sequence to repeat on every request.
Policy uses direct 30 Hz absolute joint targets and existing Kp/Kd, with zero
desired velocity/feedforward torque. Only start/return use smooth trajectories;
their duration comes from distance and configured reference limits.
Normal finish retains torque. Fault handling follows the running controller.

Inference deployment: `asus4090` from pengyue, `work-pc` from Jetson,
`/data/research/pengyue/vlai-distance-inference-20261108`.
Server source: `scripts/inference/serve_native.py`. Native pi0 uses AgentView
and right-wrist RGB, ordered joint8 degrees and separate `distance_cm`;
output is horizon32 at 30 Hz. Unified needs its actual force/FK-history interface.
The selected SSH tunnel provides loopback `ws://127.0.0.1:8808`; the client
checks checkpoint identity. Camera/model work stays outside the motor loop.

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
