# VLAI L1 workspace handoff

This is the starting context for the next agent working on the onboard VLAI L1.
It records intent, verified evidence, unresolved gates, and the staged test
sequence. Tracked configuration and executable code remain authoritative. When
this snapshot disagrees with them, stop and resolve the drift instead of
silently choosing one.

## Start here

- Onboard host: `ssh sunrise@100.75.58.105`
- Onboard workspace: `/home/sunrise/vlai-l1-runtime`
- Development branch: `feat/runtime-foundation`
- Evidence baseline: `fc3e80c7a5a2c87d8e6ee7f943891b14061628c1`
- Handoff updated: 2026-07-23

Use SSH keys when possible. Never write the login password into a repository,
shell history, service unit, or agent context file. An agent already running on
the onboard host should work directly in the workspace above.

Before doing anything:

```bash
cd /home/sunrise/vlai-l1-runtime
sed -n '1,400p' /home/sunrise/AGENTS.md
sed -n '1,240p' AGENTS.md
git status --short
git branch --show-current
git rev-parse HEAD
git submodule status
```

Read [Safety](SAFETY.md) before camera, lifecycle, CAN, calibration, or motion
work. Read [Commissioning](COMMISSIONING.md) before starting either the legacy
controller or the candidate x_air sidecar.

## Goal and design

The goal is not to invent a universal robot-hardware protocol. The reusable
part is the embodied workflow: collect an episode, validate timing and
artifacts, choose save or discard, inspect a dataset, export a derivative, and
present those operations in an Operator Panel. Robot ownership and physical
safety remain hardware-specific.

The dependency direction is:

```text
embodied-ops workflow and Operator Panel contracts
  -> VLAI L1 adapter and LeRobot dataset composition
  -> named VLAI L1 observation/action transport
  -> guarded x_air sidecars and camera owners
  -> CAN and RealSense devices
```

Repository responsibilities:

| Component | Owns | Must not own |
| --- | --- | --- |
| `vlai-l1-runtime` | L1 topology, safety, devices, sidecars, camera identities, named state/action schema, dataset composition and readiness | Generic cross-robot workflow policy |
| pinned `external/embodied-ops` | Generic episode, transaction, timing, artifact and Operator Panel primitives | L1 CAN, joints, cameras or readiness decisions |
| pinned `external/x-air-sdk` | Reviewed x_air public ABI and opaque controller dependency | Runtime orchestration or dataset behavior |
| LeRobot plugins outside this workspace | Thin Robot/Teleoperator clients of Runtime contracts | Direct ownership of CAN or cameras |

The canonical dataset is written directly as LeRobot v3.0. A v2.1 dataset is
an independently generated derivative of that canonical source. There is no
raw intermediate format and no derivative may be the source of another final
derivative.

## Physical identity

The verified CAN mapping is:

| Side | Role | Interface | USB parent |
| --- | --- | --- | --- |
| right | leader | `can0` | `1-1.4.1:1.0` |
| left | leader | `can1` | `1-1.4.2:1.0` |
| right | follower | `can2` | `1-1.4.3:1.0` |
| left | follower | `can3` | `1-1.4.4:1.0` |

The visually verified cameras are:

| Role | Model | Serial | V4L2 stream |
| --- | --- | --- | --- |
| `wrist_left` | D405 | `255323074436` | `video-index4` |
| `wrist_right` | D405 | `255323074499` | `video-index4` |
| `agent` | D455 | `251643060089` | `video-index0` |

AgentView is an optional platform role, but it is enabled for this robot's
current dataset contract.

Do not copy these values into a new script. Their single runtime owner is
[`configs/system/vlai_l1.toml`](../configs/system/vlai_l1.toml).

## Current readiness snapshot

The legacy `/opt/xarm_teleop` systemd path has been used successfully for manual
teleoperation. It is still the approved live path, but it does not publish the
new Runtime state datagrams and therefore cannot drive the new collection
pipeline by itself.

The candidate `vlai_l1_xair_sidecar` has not been commissioned:

- a left-side run reproduced visible stutter because the opaque SDK raised
  three 500 Hz workers from requested FIFO 20 to hard-coded FIFO 50;
- the sidecar now caps and verifies every process thread at FIFO 20;
- after that correction, the next left-side run stopped when `can3` reached
  live TX/RX error counts 8/84 and its cumulative passive count rose from 1 to
  4;
- the CAN guard stopped the candidate and SDK destruction disabled the motors;
- right-side and complete bimanual candidate tests have not passed.

This priority cap is a deliberate containment boundary around an opaque
dependency, not an upstream root-cause fix. Remove it only when a reviewed x_air
release exposes or corrects the internal scheduling policy.

Tracked readiness is intentionally truthful:

- `teleoperation.commissioned = false`;
- collection blocker: `teleoperation_uncommissioned`;
- policy command transport: unimplemented;
- J2 coordinate, right-follower bus stability, and provisional joint limits:
  unverified.

Policy-command blockers do not prevent teleoperation collection. They do
prevent claiming policy evaluation or production command readiness.

Camera and data evidence:

- both D405 identities were verified from saved RGB images;
- a single left V4L2 stream produced 30 fresh 640x480 RGB `uint8` frames;
- one longer dual-camera attempt preceded loss of host connectivity; causality
  was not proven, so USB power/topology and dual-stream stability remain open;
- at the last 2026-07-23 inventory, neither D405 was enumerated;
- the connected D455 AgentView produced 60 fresh 640x480 RGB `uint8` frames at
  29.82 effective FPS over USB 3; the saved image was very dark but valid;
- LeRobot 0.6.0 created, finalized and passed deep inspection of a temporary
  12-frame, two-video canonical v3 dataset on the onboard host;
- the same source exported successfully to a two-video v2.1 derivative;
- the environment uses Python 3.12.13, OpenCV 4.13.0 and
  `torch 2.11.0+cpu`; CUDA is absent.

The last full onboard software check passed 67 tests. Repository and submodule
revisions were clean. About 6.7 GB remained on the root filesystem after cache
cleanup. Do not treat that as long-term video capacity.

## Test ladder

Do not skip a failed stage. A later stage cannot be used to excuse an earlier
failure.

### 0. Hardware-free baseline

These commands must not open cameras or CAN devices:

```bash
cd /home/sunrise/vlai-l1-runtime
git pull --ff-only
git submodule update --init --recursive
just check
just sdk-verify
just sdk-describe left
just sdk-describe right
```

Expected baseline:

- tests and Ruff pass;
- the x_air revision and library hashes match tracked configuration;
- left resolves to `can1 -> can3` and right to `can0 -> can2`;
- `teleoperation_ready` and `collection_ready` remain false until live
  evidence closes the teleoperation gate.

### 1. Passive host inventory

This stage does not authorize motion:

```bash
systemctl is-active xarm-can-rename.service xarm-cpu-performance.service
just sdk-status
just camera-list
df -h /home/sunrise
```

Before any candidate launch, both legacy teleoperation services must be
inactive, no controller process may remain, and all four CAN links must be
`DOWN` / `STOPPED`.

### 2. Cameras without arm motion

Connect both D405 units and the D455 through stable USB 3 paths, preferably a
powered hub or separate root hubs. Confirm that `just camera-list` prints all
three tracked serials, then run the bounded check:

```bash
just camera-check
```

It opens each configured V4L2 stream once and validates identity, shape, dtype,
freshness, continuity and pair skew for 30 samples with a 0.25 second per-set
timeout. Record effective FPS and maximum skew. Stop and inspect kernel USB/UVC
logs if the host disconnects, a camera re-enumerates, FPS collapses, or the
check times out. Do not weaken the tracked checks to make it pass.

### 3. Candidate x_air commissioning

This stage causes immediate enable/alignment motion. It requires a fresh,
explicit operator confirmation that the workspace is clear.

Follow [Commissioning](COMMISSIONING.md) one side at a time. Important rules:

1. Stop the legacy services and verify exclusive ownership.
2. Start the observer before the candidate.
3. Test left `can1 -> can3`, stop and inspect all counters.
4. Test right `can0 -> can2`, stop and inspect all counters.
5. Run both only after both isolated stages pass.
6. Stop on unexpected direction, alignment, force, sound, vibration, stale
   state, scheduling drift, or any CAN counter increase.

For an isolated observer, use the CLI directly because the current
`just sdk-observe` recipe is bimanual-only:

```bash
.venv/bin/python -m vlai_l1_runtime.cli observe-xair \
  --config configs/system/vlai_l1.toml \
  --side left \
  --samples 20 \
  --timeout 3
```

Use `--side right` for the other isolated stage. For the final paired test:

```bash
just sdk-observe 3000 15
```

The candidate launch must be rendered from
`build/xair-assets/manifest.json` and tracked System configuration. Do not add
hardware defaults or use the upstream `start_xarm_teleop_both.sh`/ROS wrapper.
The creation call enables motors, so a launch command is never a read-only
probe.

After every stage:

```bash
just sdk-stop
just sdk-status
```

Require both services/processes inactive, no candidate remaining, and all four
links returned to `DOWN` / `STOPPED` before retrying or changing sides.

### 4. Close only the teleoperation gate

Only after left, right and paired evidence pass may a change set
`teleoperation.commissioned = true`. That change must include the tracked
config, loader/consumer expectations, relevant tests, commissioning evidence,
and affected documentation in one reviewable commit.

Do not change `command_ready`, policy transport, J2, bus-stability, or joint
limit evidence merely to enable collection. Those are separate domains.

### 5. End-to-end collection

First discard a finite episode:

```bash
just collect commissioning "hold position" 30 discard
```

Then save a short new experiment and inspect it:

```bash
just collect smoke_v1 "hold position" 60 save
just dataset-doctor smoke_v1
just export-v21 smoke_v1
```

Never reuse an experiment name after a failed transaction until hidden
`.staging-*` or `.backup-*` siblings have been inspected. Never delete a
dataset, derivative, recording, checkpoint, or user file without explicit
authorization.

### 6. Operator Panel

After collection readiness is truly closed:

```bash
just panel
```

The Panel binds to the trusted robot LAN at the tracked address and port. It
has no authentication or transport encryption. Do not expose it to an
untrusted network. Before commissioning, it correctly omits the live collection
action rather than offering a broken button.

## Definition of done for the next milestone

The new teleoperation collection path is complete only when all of the
following are evidenced:

- all three enabled camera streams pass the bounded concurrent camera check;
- left candidate runs a finite observation window with FIFO 20 and no CAN
  counter increase;
- right candidate does the same, specifically closing the prior `can2`
  concern;
- paired sidecars complete the bimanual observer window with increasing
  sequences and bounded side skew;
- stop/disable returns every owned resource to its inactive state;
- the teleoperation gate is updated through a reviewed tracked change;
- one discard and one saved real episode complete;
- the saved canonical v3 dataset passes doctor;
- its independently generated v2.1 derivative validates;
- repository, submodules and deployment checkout are clean and pushed.

Passing legacy teleoperation alone does not satisfy this definition. Passing a
synthetic dataset test alone does not satisfy it either.

## Storage and cleanup context

Failed CUDA packages are not installed. About 1 GB of obsolete uv cache and a
further 78.3 MiB of stale CUDA package cache were removed. Remaining large
items include the active 1.3 GB Python environment and required x_air build
artifacts.

Potentially recoverable space exists in old VS Code server versions, an old
Codex release, and a downloaded VS Code package. Those are user-level tools,
not Runtime garbage. Resolve exact inactive versions and obtain authorization
before deleting them. Do not delete `x_air_sdk`, `VLAI-L1_Backup`, ROS
workspaces, `data/`, `outputs/`, or model/checkpoint directories as cleanup.

## Maintaining this handoff

After each material live test, update only the evidence and next-action
sections; do not duplicate configuration values elsewhere. Record:

- timestamp and tested side;
- exact repository and submodule revisions;
- operator workspace-clear confirmation;
- process/thread scheduling result;
- CAN baseline and final counters;
- state sample count, sequence range and skew;
- camera sample count, FPS and skew when applicable;
- stop/disable result;
- pass/fail decision and the single next safe stage.

If the host-level `/home/sunrise/AGENTS.md`, tracked configuration, safety
documentation, and this handoff disagree, stop. Reconcile the inconsistency in
the repository before another live test.
