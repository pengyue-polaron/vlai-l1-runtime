# VLAI L1 workspace handoff

This is the starting context for the next agent working on the onboard VLAI L1.
It records intent, verified evidence, unresolved gates, and the staged test
sequence. Tracked configuration and executable code remain authoritative. When
this snapshot disagrees with them, stop and resolve the drift instead of
silently choosing one.

## Start here

- Onboard host: `ssh nyush-robotics-dev`
- Onboard workspace: `/home/nyu/vlai-l1-runtime`
- Default branch: `main`
- Handoff updated: 2026-07-24

Use SSH keys when possible. Never write the login password into a repository,
shell history, service unit, or agent context file. An agent already running on
the onboard host should work directly in the workspace above.

Before doing anything:

```bash
cd /home/nyu/vlai-l1-runtime
sed -n '1,260p' /home/nyu/AGENTS.md
sed -n '1,240p' AGENTS.md
git status --short
git branch --show-current
git rev-parse HEAD
git submodule status
```

Read [Safety](SAFETY.md) before camera, lifecycle, CAN, calibration, or motion
work. Read [Commissioning](COMMISSIONING.md) before starting either the legacy
controller or the commissioned x_air sidecar.

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
| right | leader | `can0` | `1-2.2.1:1.0` |
| left | leader | `can1` | `1-2.2.2:1.0` |
| right | follower | `can2` | `1-2.2.3:1.0` |
| left | follower | `can3` | `1-2.2.4:1.0` |

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

The migrated x86_64 host uses the config-rendered `run-xair` lifecycle through
`just sdk-start <left|right>`. It validates the pinned dependency and manifest,
owns only the selected CAN pair, applies the tracked CAN-FD settings and TX
queue length, launches the sidecar at FIFO 20, monitors qdisc drops, and
disables the pair before returning both links to `DOWN`.

Commissioning on 2026-07-23 found that `txqueuelen=10` dropped about 184k
qdisc packets per active bus while the ordinary CAN counters remained at zero.
The tracked queue length of 1000 eliminated new qdisc drops. Formal left,
right, and simultaneous bimanual runs were subjectively smooth, kept all buses
ERROR-ACTIVE with zero live CAN errors, and each completed its observer window.
The bimanual observer completed 3000 paired 100 Hz samples.

The sidecar still caps and verifies every SDK thread at FIFO 20. This priority
cap is a containment boundary around the opaque prebuilt dependency; remove it
only when a reviewed x_air release exposes or corrects the internal scheduling
policy.

Tracked readiness is intentionally truthful:

- `teleoperation.commissioned = true`;
- teleoperation collection has no tracked readiness blocker;
- policy command transport: unimplemented;
- J2 coordinate and right-follower bus stability: unverified.

Policy-command blockers do not prevent teleoperation collection. They do
prevent claiming policy evaluation or production command readiness.

Camera and data evidence:

- both D405 identities were verified from saved RGB images and now enumerate
  over USB 3 on the migrated host;
- the D455 AgentView runs its RGB-only 640x480 YUYV stream over USB 2; a
  900-frame isolated window sustained 29.99 FPS with no USB error;
- both D405 streams and the D455 stream completed a concurrent 300-frame
  640x480 YUYV window at 29.99 FPS with no stream failure or USB error;
- the formal Runtime check captured 30 fresh 640x480 RGB `uint8` frames from
  all three streams at 30.02 effective FPS with 21.36 ms maximum skew;
- with both 500 Hz teleoperation sides and the 100 Hz bimanual observer active,
  the parallel camera bridge captured 300 three-camera sets at 29.997 FPS with
  32.88 ms maximum skew and no new CAN or qdisc drops;
- the 10 FPS MJPEG preview and formal three-camera capture ran together for 300
  frames at 29.979 FPS with 32.38 ms maximum skew; all three preview streams
  remained fresh at 9.99 FPS and no second camera owner was created;
- a later whole-hub reset disconnected and re-enumerated both D405 cameras,
  AgentView, and all four PCAN adapters together; the resulting
  `VIDIOC_REQBUFS ... ENODEV` was a stale device node after the shared USB
  transport failed, not a role-mapping error;
- a later D455-only failure disconnected and re-enumerated AgentView three
  times while the D405 and PCAN devices remained present; USB autosuspend was
  disabled, so its cable, port, and power path remain an operational concern;
- udev binds the D455's serial-less UVC color interface to the commissioned
  AgentView identity and disables autosuspend for all RealSense and PEAK
  devices;
- LeRobot 0.6.0 created, finalized and passed deep inspection of a temporary
  12-frame, two-video canonical v3 dataset on the onboard host;
- the first 300-frame managed three-camera save exposed synchronous PNG writes
  that made wall-clock capture slower than the declared 30 FPS; that episode is
  useful for image review but not timing evidence;
- the collection path now uses the LeRobot-recommended asynchronous image
  writer with four threads per camera, rejects capture below 27 FPS, and stops
  hardware before encoding; a hardware-free 90-frame, three-camera benchmark
  enqueued at over 10,000 frames/s and completed flush plus encoding in 2.98 s;
- the same source exported successfully to a two-video v2.1 derivative;
- the environment uses Python 3.12.13, OpenCV 4.13.0 and
  `torch 2.11.0+cpu`; CUDA is absent.

The last full onboard hardware-free software check passed 92 tests. About 75 GB remained on
the migrated root filesystem before live camera commissioning.

## Test ladder

Do not skip a failed stage. A later stage cannot be used to excuse an earlier
failure.

### 0. Hardware-free baseline

These commands must not open cameras or CAN devices:

```bash
cd /home/nyu/vlai-l1-runtime
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
- `teleoperation_ready` and `collection_ready` are true;
- policy-command readiness remains false and independent.

### 1. Passive host inventory

This stage does not authorize motion:

```bash
systemctl is-active xarm-can-rename.service xarm-cpu-performance.service
just sdk-status
just camera-list
df -h /home/sunrise
```

Before any sidecar launch, both legacy teleoperation services must be
inactive, no controller process may remain, and all four CAN links must be
`DOWN` / `STOPPED`.

### 2. Cameras without arm motion

Connect both D405 units through stable USB 3 paths. The commissioned RGB-only
D455 may use its verified USB 2 path; prefer a powered hub or separate root
hubs when available. Confirm that `just camera-list` prints all three tracked
serials, then run the bounded check:

```bash
just camera-check
```

It starts or reuses the marked persistent Camera Service, then validates
identity, shape, dtype, freshness, continuity and pair skew for 30 raw frame
sets over the same local bridge used by collection. Record effective FPS and
maximum skew. Stop and inspect kernel USB/UVC logs if the host disconnects, a
camera re-enumerates, FPS collapses, or the check times out. Do not weaken the
tracked checks to make it pass. Use `just cameras stop` when the physical
handles should be released.

### 3. x_air recommissioning

This stage causes immediate enable/alignment motion. It requires a fresh,
explicit operator confirmation that the workspace is clear.

Follow [Commissioning](COMMISSIONING.md) one side at a time. Important rules:

1. Stop the legacy services and verify exclusive ownership.
2. Start the observer before the sidecar.
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

The sidecar launch must be rendered from
`build/xair-assets/manifest.json` and tracked System configuration. Do not add
hardware defaults or use the upstream `start_xarm_teleop_both.sh`/ROS wrapper.
The creation call enables motors, so a launch command is never a read-only
probe.

After every stage:

```bash
just sdk-stop
just sdk-status
```

Require both services/processes inactive, no sidecar remaining, and all four
links returned to `DOWN` / `STOPPED` before retrying or changing sides.

### 4. Teleoperation gate

Left, right, and paired evidence passed and
`teleoperation.commissioned = true`. Any future change to that gate must include
the tracked config, loader/consumer expectations, relevant tests,
commissioning evidence, and affected documentation in one reviewable commit.

Do not change `command_ready`, policy transport, J2, or bus-stability evidence
merely to alter collection. Those are separate domains.

### 5. End-to-end collection

Start with no manual x_air runtime, observer, camera reader, or competing
controller active. `just collect` starts or reuses the marked Camera Service,
connects one raw client, preflights all three cameras before motor enable,
starts both configured runtimes, and waits for fresh paired state. Use
teleoperation to place the robot at the episode start pose, then press Enter in
the terminal or Start recording in the Panel. The workflow rechecks all cameras
after confirmation and records. A normal return, error, or `Ctrl+C` closes the
collection client, disables both pairs and returns can0-can3 to `DOWN`; the
read-only preview remains available. The input gate is not an automatic reset;
no reviewed fixed-pose command path exists.

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

Collection readiness is commissioned:

```bash
just panel
```

The Panel binds to the trusted robot LAN at the tracked address and port. It
has no authentication or transport encryption. Do not expose it to an
untrusted network. Start the persistent preview with **Start cameras** or
`just cameras`; stop it explicitly with **Stop cameras** or
`just cameras stop`. Its collection action invokes the same managed session
and is shown only when the tracked readiness gates permit it. All three
previews and normalized health remain visible between collection runs. During
capture it also shows progress and the guarded Start recording input. Prompt
registration creates a new validated JSON record without modifying an existing
prompt and can fill the Collect task field.

## Current completion state

The teleoperation collection path has the following evidence:

- all three enabled camera streams pass the bounded concurrent camera check;
- left sidecar runs a finite observation window with FIFO 20 and no CAN
  counter increase;
- right sidecar does the same, specifically closing the prior `can2`
  concern;
- paired sidecars complete the bimanual observer window with increasing
  sequences and bounded side skew;
- stop/disable returns every owned resource to its inactive state;
- the teleoperation gate is tracked as commissioned;
- discard and saved real episodes complete, including a 500-frame,
  three-camera managed save;
- the saved canonical v3 dataset passes doctor;
- its independently generated v2.1 derivative validates;
- operator confirmation is shared by terminal and Panel before recording;
- collection preview, camera health, capture progress, and create-only prompt
  registration are composed through `embodied-ops`.

The remaining operational issue is intermittent RealSense USB transport
stability, especially the D455 cable/port/power path. A mid-episode disconnect
must continue to invalidate the transaction rather than trigger transparent
recovery.

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
