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
- Handoff updated: 2026-08-03

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
| pinned `external/embodied-ops` | Generic episode, task registry, transaction, timing, artifact, camera-health presentation and Operator Panel primitives | L1 CAN, joints, cameras or readiness decisions |
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
| right | leader | `can0` | `1-3.1:1.0` |
| left | leader | `can1` | `1-3.2:1.0` |
| right | follower | `can2` | `1-3.3:1.0` |
| left | follower | `can3` | `1-3.4:1.0` |

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

The startup lifecycle validates each interface against its tracked USB parent.
Before calling the motion-capable teleoperation constructor, it queries leader
and follower endpoints without enabling motors and requires all eight tracked
response IDs on every configured probe round. A startup fault now fails closed
without rebinding or retrying the adapter. Managed bimanual startup holds each
side after its complete motion-free preflight and releases both constructors
only after all four endpoints pass. Runtime CAN faults are never hot-recovered.

Motion-free diagnostics on 2026-08-02 passed 40/40 rounds with all motor IDs on
`can0`, `can2`, and `can3`. `can1` received no ACK, entered ERROR-PASSIVE with
`txerr=128`, and reproduced immediately after an isolated `peak_usb` rebind.
Single state-query frames failed in classic CAN, CAN-FD without BRS, and
CAN-FD with BRS, while `can0` answered in all three modes. Do not start the
left or managed bimanual x_air paths until `can1` again passes the complete
motion-free probe. A bounded right-only session may use
`just sdk-start-right-only`; it requires `can1/can3` to remain `DOWN` and owns
an exclusive lifecycle lock while controlling only `can0/can2`.

The first bounded right-only pipeline run on 2026-08-02 completed both 40-round
motor probes, SDK startup alignment, 2,000 monotonically sequenced and
timestamped right-side state packets at the configured 100 Hz, and guarded
shutdown. Twenty control-window health checks kept `can0/can2` ERROR-ACTIVE
with zero CAN errors and zero qdisc drops. Each right interface finished with
matching TX/RX packet counts; `can1/can3` stayed `DOWN` and their packet counts
did not change. A single `can2 failed resubmitting read bulk urb: -1` appeared
only during interface shutdown, with no USB disconnect or reset. Linux USB
core documents `-EPERM` as the expected result when a completion callback tries
to resubmit an URB while `usb_kill_urb()` is stopping it.

A later right-only end-to-end diagnostic explicitly ran right-side
`AdjustPosition`, then captured 450 synchronized sets over 15.16 seconds at
29.69 FPS. Right leader action and follower observation contained exact,
finite, increasing 8-DOF degree vectors. Right Wrist, Left Wrist, and AgentView
each produced a decodable 450-frame H.264 diagnostic video; robot/camera skew
was at most 4.99 ms and camera-pair skew at most 21.14 ms. `can0/can2` had
matching TX/RX counts with zero errors or drops, and all links returned down.
That artifact was deliberately diagnostic rather than canonical because it
preceded the tracked right-only dataset contract.

A subsequent right-side session on 2026-08-02 was mechanically constrained
while the leader was moved beyond a pose the follower could attain. The
available process evidence shows the sidecar ran for about 119 seconds and was
then stopped; the next startup failed on `can0`. No sidecar fault line was
persisted, so this timing is correlation rather than proof of the initiating
electrical failure. A targeted reset of only the configured `can0` PCAN cleared
the adapter counters. After one ordinary no-frame close/open cycle the
controller was ERROR-ACTIVE with zero counters, but the first motion-free
feedback round received none of motor IDs `0x011` through `0x018` and returned
to `txerr=128`. `can0` is down and must not be retried until the right leader
motor-bus supply/protection and harness have been inspected with the arm
unloaded. This later result supersedes any assumption that USB reset alone
repairs every `can0` passive fault.

After the operator reported a restart, one additional bounded `can0` test was
performed on 2026-08-02. USB identity still matched configured parent
`1-3.1:1.0`, all four links and both services were inactive, and a standard
interface configure/open produced an ERROR-ACTIVE baseline with `txerr=0` and
`rxerr=0`. The first and only motion-free feedback round again missed every
configured response ID `0x011` through `0x018`. Exactly eight query frames were
transmitted with zero responses; `can0` returned to `txerr=128` and its
cumulative warning/passive counts each increased from 2 to 3. The probe closed
`can0`; `can2` was deliberately not queried, no SDK handle was created, and no
alignment or motor-enable path ran. All links remain down with no controller
process. This repeat after restart confirms that host reboot/reconfiguration is
not a recovery for the current all-node Leader-bus fault.

The operator then confirmed a complete robot/motor-power restart. All four
PCAN devices re-enumerated with zero counters and retained their configured USB
parents. One `can0` probe and one `can2` probe each completed 40/40 motion-free
rounds with all IDs `0x011` through `0x018`, ERROR-ACTIVE state, and zero live
errors. An isolated right Runtime then completed SDK alignment and a protected
15-second observer window: 1,500 packets, source sequence `0..7495`, 14.990
seconds of increasing timestamps, and no joint-safety or CAN fault. Maximum
J1-J7 following errors were `[1.902, 0.284, 0.153, 1.399, 0.437, 0.590,
4.328]` degrees. Guarded shutdown left `can0` with 51,464 matching TX/RX
packets and `can2` with 51,488, zero CAN errors and zero qdisc drops; `can1` and
`can3` remained unused. All four links and both services finished inactive.
This establishes that complete motor-power restart, unlike host-only restart,
recovered the current all-node no-ACK condition.

The operator subsequently requested that the leader/follower limit detection
be turned off, then explicitly clarified that only the sustained
leader/follower following-error stop should be disabled. The hard mechanical
joint envelope remains fatal. Tracked `following_error_action = "warn"` now
emits one nonfatal warning per sustained excursion, and all CAN, freshness,
finite-value, timestamp, and hard-bound faults still stop the Runtime.

Source review also established that unilateral control is direct joint-space
reference copying, not Cartesian IK, and that the pinned JointMapper is
identity. The prior sidecar had no joint-bound or leader/follower tracking-error
guard. System schema 4 now owns conservative side-specific seven-joint bounds,
a per-joint following-error vector `[9, 8, 3, 10, 4, 2, 6]` degrees, and a
100 ms persistence timeout. The vector is above every corresponding maximum in
the saved 450-frame normal right-side diagnostic (whose maxima were
`[7.322, 6.426, 1.180, 8.655, 2.361, 0.306, 4.590]` degrees) while tightening
the low-lag wrist joints substantially below a generic 10 degree threshold.
The native sidecar fails closed with exact side/joint detail on a bound fault
or non-increasing safety timestamp. A sustained following error now emits one
nonfatal warning per excursion and does not stop the Runtime. The complete
motor-power recovery and protected low-speed right-side observer window above
provided the live commissioning evidence for this configuration.

The operator-provided official backup was re-audited at commit
`c33b653382eb9ed6fc75a5c669ad542f9f244d6c`. Its README explicitly records that
the active unilateral controller was an AArch64 binary without its C++ control
source. Its deployed wrapper only configured the two CAN interfaces and
executed that binary; it had no external joint-bound, tracking-error, or
all-motor preflight monitor. The available CAN encoder clamps only to each
motor model's wire-format position range, not the robot's URDF joint envelope.
The backup therefore cannot supply a missing IK rejection implementation and
does not disprove the containment gap found in this Runtime.

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
- teleoperation collection is exposed after the complete motor-power recovery
  and protected isolated-right observer window;
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
- the first live right-only canonical experiment,
  `blue_block_red_plate_v1`, published 30 episodes and 10,697 frames on
  2026-08-03. The final whole-dataset doctor passed with the exact configured
  task, eight right-side state/action names, and only Right Wrist plus
  AgentView videos. A detailed episode-4 audit found 351 finite 8-DOF
  state/action rows over 11.7 seconds with strictly increasing frame indices
  and timestamps; both 30 FPS AV1 videos contained and decoded all 351 frames,
  and representative start/middle/end images showed the blue block being
  placed in the red plate. The collection process then exited with no sidecar
  remaining and all four CAN links `DOWN` / `STOPPED`;
- the environment uses Python 3.12.13, OpenCV 4.13.0 and
  `torch 2.11.0+cpu`; CUDA is absent.

The pinned `external/embodied-ops` checkout was advanced from `072e847` to
`beecefa` on 2026-08-03, matching the revision already adopted by
`galaxea-a1-runtime`. VLAI now composes its shared CLI, collection interaction
and reset policy, canonical contract/file digests, and versioned Operator Panel
catalog/builders. Robot state/action semantics, AdjustPosition behavior,
readiness gates, CAN/camera ownership, and LeRobot dataset composition remain
local. The pinned `external/x-air-sdk` remains at `bf300508` because its
upstream has no newer commit.

The last full onboard hardware-free software check passed 128 tests. About
75 GB remained on the migrated root filesystem before live camera
commissioning.

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

This stage first performs a motion-free motor-feedback probe. Passing that probe
is immediately followed by enable/alignment motion, so it requires a fresh,
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
just stop
just sdk-status
```

Require both services/processes inactive, no sidecar remaining, and all four
links returned to `DOWN` / `STOPPED` before retrying or changing sides.

### 4. Teleoperation gate

Left, right, and paired evidence passed before the later `can0` incident. A
complete motor-power restart, complete right-pair probes, protected 15-second
observer window, guarded shutdown, and review of the resulting joint-error
maxima passed after the joint-safety change, so
`teleoperation.commissioned = true`. Any future change to that gate must include
the tracked config, loader/consumer expectations, relevant tests,
commissioning evidence, and affected documentation in one reviewable commit.

Do not change `command_ready`, policy transport, J2, or bus-stability evidence
merely to alter collection. Those are separate domains.

### 5. End-to-end collection

Start with no manual x_air runtime, observer, camera reader, or competing
controller active. `just collect` starts or reuses the marked Camera Service,
connects one raw client, preflights all three cameras before motor enable,
starts both configured runtimes, waits for both motion-free arm preflights,
releases both SDK constructors, and waits for fresh paired state. Use the SDK
startup `AdjustPosition` as the initial alignment, then use
teleoperation to refine the episode start pose. Enter `r` or use Reset to
repeat alignment on the active handles if needed, then press Enter in the
terminal or Start recording in the Panel. The workflow rechecks all cameras
after confirmation and records until Enter, discard, or quit. Save/discard
resets both sides on the same active handles before the next episode. A normal
return, error, or `Ctrl+C` closes the collection client, disables both pairs
and returns can0-can3 to `DOWN`; the read-only preview remains available.

While the left arm is out of service, use the separate right-only contract:

```bash
just collect-right blue_block_red_plate_v1 "put the blue block into the red plate"
```

It starts only `can0/can2` under the isolated-side lock, emits exact right-side
8-DOF vectors, and records only Right Wrist plus AgentView. Save/discard stops
frame acceptance, returns the right pair to Reset on its active handles, and
then finalizes the episode without disabling between episodes. Quit, failure,
or interruption disables the right pair. The Camera Service retains the full
physical camera contract, but Left Wrist is not a dataset feature or video.

First discard a finite episode:

```bash
just collect commissioning "hold position"
```

Then save a short new experiment and inspect it:

```bash
just collect smoke_v1 "hold position"
just dataset-doctor smoke_v1
just export-v21 smoke_v1
```

Never reuse an experiment name after a failed transaction until hidden
`.staging-*` or `.backup-*` siblings have been inspected. Never delete a
dataset, derivative, recording, checkpoint, or user file without explicit
authorization.

### 6. Operator Panel

The Panel exposes live Reset and collection because teleoperation is currently
commissioned:

```bash
just panel
```

Use `just panel-right` instead while collecting with the isolated right-only
contract; do not run both panels on the same configured port.

The Panel binds to the trusted robot LAN at the tracked address and port. It
has no authentication or transport encryption. Do not expose it to an
untrusted network. Start the persistent preview with **Start cameras** or
`just cameras`; stop it explicitly with **Stop cameras** or
`just cameras stop`. Its collection action invokes the same managed session
and is shown only when the tracked readiness gates permit it. The standalone
Reset action starts the guarded teleoperation lifecycle solely for the SDK
alignment and then shuts it down. All three
previews and normalized health remain visible between collection runs. During
the episode-start gate it offers **Start recording** and **Reset position**;
during capture it offers **Save episode**, discard, and quit while showing
indeterminate frame progress. Prompt
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
- the teleoperation implementation is commissioned after `can0` motor-power
  recovery and protected right-side recommissioning;
- discard and saved real episodes complete, including a 500-frame,
  three-camera managed save;
- the saved canonical v3 dataset passes doctor;
- its independently generated v2.1 derivative validates;
- operator confirmation is shared by terminal and Panel before recording;
- CLI presentation, collection input/decision policy, collection preview,
  camera health, capture progress, Panel schema/forms, and create-only prompt
  registration are composed through `embodied-ops`.
- the tracked right-only collection contract emits exact right-side 8-DOF
  vectors and only Right Wrist plus AgentView; 30 live episodes totaling
  10,697 frames have been atomically published and the final canonical dataset
  doctor passes;
- a hardware-free 12-frame right-only LeRobot v3 acceptance created and deeply
  inspected one episode with the blue-block/red-plate task, exact eight-name
  action/state vectors, and only `wrist_right` plus `agent` video features;
  128 Runtime tests and all 26 pinned `embodied-ops` tests pass.

The remaining operational issue is intermittent RealSense USB transport
stability, especially the D455 cable/port/power path. The prior right-leader
all-node no-ACK fault recovered only after a complete motor-power restart; a
host restart and targeted USB reset were insufficient. A recurrence must fail
closed and require the same physical distinction rather than automatic retry.
Continued right-only collection is commissioned; run the doctor after each
batch and keep watching the D455 transport. A mid-episode camera disconnect
must continue to invalidate the transaction rather than trigger transparent
recovery. Policy-command and inference motion remain separately blocked.

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
