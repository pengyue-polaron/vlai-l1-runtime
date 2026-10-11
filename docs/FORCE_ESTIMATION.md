# Follower torque feedback and endpoint wrench estimation

Current software supports a bounded, motion-free Follower feedback recording,
offline URDF geometry/Jacobians and a hardware-free numerical wrench solver.
It does **not** yet provide a
validated six-axis FT stream or modify the active teleoperation callback.
The first CSV-only capture and a subsequent right-Follower raw-frame pairing
capture passed bounded hardware tests. The operator subsequently authorized
all task-relevant non-motion hardware tests without per-test confirmation.
The newer VL1T telemetry contract remains offline-only. Guarded parameter reads
now cover all 16 Follower motors. Both sides' J3/J4 report a velocity range
different from the SDK; all position and torque ranges agree. This verifies
protocol scales, not physical torque calibration or a valid FT estimate.
Tracked per-side ranges and the offline `redecode-follower-feedback` command
now produce a separate comparison against preserved SDK values.
An offline static pipeline now connects decoding, explicit coordinate/torque
mapping, stationarity checks, gravity/friction inputs and wrench reconstruction.
It always labels numerical candidates as physically unvalidated.

An operator-authorized `automatic-right` diagnostic now also owns bounded
right-Follower PD holds and J7 trajectories without moving the Leader. Its
`sequence` mode checks startup, runs and reviews a hold, then runs and reviews
the sweep only if every preceding gate passed. `automatic_review.py` checks
saved hashes, exact named vectors, timing, raw identity/overflow/status and
final disabled feedback. It reports actual/reference spans, tracking errors,
torque, and the final half of each scheduled hold. Those are scheduled hold
windows, not an assumption that the arm was stationary. Same-command return
differences include actual posture changes and cannot identify friction alone.

The direct controller uses tracked PD gains with zero velocity/torque
feedforward. Its controller context differs from prior opaque teleoperation
captures; do not silently pool their baselines. `ft_validated` remains false
and external load is unknown unless separately documented by the operator.
This diagnostic commissions automatic data acquisition, not a general policy
transport, gravity compensation, zero calibration, or six-axis force control.
See the README for the single-command entry and SAFETY.md for the scoped
operator authorization and stop behavior.

## What is implemented

- `capture-follower-feedback` reads position (rad), velocity (rad/s), and SDK
  estimated torque (Nm) for the exact seven arm joints plus gripper. It uses
  the existing motion-free probe and its existing CAN SDK handle.
- The capture takes its physical identity, motor models/IDs, CAN setup and
  probe length/rate from System configuration. It acquires the exclusive
  lifecycle lock, rejects active controllers or links, verifies all configured
  adapter identities, and opens only the selected Follower endpoint(s).
- Every round requires all eight response IDs. Live CAN errors, increases in
  cumulative error/drop counters, or qdisc drops stop capture without retry.
  Exceptions and SIGINT/SIGTERM close owned links; no enable, disable, reset,
  alignment or teleoperation constructor is called.
- `feedback.csv` contains one row per motor per completed round. Timestamps are
  host SDK-cache read completion times, **not** per-motor hardware timestamps.
  Complete responses do not establish simultaneous acquisition or calibrated
  torque. Velocity and torque preserve the SDK's raw motor-coordinate values.
- `report.json` records configuration/library hashes, probe results and CAN
  health. Existing output directories are refused. Partial recordings are
  retained with failed status. Raw positions outside tracked arm bounds are
  listed for coordinate review; a passed communication report never authorizes
  motion. The diagnostic can observe an out-of-bounds unenabled arm without
  moving it or changing any limit.
- Report schema 2 additionally writes `raw_feedback.jsonl`: eight original
  response payloads, arbitration IDs, CAN/FD format and host socket-dequeue
  timestamps alongside the same round's SDK values. The existing probe socket
  supplies these frames; no additional CAN socket or query is added. Old frames
  are drained and discarded before each query, each motor must respond exactly
  once for unambiguous pairing, and per-motor host timestamps must increase.
  Missing/duplicate responses, invalid data lengths or missing raw/SDK pairs
  invalidate capture. The data are candidates for decoder equivalence testing,
  not proof of physical torque calibration or hardware-synchronized sampling.

The probe uses its tracked 40-round window today. Health inspection and file
writing add overhead, so this is not a 20 Hz realtime recorder. Left and right
are inspected sequentially and their data are not a synchronized pair.

## Completed right-Follower raw-pair hardware test

After explicit operator confirmation and healthy prechecks, this command ran
once on 2026-10-09 from 05:20:59 to 05:21:02 UTC. It performs **live CAN state
queries**; this is the completed command record, not an instruction to repeat:

```bash
sudo .venv/bin/python -m vlai_l1_runtime.cli capture-follower-feedback \
  --config configs/system/vlai_l1.toml \
  --side right \
  --output data/temporary/follower_feedback_raw_pair_check
```

Choose `left` or `bimanual` explicitly when those Followers are powered. Use a
new output directory for each authorized experiment. Do not automatically
repeat a failed probe, reset adapters, or start teleoperation to obtain data.
The command does not read Leader motors or start cameras.

The preceding `follower_feedback_first_code_check` directory is completed
schema-1 evidence and must remain intact. The schema-2 raw-pair test completed
40/40 rounds with all eight IDs, 320 paired raw payloads and 320 CSV rows.
Right `can2` added exactly 320 TX/RX frames, with zero CAN errors or drops and
no bounds violations. All links were verified DOWN/STOPPED afterward; left
Follower and Leader frame totals were unchanged. No enable or motion command
was issued.

The hardware-free comparison in
`data/temporary/follower_feedback_raw_pair_analysis/report.json` reproduces
every sampled SDK position, velocity and torque value exactly after float32
conversion using the pinned C binary's audited ranges. Its adjacent audit
script checks source hashes, names/IDs, timestamps and CSV/JSONL consistency.
Using the vendor Python passive ranges instead gives maximum discrepancies
of 0.025641 rad/s and 0.021978 Nm on the same frames. The sample covers a nearly
stationary arm, so agreement does not prove physical torque scale, firmware
range correctness across all values, force sensitivity or FT accuracy.
Host dequeue-to-SDK-read intervals were 56.79–58.01 ms, not motor acquisition
timing; raw status bytes are retained without inferring enable state.

Acceptance: all configured IDs on every round; finite ordered data and
strictly increasing host timestamps; ERROR-ACTIVE with zero live errors and
no error/drop counter increase; owned links DOWN after completion. Check the
report's bound violations before any later motion-capable stage. The earlier
2026-10-09 read found left J4 around -4.08 degrees, below the configured zero
lower bound, and J2 coordinate verification remains unresolved.

## Offline solver contract

`vlai_l1_runtime.wrench.estimate_wrench` accepts a verified 6x7 geometric
Jacobian and seven **already compensated external joint torques**. It returns
force in N, moment in Nm, joint-torque residuals, scaled singular values and
condition number. NumPy is imported only when solving; it is available in
the repository's development environment. Static imports and validation
remain hardware-free on Python 3.10.

- Jacobian columns: `joint_1` through `joint_7`, in radians; exclude gripper.
- Rows: linear velocity **of the specified reference point**, then angular
  velocity, both expressed in the specified frame. Do not substitute a spatial
  twist Jacobian whose linear component is about another origin.
- Wrench convention: environment acting on robot at that same point and frame,
  ordered Fx, Fy, Fz, Mx, My, Mz.
- Under `M*qdd + C + g + friction = tau_motor + tau_external`, supply
  `tau_external = model - tau_motor` after verifying motor-to-joint signs and
  scaling. The current raw CSV is not a valid direct solver input.
- Characteristic length, damping, minimum singular value, maximum condition
  number and maximum residual are required explicit inputs, with no robot
  defaults. Any future live integration must source these from reviewed tracked
  Runtime configuration. The solver scales moment/length into force units
  before damped SVD; it rejects singular/ill-conditioned geometry and excessive
  joint-torque residuals. It does not turn rejected data into zero force.

`tests/test_wrench.py` provides reproducible synthetic examples with known
six-axis loads, redundant-joint residuals and singular configurations.

## Remaining work before useful FT measurements

1. Verify raw motor coordinates against joint/model coordinates, including J2
   and left J4, without relaxing the safety bounds. Verify torque sign, scale
   and repeatability. Powered-only near-zero torque cannot establish external
   force sensitivity or prove a motor is enabled/disabled.
2. Supply velocity/torque telemetry through the existing controller owner,
   using a supported SDK accessor or a reviewed owner-contained receive-only
   decoder (see below). The current public callback carries positions only; the
   release lacks private controller headers needed to rebuild an extension.
   Runtime's active v1 state protocol and LeRobot position features are unchanged.
   The diagnostic must not be run alongside an active controller or used as a
   second CAN owner. No binary-layout casts or symbol-interception workaround
   is introduced.
3. At an independently approved safely held pose, measure an unloaded baseline,
   then apply/release a small measured endpoint load. Compare torque changes
   at matching poses and low velocities; motor actuation changes have the
   opposite sign from applied external joint torque under the convention above.
   Use a force gauge or reference FT sensor. Do not pin the arm against a hard
   stop or obstruction. The current diagnostic cannot run during that enabled
   test; stage 2 must first supply telemetry through the controller owner.
4. Verify force directions, known moment arms and multiple non-singular poses.
   Incorporate tool/payload gravity, bias and friction before moving to dynamic
   tests. A Jacobian must be computed from the verified geometry/mapping and
   tool point. `SerialChain` now computes that geometry from a rendered URDF,
   but it deliberately does not guess the motor-to-model coordinate mapping.

Six-axis estimates assume contact at the modeled tool. Link contacts, multiple
contacts, model errors and friction can violate that assumption. Residual tests
can catch some inconsistent torques but cannot prove contact location. No
force-control or safety threshold is enabled by this work.

## Offline URDF geometry

`vlai_l1_runtime.kinematics.SerialChain.from_urdf` reads an already rendered
URDF with explicit root, tip and expected moving-joint names. It follows only
that fixed/revolute chain, validates names, origins, axes, limits and topology,
and never loads meshes, invokes xacro, initializes ROS or opens devices. NumPy
is lazy and needed only for numerical evaluation. The result contains tool
position/orientation and the geometric Jacobian at the named tip origin,
expressed in the root frame. Positions are URDF radians, not raw motor values.
Out-of-limit positions are reported without clamping; this is offline geometry,
not a safety-approved pose. Prismatic and mimic joints on the selected chain
are rejected; unused gripper branches are excluded, not treated as a payload
model. This geometry module does not calculate gravity; the separate offline
subtree model described below includes masses and finger branches for that task.

For the currently rendered model the seven joint names on each side are
`xarm_<side>_joint1` through `xarm_<side>_joint7`. The SDK source selects root
`xarm_body_link0` and tip `xarm_<side>_hand`. The URDF's
`xarm_<side>_hand_tcp` origin lies another 0.08 m along the hand's local Z axis.
The Jacobian and wrench must refer to the same point. Expressed in the same
frame, `M_hand = M_tcp + r_hand_to_tcp cross F`; merely rotating the wrench
does not account for this moment-arm change.

Tests compare all linear/angular Jacobian columns with central differences,
check an analytic single-joint example, verify rotated joint origins and tool
point shifts, and recover a known synthetic wrench through the model/solver
pipeline. These are numerical checks, not evidence for actual robot calibration.

## 2026-10-09 static SDK/coordinate audit

No hardware was accessed during this audit. The configured Git remote's HEAD
and main still resolve to pinned revision `bf300508e179f652b23f0efaf3b6c9048f1f12e9`;
the operator has no additional source/version available. Only the configured
remote was checked, not every possible supplier release.

Static disassembly of the pinned aarch64 teleoperation library shows identity
copies in `JointMapper::motor_to_joint_position`, velocity and torque mapping,
and in `XArmJointConverter::motor_to_joint`. There is no extra J2 or J4 offset
in those inspected functions. This does not establish hardware encoder zeros.
The rendered J2 origin already has roll -pi/2 on the left and +pi/2 on the
right. J4 has zero origin rotation, axis (0,1,0), and model lower limit zero.
The observed left J4 value around -4.076 degrees remains outside both model
and tracked Runtime bounds under an identity coordinate assumption. Do not
add a guessed offset, change encoder zero, or relax a limit to remove it.

The saved CSV was evaluated under an explicitly **unverified identity**
motor-to-URDF mapping. The audit artifact is
`data/temporary/force_geometry_audit_20261009/report.json`; it includes the
model/input hashes, complete chains, hand/TCP transforms and Jacobians. With
an explicitly reported 1 m characteristic length used only for SVD units,
the left/right scaled condition numbers were approximately 295/667 and the
smallest singular values 0.00633/0.00280 m. These values depend on scaling and
the unverified mapping; they are not acceptance thresholds or proof of an
actual singularity. They indicate that the observed nearly straight poses
are poor candidates for an initial quantitative wrench experiment.

### A vendor passive-decoder scale mismatch

The C SDK maps its public motor-type enum to a different internal enum before
indexing `MOTOR_LIMIT_PARAMS`. The vendor Python passive decoder indexes its
table with the public enum directly. For the three configured motor models,
the resulting symmetric protocol ranges differ:

| Configured model | C SDK velocity / torque maxima | Vendor Python passive maxima |
| --- | --- | --- |
| DM8009 | 45 rad/s / 54 Nm | 10 rad/s / 28 Nm |
| DM4340 | 8 rad/s / 28 Nm | 30 rad/s / 10 Nm |
| DM4310 | 30 rad/s / 10 Nm | 50 rad/s / 5 Nm |

The right-Follower paired capture also reproduced these C SDK ranges on its
sampled values. This evidence applies to the pinned aarch64 binary only; it does not
validate either scale against the motor firmware or physical output torque.
Current Runtime captures use the C SDK getters, never the Python passive
decoder. Do not adopt that decoder as an online workaround or silently rescale
existing data. Any future raw-frame path must reconcile firmware/protocol
ranges, store reviewed values in tracked configuration, and test against the
same frames decoded through the existing owner.

Reproducible binary evidence: CAN library SHA256
`cd687a566d03307e449af451b8f683f3f3bffd266675f49c2679d5a7d2e67e35`;
`xarm_sdk_init_arm_motors` at 0xcc60 maps public code 0 to internal 1, and public
codes 1..9 through bytes at virtual/file offset 0x177f8. State decoding at
0x16af0 indexes triples of doubles in the 312-byte `MOTOR_LIMIT_PARAMS` object
at virtual/file offset 0x17ae8. The configured public codes 0/1/4 map to internal
1/3/7. These addresses are audit evidence, not Runtime ABI or production
configuration. The existing decoder was not edited, and no binary was loaded
or invoked to obtain this evidence.

### Online integration still required

The public teleoperation C header and exported `xarm_teleop_*` symbols contain
no velocity/torque state accessor. One possible route is an owner-contained,
receive-only raw-frame reader inside the existing native sidecar process;
the paired capture is preparation for validating its decoder. It would not
create a second physical-device-owning process. Before implementing it, verify
the protocol/ranges, frame status and freshness using same-round raw/C-SDK pairs,
keep its configuration and identities tracked, and define a versioned transport
to consumers. This native reader is not yet implemented or commissioned.

Another route is a supplier-supported SDK extension. It should copy
position, velocity, measured/estimated torque, validity/error status and
per-motor update timestamps/sequences from the already-owned handles; distinguish
feedback torque from commanded torque; document coordinates, signs, units,
range and cache freshness; and publish without blocking the control callback.
An extra getter must neither create another CAN owner nor refresh/send frames.
It should fail on stale/incomplete data rather than report zero force. Once
available, the extension needs protocol-versioned consumer support and a new
bounded validation before the enabled known-load experiment. No messages were
sent to the supplier, and no undocumented C++ object layout was accessed.

## Pure native feedback core (2026-10-09)

`native/xair_sidecar/motor_feedback.{hpp,cpp}` now supplies a standalone C++17
decoder and a single-thread-owned candidate accumulator. Neither opens sockets,
loads the SDK, reads clocks nor creates threads. The module is **not linked to
the live sidecar executable** and adds no command/enable path or v1 wire change.
Motor identities, ranges and age/skew limits are explicit constructor inputs;
there is no motor-model range lookup or fallback. Future live callers must
obtain reviewed settings from tracked System configuration. Motor ranges and
their evidence are now tracked under `follower_feedback`; live age/skew/status
policy and native owner integration remain unimplemented.

The decoder accepts configured standard data IDs with exactly eight payload
bytes, validates the slave nibble and positive timestamp, and keeps the original
payload, CAN/FD indication, raw status/temperature bytes and integer endpoint
indicator. Protocol endpoint values can indicate saturation but do not prove it.
Status nibbles, including unknown/fault values, remain visible for analysis;
**successful decoding or assembly never means healthy/enabled or valid for FT**.

The accumulator requires exact eight-motor ordering and distinct IDs. Every
motor must have updated since the preceding batch. Missing, expired or excessively
skewed candidates return no batch, never zeros or a republished cached batch.
Invalid target frames, CAN flags, future timestamps, non-increasing per-motor
or assembly times, and explicitly reported input faults permanently latch the
session; later data cannot clear it. Old/skewed candidates can be replaced by
fresh data, but a live session will additionally need an owner-enforced startup
deadline and ongoing no-batch timeout. The Python consumer session implements
deadline checks, but their integration into a live owner is not implemented.

These checks bound **host dequeue** age/skew only. Old packets drained from a
backlogged socket could still look fresh; online ingress must separately handle
receive queue overflow, kernel receive timestamps/clock conversion, bounded
draining, and any parameter-reply traffic sharing feedback IDs. Never place
blocking receive or parsing work inside the realtime SDK callback. Normal CAN
health checks and motor-status validation remain independent requirements.

Hardware-free build and tests (no SDK root needed):

```bash
cmake -S native/xair_sidecar -B /tmp/vlai-feedback-pure-build \
  -DVLAI_L1_SAFETY_ONLY=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/vlai-feedback-pure-build --parallel 2
ctest --test-dir /tmp/vlai-feedback-pure-build --output-on-failure
```

The native decoder also exactly reproduced all three float32 SDK fields in all
320 saved raw frames using the audited ranges supplied as replay inputs. The
local replay file is
`data/temporary/follower_feedback_raw_pair_analysis/native_replay.tsv`; run it
with `vlai_l1_motor_feedback_test --replay FILE` from the build directory.
Each whitespace-separated row carries name, response ID, slave ID, position/
velocity/torque maxima, host receive time, payload hex, then three SDK floats.
It is an offline test input, not a Runtime configuration or live capture tool.
Its source JSONL SHA256 is
`59cdd87177258d1e3af229a8a90172143341859d73add34a7b935fba3eac331b`.
The permanent C++ regression fixture retains one right J1 frame from this file;
synthetic tests cover endpoint quantization, status preservation and invalid/
missing/stale/skewed/timestamp-regressing data.

### Protocol evidence and next commissioning stages

The manufacturer-authored [DM-J8009-2EC V1.0 manual, pages 4–6, mirrored by OpenArm](https://docs.openarm.dev/assets/files/dm8009-90ee7f15d06666e15afd49e5d5417150.pdf)
supports the observed feedback bit packing and states that P_MAX/V_MAX/T_MAX
are configurable. It lists fault codes 8 through E, but does not settle normal
status 0/1 semantics for this installed firmware. This is documentation evidence,
not proof that all installed models/firmware match that manual. No enable-state
claim or live fault-code policy is inferred from the captured zero high nibbles.

Before online activation, establish installed model/firmware and actual protocol
ranges, then track their provenance, valid status policy and telemetry timing
limits in System configuration. The current public C header has no parameter-read
API. The separate guarded raw register reader described below has now been
reviewed and exercised under operator authorization on all Follower motors.
Live telemetry configuration and status policy remain pending. Do not write
parameters to make the motor agree with an SDK table.

For coordinate commissioning, start with right Follower only and document a
supported, stationary physical pose and the corresponding bounded diagnostic
capture. Compare measured link/joint geometry to URDF FK; a second independently
approved supported pose with a small measured change in one joint establishes
that joint's sign/scale. Absolute offsets require a physical reference pose as
well, not just differencing two encoder readings. The operator must approve the
support and movement procedure before manipulating the arm; no zeroing or SDK
enable/alignment is part of that diagnostic. Leave the unresolved left J4 bounds
violation untouched. Only after mapping, telemetry and compensation are verified
should a held-pose load test compare a measured applied force/moment against FT.

## VL1T raw telemetry contract and consumer session

`native/xair_sidecar/feedback_wire.{hpp,cpp}` serializes the native candidate
batch; `vlai_l1_runtime.feedback_wire` parses and validates it. This is a
separate **VL1T version 1** contract for one Follower side, not a change to live
VL1S position state. There is no live publisher/receiver socket or LeRobot
integration yet. Native encoding validates names, configured identities and
raw/decoded agreement before writing explicit little-endian bytes.

Every packet is exactly 328 bytes: a 72-byte header plus eight 32-byte records
in `joint_1` through `joint_7`, `gripper` order. Role is always Follower;
coordinates remain raw motor rad/rad/s/SDK-estimated Nm. These are not LeRobot
position features or a calibrated wrench.

| Header offset | Type | Meaning |
| --- | --- | --- |
| 0 | 4 bytes | `VL1T` magic |
| 4 | uint16 | Version 1 |
| 6 | uint8 | Side 0 left / 1 right |
| 7 | uint8 | Reserved, must be zero |
| 8 | 16 bytes | Nonzero session identity |
| 24 | 32 bytes | Nonzero expected configuration SHA256 |
| 56 | uint64 | Positive source sequence |
| 64 | uint64 | Host batch-assembly monotonic ns |

| Per-motor offset | Type | Meaning |
| --- | --- | --- |
| 0 | uint16 | Standard response ID |
| 2 | uint8 | Bit 0 CAN-FD, bit 1 integer protocol endpoint; other bits zero |
| 3 | uint8 | Reserved zero |
| 4 | uint64 | Positive host socket-dequeue monotonic ns |
| 12 | 8 bytes | Original feedback payload, including raw status and temperatures |
| 20 | 3 x IEEE754 float32 | Position, velocity, torque |

The future owner must issue a fresh session identity, distribute it and the
reviewed config digest to consumers, and ensure a shared host monotonic clock.
These fields establish expected provenance, not cryptographic authentication.
The config must cover ranges and status/timing policy before live activation;
no such runtime settings are silently inferred from the current config file.

`FeedbackSession` requires explicit side/session/config, ordered response/slave
IDs, per-motor allowed raw-status sets, session start, maximum age/skew, startup
deadline and silence deadline. It rejects old/wrong sessions, malformed or
non-finite packets, protocol endpoints, unacceptable status codes, timestamp
regressions, duplicate sequences and incomplete fresh updates. Source sequence
gaps are allowed within the freshness limits; this does not relax CAN-drop
rules. The consumer must call `check_deadline(now_ns=...)` periodically even
when no packets arrive. Any failure latches the session and clears its cached
packet. A later good packet cannot recover it. Transport/owner faults can be
latched explicitly through `invalidate`.

All time limits concern host observations. Kernel receive-queue age/overflow,
installed firmware status semantics and physical torque calibration remain
separate gates. An accepted packet means this explicitly supplied raw telemetry
policy passed, not that the arm is enabled or FT-valid. The tests' allowed
status `{0}` is synthetic policy only, not a deployed enabled-state assertion.

Cross-language tests compile only the pure native target, compare every byte
against an independent Python specification fixture, then pass the result
through the Python session checks. Saved-data replay also passed all 40 groups/
320 motors through native decode/serialization and Python session validation,
preserving original payloads, timestamps and SDK float32 values exactly. Evidence
is `data/temporary/follower_feedback_raw_pair_analysis/wire_replay_report.json`.
Its session/config bytes and assembly timestamps are explicitly synthetic, and
its timing bounds are derived solely for offline replay, not robot defaults.
The test executable's `--replay-wire INPUT OUTPUT` option generates this offline
wire stream; it never opens CAN.

## Optional parameter diagnostic — first right-J3 test completed

The operator confirmed no parameter records are available. The pinned
[manufacturer source](https://github.com/dmBots/motor-sdk/blob/0b2ede457bdbf0882e29ab9958ab8fda047b7f4a/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)
defines a parameter-read transaction to standard ID `0x7ff` with payload
`slave_lo slave_hi 33 register 00 00 00 00`. Replies echo the slave, operation
and register before a little-endian value. Registers 7/8 hold master/slave IDs,
13/14/36 hardware/firmware/sub-version integers, and 21/22/23 the three float32
ranges. Its DM4340 example velocity maximum is 10 rad/s, while the pinned x_air
C binary uses 8; this reinforces the need to read the installed configuration.
Neither source proves what this robot's firmware supports or stores.

`motor_parameters.ParameterRead` only builds the eight allowlisted read
transactions above and validates matching replies. It exposes no write,
save, enable, mode-change or zeroing operation. Versions remain raw integers;
range values must be finite/positive and reported identities must agree with
tracked configuration. No vendor code was downloaded into the repository,
imported or executed.

`capture-follower-feedback --read-parameters-for MOTOR` runs only for one side
and one named motor, under the existing exclusive lock and guarded lifecycle.
It first completes the full 40-round/eight-motor feedback capture and releases
that SDK handle. Only then does the same process open a temporary raw parameter
socket on the selected Follower. It sends one request per register, sequentially,
without retransmission. CAN/FD framing follows tracked `can.fd`. The parameter
phase reuses the tracked diagnostic budget (`motor_probe_duration_s`, currently
2 s) and per-transaction observation window (`1 / motor_probe_rate_hz`, currently
50 ms); it does not add independent timing defaults.

Every transaction requires exactly one valid matching reply within its window.
Unsolicited queued data before a query, duplicate/late/malformed responses,
CAN error frames, queue overflow, timeout, invalid values or health failures
stop the run without retry. CAN identity/counters/qdisc are checked around each
transaction. An entire reply window is observed to catch duplicates. Normal
state broadcasts or parameter replies sharing IDs are not silently interpreted
as successful register reads. Before activation, any future continuous telemetry
path must exclude parameter transactions from its state decoder.

Schema 3 records parameter readings in `parameters.jsonl` and `report.json`,
including raw request/reply bytes and host timestamps. Partial valid readings
are retained on failure; malformed/timeout transactions yield a failed report,
not invented values. Existing state-only behavior retains schema 2. Links are
closed on completion/failure/signals using the existing capture cleanup. A pass
does not change configuration, resolve coordinate offsets, or validate physical
torque scale.

The first test was **right Follower J3 only**, to check the DM4340 discrepancy.
After operator approval and fresh healthy prechecks, the following command ran
once on 2026-10-09 at 06:16 UTC. This records a completed run; do not repeat it:

```bash
sudo .venv/bin/python -m vlai_l1_runtime.cli capture-follower-feedback \
  --config configs/system/vlai_l1.toml --side right \
  --read-parameters-for joint_3 \
  --output data/temporary/follower_right_j3_parameter_check
```

Expected scope: 320 state queries/responses during preflight, then at most eight
parameter queries to the configured J3 address. Require all preflight motors,
eight matching parameter replies, zero errors/drops and links DOWN afterward.
No Leader query, other-side query, enable, alignment, parameter write or motion
command is part of this test. A timeout or unsupported reply ends the experiment;
do not retry, switch framing, change registers or reset hardware automatically.
The operator approved this exact scope and it passed. The later standing
authorization covers further task-relevant non-motion diagnostics; it does not
authorize enable, movement, loading, parameter writes or recovery.

### Results: installed right J3 differs from SDK velocity decoding

From 06:16:13–06:16:16 UTC, all 40 state rounds and eight J3 reads completed.
All parameter responses were eight-byte CAN-FD payloads on configured ID 0x013;
observed host send-to-dequeue intervals were 0.742–0.835 ms. The right bus added
exactly 328 TX/RX frames with zero CAN errors/drops or qdisc drops. No joint bounds
violations were reported. All interfaces were DOWN/STOPPED afterward; other
endpoint frame counts were unchanged.

| J3 register | Reported value | Pinned C SDK decoding range |
| --- | --- | --- |
| Master/slave IDs | 19 / 3 | Matches configured IDs |
| Hardware / firmware / sub-version | 0 / 925970741 / 53 (raw integers) | Not inferred |
| P_MAX | 12.5 rad | 12.5 rad |
| V_MAX | **10 rad/s** | **8 rad/s** |
| T_MAX | 28 Nm | 28 Nm |

The parameter records are in
`data/temporary/follower_right_j3_parameter_check/parameters.jsonl`, alongside
the state CSV/raw JSONL and schema-3 report. The raw firmware bytes are
`35 31 31 37`; their version-string interpretation is not established. Hardware
version zero is preserved as reported, not replaced with a guessed revision.

Offline comparison in
`data/temporary/follower_right_j3_parameter_analysis/report.json` verifies all
query/reply identities and source hashes, then re-decodes only J3 with its read
ranges. Position and torque exactly match all 40 SDK samples; velocity becomes
1.25 times SDK velocity within float32 rounding (maximum difference in this
near-stationary sample: 0.0014652 rad/s). The original source files are unchanged.
The actual eight response payloads are retained as pure regression fixtures.

This proves the J3 register/SDK range disagreement, not physical calibration or
full-range accuracy. Do not patch the opaque SDK, rewrite motor parameters,
silently rescale historic data, or apply J3's result to other endpoints/motors.
The future native reader must take reviewed per-motor ranges from tracked
configuration and distinguish those raw decoded values from SDK cache values.
The subsequent survey below independently checks each remaining motor. Live
telemetry integration and coordinate/known-load checks are still pending.

### Results: complete two-Follower parameter survey

Under the updated non-motion authorization, 15 sequential captures ran on
2026-10-09 from 06:27:12 through 06:28:34 UTC: seven remaining right motors,
then all eight left motors. The earlier right-J3 capture completes the set.
Every new capture passed 40 rounds with all eight state responses, followed by
eight allowlisted parameter reads for its selected motor. Each added exactly
328 TX and 328 RX frames; the new captures total 4,920 in each direction.
CAN live/cumulative errors and interface/qdisc drops remained zero. Afterward,
all links were DOWN/STOPPED; right/left Follower totals were 3,584/3,264 frames
in each direction, with both Leaders unchanged at zero.

Corresponding motors on both sides report the following values. All report
P_MAX=12.5 rad, hardware version 0, and master/slave IDs matching configuration.
Firmware/sub-version values below are raw integers, without inferred semantics.

| Motors on each side | Configured model | Reported V_MAX (rad/s) | Reported T_MAX (Nm) | C SDK V_MAX (rad/s) | Firmware / sub-version |
| --- | --- | --- | --- | --- | --- |
| J1, J2 | DM8009 | 45 | 54 | 45 | 925971510 / 52 |
| J3 | DM4340 | **10** | 28 | **8** | 925970741 / 53 |
| J4 | DM4340 | **10** | 28 | **8** | 959525173 / 53 |
| J5, J6, J7, gripper | DM4310 | 30 | 10 | 30 | 925970485 / 52 |

All reported torque ranges agree with the C SDK. New source captures are in
`data/temporary/follower_parameter_survey_20261009/{right,left}/{motor}`.
The offline audit and report are in
`data/temporary/follower_parameter_survey_analysis_20261009/`. They verify
16 captures, 128 register values and 5,120 state frames, including the earlier
right-J3 run. Source hashes, identities, request/reply matching, timestamps,
health and raw/CSV consistency are checked. Binary table inspection loads no
SDK. Decoding all saved frames with the audited C SDK ranges reproduces every
SDK float32 exactly. Using the reported ranges preserves all positions and
torques; both sides' J3/J4 velocities become 1.25 times SDK values within
float32 rounding. Source data and active runtime configuration remain unchanged.

Observed torque quantization steps are approximately 0.026374 Nm (DM8009),
0.013675 Nm (DM4340), and 0.004884 Nm (DM4310). Across these stationary samples,
the largest per-motor population standard deviation is 0.010611 Nm. These are
protocol resolution and observed variation, not calibrated accuracy or proof
of response to external load. All observed status high nibbles are zero;
their meaning for installed firmware remains unverified.

Left J4 remains approximately -4.076 degrees, below its tracked zero lower
bound. No limit or offset was changed. Coordinate mapping, gravity/friction
compensation, live telemetry ownership/timing/status validation, and physical
known-load validation remain gates before useful six-axis FT can be claimed.
Movement, supported repositioning and applied loads require an agreed physical
test procedure outside the current non-motion authorization.

## Tracked ranges and offline re-decoding

The optional `follower_feedback` section extends System schema 4 for diagnostic
decoding only. Each side must supply the exact eight named motors, with positive
finite float32-compatible P/V/T ranges, raw hardware/firmware/sub-version
integers, and the SHA256 of its original parameter JSONL. The section also
records the original survey configuration hash. Missing evidence leaves this
decoder unavailable; no model lookup or SDK fallback is used. Configuration
fingerprinting includes the entire section. Existing command readiness gates
and control gains are unchanged.

`feedback_decode.decode_follower_feedback` decodes paired raw frames using
these ranges, checking configured response IDs, slave nibbles and frame format.
It preserves raw status/temperature bytes and protocol-endpoint flags without
interpreting them as enabled, healthy or calibrated. Position and torque are
still in unverified motor coordinates. The SDK samples remain separate.

For an existing capture, this hardware-free command creates a new comparison.
It has been exercised as shown; choose a fresh output path on subsequent runs:

```bash
.venv/bin/python -m vlai_l1_runtime.cli redecode-follower-feedback \
  --config configs/system/vlai_l1.toml \
  --source data/temporary/follower_right_j3_parameter_check \
  --output data/temporary/follower_right_j3_tracked_decode
```

The source must be a passed schema-2/3 capture from the current or original
survey configuration. The command checks endpoint identities, complete ordered
rounds, exact vectors and increasing per-motor/producer timestamps before
creating output. Inputs use the regular-local-file restriction of the config
loader; device files/FIFOs are not opened. Existing outputs are refused.
`decoded.csv` retains SDK fields alongside `reported_*` fields, original raw
payloads and host receive timestamps. The report records source/decoding config
hashes, input hashes, full range evidence and the source bounds findings. It is
an offline comparison, not renewed CAN-health or telemetry-freshness evidence.

Completed replay evidence is in
`data/temporary/follower_tracked_range_replay_20261009/`: all 16 captures and
5,120 motor frames pass, with exact position/torque preservation and J3/J4
velocity correction by 1.25 within float32 rounding. The pure native decoder
also agrees exactly with all 5,120 Python values using the same tracked ranges.
Its generic test executable prints "SDK float32 agreement"; in this replay its
reference columns deliberately contain **reported-range decoded values**, not
the original SDK velocities. No SDK library or hardware was opened in replay.

This section changes the configuration file hash. Historical source captures
retain their original hash and bytes. Range provenance is an observation at
the survey date; it does not detect a later firmware or parameter change.
Live activation still requires parameter/status verification, owner timing
integration and coordinate/compensation/known-load validation.

## Offline gravity model

`vlai_l1_runtime.gravity.GravityModel` reads one complete subtree from a
rendered URDF. The selected branch must be a direct child of an explicit root;
all its descendants are included, including fixed attachments and mimic
prismatic fingers. It validates joint topology, explicit origins/axes/limits,
positive finite masses and COM locations, and mimic dependencies. Missing mass
data are rejected; only explicitly listed empty frame links can be massless.
It loads no meshes, external XML, SDK, ROS, camera or CAN resources. NumPy is
lazy and required only for numerical evaluation.

The caller supplies exact independent URDF coordinates, a gravity acceleration
vector expressed in the root frame, and an explicit tuple of additional
`PointMass` payloads (empty if none are included in the experiment). Revolute
inputs/efforts use rad/Nm and prismatic ones use m/N. No gripper encoder-to-finger
mapping, motor zero offset, installed payload or robot mounting direction is
guessed. Added masses must exclude components already represented in the URDF;
physical double counting cannot be detected from names alone.

The result is the **holding effort opposing gravity**, `g(q) = dU/dq`, together
with the potential energy, individual mass contributions, total model mass,
joint units and any model-limit violations. COM locations are transformed in
their link frames; inertial-origin RPY does not rotate the COM translation.
Mimic-joint derivatives are included in the source joint's generalized effort.
Inertia tensors are unnecessary for static gravity and are not validated by
this model. This is not an inverse-dynamics implementation.

With verified joint/torque coordinates and a stationary, actively held pose,
the ideal friction-free relation is `tau_external = g(q) - tau_motor`.
Friction, bias, tool/camera/cable loads and external support must be resolved
before applying it to real telemetry. The current powered-only capture does
not establish those conditions. No raw-to-compensated live pipeline was added,
and the existing controller friction constants were not treated as calibrated
force-estimation parameters.

The saved rendered model SHA256 remains
`a4470dbc12bc09c879af188cded91d82e5164d0d52c47dbdfc8884fb87445596`.
Each side has 11 mass-bearing links totaling 5.5057424269 kg: arm link0 through
link7, hand and two fingers. This includes the fixed arm mounting link; it is
not a statement that all 5.506 kg load every joint. Each finger is modeled as
0.0360254534 kg. These are model values, not measurements of this robot. The
vendor arm YAML's calibration comment does not establish current-device
provenance, and actual wrist cameras/tools may require additional payloads.

Offline evidence is in `data/temporary/follower_gravity_audit_20261009/`.
For 16 synthetic poses per side, all eight generalized gravity efforts agree
with central differences of potential energy to below 1e-9 (Nm or N depending
on the joint). An additional hypothetical 0.5 kg TCP load agrees with its
Jacobian-projected gravitational wrench within 1.4e-15 Nm. A separate synthetic
seven-joint regression recovers a known six-axis wrench after gravity
subtraction. None of these numerical checks validates physical FT accuracy.

The audit also evaluates the saved pose only under an explicitly unverified
identity motor-to-URDF mapping, assumed body-frame gravity `(0, 0, -9.80665)`,
zero modeled finger opening and no additional payload. Left J4's bounds
violation remains visible. No gravity vector, mapping, payload or model
calibration was activated in Runtime configuration; all live gates remain.

## Offline static pipeline and current-pose rejection

`static_wrench.OfflineStaticWrench` processes one ordered saved stream. All
seven arm coordinate maps, auxiliary model coordinates, gravity/payloads,
static friction efforts, raw-status sets and numerical/stationarity/error
budgets are explicit constructor inputs. The stream takes physical motor IDs,
reported ranges and hard joint bounds from validated System configuration.
No values are automatically copied from the controller's friction model.

For each arm joint, `q_model = a*q_motor + b`; the corresponding generalized
effort is `torque_gain * (tau_motor - torque_bias) / a`. This separates an
encoder zero offset from torque calibration and preserves power under the
coordinate transformation. Tests include negative scales and non-unit ratios.
The model chain and gravity subtree must share a root and URDF hash, and their
named joint mappings must agree. Extra model coordinates, such as finger
opening, are explicit hypotheses until a physical mapping is established.

Wrong side/IDs/status, protocol endpoint values, old/skewed pairs, regressing
timestamps and raw/model bound violations latch an invalid stream. Velocity
or position drift clears the stationary window; gaps require a fresh window.
Only after the explicit continuous window does the pipeline compute
`tau_external = gravity + static_friction - calibrated_motor_effort`. Singular
geometry, excessive condition number, unexplained joint-torque residual or
excessive propagated input error rejects the result. Rejection returns no
estimate and never repeats a previous force or substitutes zero.

`WrenchEstimate.torque_to_wrench` exposes the fixed damped solve's linear gain.
`propagated_torque_error` maps independent absolute joint-torque bounds to
componentwise force/moment bounds through the absolute gain. The pipeline
includes one **full** protocol torque step (firmware rounding is unverified)
plus explicit unmodeled-torque bounds. These are not statistical confidence
intervals; unknown geometry error, inadequate model-error bounds and damping
bias remain outside that calculation. Adversarial synthetic torque-error tests
attain each predicted component bound.

The operator subsequently confirmed attached cameras, an upright base and a
safe scene. Camera/bracket masses/COM and mechanical support/limit contact were
not specified. The existing URDF must not be treated as a complete measured
payload model, and a powered-only reading must not be zeroed as an unloaded
baseline merely because the workspace is safe.

Under the standing non-motion authorization, a fresh bimanual capture completed
2026-10-09 07:06:56–07:07:01 UTC. Each side passed 40/40 rounds and added exactly
320 TX/RX frames, with zero CAN errors and interface/qdisc drops. All links were
DOWN/STOPPED afterward; right/left totals were 3,904/3,584 each direction and
both Leaders remained at zero. No enable, alignment, parameter write or motion
command occurred. Source: `data/temporary/follower_static_pipeline_check_20261009/`.

The replay in `data/temporary/follower_static_pipeline_analysis_20261009/`
contains original/decoded comparisons, per-record pipeline outcomes, policy
inputs/hypotheses, hashes and a saved-pose reference plot. All 40 left records
are invalid because J4 is below the existing bound. On the right, eight records
fill the 0.5 s diagnostic stationary window and the remaining 32 are rejected
for conditioning. **Neither side produces a numerical FT candidate.**

At an explicitly diagnostic characteristic length of 0.2 m and unverified
identity coordinates, the scaled condition numbers are about 130 (left) and
294 (right). One full torque quantization step alone propagates to a worst-case
body-Z force component bound of roughly 3.79 N/8.66 N respectively. Adding a
purely illustrative 0.1 Nm error per joint increases these bounds to roughly
29.8 N/69.5 N. These are sensitivity calculations, not measured forces or
accuracy claims. The right arm's current nearly straight model pose is a poor
place to demonstrate full six-axis reconstruction; damping cannot recover a
weakly observable direction.

### Right active position observation (2026-10-09)

After physical Follower identity and suspended endpoints were confirmed and
right Reset motion was observed as normal, a guarded isolated-right session
recorded 1501 position samples across 15 seconds. Evidence and offline analysis
are in `data/temporary/right_active_observation_20261009/`. The session ended
with verified shutdown and all CAN links DOWN; the left side was not operated.

Right Follower J4 was about 35 degrees. With the same unverified identity
motor-to-model map and 0.2 m characteristic length, first/last-pose Jacobian
condition numbers were 15.98/16.11, substantially better than the earlier
nearly straight pose. Position variation was still measurable: peak-to-peak
J7 1.333 degrees and J1 1.137 degrees. This was normal Leader tracking, not an
independent fixed-hold mode. Both right role vectors remained in bounds.

Only position was captured during active control. This does not establish
external force, torque signs, payload calibration or FT accuracy. The capture
script's sequence fields represent the assembler output counter, so its
zero skipped-sequence report must not be used as native packet-loss evidence.
The recorded source timestamp gaps were 9.985–10.016 ms.

### Active raw torque and same-pose tare (2026-10-09)

The native sidecar now has an opt-in diagnostic raw receiver in the same process
as the SDK motor owner. It sends no CAN frames and opens no additional SDK
handle. The existing `run-xair` motion workflow accepts `--feedback-log PATH`
and `--feedback-max-frames COUNT`; this does **not** make `run-xair` read-only.
The managed runtime exposes the options only for an isolated single side.
Motor response IDs and timing budgets come from Runtime configuration. Default
launches are unchanged. Logging starts after initial SDK alignment, before the
regular control loop starts.

The receiver stores host monotonic/realtime dequeue times, kernel realtime
receive times, raw CAN bytes/IDs/format and socket overflow counts. Kernel
receive times are not motor acquisition times. Missing/stale responses,
overflow or malformed frames/timestamps fail the diagnostic; the sidecar stops.
In default full-retention mode, a full buffer also stops the diagnostic. The
explicit `--feedback-retention latest` mode instead evicts oldest records in
constant time, preserves chronological export, and adds a
`# retained_latest_evicted=N` CSV header. Reports must declare matching
`feedback_retention=latest` and `evicted_raw_frames`; the offline parser refuses
mismatched or undeclared retention. The frame sequence is local to the retained
window, not a claim that the entire session was saved. All live input/watchdog
checks run even on data that is later evicted. It performs no file writes during control.
The create-only CSV is written after SDK handle release; failed runs must not
be used as valid capture evidence. This is separate from the still-unconnected
VL1T live telemetry path.

An authorized right-only 10-second run captured 16706 frames and 1001 native
position packets, with zero receive socket overflow, maximum per-motor gap
5.102 ms and maximum eight-motor receive skew 4.508 ms. All raw status nibbles
were 1; no universal status meaning was inferred. Data, provenance, postchecks
and analysis scripts are in `data/temporary/right_active_torque_20261009/`.

Using the existing unverified identity coordinates and incomplete gravity/
friction model, all stationary full-wrench attempts failed the 0.1 Nm joint
residual threshold. A separate **offline same-pose tare** fit a constant
model-minus-motor baseline on seconds 2..4, then evaluated seconds 6..10 within
an explicit 0.002-rad pose envelope. The 833 held-out numerical candidates had
delta-force standard deviations approximately [0.110, 0.083, 0.206] N and
delta-moment standard deviations [0.0119, 0.0073, 0.0132] Nm. These measure
zero-load repeatability only; no known force, gain, direction, accuracy or
multi-pose calibration has been verified. Nothing was written to motor zero,
torque calibration, controller gains or Runtime config. All candidates remain
`ft_validated=false`; do not treat the baseline as a general sensor calibration.

### Repeatable unloaded version (2026-10-11)

`vlai_l1_runtime.unloaded_baseline` is a hardware-free analyzer for the existing
same-owner right-Follower passive capture. It reports **delta wrench relative
to the unloaded posture**, with camera, gripper and brackets still installed.
It does not provide an online FT topic or write motor/controller calibration.
Changing posture, attachments, gripper opening, or restarting the session needs
a new baseline. No baseline import into another capture is supported.

Example (output directory must not already exist):

```bash
.venv/bin/python -m vlai_l1_runtime.unloaded_baseline \
  --config configs/system/vlai_l1.toml \
  --capture data/temporary/right_unloaded_20261011 \
  --urdf build/xair-assets/v10_follower.urdf \
  --policy configs/diagnostics/right_unloaded.json \
  --output data/temporary/right_unloaded_20261011/reanalysis
```

The tracked diagnostic policy fits seconds 2..4 and evaluates seconds 6..10;
time starts at the first complete raw eight-motor batch. These windows are
disjoint and require minimum sample counts and coverage. All fit samples must
be stationary. After fitting, every intervening batch, including the gap before
the validation window, is checked against the reference posture for all eight
motors. Movement outside 0.002 rad (about 0.115 degrees), loss of stationarity or
a sample gap invalidates the baseline for the rest of the window, even if the
arm returns. Rejected rows contain a reason and `delta_wrench=null`.

`report.json` records hashes of capture/config/model/policy, raw capture health,
reference posture, baseline residual, resolved policy, held-out statistics and
assumptions. `delta_wrench.jsonl` preserves every validation decision, ordered
Fx/Fy/Fz in N then Mx/My/Mz in Nm, at `xarm_right_hand_tcp` expressed in
`xarm_body_link0`. `static_results.jsonl` retains the untared pipeline decisions.
Every result has `ft_validated=false`. A passed report means this diagnostic's
data, stationarity and numerical checks passed; it does not establish force
accuracy, sign or gain. The bounded local-file reader refuses symlinks/devices;
configuration retains its original 1 MiB limit while capture input permits
64 MiB. Raw digest/identity/timing/overflow/protocol failures are rejected.
Captures explicitly marked `external_load_present=true` (or a present value
other than boolean false) are refused by unloaded analysis. Legacy captures
without this field rely on the documented operator unloaded declaration; the
analyzer cannot infer absence of load from motor data.

Acquisition remains the existing guarded isolated-right lifecycle, with its
motion-free preflight before enable/alignment and automatic disable/down after
the bounded observation. The reviewed diagnostic invocation is preserved in
`data/temporary/right_unloaded_20261011/capture.py`; **running it moves the right
Leader and Follower** and requires current operator workspace-clear readiness.
It is ordinary Leader tracking, not an independently commanded fixed hold.
It creates evidence beside the script and refuses existing result files; use
a new experiment directory for a new authorized capture. Do not run a second
CAN querying tool beside this lifecycle. Offline analysis above requires no
powered hardware.

### Motion-tolerant estimation review (2026-10-11)

The same-pose diagnostic envelope is a limitation of the constant tare, not a
physical requirement that a usable estimator keep the wrist within 0.115
degrees. Normal posture changes must be represented by the estimator. Merely
widening that envelope would reuse an unvalidated bias outside its evidence.

An offline review of the completed same-session load/unload/reload capture is
saved as `data/temporary/right_remote_paired_20261011/motion_diagnostic.py` and
`motion_diagnostic.json`. It reads local artifacts and evaluates the model at
each stage's mean posture. Config/model hashes match the original analysis.
Between the two loaded stages, J3 changes by -0.21844 degrees and its reported
motor torque increases by 0.91217 Nm. The current incomplete gravity model
predicts only -0.00481 Nm change. Nominal configured Kp times the change in
Leader/Follower tracking error is +0.91499 Nm. J2 and J6 also closely follow
their nominal proportional response. This supports investigating servo effort
and friction/history-dependent equilibrium; it does not identify the opaque
controller, prove friction, or verify the feedback's physical torque accuracy.
Do **not** subtract Kp times tracking error as a correction: real external load
also changes that error, so this could remove the desired signal.

Updating the Jacobian with the measured posture gives a scaled six-axis
condition number of about 16.19 in both loaded stages. The joint-space residual
norm nevertheless changes from 0.0264 to 0.4521 Nm. These mean-pose numbers
differ slightly from the prior averaged-sample calculation. Assuming zero TCP
moment and solving only three force components increases those residuals to
0.1222 and 0.6754 Nm; reducing dimension alone does not fix this capture, and
the remote attachment does not establish zero TCP moment. No fitted wrench is
released and no existing acceptance or physical safety gate is changed.

The next estimator design should proceed through these offline-verifiable steps:

1. Verify motor/model coordinates and torque scale/sign, then model the installed
   gripper/camera/bracket mass and COM. Recompute gravity and the Jacobian at
   every posture. Distinguish permanent tool gravity from the external load
   being measured; subtracting a test object's own weight would erase that test.
2. Identify a no-external-load residual model over multiple postures and both
   approach directions, including slow motion, reversals, and settled holds.
   Include velocity and motion history where supported, with measured uncertainty
   near stiction. A single stationary record cannot identify this model. Fit on
   explicitly unloaded data and freeze during load tests so adaptation cannot
   absorb sustained contact force. Keep held-out poses, reversals and sessions.
3. Start with quasi-static/slow-motion estimates inside the calibrated domain.
   Solve the current Jacobian with joint uncertainty weights and regularization,
   report directional uncertainty, and reject unexplained residuals. Weighting
   and damping limit noise amplification; neither removes unknown torque bias.
   A restricted force-only estimator additionally needs a justified contact point
   and moment model. Single-contact-at-TCP assumptions must be explicit.
4. For faster motion, add inertial/Coriolis compensation and a momentum observer
   after dynamics and velocity/timestamp quality are established. An observer
   also estimates unmodeled friction and dynamics as disturbances; it does not
   automatically separate those from external contact.

Validation should deliberately include small wrist changes under repeated,
measured loads, rather than require an immobile wrist. An initial experiment
can compare unloaded and weighed-load data at several safe bent-arm poses,
approaching each from both directions, using the same attachment and then
held-out poses. Gravity-only loading cannot validate arbitrary six-axis wrench
accuracy; different force directions and known moment arms are needed later.
No new hardware session or motion is started by this review.

Relevant primary references:
[DLR force sensitivity and joint hysteresis study](https://elib.dlr.de/105945/)
and [ICRA 2019 momentum observers](https://elib.dlr.de/129060/1/root.pdf).
Their assumptions and sensor architectures do not establish this arm's accuracy.

### Offline posture/history prototype (2026-10-11)

`vlai_l1_runtime.residual_model` now supplies a pure offline local residual
model. It is separate from `unloaded_baseline`: the existing tare policy,
physical configuration, SDK commands and live lifecycle are unchanged.
NumPy is imported only when fitting/predicting; importing the module does not
open hardware or require ROS. All fit policies are explicit caller inputs;
the synthetic test tolerances are not robot operating parameters.

For each joint, the prototype predicts the unloaded `g(q) - tau_motor`
residual using an intercept, all seven local joint positions, that joint's
velocity and its last resolved approach direction. This is a limited affine
approximation, not a complete friction or dynamics model. The corrected
candidate external torque is the measured `g(q) - tau_motor` minus this
predicted residual. It can then be passed to the existing current-posture
Jacobian solver. No coefficients have been fitted for the physical robot.

API sequence:

1. Decode and validate the capture through existing raw identity, timing,
   status, joint-bound and coordinate checks. Evaluate gravity at each posture.
   This module does not replace capture checks or evaluate motor safety.
2. Construct `ApproachHistory` with explicit per-joint excursion thresholds
   above the measured encoder noise and an explicit maximum timestamp gap.
   Feed ordered position samples. It retains direction while stationary,
   requires cumulative travel from an extremum to resolve reversals, clears
   direction to unknown after a gap, and latches malformed/regressed input.
   It does not infer motion direction from quantized velocity sign alone.
3. Construct immutable `ResidualSample` records with exact `ARM_JOINT_NAMES`,
   source timestamp, recording identity, q/dq, resolved direction and
   `model_minus_motor_nm`. The caller's `context_key` must identify the side,
   model, mapping, tool assembly, gripper opening and controller configuration.
   Explicit external-load labels are required; unknown is not unloaded.
4. Call `fit_residual_model(training=..., validation=..., policy=...)` with
   independent unloaded recordings. Training/validation recording IDs must not
   overlap. The fitter rejects insufficient posture coverage, rank-deficient or
   poorly conditioned features, unknown history, unsupported validation samples,
   excessive speed or excessive held-out residual. It does not refit on the
   validation data.
5. Call the frozen model's `predict(sample)` on loaded data with the same
   context. Out-of-range postures, unsupported local posture/direction/velocity
   combinations, unknown direction and excessive speed are rejected. Results
   remain `ft_validated=false`. Empirical held-out maximum error is reported
   explicitly as an observed error, not a guaranteed torque/force bound.

Prediction is an offline point evaluation, not an online freshness/session
guard. Cross-joint motion history, acceleration, temperature and general
stiction behavior remain unmodeled. Bounded speed alone does not establish
negligible acceleration. A real experiment still needs slow, smooth motion and
held-out validation; the model must not be used for force control or safety.

Synthetic tests inject a known six-axis wrench while posture and direction
change and verify recovery through the existing Jacobian solver. Separate tests
exercise direction memory at a stop, reversal noise rejection, timestamp faults,
data leakage, loaded-fit rejection, deficient excitation and domain rejection.
These establish software behavior under the synthetic model, not real accuracy.

The real-data coverage audit is saved in
`data/temporary/right_remote_paired_20261011/residual_coverage.py` and
`residual_coverage.json`. It validates and reads the Oct 9 unloaded capture,
the Oct 11 unloaded capture and the paired capture's operator-confirmed
unloaded stage. Their final four-second windows have posture-feature ranks
4/8, 4/8 and 5/8; all have several exactly constant joint-position channels.
The varying channels span only about one encoder step (0.02186 degrees), and
no resolved bidirectional approach history is established by these windows.
They cannot identify this model. Raw velocity variation in these stationary
records must not be interpreted as reliable small-motion excitation.

The next physical participation is removal of the remote/external load, followed
by confirmation of right-side power and clear alignment space. A subsequent
bounded acquisition must use the existing isolated-right controller owner and
its passive logger. After alignment, operator-controlled small Leader motions
can collect multiple nearby poses, approach directions, slow transitions and
settled holds; do not push the Follower as if that were unloaded training.
Keep a complete independent traversal out of training. Sampling all seven
position features and separating velocity/history effects requires more than
one wrist-only perturbation. No autonomous motion/zeroing/calibration path has
been added, and no new live test has run for this prototype.

### Guided unloaded follow-up (2026-10-11)

The operator-ended unloaded capture has 78 qualified settled windows but only
two pose clusters. In `right_unloaded_operator_stop_20261011/model_comparison.json`,
transferring the first pose's constant tare leaves a seven-joint residual norm
of 0.728 Nm at the second. Fitting an unrestricted TCP rigid payload (mass and
three first moments) after differencing out constant joint bias still leaves
0.346 Nm, including 0.339 Nm at J3. This is a hypothesis check under the current
mapping/gravity assumptions, not identification of the camera or proof of a
particular friction mechanism. Repeated windows at one initial hold are not
independent returns to that pose.

The next experiment first checks whether a small deliberate wrist loop produces
repeatable settled residuals. It does not require a weighed load yet:

1. Confirm the remote is removed, camera/bracket and gripper opening unchanged,
   both right arms powered and clear, and the operator available at the Leader.
   Use a new capture directory and the existing guarded right-only owner with
   its passive logger. Startup includes alignment. Left remains inactive.
2. After startup, inspect current named positions and margins against tracked
   bounds. Choose a comfortably bent, suspended posture. Do not ask the operator
   to reproduce the previous near-straight final pose or change any limits.
3. Guide one wrist joint at a time, beginning with J7: reference hold, small
   excursion in one direction, return and hold, excursion in the other
   direction, return and hold. A few degrees over several seconds with roughly
   five-second holds is the initial operator instruction, not a controller
   target or safety setting. The operator moves only the Leader, releases it
   for holds when safe, and never touches/supports the Follower. Record actual
   seven-joint positions and approach history; manual motion need not isolate
   one joint perfectly. Do not call unequal return poses identical.
4. On operator completion and hands clear, record the final hold and shut down
   through the existing owner. No elapsed-time cutoff is added. Check retained
   raw time coverage and explicit eviction counts: an evicted start/return
   invalidates that comparison even if capture health passed. Start a fresh
   session for further coverage rather than silently use missing history.
5. Offline, validate transport/status, settled-window qualification, both
   approach directions and repeated-pose coverage. Compare within-hold scatter,
   opposite-direction return offsets and posture changes separately. A held
   wrist loop may be diagnostic even with unresolved history on other joints;
   it must not bypass the full residual fitter's all-joint history checks.
6. If this first loop is usable, expand to other wrist, elbow and shoulder
   coordinates in separate guided blocks, then repeat a complete traversal in
   a separate acquisition reserved for validation. Check rank/support rather
   than assuming a specified number of poses guarantees identifiability.

Settled holds cannot identify a velocity coefficient. First assess static
posture/history compensation; fitting the existing ten-feature prototype also
requires qualified slow transitions and independent validation, or a separately
reviewed static-only model. Raw quantized speed noise is not velocity excitation.
Only after freezing an accepted unloaded model should a weighed load and small
wrist changes test signal preservation. Ultimately validation must include motion;
these holds separate error sources while building that model.

### Cross-recording payload hypothesis check (2026-10-11)

`right_unloaded_wrist_loop_20261011/cross_capture_comparison.py` verifies raw,
configuration and URDF hashes plus operator unloaded eligibility for the latest
two recordings. It fits five equally weighted pose means from the wrist-loop
recording, freezes coefficients, and compares against the earlier recording's
two pose means. Repeated windows do not receive independent-pose weight.

| Hypothesis | Five-pose fit error | Other-recording error |
| --- | ---: | ---: |
| Constant joint biases | 0.777 Nm | 0.557 Nm |
| Biases plus rigid TCP payload | 0.618 Nm | 0.819 Nm |

Each number is the RMS of the per-pose seven-joint torque error norm, not a
force error or per-joint accuracy. The unrestricted payload hypothesis has
11 independent columns including seven biases; the four payload coefficients
represent mass and first moments. Its coefficients are not accepted physical
parameters. Rigid TCP attachment is assumed, not evidence of the actual camera
mount link. The two comparison poses are outside the training position box,
with nearest maximum-joint differences 0.940 and 19.912 degrees, so this is
an exploratory cross-recording comparison including extrapolation. It cannot
validate supported-domain accuracy or motion performance.

The two initial bent-arm poses differ by at most 0.940 degrees per joint, while
their residual vectors differ by norm 0.279 Nm. This establishes a repeatability
question for nearby poses, not a measured same-pose hysteresis value. Posture,
approach direction, session effects, coordinate errors and friction remain
confounded. Neither simply widening the original tare envelope nor installing
the payload fit is justified by these results. No calibrated FT was accepted
and no new hardware operation was performed for this comparison.

The next useful experiment should record one return at a time: a settled
reference, an operator-controlled nearby excursion and return, then an approach
from the other direction. Check the actual seven-joint posture before labelling
a return; retain deviations rather than pretending manual returns are exact.
If coordinated joint movement prevents matching, record the deviation and
design a matched local posture comparison instead of requesting larger motions.

### Automatic experiment request and current interface gap (2026-10-11)

The operator has authorized automatic experiments instead of repeatedly moving
the Leader manually. Repeatable programmed motion is the intended acquisition
direction. No new motion command was sent during this interface audit.

The commissioned native owner accepts `ADJUST_POSITION` only; it has no joint
trajectory command. The pinned teleop C interface provides lifecycle, feedback
callbacks and home. Internal reference copying continually follows the Leader.
An external writer would not acquire exclusive control merely by publishing a
new position. In particular, the vendor's `xarm_trajectory_executor.py` explicitly
opens a second Leader CAN socket and sends MIT commands while teleoperation
continues. Its hardcoded right-Leader can1 also disagrees with tracked can0.
Changing that port does not resolve competing writers, command arbitration,
watchdog ownership or shutdown semantics. This script is not a deployable path.

`configs/diagnostics/right_wrist_sweep_preview.json` and the pure offline script
under `data/temporary/right_automatic_sweep_preview_20261011/` prepare a concrete
candidate: hold, J7 +2 degrees, reference, J7 -2 degrees, reference, repeating
twice. Every transition uses a four-second quintic smoothstep and each hold is
five seconds. All other joints and the gripper retain their archived reference.
The resulting 77-second preview checks exact finite named vectors, increasing
relative times, configured position bounds and unchanged inactive coordinates.
It is neither a live command stream nor a new motion adapter. Sampling at 100 Hz
is for the preview artifact, not a configured motor-control rate. Amplitude and
timing are candidate experiment inputs, not validated operating limits.

Before execution, the unresolved command readiness gates still need evidence:
reviewable controller source, coordinate reconciliation including J2, right
Follower bus stability under the required load conditions, and a reviewed
single-owner command lifecycle. The repository's current schema deliberately
rejects enabled command transport; a config flag change is not a solution.
The lack of complete controller source is still unresolved after the operator
reported having no other SDK version. Do not relabel a short clean telemetry
capture as closure of the source/coordinate/sustained-load gates.

Once those prerequisites have evidence, a separately reviewed implementation
must accept named, bounded targets inside the sole physical owner, arbitrate
teleoperation versus trajectory mode, begin from fresh measured position, and
enforce command/state timeouts and deterministic disable/release. Simulate bad
timestamps, stale feedback, malformed targets, process loss and control-mode
switches before commissioning. A first physical trial should be one bounded
excursion with observed stop behavior, then the repeatable acquisition sequence.
No automatic retry after a hardware fault and no continuous unattended motion
are inferred from the request to reduce manual participation. The current
preview does not validate workspace clearance, effort limits or stop response.

### Reusable offline automation core (2026-10-11)

`vlai_l1_runtime.trajectory` now provides the reusable portion of the automatic
test sequence without a live adapter:

- `SweepSpec`: all amplitude, transition/hold duration, cycle count and analytic
  speed/acceleration budgets are explicit. Candidate values are in the tracked
  diagnostic preview JSON, not hidden controller defaults.
- `WristSweep`: snapshots a loader-validated configuration and exact named
  eight-position vector. It checks the full monotonic quintic segments against
  tracked arm bounds, keeps the gripper and other joints fixed, and evaluates
  positions, velocities and accelerations at explicit relative nanoseconds.
  Returned mappings and the plan are immutable. No interpolation extrapolation.
- `TrajectoryRehearsal`: consumes explicitly simulated health and named feedback.
  The first reference must match fresh feedback. Wrong identity, stale/future/
  regressed timestamps, bounds, transport faults and sustained per-feature
  tracking error latch release with no further output targets. Source-time
  comparison avoids interpreting a delayed position as the current pose.
  A separate polling call models watchdog expiry without renewing feedback.

Completion and stop enter `RELEASE_REQUIRED`, not a claimed disabled state.
Both disable and resource-release acknowledgements must be exactly true to
reach `CLOSED`; failed release is terminal. These are simulated lifecycle facts,
not motor operations. Normal following-error warnings in the commissioned
teleoperation path remain unchanged. The stricter experimental rehearsal
tracking thresholds do not silently change its configured behavior.

The 38 new tests include all 7,701 samples of the candidate 77-second sequence,
smooth boundaries and derivative limits, immutable references, unchanged
production readiness, future/stale/regressed feedback, wrong identity, gripper
deviation, independent watchdog expiry, explicit stop and failed release.
The full synthetic scenario report is
`data/temporary/right_trajectory_rehearsal_20261011/report.json`. Feedback is
ideal position tracking or an injected fault; no torque, actuator dynamics,
collision, physical stop response or realtime scheduling is validated.

The native control socket has not gained a trajectory command. Upstream main
is still pinned at `bf300508e179f652b23f0efaf3b6c9048f1f12e9`, and local source
searches did not find the internal controller/dynamics implementations.
Vendor CMake imports the binary instead of compiling its visible SDK wrapper
without those internal sources. Live integration remains unfinished pending a
reviewable single-owner controller and evidence for the tracked readiness gates.

### Physical coordinate and load validation

Front/side photos have been reviewed and the operator confirmed suspended
Follower endpoints and the CAN cable identities. These observations do not
calibrate joint coordinates. Record camera/bracket/other tool mass and COM
evidence when available; do not infer the whole assembly's properties from a
camera product weight.

Only after that review should a supported, one-joint-at-a-time coordinate check
be agreed with the operator. No offset/limit change is justified merely to
make the existing sample pass. A later known-load experiment needs a suitable
bent-arm pose, verified coordinate/torque signs and telemetry inside the active
controller owner. The standalone querying diagnostic still rejects active
controllers and cannot be run beside teleoperation. Use only the optional
same-owner passive raw logger for an active diagnostic capture. A weighed small
mass/line or calibrated handheld force gauge, a reviewed attachment/direction,
and an agreed bounded experiment are needed to validate force changes. Live
VL1T ingress/status commissioning and calibration remain separate prerequisites
to a validated online FT stream; no force-control path exists.
