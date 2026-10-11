# VLAI L1 workspace handoff

This is the starting context for the next agent working on the onboard VLAI L1.
It records intent, verified evidence, unresolved gates, and the staged test
sequence. Tracked configuration and executable code remain authoritative. When
this snapshot disagrees with them, stop and resolve the drift instead of
silently choosing one.

## Start here

**Latest implementation and live commissioning (2026-10-11):** The operator's
latest direction explicitly authorizes implementing and running the complete
bounded automatic diagnostic, waives the earlier complete-controller-source
prerequisite for this experimental path, and authorizes editing AGENTS.md.
The scoped exception and SAFETY.md describe `automatic-right`, its native
single motor owner, independent supervisor, strict feedback guards and verified
disable/down cleanup. Production policy readiness remains false. The earlier
offline-only prohibition below is historical and superseded for this path.
The complete single-command `--mode sequence` now performs preflight, hold,
verified disable/down, offline artifact review, then preflight/sweep/disable/
review. A failed gate ends the sequence without retry. The native owner uses
current measured raw motor positions, tracked PD gains and zero feedforward;
it never homes/lifts/zeros or moves the Leader. Controller context differs
from the previous opaque teleoperation records. No cameras were opened.

**LATEST HARDWARE STOPPED: 2026-10-11 08:54:47 UTC.** All configured CAN links
are DOWN/STOPPED, exact USB parents, zero live/cumulative errors and interface
drops; can2 qdisc drops zero. No controller remains. Final TX/RX: can0
1555912/1555855, can1 0/0, can2 1670504/1670504, can3 3904/3904. Only can2
changed during these tests; its initial TX/RX was 1560600/1560600. No adapter
reset, calibration, bound relaxation, or automatic fault retry occurred.
User authorized autonomous tests with the previously cleared right workspace.
Host-level AGENTS files are absent. Tests ran on repository base
438bb696935a1f379cc0ffc1aafbc469a986a44c with the diagnostic changes in the
working tree; pinned SDK remains bf300508e179f652b23f0efaf3b6c9048f1f12e9.
The diagnostic source, configuration, tests and documentation are versioned
together. Raw recordings and generated builds remain local in ignored paths.

Evidence under `data/temporary/`:

- `right_automatic_hold_20261011`, 08:45:19–08:45:27 UTC: 40/40 motor-probe
  rounds, five-second hold, 500 samples, 4,048 raw frames; maximum position
  tracking error 0.02186 degrees. Passed, all eight motors confirmed disabled.
- `right_automatic_sweep_20261011`, 08:45:45–08:46:29 UTC: 40/40 preflight,
  +/-0.5-degree J7 reference sweep, 4,100 samples / 32,848 raw frames, actual
  J7 span 0.67757 degrees and maximum error 0.48508 degrees. Passed and disabled.
- `right_automatic_sequence_20261011`, 08:51:30–08:52:58 UTC: one command
  automatically passed hold/review, then two +/-1-degree cycles/review. Sweep
  has 7,700 samples / 61,648 raw frames, J7 span 1.77041 degrees, maximum error
  0.69191 degrees (tracked limit remains 0.8). Non-J7 position spans at most
  0.02186 degrees, unchanged non-J7 references. Both native threads observed
  FIFO 20; native startup verifies all threads. Maximum raw motor gap 17.255 ms.
  `sequence.json`, per-stage `analysis.json`, and `sweep/repeat_review.json`
  contain integrity, tracking, hold and repeated-approach results. Sweep raw
  SHA256: 0cbd466f7b5a49b453a586896110089da24515c194b921cb5fa161de0e5bc8fd.
- `right_automatic_stopcheck_20261011`, 08:54:37–08:54:47 UTC: scripted
  SIGTERM to the supervisor after seven seconds of the smaller sweep. Verified
  disabled feedback after 29.980 ms, then link DOWN. `stop_verification.json`
  passes; the intentionally interrupted capture truthfully remains failed and
  must not be used as a complete baseline. Adjacent `.py` reproduces the test.

Initial successful sessions used native SHA256
68ec8920438337a808e8a28ae9907e504a4f2ba60474d2a0e2bfd109bdc8d9c4.
The final build d2209c8802c179b588ad792e4fe14cf3bfed800fc0f48b30febff7c6690fb8ce
changes only the disabled event's position field to the freshly measured stop
position (earlier disabled events printed the initial reference). Its stop path
passed the live interrupted test. Actual trajectory samples were unaffected.

Same-approach J7 hold means repeat within 0.02038 degrees and 0.00490 Nm across
the two cycles. However, the return after the negative excursion differs from
the initial hold by about -0.56 degrees and +0.24 Nm. This is direction/history
dependent data with differing actual postures, not proof that friction alone
causes the effect or an accepted force calibration. Load presence is recorded
as unknown. Next work can use this bounded owner and saved repeated holds to
develop a direction-aware baseline; calibrated FT still needs physical model
and known-load validation. Do not pool the direct-PD data with old controller
baselines or treat a passed acquisition report as FT acceptance.

Hardware-free verification: 550 pytest tests, four native CTests, Ruff check/
format and `git diff --check` pass. Tests include pure guarded trajectories,
failed preflight, sequential disable-only cleanup, interrupted/failed release
reporting, saved-data corruption, and stopping the sequence at a failed gate.
See README for the launch command. No user action is currently pending.

**Previous pure implementation (2026-10-11, hardware-free):** The operator explicitly
asked to build the missing automation rather than stop at a preview. New
`src/vlai_l1_runtime/trajectory.py` contains immutable `SweepSpec` / `WristSweep`
and `TrajectoryRehearsal`. It is pure mathematics and a fault-state model,
not a live command adapter. The existing native owner, CAN send path,
CommandSession readiness, system schema and teleoperation warning policy were
not changed. The robot has not been enabled by this work.

The plan uses exact side-specific eight-feature degree vectors, immutable
reference snapshots, existing tracked arm bounds, analytically checked quintic
speed/acceleration budgets, fixed gripper/other joints, and explicit relative
times. Rehearsal rejects old/future/regressed/wrong-side feedback, mismatched
first hold, bounds and transport faults; per-feature persistent tracking errors
request release with no more targets. Tracking compares against feedback source
time. Polling models an independent watchdog but cannot refresh feedback.
Completion and operator stop also require a separate simulated disable/release
acknowledgement; failed release is terminal and does not auto-restart. No actual
disable, process supervisor, transport, exclusive hardware mode or actuator
dynamics is implemented by this pure model. Do not connect its output to CAN.

520 hardware-free pytest tests pass overall, with Ruff check/format and
`git diff --check` passing. The 38 new targeted tests include a full 7,701-point sweep and
fault injection, unchanged production readiness, immutable inputs, strict names,
Python 3.10 import/configuration/endpoint validation. Reproducible six-scenario
report: `data/temporary/right_trajectory_rehearsal_20261011/report.json`, with
adjacent script. Candidate speed/acceleration/tracking rehearsal inputs are
explicit in `configs/diagnostics/right_wrist_sweep_preview.json`; they are not
commissioned physical limits. This updates that policy's hash; older preview
artifacts remain historical, not regenerated or overwritten.

Source investigation also checked local openarm/install/build/Downloads/Documents/
twork paths and upstream branch refs. No internal Control/Dynamics/JointMapper
sources found. `git ls-remote --heads origin` still reports only main at
bf300508e179f652b23f0efaf3b6c9048f1f12e9. Vendor CMake explicitly imports a prebuilt
library when `src/controller/control.cpp` is absent. The supplied wrapper calls
missing internal headers, so editing its reference-copy function is not a
rebuildable arbitrary-trajectory integration. Low-level MIT C functions exist,
but using them beside teleoperation would compete for motor ownership. Remaining
work requires a reviewable owner/controller implementation and coordinate/bus/
stop evidence before a live adapter. No new SDK/source availability or readiness
gate has been inferred from authorization or synthetic tests.

**Latest operator direction (2026-10-11):** The operator explicitly authorized
automatic motion experiments to avoid repeated manual Leader manipulation.
This supersedes the pending request to start another manual return test; no
new hardware session has started. Passive host inspection after the interrupted
turn found all four configured endpoints DOWN/STOPPED, exact USB parents,
zero live CAN error counters and no competing controller. Authorization is
recorded; do not repeatedly ask whether software-controlled experiments are
desired. Existing readiness gates remain unresolved, not implicitly cleared.

Source audit: the native control socket accepts only `ADJUST_POSITION`; the
pinned teleop C ABI has callbacks/lifecycle/home but no arbitrary named joint
trajectory input. Vendor `publish/lerobot_collector/xarm_trajectory_executor.py`
opens a second Leader CAN socket while teleoperation still publishes MIT
commands and hardcodes right Leader as can1 (this machine is can0). Do not run
that script, patch just its port, or inject raw commands beside the owner.
Tracked blockers are command_transport_unimplemented, production_source_unavailable,
j2_coordinate_unverified and follower_right_bus_stability_unverified.
AGENTS.md prohibits adding live command/enable/calibration/reset adapters before
the tracked readiness gates are resolved. No readiness flags or limits changed.

An offline-only candidate is now in
`data/temporary/right_automatic_sweep_preview_20261011/preview.py`, `preview.json`
and `preview_points.jsonl`, with explicit preview inputs in
`configs/diagnostics/right_wrist_sweep_preview.json`. It uses archived startup
Follower positions only, varies J7 by +/-2 degrees with four-second quintic
transitions, five-second holds and two cycles (77 seconds, 7,701 preview points).
Analytic peak speed is 0.9375 deg/s and acceleration 0.72169 deg/s^2; these are
candidate trajectory properties, not commissioned hardware limits. Exact named
eight-position vectors, unchanged other joints/gripper, increasing relative
times and configured joint bounds are checked. Quintic monotonicity also covers
continuous position bounds between samples. Output explicitly says not
executable/live-ready; archived positions cannot become the first live command.
No physical collision, torque or stop-response validation is claimed. Follow
the automatic-experiment section of FORCE_ESTIMATION.md for the remaining work.

**Previous manual session stopped (2026-10-11 07:20:39 UTC):**
`right_unloaded_wrist_loop_20261011` completed after operator confirmation of
hands clear, Leader-only motion and no abnormality. Exec 19083 exited with
status zero; report passed/completed, 10,304 position samples, no raw/position
eviction. All configured CAN links verified DOWN/STOPPED, exact USB parents,
zero live/cumulative errors and interface/qdisc drops, no controller process.
Final TX/RX: can0 1555912/1555855, can2 1560600/1560600, can1 0/0,
can3 3904/3904. Completion and operator confirmation markers are present; do
not write another or restart this directory. Offline integrity/settled analysis
is complete in `analysis.json` and `settled_review_v2.json` with adjacent
reproducible scripts. Raw SHA256:
`28a348621e245933b19a9eb52d7e4b8b0cc6a25086f1db079215be175d47c6ea`.
171,717 raw frames form 21,464 complete batches plus five trailing motor frames
(reported partial tail, not a completed eight-motor sample). Maximum motor gap
6.34378 ms and queue age 368.586 us; all raw status nibbles 1. There were 16
following-error warnings, with operator-confirmed no external contact or
abnormal force; hard bounds and transport health passed. Leader J2 reached
-8.776 degrees (configured minimum -10); the operator was told to stop moving
when it was observed near that boundary, then independently reported completion.

The intended small wrist loop actually moved multiple joints: Follower spans
approximately [9.70,25.24,14.21,11.74,30.38,33.97,30.38] degrees. Peak raw wrist
speed reached 57.51 deg/s. Keep this as unloaded multi-pose evidence, not a
successful isolated wrist/reversal experiment. Existing stationary criteria
selected 25 half-second windows in five clusters, counts [16,1,2,1,5], scaled
Jacobian condition 16.13–25.76. Initial hold dominates cluster 0; there is no
opposite-direction return to the initial pose. Motion feature ranks are
[10,10,9,9,10,10,10], with 8,169 samples having known all-joint direction.
No real residual fit or calibrated FT accepted. Cross-recording comparison is
now complete in `right_unloaded_wrist_loop_20261011/cross_capture_comparison.json`
and its adjacent script. Five equally weighted pose means from this recording
fit (a) seven constant joint biases and (b) those biases plus four unrestricted
TCP payload parameters. Freeze each fit and evaluate the earlier recording's
two pose means. RMS of seven-joint error norms, in Nm:
constant bias train 0.7769 / other recording 0.5568;
bias plus TCP payload train 0.6180 / other recording 0.8194.
The second fit has rank 11/11; lower training error does not establish accuracy.
Both comparison poses are outside the training position box; their nearest
maximum-joint differences are 0.9397 and 19.9124 degrees. This is a hypothesis
comparison including extrapolation, not supported-domain validation.
Initial-pose residual vectors differ by norm 0.2785 Nm across recordings, with
maximum joint-position difference 0.9397 degrees. These are nearby rather than
identical poses, so posture, history and session effects cannot be separated.
The fitted payload is hypothetical and assumes rigid attachment to the TCP;
attachment link, coordinates, gravity and torque mapping remain unverified.
Do not apply its fitted mass/COM or biases to runtime. This result does not
rule out other physical models or identify friction as the sole cause.

Next useful physical data is a repeated-pose, opposite-approach comparison:
issue one movement/hold at a time and inspect actual posture before continuing.
Do not require perfectly isolated manual joint movement or claim that five
poses identify the full seven-position/velocity/history model. Hardware remains
stopped; no further action currently required from the operator.

Historical launch context for this now-stopped session: the operator confirmed ready for the
guided unloaded wrist loop. `data/temporary/right_unloaded_wrist_loop_20261011/`
ran through unified exec **19083**, using the previously tested
operator-ended capture (formatting only, no control changes). No elapsed-time
cutoff. Initial stationary stage completed and `manual_unloaded_motion` began
at host monotonic 228505819087197 ns. The operator has been instructed through
an async question to move only the right Leader's distal wrist joint a few
degrees slowly in each direction, with five-second holds at excursions and
returns, repeating once, keeping shoulder/elbow/gripper unchanged as practical.
Follower must remain untouched. On completion/hands clear, inspect fresh
`status.json`, then create this directory's unique `motion_done` marker with
the reply and monotonic time; the owner records five seconds then shuts down.
For abnormality/stop use `stop_requested`, not the completion marker. Verify
actual process exit, report and all link/counter/qdisc states before claiming
disable. No automatic restart. The old capture below is already stopped.

Precheck and running_check artifacts record exact USB parents and zero
live/cumulative CAN errors and interface/qdisc drops, right ERROR-ACTIVE and
left STOPPED. At the running check, Follower q was approximately
[-0.67,0.12,-0.16,34.59,-0.43,-0.47,-1.93] degrees; Leader was
[-0.03,0.14,-0.27,35.22,-0.45,-0.34,-0.86]. All tracked bounds passed.
482 hardware-free pytest tests, Ruff check/format and diff checks passed before
startup. Raw/report bulk output is now finalized after owner shutdown.

**SESSION STOPPED after operator completion (2026-10-11):**
`right_unloaded_operator_stop_20261011` ended normally at 06:56:45 UTC after
the requested final five-second hold. All configured CAN links are DOWN /
STOPPED with exact USB parents, zero live/cumulative CAN errors and zero
interface/qdisc drops; no controller process remains. Exec session 19849 has
exited. Final can0 TX/RX 1381808/1381757, can2 1386480/1386480, can1 0/0,
can3 3904/3904. Do not write another completion marker or restart this capture.
The operator confirmed only Leader motion, Follower untouched and no collision
or abnormal force. This resolves the previous endpoint misunderstanding.

Capture is `passed`, experiment `completed`: 27,951 native position packets,
465,776 raw frames, 58,222 complete batches, no partial tail or evicted data.
Maximum motor gap 5.7634 ms, kernel queue age 633.423 us, status nibbles all 1.
Follower joint spans were approximately [66.86,27.96,43.45,32.48,68.81,16.41,92.15]
degrees; Leader spans were comparable. The 54 following-error warnings are
retained, with no configured hard-bound fault. All seven feature matrices have
rank 10, but only 5,799 manual-phase batches have resolved history on every
joint. Rank does not validate inertia/friction compensation. No calibrated
force output or physical residual model has been accepted. Detailed evidence
is in analysis.json, postcheck.json and operator_confirmation.json alongside
the capture. Offline settled review is now complete in `settled_review_v2.json`
(the first `settled_review.json` write was incomplete; do not use it).
Applying the existing 0.5-second static-window speed/span criteria yielded
78 selected windows in only two pose clusters (0.5-degree reporting radius):
the initial bent pose and the final near-straight pose. Scaled Jacobian condition
numbers range 16.16–91.83. Peak raw joint speed reaches 245.55 deg/s at J5;
the moving data cannot be treated as quasi-static without inertial modeling.
The data is useful for motion-model investigation, but two settled poses and
no independent traversal cannot validate the proposed full local residual fit.
Offline model/trajectory comparison is complete in `model_comparison.json`
and its adjacent reproducible script. Between the two settled clusters, the
seven-joint residual changes by [0.356,-0.298,-0.309,0.455,-0.045,-0.071,-0.069]
Nm (Euclidean norm 0.728 Nm). An unconstrained four-parameter TCP rigid payload
plus constant joint bias hypothesis leaves 0.346 Nm, predominantly J3
(0.339 Nm), under the current coordinate/gravity assumptions. Its rank is four;
the coefficients are diagnostic, not a measured camera mass or calibration.
This does not isolate friction from coordinate/model/controller errors.
Position differences over 0.12 seconds at 50 Hz describe only 15.50 seconds
above 1 deg/s and 0.42 seconds in the 1–5 deg/s band (longest run 0.14 seconds).
These bands are analysis settings, not safety limits or evidence of negligible
acceleration. Most of the 279.5-second capture was waiting at the first pose.

Next physical stage is a guided unloaded posture/approach-direction experiment,
specified in `docs/FORCE_ESTIMATION.md` under "Guided unloaded follow-up".
Begin with a small wrist loop and inspect its recorded holds before asking for
broader joint coverage. Separate acquisitions supply independent repeats; do
not relabel randomly split windows as independent recordings. Keep the user's
no-time-cutoff preference and inspect ring eviction before using any stage.
The latest work was offline only; no new controller or hardware test ran.

Historical launch context for this now-stopped session: the operator explicitly
requested restart with no elapsed-time cutoff, to remain on until they report
completion. `data/temporary/right_unloaded_operator_stop_20261011/capture.py`
ran via unified exec session 19849. Status/console are in the same
directory. It passed the existing guarded right-only startup and reported fresh
positions, can0/can2 ERROR-ACTIVE, zero counters. Left CAN links remain down.
Ordinary joint-space Leader tracking was used, not an independent hold mode.

The operator was told to move only the right Leader, leave the Follower
untouched and keep gripper opening unchanged. When they report completion and
hands clear, first inspect `status.json`. If still `manual_unloaded_motion`,
create the unique `motion_done` marker with their reply and host monotonic time;
the capture records five seconds then releases the existing runtime normally.
For an immediate operator stop/fault report, create `stop_requested` instead.
Then wait for process exit and verify all owned CAN links down, counters/qdisc
drops and absence of controller processes. Never claim shutdown merely from a
marker. No report/raw bulk data is finalized until shutdown; `status.json`
is the current observation source. Do not mistake empty open files for missing
telemetry. Faults still stop the existing lifecycle; do not auto-restart them.

To support the explicit no-time-limit request without unbounded memory/disk
growth, opt-in `feedback_retention="latest"` retains the last 600,000 raw frames
(approximately six minutes at the observed rate); the operator capture retains
the last 40,000 position packets in memory. Both are written after shutdown.
Native logging exports `# retained_latest_evicted=N`; the report declares
matching retention/eviction counts. Offline parsing verifies that metadata and
all retained raw integrity checks. Discarded old history is explicitly counted;
socket overflow, time regressions, missing motors, CAN errors and hard bounds
still fail. Default full-retention behavior is unchanged. No SDK enable/reset
or torque path was added. Native circular-buffer tests cover multiple wraps,
chronological export, timestamp regression, socket loss and stale motor faults.
482 hardware-free pytest tests and three native CTests, Ruff and diff checks
passed; the sidecar was rebuilt before this session. The real wrap has not yet
been observed at the startup handoff.

The prior `_03` attempt (06:42:54–06:46:03 UTC) ended at its 180-second limit:
18,001 position packets, 299,985 raw frames, 37,498 complete batches, no meaningful
manual Leader excursion, and `experiment_status=incomplete`. All links then
stopped with zero errors/drops; can0 TX/RX 913672/913633, can2 918304/918304.
The operator subsequently asked to remove the duration cutoff; the operator-ended
session above implemented that request.

Latest live stage (2026-10-11, 06:32:40–06:35:49 UTC): after the operator
confirmed the remote removed, right power on and workspace clear, the existing
guarded isolated-right owner ran a bounded unloaded manual-motion observation.
Evidence: `data/temporary/right_unloaded_motion_20261011_02/`. The first attempt
in `right_unloaded_motion_20261011/` failed binding the root-owned state socket
before any CAN startup/SDK construction; all link counters stayed unchanged.
The second used the established sudo invocation, not an adapter reset/retry.

The second capture passed transport/cleanup with 18,001 position packets,
299,959 raw frames, 37,494 complete batches and seven trailing partial frames.
Maximum motor gap was 5.291 ms, maximum kernel queue age 361.865 us, and all
observed raw status nibbles were 1. Runtime hard bounds were respected, but
J6/J7 following-error warnings occurred (configured action remains warn).
The operator's completion marker arrived after the 180-second limit, so the
script had already stopped; the requested final five-second labeled hold was
not captured and `experiment_status=incomplete`. No subsequent restart ran.

**Do not use this capture to train unloaded compensation.** Leader joint spans
were only 0–0.066 degrees, while Follower spans were approximately
[5.29, 5.27, 1.57, 2.89, 3.21, 10.49, 11.78] degrees. Follower raw data and native
positions agree. The operator subsequently confirmed directly pushing the
Follower (not the Leader). `training_eligibility.json` records this confirmation
and supersedes the capture's initial no-external-load declaration; this run
contains unknown human external forces and is rejected for unloaded-fit use.
Local feature ranks being 10/10 does not establish unloaded identifiability
when human external torque may be present. Do not infer zero applied force from
the remote being removed. No physical residual model was fitted.

Postcheck: all configured adapters retained exact USB parents, all links DOWN /
STOPPED, no controller remained, zero live/cumulative CAN errors and zero
interface/qdisc drops. Final can0 TX/RX 611328/611310; can2 TX/RX 615912/615912;
can1 0/0; can3 3904/3904. Repository/SDK revisions remain
438bb696935a1f379cc0ffc1aafbc469a986a44c and
bf300508e179f652b23f0efaf3b6c9048f1f12e9 respectively. Hardware-free verification:
476 pytest tests, Ruff check/format and `git diff --check` passed. Next safe
stage is a fresh guarded preflight followed by an unloaded trial with manual
Leader motion and no contact with the Follower. The endpoint misunderstanding
is resolved; no physical mapping or calibration change is justified.

Latest implementation (2026-10-11): `residual_model.py` adds a hardware-free
local affine unloaded residual prototype with posture, own-joint velocity and
last approach direction. `ApproachHistory` preserves signs through stops,
resolves reversals above explicit excursion thresholds, clears on gaps and
latches timestamp/input faults. Fits require independent unloaded recordings,
excitation/rank/conditioning and held-out error checks. Frozen predictions
reject unsupported posture/history/speed/context; all results remain
`ft_validated=false`. Synthetic tests verify retention of an injected six-axis
wrench across pose changes/reversals; no physical model has been fitted.
The real-data audit `right_remote_paired_20261011/residual_coverage.json` finds
posture ranks 4/8, 4/8, 5/8 in three existing unloaded windows, with unresolved
approach histories. New unloaded multi-pose bidirectional data and independent
validation are required. No motors/CAN were opened during this work. Next
operator action: remove the external remote/load and confirm the right pair's
power/clear workspace, then arrange a bounded existing-lifecycle acquisition
with manual Leader motion (not pushing the Follower). See FORCE_ESTIMATION.md
for prototype assumptions and the capture/validation sequence.
Verification: 476 hardware-free pytest tests passed (27 new prototype tests),
Ruff check/format and `git diff --check` passed. The operator has been asked to
remove the remote and confirm current right-side power/clearance; no readiness
reply or new hardware startup is recorded by this entry.

Latest FT follow-up (2026-10-11): the operator correctly challenged requiring
an immobile wrist for practical estimation. Offline-only mean-pose analysis in
`data/temporary/right_remote_paired_20261011/motion_diagnostic.json` finds J3's
repeat-loaded torque change (0.91217 Nm) closely matches nominal proportional
tracking-error response (0.91499 Nm), while modeled gravity changes -0.00481 Nm.
This is a diagnostic correlation, not an identified friction/torque calibration.
Current-posture Jacobians remain similarly conditioned (~16.19); repeat-loaded
six-axis residual remains 0.4521 Nm. Force-only/zero-TCP-moment fitting does not
resolve it. Do not subtract servo P effort or loosen the same-pose tare gate.
See `docs/FORCE_ESTIMATION.md` for the proposed posture/history-aware unloaded
model, held-out validation, and eventual dynamic observer. No hardware was
started in this follow-up. The next software stage is a calibrated-domain
offline estimator; new multi-pose unloaded and measured-load evidence is needed
before claiming it works. Future validation should include wrist perturbations.

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
| pinned `external/embodied-ops` | Generic episode, task registry, transaction, timing, artifact, camera-health, Operator Panel and LeRobot format mechanics | L1 CAN, joints, cameras, task/provenance policy or readiness decisions |
| pinned `external/x-air-sdk` | Reviewed x_air public ABI and opaque controller dependency | Runtime orchestration or dataset behavior |
| Future LeRobot plugins (not yet shipped) | Thin Robot/Teleoperator clients of Runtime contracts | Direct ownership of CAN or cameras |

The canonical dataset is written directly as LeRobot v3.0. A v2.1 dataset is
an independently generated derivative of that canonical source. There is no
raw intermediate format and no derivative may be the source of another final
derivative.

## Physical identity

The verified CAN mapping is:

| Side | Role | Interface | USB parent |
| --- | --- | --- | --- |
| right | leader | `can0` | `1-3.1.4:1.0` |
| left | leader | `can1` | `1-3.1.1:1.0` |
| right | follower | `can2` | `1-3.1.2:1.0` |
| left | follower | `can3` | `1-3.1.3:1.0` |

The table includes the 2026-09-17 upstream-port move; the operator confirmed
that adapter positions within the CAN hub and all arm connections were unchanged.

On 2026-09-16, the four PCAN-USB FD adapters were recommissioned on the
`nyush-jetson-dog` host through the external four-port hub. Motion-free motor
feedback and operator-observed manual joint changes established the mapping
above. The Jetson onboard `mttcan` interface is named `canboard0`; the enabled
`xarm-can-rename.service` reserves `can0` through `can3` for the four configured
PEAK adapters and leaves every interface down. All four endpoints completed a
40-round, eight-motor motion-free response check with zero live CAN errors.
No motor enable, alignment, or teleoperation launch was performed during this
host setup.

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

### 2026-10-11 operator-staged same-session remote load/unload/reload

Operator authorized proceeding and stopping for manual steps. Prepared a
bounded 180-second observation script around the existing isolated-right
ManagedXAirRuntimes, with same-owner passive logging capacity 600000 frames.
No runtime/controller/SDK code changed in this stage. File markers label
operator completion only; they do not command motors. Announced alignment,
continued tracking while waiting, and automatic disable after the last 10-second
observation (or the time budget). Existing secured R3-1 payload remained on the
right Follower at startup. User was instructed to keep Leader untouched.

The one session ran **05:57:19–05:59:38 UTC**, including startup/shutdown, with
130.48 seconds of active observation and **13048 position packets**. Operator
explicitly confirmed removing the remote with hands clear, then replacing it
in the same position/orientation/fixation with hands clear. Each confirmation
began a fresh 10-second observation. No Reset or process restart occurred
between stages. All three stages completed before the time limit. Artifacts:
`data/temporary/right_remote_paired_20261011/` includes `capture.py`, console,
raw feedback, native positions with stage labels, stage events in `report.json`,
operator confirmation markers, final `status.json`, `postcheck.json`,
`analyze.py`, `analysis.json`, `static_results.jsonl`, `wrench_decisions.jsonl`
and `consistency.json`. Capture metadata marks load as mixed operator stages;
never pass the whole capture through the unloaded-only analyzer.

Raw capture passed: **217442 frames**, **27180 complete batches**, two trailing
partial frames, zero duplicates/overwrites/socket overflow. Maximum per-motor
gap 5.270 ms, receive skew 4.562 ms, kernel queue age 475.246 microseconds; all
raw statuses observed 1. Both native role vectors and all decoded raw Follower
positions stayed in configured bounds. The existing guarded lifecycle reported
ready, running and successful shutdown without a CAN/scheduling fault. HEAD
`438bb696935a1f379cc0ffc1aafbc469a986a44c` plus current working changes and SDK
`bf300508e179f652b23f0efaf3b6c9048f1f12e9` remain unchanged. Raw SHA256:
`0488cf5376780e4dc6a874cdd8e5c8146786a3c047309a770fb057b1d4f08616`.

Analysis used the last four seconds of each labeled observation, excluding
operator handling and settling. Window sample counts 834/833/833; all eight
coordinates passed whole-window stationary span/velocity checks. Whole-capture
identity, timing and bounds checks cover every raw frame/batch. Model evaluation
uses one in four batches (~52 Hz), explicitly recorded; each stage contains
182 stationary eligible model samples after warmup. Baseline is fitted only to
the unloaded stage of this same session. No prior-session tare was imported.

J1 mean feedback torque follows **3.0265 -> 2.4456 -> 3.0137 Nm**, giving
loaded-minus-unloaded changes +0.5809 and +0.5681 Nm. J7 changes +0.0889 and
+0.0800 Nm. This supports repeatable load sensitivity on those joints only.
J4 remains about 3.0264 -> 3.0045 -> 3.0056 Nm; the cross-session J4 difference
reported in the earlier entry must not be interpreted as the true load torque.
J2/J3/J6 differ materially between first and second loading (J3 +0.9122 Nm),
despite near-constant Leader means. Hand/gripper angle remains essentially the
same, but Follower posture changes: initial load vs unloaded exceeds the
0.002-rad envelope on J1/J6/J7; repeat load exceeds it on J1/J3/J6/J7. Largest
mean change is J6 ~0.590 degrees. **All 364 delta-wrench decisions are rejected**,
with null wrench and explicit pose reasons; `ft_validated=false` throughout.

A separate diagnostic least-squares projection onto the seven-joint Jacobian
transpose subspace, with no wrench/mass accepted, gives joint inconsistency
norms 0.0265 Nm for initial loading and **0.4517 Nm** for repeat loading, versus
the existing 0.1 Nm residual threshold. This is additional evidence against
merely widening the pose envelope. Friction/history effects, changed contact
or force application, and unverified model/sign behavior remain possible; the
capture does not uniquely identify the cause. No limits or thresholds were
relaxed to produce a force number. R3-family 300 g remains nominal reference,
not actual measured remote-plus-fixation mass or a validated FT calibration.

Shutdown passed; no controller remains; configured can0..can3 all DOWN/STOPPED,
USB identity preserved, zero live/cumulative CAN errors and interface/qdisc
drops. Totals: can0 TX/RX 308992/308981 (delta 219840/219834), can2
313552/313552 (219848/219848); left unchanged (can1 zero, can3 3904/3904).
Aggregate Leader differences are not proof of which motor/packet was missed.
Hardware-free checks before launch: 449 pytest tests, Ruff and diff check;
the temporary capture script separately passed lint/format and syntax checking.

Operator need not act further now. Next meaningful experiment needs a defined,
repeatable force application point that can be unloaded/reloaded without pushing
or twisting the wrist, plus investigation of joint-torque hysteresis and model
mapping. Do not repeatedly restart or loosen pose/residual gates. Current
evidence demonstrates torque response but does not establish reliable six-axis
force or weight measurement. Gripper-button work remains cancelled.

### 2026-10-11 right-arm remote payload observation

Operator cancelled the proposed gripper-button feature and requested lifting
the right arm with a Unitree G1 remote already attached. Before motion the
operator confirmed it was secured on the right Follower. Announced a bounded
10-second observation with existing right Leader/Follower startup alignment
and automatic disable/down, keeping hands clear. Ran the same guarded
ManagedXAirRuntimes lifecycle and same-owner passive logger at 05:49:02–05:49:21
UTC. All software/SDK revisions remain as in the unloaded entry below; no new
command/hold/gripper path, gains, zero or limits were introduced. The operator
subsequently confirmed the remote stayed fixed and motion was normal, with no
slip/collision/abnormal sound or force, and identified the label as **R3-1**.

Evidence: `data/temporary/right_remote_load_20261011/` includes executable
capture/analysis scripts, raw CSV, native positions, console, capture report,
postcheck, analysis and a separate post-capture operator confirmation. Capture
metadata explicitly says `external_load_present=true`; **do not pass it through
an unloaded tare**. The reusable unloaded analyzer now refuses explicitly loaded
or non-boolean/unknown load metadata, while legacy captures without that field
continue to rely on the original documented operator unloaded declaration.

Captured 1001 position packets, 16715 raw frames and 2089 complete batches,
with three retained trailing partial frames, zero overwritten frames or socket
overflow. Maximum per-motor gap 5.154 ms, batch skew 4.477 ms, kernel queue age
288.841 microseconds. All raw statuses observed were 1. Both native arm vectors
and raw Follower positions stayed in configured bounds; lifecycle reported
ready/running and successful shutdown with no scheduling/CAN fault. Native
position callback sequence range 5..5005 is decimated publication, not evidence
of 4000 lost published packets. Raw SHA256:
`e3b61ee46dcadfdf0213f8c42dccc7c41308137e6d8bed1de981b60cf4282311`.

Compared with the earlier unloaded run's last two seconds, loaded last-two-second
mean torque changes in J1..J7 are approximately
[+0.6289,-0.0118,-0.0255,+0.6603,-0.1143,-0.0310,+0.0861] Nm.
This is cross-session raw torque comparison only, not external FT. J1/J4/J5/J7
depart from the previous reference by more than the 0.002-rad baseline envelope
(approximately -0.136/-0.153/+0.282/-0.198 degrees). A restart also invalidates
reuse of that baseline. `analysis.json` deliberately reports `delta_wrench=null`
and `ft_validated=false`; no loaded samples were tared and no mass estimate was
inferred. The final two seconds were stable (largest position span ~0.022 deg).

Unitree's official R3-family page <https://www.unitree.com/cn/R3/> lists 300 g,
corresponding to nominal gravity 2.942 N. Operator label R3-1 is recorded;
catalog family mass remains a reference, not a measured mass including any
ties/hooks. Attachment point/COM and added fixation mass remain unmeasured.

All four links are now DOWN/STOPPED, exact USB parents preserved, zero live/
cumulative CAN errors and interface/qdisc drops, no remaining controller.
can0 TX/RX 89152/89147 (session delta 19120/19119), can2 93704/93704
(19120/19120); left unchanged (can1 zero, can3 3904/3904). Aggregate Leader
counter difference does not identify a lost motor response. Hardware-free
verification after the loaded-tare rejection change: 449 pytest tests, Ruff
lint/format and diff check passed. Next useful stage requires operator handling:
same-session load/unload comparison with explicit stable windows, after agreeing
how the remote can be removed without changing arm/gripper posture. Do not
restart motion just to produce another cross-session baseline or claim a 300 g
weight measurement from this run.

### 2026-10-11 repeatable unloaded version and fresh right-only capture

Operator requested a no-external-load version and freshly confirmed both right
Leader/Follower powered, untouched, suspended without added load and with clear
alignment space. Existing camera/gripper/brackets stayed installed. Implemented
the hardware-free `python -m vlai_l1_runtime.unloaded_baseline` analyzer and
explicit `configs/diagnostics/right_unloaded.json`; usage is documented in
`docs/FORCE_ESTIMATION.md`. No new motor command, enable, Reset, calibration,
force-control, camera or live FT path was added. Runtime physical identity,
bounds, torque ranges and source freshness remain authoritative.

The analyzer checks capture/config/model provenance, all raw identities,
sequence/clocks, protocol endpoints, overflow, per-motor freshness and exact
eight-motor batches. It refuses duplicate motor frames before a complete batch
instead of silently overwriting them. It fits only stationary seconds 2..4 and
evaluates independent seconds 6..10, with minimum counts and time coverage.
All eight motor positions, including the gripper, must remain within 0.002 rad
of the reference after fitting. Pose departure, stationarity loss or a sample
gap invalidates the baseline for the remainder even after a return to pose.
Rejected validation rows contain null wrench plus a reason. Create-only output
contains report, validation rows and untared static-pipeline decisions. Every
result remains `ft_validated=false`; baseline cannot be loaded across sessions.
The local regular-file reader now permits an explicit artifact byte cap while
retaining the original 1 MiB configuration default and device/symlink rejection.

Historical Oct9 replay in `data/temporary/right_unloaded_replay_20261011/`
reproduced the 417-fit/833-validation result and prior noise statistics. Then
the existing bounded right-only diagnostic ran 05:29:11–05:29:29 UTC including
preflight, alignment and shutdown. Artifacts and exact invocation are under
`data/temporary/right_unloaded_20261011/` (`capture.py`, `console.log`,
`report.json`, `raw_feedback.csv`, `positions.jsonl`, `postcheck.json`,
`baseline/`). This reused the existing ManagedXAirRuntimes and same-owner
passive logger; no other CAN querying owner, left motion or camera capture.
Repository HEAD was `438bb696935a1f379cc0ffc1aafbc469a986a44c` plus the recorded
working changes; SDK remains `bf300508e179f652b23f0efaf3b6c9048f1f12e9`.

Fresh capture passed: 1001 position packets, 16688 raw frames, 2086 complete
batches over 10.0140 seconds, no partial/overwritten batches or socket overflow.
Maximum per-motor gap 5.259 ms, receive skew 4.613 ms and kernel queue age
168.868 microseconds. All observed raw statuses were 1 (diagnostic acceptance,
not commissioned semantic health). Position packet native callback sequences
range 5..5005; do not describe their increments as 100 Hz packet loss because
native callbacks and publication run at different rates. The guarded lifecycle
reported ready, running, and successful disable/down; no scheduling/CAN fault
was reported. No separate per-thread scheduler snapshot was taken this run.
Raw SHA256 is
`28b0dfa2d2e4773a7150080643ceed2c8146c7acdace9c0425ed03126d72cda8`.

Fresh baseline report passed with 417 fit samples and 833/833 held-out delta
candidates, no pose invalidation or numerical rejection. Delta-force standard
deviations [Fx,Fy,Fz] = [0.13654,0.07553,0.23595] N; delta-moment standard
deviations = [0.01077,0.00762,0.01199] Nm. Mean deltas are near zero; peak absolute
force was 1.18087 N. These are same-pose zero-load repeatability, not accuracy,
gain/direction calibration or permission for contact control. Untared stationary
estimates still failed the joint residual criterion (1911 rejections, 54 moving
and 121 warmup samples), so absolute FT remains unsupported.

Postcheck: no controller remains, all four configured links DOWN/STOPPED, exact
USB parents preserved, all live/cumulative CAN errors, interface and qdisc drops
zero. can0 TX/RX 70032/70028 (delta 19096/19096), can2 74584/74584
(delta 19096/19096); left unchanged (can1 zero, can3 3904/3904). No retries,
adapter reset, hardware zero, limits or gains changed. Software checks were
hardware-free: 448 pytest tests, 3 native CTests, Ruff lint/format and
`git diff --check`. Pytest was run with external plugin autoload disabled and
inherited ROS PYTHONPATH removed; the first default invocation failed during
ROS plugin import, before running tests or initializing hardware.

Next safe stage: review a known load/attachment/direction with the operator to
test delta-force sign and gain at a freshly established same-pose baseline.
No added-load test has run. Left J4 remains unresolved; do not move it or alter
zero/limits to make the reading pass. Restarting a session or changing posture,
tool or gripper opening requires a fresh baseline, not reuse of these numbers.

### 2026-10-09 same-owner active right torque capture and offline tare replay

On the operator's request to continue, implemented opt-in passive Follower raw
logging inside the existing native SDK-owning sidecar. `run-xair` accepts
`--feedback-log` plus explicit `--feedback-max-frames`; the managed wrapper
allows the same options only for an isolated single side. Response IDs come
from System config, and timeout/poll budgets reuse tracked state freshness and
CAN health intervals. No additional motor handle or CAN transmitter, command
adapter, enable/Reset/hold/calibration path, camera access or status inference
was added. Default launches do not open this extra receive socket.

`PassiveFeedbackLog` opens its filtered, nonblocking CAN FD receive socket after
the SDK constructor returns and before `start`; thus initial alignment itself
is not captured. It records raw bytes, host monotonic/realtime dequeue clocks,
SO_TIMESTAMPNS kernel realtime timestamps and SO_RXQ_OVFL. A bounded preallocated
buffer avoids filesystem writes in the control window. Missing/stale motor
responses, capacity exhaustion, invalid timestamps/frames, overflow or receive
errors latch failure; the sidecar loop stops on capture failure. Stop joins the
receiver before SDK loop shutdown. Log serialization occurs only after `run`
has released SDK handles. File output is create-only, with the invoking user's
ownership. Parent/lifecycle shutdown now also propagates a failed native child
shutdown instead of discarding its exit status. VL1T live publication is still
not connected; this is a diagnostic raw CSV path, not an online FT stream.

After software verification, the announced 10-second isolated-right capture
ran 10:44:00–10:44:18 UTC including startup/shutdown. The operator was told not
to move Leader or touch/load Follower. Evidence and executable experiment/
analysis scripts are in `data/temporary/right_active_torque_20261009/`.
Captured 1001 native position packets and 16706 raw Follower frames over
10.0256 seconds: 2088 complete eight-motor batches and two trailing partial
frames, retained but not emitted as a complete batch. All status nibbles are
1; this is an observation, not a commissioned enabled/healthy-status policy.
Maximum per-motor gap is 5.102 ms, maximum batch receive skew 4.508 ms, maximum
kernel-to-dequeue age 244.039 microseconds, and reported socket overflow is zero.
Nearest-time raw/SDK position comparison matched all 1001 packets for every
motor, with largest discrepancy 0.1093 degrees (not acquisition synchronization).

Stable last-two-second decoded arm torque means in joint order are approximately
[2.408, -0.042, -0.463, 2.303, -0.102, -0.141, 0.472] Nm. Unverified identity
coordinates, URDF gravity without camera/bracket payload and zero nominal
friction yielded 55 nonstationary samples, 140 warming-up samples and 1893
residual rejections; no untared FT candidate was accepted. Do not relax the
residual threshold to report those numbers as forces.

A separate offline same-pose tare used only seconds 2..4 (417 samples), then
evaluated seconds 6..10 (833 candidates) within an explicit 0.002-rad pose
envelope. It subtracts the baseline model-minus-motor residual; nothing is
written to motors or Runtime config. Mean untared joint residual norm was
0.2624 Nm; maximum tared residual norm was 0.0314 Nm. Held-out delta-force
standard deviations [Fx,Fy,Fz] were [0.1099,0.0829,0.2059] N and delta-moment
standard deviations [Mx,My,Mz] [0.01194,0.00729,0.01323] Nm. Peak absolute
delta force was 0.7545 N. These are zero-load repeatability under hypotheses,
not gain/direction/accuracy verification or a general payload/friction model.
Every result remains `ft_validated=false`. `tare_analysis.json` and
`held_out_tare_results.jsonl` preserve the independent time split and caveats.

No controller remains; all four links are DOWN/STOPPED with zero live/cumulative
CAN errors and interface drops; right qdisc drops are zero. Left counters were
unchanged (can1 0, can3 3904 TX/RX). Final totals: can0 TX 50936/RX 50932,
can2 TX/RX 55488. Session deltas: can0 19112 TX/19110 RX, can2 19112/19112;
aggregate Leader differences are not evidence of a specific missing motor
response and must not be called complete matched active telemetry.

Software verification: 432 hardware-free pytest tests, three hardware-free
native CTest tests, Ruff lint/format and diff whitespace checks passed. Tests
cover optional/bounded log arguments, pure buffer timestamp/drop/capacity/
silence faults and latching; live socket reception was exercised only by the
separate authorized capture above. The operator has been asked whether a
weighed small mass with a hanging line or a handheld force gauge is available,
and told not to apply it yet. A known-load experiment needs that information
and an agreed attachment/direction; no load test has been run. Camera/bracket
mass/COM, torque gain/sign and coordinate validation remain unresolved.

### 2026-10-09 isolated right 15-second active position observation

The operator requested continuing tests and explicitly allowed focusing on
the right arm. Existing right Leader power/workspace and normal Reset motion
confirmations remain applicable. Announced a bounded 15-second observation
through the existing unilateral lifecycle, with no manual movement or external
load and automatic disable/down at completion; this is not fixed-pose hold.

First, `data/temporary/right_before_observation_20261009/` captured one
configured 40-round right-Follower motion-free window: all eight IDs each round,
no bound violations or CAN errors/drops. Right J4 was approximately 22.24
degrees after the earlier Reset/shutdown; do not assume every disabled pose
returns to the earlier nearly straight pose.

The active experiment ran 07:54:26–07:54:50 UTC including guarded startup and
shutdown, with a 15-second position observation window. It used the existing
`ManagedXAirRuntimes` right-only configuration and `XAirStateReceiver`, without
another CAN handle, camera access, payload, calibration, new command path or
runtime code change. Artifacts:
`data/temporary/right_active_observation_20261009/{observe.py,report.json,positions.jsonl,analyze.py,analysis.json,postcheck.json}`.
1501 samples span 14.999999794 seconds. Source timestamp intervals are
9.98486–10.016061 ms and receive ages 0.584–0.898 ms; no timeout or timestamp
regression was seen. **Sequence caveat:** the diagnostic used assembled
`NamedJointVector` metadata, whose sequence is an assembler output counter,
not the native packet source sequence. Its `source_sequence` field and
`skipped_source_sequences = 0` must not be described as native zero-loss
evidence. `analysis.json` records this limitation; timestamps remain source
timestamps. Future source-loss diagnostics must retain raw packet sequences.

Both right Leader and Follower recorded seven-joint positions stay within
tracked bounds. Follower J4 moved from about 35.09 to 34.54 degrees; largest
joint peak-to-peak spans were J7 1.333 degrees and J1 1.137 degrees. Maximum
absolute following error was 1.355 degrees on J7. Leader spans were under
0.088 degrees. This is drift in an ordinary tracking session, not proof of
fixed-pose hold or an unloaded calibrated static baseline.

Offline geometry under the same unverified identity coordinate mapping and
0.2 m characteristic length gives condition number 15.98–16.11 at the
first/last pose, versus about 294 in the earlier nearly straight right pose.
Quantization-only full-step worst-case force-Z sensitivity is about 0.41–0.42 N;
this is not measured FT or accuracy. No active motor torque was captured, no
load was applied, and no FT candidate is validated. A same-owner active torque
stream, coordinate/torque-sign validation, and camera/bracket payload evidence
remain prerequisites to a known-load wrench experiment.

After shutdown no controller remains and all configured links are DOWN/STOPPED
with zero live/cumulative errors and interface drops; active right qdisc drops
are zero. Left counters are unchanged during this turn (can1 0, can3 3904
TX/RX). Final right totals are can0 TX 31824/RX 31822 and can2 TX/RX 36376;
active-window deltas are 27424/27423 and 27432/27432 respectively. Aggregate
counter differences do not identify a missing motor/sample. No retries,
adapter resets, zero changes, limit changes or left motion occurred.

### 2026-10-09 physical identity confirmed; right Reset and shutdown verified

The operator confirmed hub ports 2/3 connect to the intended Followers and
that the Follower side assignments are correct. Both Follower endpoints are
fully suspended. The operator also confirmed right Leader power on, no person
holding it, and clear motion space. Reset experiment authorization is explicit;
do not ask for the same authorization again. The desired eventual raised hold
is still outstanding, and no calibration reference was established for a zero
write.

A bounded Follower-only capture at 07:36:30–07:36:36 UTC is saved in
`data/temporary/follower_before_requested_reset_20261009/`. Both sides passed
40 complete eight-motor rounds with zero CAN errors/drops. Left J4 remains
approximately -4.08 degrees; right joints are inside configured bounds.
The left side was not enabled or reset and no zero or bound was changed.

The first unprivileged right Reset invocation failed before hardware startup
because `/run/vlai-l1` is root-owned. The sudo invocation at
07:38:26–07:38:35 UTC completed startup alignment and received fresh state,
then returned an error because the managed parent tried to execute the absent
legacy `/opt/xarm_teleop/disable_unilateral_pair.sh`. The operator observed
normal right Follower raising and subsequent lowering with no abnormal motion,
noise or collision. The lifecycle child had already performed its existing
disable/down cleanup; no controller remained and all links were DOWN with zero
errors. Artifacts are in `data/temporary/right_reset_experiment_20261009/`
and `data/temporary/right_reset_experiment_privileged_20261009/`.

Fixed managed shutdown to let the existing lifecycle retain sole motor cleanup
ownership, remove the duplicate legacy-script call, and require a successful
child exit plus passively verified DOWN state for both owned links. A forced
shutdown, abnormal exit or unreadable/UP link fails verification. Only actually
started sides are checked/cleaned. The existing lifecycle disable routine now
checks command failures, still attempts remaining motors/links on an error,
and verifies links DOWN; it releases sockets/handlers/lock even if that routine
raises. This is not a new enable, Reset, zeroing or fixed-hold path. Failed
verification does not establish motor disable and must not trigger automatic
restart. Hardware-free tests cover these failure branches.

A second isolated right Reset at 07:42:05–07:42:14 UTC passed the full fixed
workflow (exit 0): guarded startup, alignment, fresh state, shutdown, DOWN
verification. Evidence: `data/temporary/right_reset_cleanup_verified_20261009/`.
All four configured links are DOWN/STOPPED, no controller remains, live and
cumulative CAN error counters and interface drop counters are zero; right
qdisc drops are zero. Left link TX/RX counters did not change during either
right Reset. Final counters: can0 TX 4400/RX 4399, can2 TX/RX 8624, can1 zero,
can3 TX/RX 3904. The second active session had 2200 TX and 2199 RX on can0;
aggregate counts do not establish which frame lacked a received counterpart
and must not be presented as complete matched live telemetry. The guarded
startup required complete motor responses before alignment.

The arms are not continuously held after standalone Reset. Left J4 coordinate
calibration and a reviewed fixed-hold capability remain unresolved; do not
substitute arbitrary zeroing or ordinary Leader tracking for them. Verification
after code changes: 423 hardware-free pytest tests passed, Ruff lint/format and
`git diff --check` passed. Live validation is the separate right Reset above.

### 2026-10-09 operator challenged physical Follower/Leader identity

The operator explicitly confirmed both Follower endpoints are fully suspended,
authorized a Reset experiment, and initially requested a zero change before
questioning whether the sampled hardware was actually Leader rather than
Follower. Resolve physical identity before any zero write or motion; do not
treat permission as a calibrated zero reference. The earlier unresolved
external-support question is now answered by the operator.

Read-only inspection confirmed `capture_follower_feedback` selects endpoints
by `role == "follower"` and records side/interface from that endpoint. The last
capture used can3/USB `1-3.1.3:1.0` for configured left Follower and can2/USB
`1-3.1.2:1.0` for configured right Follower. Live OS link metadata still matches
all four configured USB parents. can2 remains at 3904 TX/RX, can3 at 3584
TX/RX, and can0/can1 at zero TX/RX. All four are DOWN/STOPPED with zero live
errors; no controller was found in the process listing. No CAN messages were
sent during this review.

This establishes software selection and USB identity, not the physical cable
destination. Historical commissioning documents hub ports 2/3 as right/left
Follower and 1/4 as left/right Leader, but the operator's new identity question
requires a fresh physical check. Asked for visual cable tracing without
disconnecting cables or moving the arms. Do not claim the motor response IDs,
firmware ranges, or the configured role label independently identify the
physical arm. No zero, bounds, configuration, Reset or runtime behavior changed.

### 2026-10-09 operator photos reviewed without hardware access

Viewed both WebP images supplied in
`/home/nvidia/Pictures/received-2026-10-09/`: the file ending
`09446cc9077g.webp` shows a frontal view and the file ending
`5efca6734c3g.webp` shows a side view. Both Followers appear nearly straight
and hanging down, with wrist cameras/brackets and attached cables. This is
qualitatively consistent with the saved unverified-coordinate pose plot in
`data/temporary/follower_static_pipeline_analysis_20261009/pose_reference.png`;
it does not establish joint zero, sign, exact angles or a calibrated mapping.
No obvious gross abnormal bend is visible. Internal mechanical-stop contact
and small angular discrepancies cannot be ruled out from these images.

The grippers are close to the black base platform in projection; clearance
and load-bearing contact are not established because of perspective and
occlusion. Asked the operator to confirm contact/support without moving the
arms. Do not assume an unloaded or unsupported state from the images.
The photos do not resolve the reviewed Reset/hold limitations below. No
CAN access, enable, Reset, limit change or motion occurred in this review.

### 2026-10-09 requested Reset/raised hold reviewed; no motion executed

The operator requested Reset to raise both Followers, reported no visually
apparent physical limit violation, and clarified the desired result is raised
and continuously held. This explicitly authorizes the requested motion intent;
do not repeat the old claim that only non-motion work was requested. It does
not authorize changing/ignoring joint bounds or substituting an arbitrary
target or unreviewed command path.

Read-only review found that standalone `reset_managed_teleoperation` starts
selected runtimes, waits for startup alignment/state, then shuts them down
(disable). Keeping the existing unilateral runtime running also enables Leader
tracking; it is not an independent fixed-Follower-pose hold. The public SDK
constructor initializes/enables Leader and Follower and calls both opaque
`Control::AdjustPosition` routines before returning. Native callback/bounds
monitoring is installed after that constructor. Thus the known left-J4 raw
bound discrepancy cannot safely be resolved by assuming Reset will move it
inside the envelope before protection becomes active. A brief static binary
inspection did not establish a reviewed raise target/trajectory; no SDK was
loaded or executed in this review.

No CAN query, enable, Reset, runtime start, limit/offset change or motion
occurred in this review. The last sample remains approximately -4.08 degrees
against the configured J4 minimum of zero. This is a numerical bound finding,
not proof of mechanical overtravel. Physical coordinate/support review is
still required; do not promise that starting Reset alone validates the zero
or that a stopped/disabled arm will remain raised.

Hardware-free verification after this review: full pytest 413 passed; Ruff
lint and format checks and `git diff --check` passed. Only this handoff entry
was changed during the Reset review; runtime behavior was not changed.

### 2026-10-09 continuous-work authorization and integrated static replay

The operator asked to continue autonomously until physical participation is
needed. Existing non-motion test authorization persists; no enable, alignment,
manual manipulation, applied load or recovery authorization was added. The
operator confirmed attached cameras, upright base and a safe scene, but did
not establish camera/bracket mass/COM or whether the arms are externally
supported or resting at limits. Do not equate that reply with active holding
or an unloaded baseline.

New pure `static_wrench.py` connects tracked raw decoding, seven explicit
power-consistent motor/model maps, an observed stationary window, URDF gravity,
explicit static-friction inputs and the existing damped wrench solve. Invalid
identity/status/timing/bounds latches a rejected stream; motion/gaps clear the
stationary window; bad conditioning/residual/error bounds reject a candidate.
No rejected record emits a zero or stale wrench. All outcomes, even numerical
candidates, retain `ft_validated = false`. No live publisher, command path or
calibration defaults were introduced.

The wrench result now includes the solve's torque-to-wrench gain and a pure
componentwise error-propagation helper. The pipeline accounts for one full
protocol torque step plus explicit unmodeled-effort bounds. Unknown model and
coordinate errors/damping bias are not implicitly bounded. Synthetic tests
cover raw quantized packets through six-axis recovery, torque/power signs,
stationarity resets, latched faults, uncertainty rejection and adversarial
error propagation.

A fresh guarded bimanual capture ran 07:06:56–07:07:01 UTC with all 40 rounds
and all eight IDs per side. Each Follower added 320 TX/RX frames, with zero CAN
errors and interface/qdisc drops. Final all-links DOWN/STOPPED, can2 totals
3,904/3,904 and can3 3,584/3,584, Leaders zero. No enable, disable, alignment,
parameter write, movement command or camera access occurred.
Source: `data/temporary/follower_static_pipeline_check_20261009/`.

Replay: `data/temporary/follower_static_pipeline_analysis_20261009/`.
Left: 40 invalid records due to existing J4 bounds. Right: eight warming-up
records and 32 conditioning rejections. There are zero FT candidates. Under
the explicit identity-mapping hypothesis and 0.2 m diagnostic length, condition
numbers are approximately 130/294 (left/right). Torque quantization alone can
produce worst-case Z-force component error bounds around 3.79/8.66 N in these
poses; these are sensitivity bounds, not measured force or calibrated accuracy.
The `pose_reference.png`/SVG plot shows this hypothesis, never a target pose.

Next requires operator-assisted physical geometry/support review (front/side
photos) before any manipulation. Camera/bracket payload evidence is also
missing. The active SDK still lacks a public torque callback, and live native
telemetry ingress/status commissioning remains necessary before an enabled
known-load experiment. Do not run the diagnostic beside an active controller.

Hardware-free regression checks passed: 413 pytest cases, Ruff lint/format and
`git diff --check`. Python 3.10 static policy validation and coordinate/power
conversion also passed with socket/process access forbidden. The pose plot was
rendered in an isolated plotting environment; no deployment dependencies were
changed. No physical FT claim or calibration/readiness gate was promoted.

### 2026-10-09 offline gravity calculation and model audit implemented

The operator asked to continue. New pure `gravity.py` calculates URDF subtree
holding efforts `g(q) = dU/dq`, including fixed hand/tool masses, independent
prismatic joints and mimic finger branches. Gravity vector, exact URDF joint
coordinates and additional payloads are explicit caller inputs. Missing mass/
COM data fail validation unless an empty frame is explicitly listed as
massless. Root/branch topology, mimic dependencies, finite values and payload
attachments are checked. Results preserve units, per-mass contributions and
out-of-limit coordinates; they do not authorize motion or supply calibrated FT.

The saved rendered URDF has 11 mass-bearing links per side totaling
5.5057424269 kg, including the fixed arm mounting link and both fingers. Its
source vendor YAML contains a calibration comment, but those mass/COM values
have not been verified on this installed robot. Camera/tool/cable payloads,
mount orientation, motor-to-model coordinates and friction/bias remain open.
No SDK friction defaults were adopted as calibrated estimator parameters.

Evidence: `data/temporary/follower_gravity_audit_20261009/`. Each side passed
16 synthetic poses, checking all eight generalized efforts against potential
energy differences (max error below 1e-9), and a hypothetical 0.5 kg TCP load
against Jacobian projection (max error below 1.4e-15 Nm). The recorded pose is
an explicit identity-mapping hypothesis only, with assumed body-frame gravity,
zero modeled finger opening and no additional payload. Left J4's violation
persists. A synthetic seven-joint test also verifies gravity subtraction and
six-axis wrench sign end to end.

This stage used files and numerical computation only. No gravity-compensation
controller, live hardware operation, coordinate offset, physical calibration,
or new live telemetry path was introduced. The next physical validation still
requires a supported pose/mapping and known-load procedure; non-motion hardware
diagnostics remain authorized without repeated confirmation.

Hardware-free validation passed 396 pytest cases, Ruff lint/format and
`git diff --check`. Python 3.10 loaded and validated the complete right subtree
without NumPy or hardware dependencies; sockets, SDK loads and subprocesses
were explicitly forbidden during that check.

### 2026-10-09 tracked decoding ranges and offline comparison implemented

After the operator asked to continue, the real parameter survey was promoted
into the optional diagnostic `follower_feedback` System-config section: exact
eight-motor records per side, P/V/T ranges, raw version registers, parameter-log
SHA256 and the survey config SHA256. Configuration validation rejects missing,
misordered, unknown, non-finite or unrepresentable range data and includes the
section in its mutation fingerprint. Absent evidence provides no decoder fallback.
The command schema/readiness and active SDK control path are unchanged.

New pure `feedback_decode.py` and `redecode-follower-feedback` CLI decode saved
raw captures with those ranges, preserving SDK values in separate CSV columns.
The CLI rejects wrong endpoint/config identities, incomplete rounds and
regressing timestamps, uses only regular local files and creates new output.
It does not open CAN/SDK/camera resources or authorize force estimation.

Offline audit and per-capture comparisons are in
`data/temporary/follower_tracked_range_replay_20261009/`. All 16 captures/5,120
motor frames pass: positions and torques match the SDK exactly; J3/J4 velocities
on both sides use the reported maximum of 10 rather than the SDK's 8. The pure
native decoder agrees exactly with the Python decoder across all saved frames.
All 16 tracked parameter hashes and register values were checked against the
original survey. Source captures are unchanged; the config now has a new hash.

Live telemetry status/timing integration, joint-coordinate verification,
gravity/friction/payload compensation and known-load FT testing remain open.
Left J4's bounds finding is preserved. No hardware operation occurred in this
software/replay stage; the prior non-motion authorization remains in effect.

Hardware-free validation passed 380 pytest cases, Ruff lint/format and
`git diff --check`. The offline CLI was also exercised on the saved right-J3
capture. Python 3.10 config loading and raw decoding passed with sockets,
subprocesses and SDK-library loading forbidden (process libc remains available
for the existing regular-file filesystem check).

### 2026-10-09 updated operator authorization: motion-free diagnostics

The operator now explicitly authorizes all task-relevant **non-motion hardware
tests** without further per-test confirmation. This supersedes the earlier
stop-and-ask instruction for such diagnostics in the historical entries below.
It covers guarded Follower state/parameter reads, not enable, alignment,
teleoperation startup, manual repositioning, loading, zeroing, parameter writes
or adapter recovery. All existing CAN/ownership/identity/freshness gates remain;
a failed or missing response stops the test sequence without automatic retry or
reset. Both Followers were previously confirmed powered. The current survey
used existing guarded single-motor captures sequentially, right then left.

### 2026-10-09 both Follower parameter surveys completed without motion

From 06:27:12 through 06:28:34 UTC, all 15 remaining single-motor captures
passed: seven right motors and all eight left motors. Together with the earlier
right-J3 result, all 16 motors now have eight verified parameter reads each.
Each new capture completed all 40 eight-motor state rounds, then eight parameter
transactions, adding exactly 328 TX/RX frames (4,920 each direction in total).
All live/cumulative CAN errors and interface/qdisc drops stayed zero. Final
links were DOWN/STOPPED, right/left Follower counters 3,584/3,264 each direction,
and both Leader counters remained zero. No enable, disable, alignment, motion,
parameter write, reset or camera access occurred.

Both sides agree by corresponding joint: P_MAX=12.5 for all; J1/J2 V_MAX=45,
T_MAX=54; J3/J4 V_MAX=10, T_MAX=28; J5/J6/J7/gripper V_MAX=30, T_MAX=10.
Thus both J3 and J4 on both sides differ from the pinned C SDK's velocity
maximum of 8. All position and torque ranges agree. Raw hardware version is
zero for all; firmware/sub-version values are recorded without interpretation
in [Force estimation](FORCE_ESTIMATION.md). Left J4 remains about -4.076 degrees
below its configured lower bound; no coordinate offsets or bounds were changed.

New captures: `data/temporary/follower_parameter_survey_20261009/`.
Offline audit: `data/temporary/follower_parameter_survey_analysis_20261009/`.
The audit validates all 128 parameter values and 5,120 saved state frames across
the 16 captures. All C SDK float32 values are reproduced from original payloads.
Using reported ranges preserves positions/torques and rescales J3/J4 velocities
by 1.25 within float32 rounding. Observed stationary torque variation does not
validate external-force sensitivity or physical calibration. Native live
telemetry, status semantics, coordinates and gravity/friction compensation
remain unresolved; no valid online six-axis FT stream is available yet.

Next physical calibration involves supported pose/coordinate checks and known
loads, requiring an agreed movement/loading procedure. Existing authorization
continues to cover task-relevant non-motion diagnostics without repeated asks.

After documentation updates, hardware-free checks passed: 363 pytest cases
(including native cross-language checks), Ruff lint/format and
`git diff --check`. A final read-only interface inspection confirmed all links
still DOWN/STOPPED with zero errors and unchanged frame counts.

### 2026-10-09 right J3 parameter read passed; velocity scale mismatch confirmed

The operator explicitly approved the proposed right-only state/parameter test.
One schema-3 capture ran from 06:16:13 to 06:16:16 UTC, after healthy baseline,
inactive services, exclusive ownership and adapter identity checks. All eight
right Follower motors passed all 40 state rounds, then all eight allowlisted
J3 parameter reads succeeded. Right `can2` added exactly 328 TX and 328 RX
frames (320 state plus eight parameter transactions), with zero live/cumulative
CAN errors and zero interface/qdisc drops. No bounds violations occurred.
All links were verified DOWN/STOPPED afterward; right Follower totals are now
1288/1288, left remains 640/640, Leaders remain at zero. No enable, disable,
alignment, parameter write, reset, movement command or camera access occurred.

J3 reported master/slave IDs 19/3, raw hardware version 0, raw firmware integer
925970741, sub-version 53, and ranges P_MAX=12.5, V_MAX=10, T_MAX=28. Version
format/meaning beyond those raw values is unverified. This **confirms a mismatch**
with the pinned C SDK's J3 velocity decoder maximum of 8 rad/s; position and
torque ranges agree. Offline re-decoding of the 40 new J3 state samples with
the reported ranges preserves position/torque exactly and gives velocity 1.25
times the SDK values within float32 rounding. This is a decoder-scale finding,
not a physical speed/torque calibration or validated FT estimate. It must not
be extrapolated to J4, other motors or the other side.

After the live run, hardware-free validation passed 363 pytest cases (including
native cross-language checks), Ruff lint/format and `git diff --check`.

Capture evidence: `data/temporary/follower_right_j3_parameter_check/`.
Offline comparison/hashes: `data/temporary/follower_right_j3_parameter_analysis/report.json`.
Original captures, the pinned SDK and Runtime configuration were not rescaled
or edited. The eight actual reply payloads now have hardware-free regression
tests. The control SDK's velocity scaling remains unchanged and unverified for
use in dynamics/compensation; native telemetry must eventually use reviewed
per-motor evidence, not simply reproduce that SDK mismatch.

Next safe stage: software review of per-endpoint/motor range provenance and a
separately authorized bounded parameter capture for the other required motors.
The diagnostic is proven only for this one J3/firmware. Stop before another
hardware test per the operator's standing request. Coordinate, live telemetry,
gravity/friction/payload and known-load FT checks remain outstanding.

### 2026-10-09 telemetry wire/session and guarded parameter diagnostic prepared

The operator asked to continue and confirmed there are no motor parameter
records. Work remained hardware-free. The native decoder now has a separate
VL1T/1 serializer and Python parser/session validator; the live VL1S position
stream and sidecar main are unchanged. New packets carry explicit session/config
identity, sequence, per-motor raw frames/timestamps and float32 feedback.
Consumer checks latch failures for wrong identities/statuses, stale/repeated
samples, protocol endpoints and startup/silence deadlines. Runtime must invoke
the deadline poll even when packets stop. No live telemetry socket is wired in.
All policy inputs remain explicit; no enabled-state or calibrated-FT assumption
is made. Cross-language fixtures and offline replay of the saved 40 groups/320
motors preserve all raw frames and SDK values exactly. Local replay evidence:
`data/temporary/follower_feedback_raw_pair_analysis/wire_replay_report.json`.

Read-only protocol research found the manufacturer parameter interface at
`dmBots/motor-sdk` revision `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a` (linked in
FORCE_ESTIMATION.md). A pure allowlisted request/reply codec and guarded optional
`--read-parameters-for MOTOR` stage are implemented. It requires exactly one
Follower side/motor, completes full state preflight first, releases the SDK
handle, then issues at most eight parameter reads from that same owner process.
It uses tracked diagnostic timing budgets and CAN/FD setting; errors, queue
overflow, duplicate/late/unsupported replies and timeouts fail without retry.
No parameter write/save, enable, mode change or calibration code was added.
Schema 3 saves raw parameter evidence; state-only captures remain schema 2.

Final hardware-free validation: 355 pytest cases, both native CTest targets,
Ruff lint/format and `git diff --check` passed. Python 3.10 pure contracts and
syntax passed with socket/process/SDK loading denied. Tests additionally cover
parameter socket setup cleanup, health rejection and global-budget expiry before
send. No parameter query has been transmitted to the robot yet.

**Next hardware stage pending operator confirmation:** one right-Follower
capture with `--read-parameters-for joint_3`, output
`data/temporary/follower_right_j3_parameter_check`. The command is documented
in FORCE_ESTIMATION.md. Preflight covers all eight right motors for 40 rounds;
then only J3's identity/version/range registers are read. This is new hardware
work, not authorized by the earlier state-only test. No live operation was
performed in this stage. DM4340 is selected because its SDK/example velocity
ranges differ; actual configured values and firmware support are still unknown.
After any failure stop; do not automatically retry or reset. Existing coordinate,
physical torque, kernel queue timing and online-owner integration gates remain.

### 2026-10-09 native feedback core and offline replay

The operator asked to continue software work. No CAN interface, motor handle,
ROS node, camera or service was opened in this stage. A standalone C++17
`motor_feedback` library now decodes raw candidates and assembles exact named
eight-motor batches with explicit ranges and age/skew limits. It is not linked
into the live sidecar and adds no socket, enable path or v1 transport change.
Unknown/status-fault nibbles remain raw data; a decoded batch is not healthy,
enabled or FT-valid by implication. Missing/stale/skewed candidates emit no
batch. Invalid target frames/flags or timestamp regressions latch the session.

The module compiled with strict warnings; native CTest covers decoding,
integer endpoints, status preservation, incomplete/old/skewed batches, time
regressions, malformed frames and persistent faults. Offline native replay of
all 320 saved right-Follower raw/SDK pairs matched all position/velocity/torque
float32 values exactly. Replay input is
`data/temporary/follower_feedback_raw_pair_analysis/native_replay.tsv`.
See FORCE_ESTIMATION.md for build commands, input format and source hash.

Final validation was hardware-free: both native CTest targets, 235 pytest
cases, Ruff lint/format and `git diff --check` passed. Generated native build
artifacts are under `/tmp/vlai-feedback-pure-build`, outside the repository.

A manufacturer DM-J8009-2EC manual supports the feedback packing and notes
configurable P_MAX/V_MAX/T_MAX, but does not establish the installed firmware's
normal status semantics or actual configured ranges. These are still live
integration gates, along with queue overflow/kernel timestamp handling, versioned
consumer transport, owner-enforced missing-data deadlines and motor-status policy.
No register-query implementation, firmware change or live telemetry reader was
added. Future settings must come from tracked Runtime configuration rather than
the binary audit table embedded as defaults.

Next safe work: resolve model/firmware/range evidence and review the owner-only
telemetry ingress/transport before connecting this core. The physical coordinate
verification procedure is outlined in FORCE_ESTIMATION.md. Stop and call the
operator before any further hardware test or supported-pose manipulation, as
requested. Existing motion, J2, left J4 and calibrated-torque gates remain open.

### 2026-10-09 right-Follower raw/SDK pairing passed (05:21 UTC)

After the operator explicitly approved the proposed right-only motion-free
test, schema-2 `capture-follower-feedback` ran once from 05:20:59 to 05:21:02 UTC.
Prechecks found inactive controller services, no controller process, verified
adapter identities, all links DOWN and zero errors. Right Follower `can2`
returned all eight configured response IDs on all 40 rounds, adding exactly
320 TX and 320 RX frames. CAN remained ERROR-ACTIVE with zero live/cumulative
errors and no interface or qdisc drops. No bounds violations were reported.
Post-capture inspection confirmed every link DOWN/STOPPED, `can2` TX/RX totals
960/960, untouched left Follower totals 640/640, and zero Leader frames. No
enable, disable, alignment, reset, motion-capable constructor or camera was used.

Artifacts are in `data/temporary/follower_feedback_raw_pair_check/`: report,
320-row CSV and 40-line raw/SDK JSONL. The offline reproducible audit script
and report are in `data/temporary/follower_feedback_raw_pair_analysis/`.
It verifies capture/config/library hashes, exact named vectors/IDs, increasing
host timestamps and CSV/JSONL agreement. All 320 raw payloads decoded with the
pinned C binary's audited range table reproduce every SDK position, velocity
and torque value exactly after float32 conversion (zero maximum error).
The vendor Python passive ranges instead differ by up to 0.025641 rad/s and
0.021978 Nm on these same frames. No library was loaded by the offline audit.

This establishes decoder agreement only on the sampled near-stationary right
arm, not full-range protocol correctness, physical torque calibration or FT
accuracy. Host dequeue-to-SDK-read intervals were 56.79–58.01 ms; neither is
a motor acquisition timestamp. Raw status bytes were preserved without
inferring enable state. No six-axis FT measurement was produced.

Post-test software validation was entirely hardware-free: 235 pytest cases
passed, Ruff lint and formatting passed, and `git diff --check` was clean.

The next stage is software/design work on verified protocol/status semantics
and telemetry inside the existing controller owner, plus a concrete coordinate
verification procedure. Do not repeat capture or begin a motion/load test
without the operator's requested new-test confirmation. J2 mapping, left J4
below the configured lower bound, torque sign/scale, and payload/gravity/friction
compensation remain unresolved. A known-load experiment needs that telemetry
and a separately approved safely held pose; this diagnostic cannot run alongside
an active controller.

### 2026-10-09 offline geometry and raw-frame pairing; next hardware test pending

The operator asked to continue and confirmed no additional SDK source/version
is available. This stage remained entirely hardware-free. The configured
mirror's HEAD/main still match `bf300508`; no dependency revision was changed.
Static inspection confirms the public teleoperation API has no velocity/torque
accessor and the inspected JointMapper/XArmJointConverter functions copy
coordinates directly. J2's +/-pi/2 origin rotation is already in the URDF;
neither that nor left J4's -4.076 degree feedback authorizes a guessed offset.

`kinematics.SerialChain` now loads explicit fixed/revolute URDF chains and
computes root-frame tool poses and geometric Jacobians without ROS or SDK
initialization. It validates joint order/topology/values and reports model-limit
violations without clamping. Finite-difference, analytic and synthetic wrench
tests validate the math. Offline evaluation of the earlier CSV under a clearly
unverified identity motor-to-URDF assumption is retained in
`data/temporary/force_geometry_audit_20261009/report.json`. The SDK hand origin
and URDF TCP differ by 0.08 m; moment translation must accompany any point
change. At an explicitly chosen 1 m SVD scaling, left/right condition numbers
were about 295/667, suggesting weak observability at those assumed model poses,
not a physical calibration or safety verdict. No FT measurement was produced.

A substantive decoder discrepancy was found in the pinned aarch64 release:
the C API remaps public motor-type codes before indexing its internal range
table, whereas the vendor Python passive decoder does not. For configured
DM8009/DM4340/DM4310 the C-decoder torque maxima are 54/28/10 Nm versus
28/10/5 Nm in the Python passive path; velocity ranges differ too. These are
binary/protocol findings, not verified physical torque ranges. Existing captures
use the C API. Do not import the vendor passive decoder or silently rescale data.
The pinned hash, addresses and limits are documented in FORCE_ESTIMATION.md.

Capture report schema 2 now additionally saves `raw_feedback.jsonl`, pairing
each completed round's SDK values with all eight raw response payloads, IDs,
CAN/FD format and host dequeue timestamps. It reuses the existing probe receive
socket and sends no additional queries. It clears pre-round raw data, rejects
invalid payload lengths or duplicate responses for pairing, checks named
vectors and increasing per-motor host timestamps, and preserves existing CSV
fields. This prepares a possible receive-only telemetry reader within the
existing sidecar owner process; that online reader is not implemented yet.
Neither the raw-pair capture nor any new native telemetry has been hardware
tested. Earlier schema-1 capture evidence does not cover this extension.

Hardware-free checks pass 235 pytest cases, Ruff, formatting and diff checks;
Python 3.10 pure model/raw-feedback parsing and syntax also pass using the
declared tomli dependency. No CAN interface, controller, camera or ROS node
was opened in this stage. Next action: stop for operator readiness confirmation,
then one bounded **right-Follower-only** schema-2 capture to a new output
directory (`follower_feedback_raw_pair_check`) and offline comparison of raw
frames against C-SDK values. Do not enable or move an arm during that test.
Later motion/load tests still require verified coordinates, protocol/torque
semantics and telemetry through the existing controller owner.

### 2026-10-09 new-code Follower capture passed (04:53 UTC)

The operator explicitly confirmed readiness for the previously proposed
motion-free new-code test. With both controller services inactive, no controller
process, all tracked adapters correctly mapped and all links DOWN with zero
errors, `capture-follower-feedback --side bimanual` completed once under the
exclusive lifecycle lock. Left `can3`, then right `can2`, each returned every
configured motor ID on all 40 rounds, adding exactly 320 TX and 320 RX frames.
Both remained ERROR-ACTIVE during capture with zero live errors, no cumulative
error increase and no qdisc drops. Both returned DOWN/STOPPED after completion.
Leader links remained DOWN with zero frames. No enable, disable, alignment,
reset, controller constructor, camera operation or force command was issued.

Artifacts are `data/temporary/follower_feedback_first_code_check/feedback.csv`
and `report.json`. Offline inspection verified all 640 rows, exact eight-motor
order in each round, finite values and strictly increasing host read timestamps
per side. Actual capture rates were 14.51 Hz left and 14.49 Hz right including
health-check/recording overhead; these are sequential diagnostic windows, not
realtime or synchronized telemetry. Arm torque magnitudes remained near zero
(maximum 0.0396 Nm left, 0.0342 Nm right). Left J4 remained about -4.076 degrees
and was correctly listed as a raw-coordinate bound violation; right had none.
This validates the new query/recording path, not external-force sensitivity,
torque calibration, or six-axis FT accuracy. Do not advance directly to an
enabled load test: the next safe stage remains coordinate review plus supported
online torque feedback through the controller owner. The operator's readiness
confirmation covered only this completed motion-free capture.

Source/submodule revisions are unchanged from the entries below, with the new
tooling uncommitted. Hardware-free pytest again passed 202 tests with ROS plugin
autoload disabled; Ruff and diff whitespace checks passed.

### 2026-10-09 torque tooling prepared; hardware test explicitly pending

After the earlier live inspection below, the operator authorized code work and
explicitly requested a stop before testing the new code on hardware. No CAN,
camera, motor, ROS node or service operation was performed during this code
stage. Do not interpret the earlier feedback pass as validation of this code.

`capture-follower-feedback` is now an explicit bounded live-query CLI: it
reuses the existing probe/SDK owner to record named position, velocity and
estimated torque, requires all motor responses every round, checks CAN health
before/after rounds, and returns owned links DOWN on success or failure. It
uses tracked configuration, exclusive lifecycle ownership, create-only output,
and saves CSV plus a provenance/health report. Raw coordinate-bound violations
are reported for review; a communication pass does not authorize motion.
The capture refuses active teleoperation and never enables or resets motors.

`vlai_l1_runtime.wrench.estimate_wrench` provides a pure damped-SVD solver with
explicit point/frame, seven named compensated external torques, 6x7 geometric
Jacobian, dimensional scaling and rejection thresholds. It rejects singular
geometry, poor conditioning and excessive residuals. It does not compute
robot-specific gravity/friction compensation or guess motor/model coordinates.
The current SDK release still only exposes positions in its teleoperation
callback and lacks the internal headers needed to rebuild that API. The active
v1 state stream remains unchanged; supported online torque telemetry is a
separate unresolved SDK prerequisite before an enabled known-load experiment.

All 202 hardware-free tests pass with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, as do
Ruff, formatting and diff whitespace checks. A separate isolated Python 3.10
environment with the declared `tomli` dependency passes imports, feedback
validation and new-module syntax checks. Tests cover synthetic six-axis wrench
recovery, singular/residual rejection, missing responses, non-increasing host
timestamps, data validation, ownership/identity/CAN faults, interruption and
cleanup, existing-output protection, and import/CLI parsing without hardware
initialization. See [Force estimation](FORCE_ESTIMATION.md) for the exact next
query command and staged validation. Next action is operator confirmation for
the bounded new-code Follower capture only; motion/known-load testing still
requires coordinate review (including left J4), telemetry support, and a
separate cleared-workspace confirmation.

### 2026-10-09 Follower feedback and torque inspection (04:36 UTC)

The operator confirmed power on for both Followers and requested feedback
inspection for possible endpoint wrench estimation. On the current local host
`nyush-jetson-nx`, all four tracked PEAK USB identities matched, all links were
initially DOWN/STOPPED with zero counters, and no controller was active. Under
the exclusive lifecycle lock, a bounded diagnostic invoked the existing
motion-free probe on only right Follower `can2`, then left Follower `can3`.
Each passed all 40 rounds for all eight response IDs 0x011-0x018, with exactly
320 transmitted and 320 received frames, ERROR-ACTIVE, zero live/cumulative
CAN errors and zero qdisc drops. Only state queries were sent; no motor enable,
disable, alignment, reset, teleoperation constructor, or camera operation ran.
Both Follower links were returned DOWN; both Leader links remained DOWN.

The diagnostic also read finite position, velocity and torque values from the
existing CAN SDK handle after each complete feedback round. The retained
`data/temporary/follower_feedback_20261009_043608/feedback.json` contains those
SDK cache snapshots, host monotonic read timestamps, and before/after link
statistics. These are not per-motor hardware timestamps or validated FT data.
Arm torque magnitudes stayed below 0.0206 Nm right and 0.0396 Nm left in this
powered-only window. Near-zero feedback does not establish external-load
sensitivity, torque scaling, or gravity compensation accuracy. Left J4 raw
position averaged -4.08 degrees, below the tracked 0-degree lower limit;
reconcile raw motor/model/control coordinates before a motion-capable test.
Do not relax bounds to proceed. The next safe stage is hardware-free torque
telemetry/coordinate review, followed by a separately authorized, workspace-
cleared test with a known small load after the coordinate issue is resolved.

Repository revision remains `438bb696935a1f379cc0ffc1aafbc469a986a44c` with local
changes; embodied-ops remains `6eeb9e04b05cd59f8854efd1cb08572e08e5a4fa` and
x-air-sdk `bf300508e179f652b23f0efaf3b6c9048f1f12e9`. Static SDK verification
passed. Hardware-free checks passed all 148 tests, Ruff and formatting. Pytest
required `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` because the ambient ROS pytest
plugin imports Python 3.10 launch-testing dependencies into Python 3.12 and
fails on missing `lark`; no ROS node or hardware was initialized by the tests.

### 2026-09-29 cameras moved to USB2

At the operator's request, this check did not exercise CAN. Both D405 cameras
enumerated through the USB2-labelled hub on bus 2, whose upstream link reports
10 Gbps. Right serial `260422274280` is at `2-2.2` and left serial
`260322278153` is at `2-2.3`; each camera reports USB 3.2 and a 5 Gbps link.
The temporary two-wrist recording paths were updated by exact serial identity.
With no camera owner or controller active, the camera-only recorder opened only
the 640x480 YUYV color nodes at 30 FPS (no depth) and ran both concurrently for
30 seconds. Each side saved and decoded all 900 frames: right 29.993 FPS with
34.04 ms maximum gap, left 29.994 FPS with 34.29 ms maximum gap. The completed
75 MiB diagnostic is retained at
`data/temporary/usb2_bandwidth_20260929_035017`. Kernel logs showed the known
RealSense unsupported-format enumeration notice, but no USB disconnect, reset,
bandwidth-allocation error, or failed transfer during the window. Camera handles
were released. This validates bandwidth for the current two-camera RGB-only
640x480/30 mode; it does not validate depth streams, other resolutions, or
simultaneous teleoperation. All 148 hardware-free tests, Ruff, formatting and
diff whitespace checks pass after the path update.

### 2026-09-29 CAN hub moved to USB1

The operator moved only the four-port CAN hub upstream connection to the port
labelled USB1 and explicitly asked to reconcile CAN identity without inspecting
cameras. The four unchanged hub positions now enumerate below USB `1-3.1`.
Tracked parents and their hardware-free expectations were updated while
preserving the established hub-port mapping: port 4/right leader -> can0,
port 1/left leader -> can1, port 2/right follower -> can2, port 3/left follower
-> can3. With no controller running and all links down, the persistent naming
service was restarted under the exclusive lifecycle lock. Each resulting
logical interface passed 40/40 motion-free feedback rounds for all eight motor
IDs: 320 matching TX/RX frames per bus, ERROR-ACTIVE, txerr/rxerr zero, and
zero qdisc drops. All four links were returned DOWN. No camera was opened, and
no motor enable, disable, alignment, reset, or teleoperation constructor ran.
The CAN preflight requirement for a guarded teleoperation start is satisfied;
an actual start still requires an operator-cleared workspace because it causes
enable/alignment motion. All 148 hardware-free tests, Ruff, formatting, and
diff whitespace checks pass.

### 2026-09-17 loaded capture checks (07:26 UTC)

After the operator started both guarded sidecars, a 10 s full diagnostic
attempt at the configured 640x480/30 color mode failed before the gate:
both motor sides supplied fresh state and right video became ready, but left
USB 2 video timed out. Output is retained and marked failed at
`data/temporary/combined_check_20260917_072641`; no valid motor/video samples
were recorded. All four CAN links were ERROR-ACTIVE with zero live errors
before/after; the postcheck reported zero qdisc drops.

Without restarting controllers or changing cabling, a separate bounded direct
V4L2 diagnostic requested 424x240 YUYV at 15 FPS on both cameras. Both devices
reported the exact requested mode and simultaneously returned 45 color frames
each, left 15.02 FPS and right 15.01 FPS, with no read failure. Handles were
released; no images or motor data were saved in this low-load test. This
supports trying lower RGB load, not proof of a specific USB failure mechanism
or of a complete/sustained recording. Tracked recorder modes remain unchanged.
Next stage: operator choice of lower-quality capture, an explicit temporary
recording preset, and a full bounded motor-plus-video recording validation.
Both operator-started controllers were left running, with no enable, reset,
stop, or direct CAN commands issued by these checks.

### 2026-09-17 current CAN mapping applied (07:16 UTC)

On explicit operator request, and their confirmation that the four adapter
positions/arm cables were unchanged, CAN parents were updated from the old
upstream path to `1-3.2.3.1` as shown in the physical table. Tests were updated
without weakening identity validation. Under the exclusive lifecycle lock,
with no controllers and all links down, the enabled naming service was
restarted and restored can0 right leader, can1 left leader, can2 right
follower, can3 left follower. Each bus then passed 40/40 motion-free feedback
rounds for all eight IDs 0x011-0x018: 320 TX and 320 RX per bus, zero live or
cumulative CAN errors, packet drops, or qdisc drops. All four were returned
DOWN. No enable/disable, alignment, USB reset, or teleoperation was performed.
Existing launch manifests match; all 148 hardware-free tests, Ruff and diff
whitespace checks pass. Source/submodule revisions remain as recorded below.
Earlier notes about unresolved CAN mapping are superseded by this result.
Current temporary camera paths remain left `1-3.1`, right `2-2.4`; their
30 s camera-only pass below does not establish loaded coexistence. Next stage
is a fresh operator-cleared guarded teleoperation test, followed by a bounded
combined recording; do not claim simultaneous capture validated yet.

### 2026-09-17 RGB-only check after reconnect (07:07 UTC)

The operator reconnected the interfaces and requested capture without depth.
The temporary recorder already opens only the YUYV color node, not depth.
Current enumeration places left `260322278153` at USB `1-3.1` (480 Mbps) and
right `260422274280` at `2-2.4` (5 Gbps). The temporary camera config was
updated by serial identity. A concurrent 30 s RGB-only recording passed,
saving and decoding 902 left frames at 30.03 FPS and 901 right frames at
29.99 FPS, with no errors. Output is
`data/temporary/rgb_only_20260917_070708`; this contains no motor data.
No controllers were active and all CAN links stayed down; loaded coexistence
was not tested. All 148 hardware-free tests and Ruff passed.

Four PEAK adapters are present again, now under USB `1-3.2.3.1`, enumerated
can0-can3 in hub-port order 1-4. This is NOT the commissioned logical mapping.
Tracked CAN parents still describe the previous location, so do not launch
teleoperation before identity reconciliation/naming and a motion-free probe.
No CAN mapping, link, or motor changes were made during this RGB-only check.
Turning off depth cannot reduce the existing recorder load; a lower RGB mode
would be a separate explicit capture setting and requires its own validation.

### 2026-09-17 OTG wiring check (07:00 UTC, superseded)

The operator moved the two cameras to the ports labelled USB1/USB2 and the
CAN hub to OTG. Enumeration now places left serial `260322278153` at `1-3.1`
and right `260422274280` at `1-3.2`, both 480 Mbps under upstream hub `1-3`.
Each bounded direct V4L2 640x480 YUYV/30 check returned 90 frames at 30.06 FPS;
tool approval timing does not establish simultaneous capture in these checks.
No image files were saved; both handles were released. No controller or video
owner was active. The four PEAK adapters and their hub are absent from USB
enumeration; only onboard `can4` exists. The exposed USB role switch reads
`host`, but this does not prove the labelled connector/cable path is working.
No USB role, CAN, or configuration changes were made. Current tracked CAN
paths are absent and the temporary left-camera path is stale. Do not start
teleoperation or bypass identity checks. Next step is operator inspection of
the OTG-to-hub upstream data connection and power, then re-enumeration and
identity reconciliation before any motor probe or full recording.

### 2026-09-17 CAN hub relocation

After the operator confirmed unchanged hub-internal wiring, tracked CAN parents
and test fixtures were updated to the current physical table. The enabled
`xarm-can-rename.service` was restarted under the exclusive lifecycle lock,
with no controllers active and all four links down. It restored the original
logical right/left leader/follower names. Each endpoint then passed the tracked
40-round motion-free feedback probe for all eight response IDs: 320 TX/320 RX,
zero live/cumulative CAN errors, zero packet drops and zero qdisc drops. All
four links were returned DOWN; no enable, disable, alignment, USB reset, or
teleoperation constructor ran. The operator authorized mapping/probe work,
not a motion test. Controller scheduling and loaded coexistence were not tested.
Source remains `438bb696935a1f379cc0ffc1aafbc469a986a44c` with local changes;
submodules remain embodied-ops `6eeb9e04b05cd59f8854efd1cb08572e08e5a4fa`
and x-air-sdk `bf300508e179f652b23f0efaf3b6c9048f1f12e9`.
Both existing launch manifests match the updated configuration. Hardware-free
checks pass all 148 tests plus Ruff. Temporary recording camera paths now match
left USB `2-1.4` and right USB `1-3.2`, preserving the verified serial roles.
Earlier camera-path notes below are historical, not the current configuration.
At 06:50 UTC, a 10 s camera-only recorder check with the CAN hub connected
but all links down saved and decoded 301 frames per camera: left 29.99 FPS,
right 30.03 FPS, no errors. Output is retained at
`data/temporary/rewired_cameras_20260917_065052`. Camera handles were released.
The subsequent operator-started full recording failed: both motor sides and
left video became ready, but right video timed out before the recording gate.
At 06:54 UTC all CAN controllers were ERROR-ACTIVE with zero live errors and
zero qdisc drops, while kernel logs repeatedly reported PEAK USB fragmented
receive records. Right video and all CAN adapters share upstream USB hub
`1-3`; this is evidence to investigate shared-path operation, not proof of
a particular bandwidth, power, or driver cause.

At 06:56 UTC the operator stopped both teleoperation processes. Read-only
checks confirmed no remaining sidecar and all four CAN links DOWN/STOPPED
with zero live errors. With unchanged USB topology and no reset/replug, the
10 s camera-only check again passed: 301 saved/decoded frames per camera,
left 29.99 FPS and right 30.03 FPS, no errors. Files are retained at
`data/temporary/after_teleop_stop_20260917_065647`. No CAN commands were sent
by this check, no arms were enabled, and camera handles were released.
The configuration is therefore not validated for simultaneous teleoperation
and cameras. Next stage: isolate the right camera from the shared USB 2 path
and reverify identity/camera operation before an operator-cleared loaded test.
The naming service is a boot/explicit-start oneshot, not a hotplug watcher.

### 2026-09-17 temporary two-camera recorder

`vlai_l1_runtime.temporary_recording` and `configs/recording/two_wrist.json`
provide an operator-requested standalone CSV/MJPEG recording mode without
AgentView. See `docs/TEMPORARY_RECORDING.md`. It verifies camera serial/USB
path identity, owns two otherwise-unused cameras, and receives existing
sidecar states over the normal state socket without opening CAN. It never
enables, resets, or stops arms. All 32 leader/follower motor positions are
stored across two per-side CSVs with source/receive timestamps; independent
video timestamps are host read-completion times, not hardware synchronization.
Existing output directories or occupied state sockets are not overwritten.

Hardware-free tests passed 148 cases, including synthetic two-side recording,
single-side state loss, packet validity, and resource/old-data preservation;
Ruff and diff whitespace checks passed. The first real camera test failed
because left UVC returned no frames; an isolated left capture also timed out.
The operator replugged only the left camera. The subsequent explicit
camera-only check saved and decoded 301 frames from each camera over 10 s,
left 30.03 FPS and right 29.99 FPS, with no errors. Its files are retained at
`data/temporary/camera_check_20260917_02`; the failed `_01` directory is also
retained and marked failed. These are camera-only diagnostics, not all-motor
experiments. A separate 3 s real-state check received 301 left packets but
zero right packets, so full real two-side-plus-camera recording has not been
validated. No arms were started or restarted by these checks. The next full
recording needs both operator-started sidecars supplying fresh state.

At 06:37 UTC, after another operator camera reconnect, the bounded full check
in `data/temporary/replug_check_20260917_063755` failed before its recording
gate: both motor sides supplied state and the right camera became ready, but
left `/dev/video10` timed out without a frame. An isolated left-only V4L2
640x480 YUYV/30 read also opened successfully but timed out after 10 s. Thus
the failure is not limited to simultaneous two-camera capture. Left remains
on the multi-hub 480 Mbps path; right is on 5 Gbps. The exact USB/device cause
is unconfirmed. All four CAN links were ERROR-ACTIVE with zero live error
counters at the precheck; existing operator-started controllers were left
untouched. No CAN commands, USB resets, or arm starts/stops were performed.
The failed diagnostic files are retained. Next stage: inspect/change only the
left camera cable/USB path with the system safely stopped, reverify identity
if its port changes, then repeat a bounded camera check before full recording.

At approximately 06:40 UTC the operator swapped the camera ports. Serial
enumeration now places left `260322278153` on USB `2-1.4` (`/dev/video4`,
5 Gbps), and right `260422274280` on `1-3.1.3.1` (`/dev/video10`, 480 Mbps).
With no preexisting video owners, concurrent bounded direct V4L2 checks at
640x480 YUYV/30 returned 90 left frames at 30.02 FPS; right opened but its
first read timed out after 10 s. Both handles were released, no images saved,
and CAN/controllers were untouched. Failure followed the USB connection path,
not the left camera identity; the particular hub/cable/power/driver cause is
not isolated. The temporary recording config still describes the earlier
ports and will reject this swapped wiring. Do not bypass its identity check;
reconcile paths after the operator settles a working connection for both.

At approximately 06:43 UTC the operator temporarily unplugged the CAN hub
and connected the right camera in its place. Enumeration confirmed right
serial `260422274280` at USB `1-3.2` (`/dev/video10`, still 480 Mbps) and left
`260322278153` unchanged at `2-1.4` (`/dev/video4`, 5 Gbps). No sidecar or
video owner was found. Concurrent direct 640x480 YUYV/30 tests returned 90
frames per camera, left 30.02 FPS and right 30.05 FPS, without read failures.
No image files or motor data were recorded; handles were released. This is a
short camera-only pass, not evidence for sustained capture or CAN/camera
coexistence. USB 2 speed alone cannot explain the preceding timeout. Final
wiring and temporary-recorder paths remain to be reconciled; reconnecting CAN
requires mapping/health checks before any motion-capable startup.

### 2026-09-17 J5/J6 stop-threshold restoration

At the operator's request, the temporary 0.1 degree expansion was reverted.
Both sides again use the original J5 +/-90 and J6 +/-45 degree stop
thresholds. The local launch manifest and test expectations match these
values. No controller restart or motion test was performed for the change
or its reversal; a running controller retains its launch-time settings.

### 2026-09-17 Jetson reconnect verification

At approximately 05:46-05:49 UTC, the operator restored the USB connections
and requested CAN and camera checks. The four PEAK USB parents again matched
tracked configuration, but hotplug had assigned enumeration-order CAN names.
Re-running the existing enabled `xarm-can-rename.service` succeeded and restored
the configured mapping. The service is oneshot, not a hotplug watcher; future
unplug/replug operations still require identity verification. The unused onboard
`mttcan` is currently `can4`, not `canboard0`.

With no controller running and an exclusive lifecycle lock held, each configured
CAN endpoint passed all 40 motion-free rounds for all eight response IDs. Each
started with zero packet/error counters and finished with 320 TX and 320 RX
frames, zero packet errors/drops, zero qdisc-drop increase, and ERROR-ACTIVE
with zero live error counters. Each endpoint was returned DOWN/STOPPED; no
enable, alignment, or controller scheduling test was performed. Source revision
was `438bb696935a1f379cc0ffc1aafbc469a986a44c` with existing local changes;
submodules were embodied-ops `6eeb9e04b05cd59f8854efd1cb08572e08e5a4fa`
and x-air-sdk `bf300508e179f652b23f0efaf3b6c9048f1f12e9`.

Both D405 cameras concurrently returned 300 RGB uint8 frames each at 640x480,
configured 30 FPS: left measured 30.03 FPS (maximum read interval 34.08 ms),
right 29.99 FPS (33.98 ms). No read failed; both handles were released. Pair
skew and sustained teleoperation-plus-camera load were not measured. Kernel
logs over the checks showed no USB disconnect or CAN/UVC fault.
Operator unplug identification established left RealSense serial `260322278153`
(ASIC `255323074499`, USB 2 path) and right `260422274280`
(ASIC `255323074436`, USB 3 path). This supersedes the older camera role table
above for this host. Runtime camera configuration is still unchanged and has
the wrist identities reversed; left also lacks a serial-bearing V4L by-id link,
and the configured AgentView is absent. Successful direct camera checks do not
establish collection readiness. The next camera stage is to reconcile stable
identities and runtime roles before a formal collection camera check.

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
releases both SDK constructors, waits for fresh paired state, and explicitly
runs the tracked pre-collection Reset on the active handles. Then use
teleoperation to refine the episode start pose. Enter `r` or use Reset to
repeat alignment on the active handles if needed, then press Enter in the
terminal or Start recording in the Panel. The workflow rechecks all cameras
after confirmation, buffers the stationary prefix until sustained action
motion, keeps the configured preroll, and records until Enter, discard, or
quit. Save/discard
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
