# Safety

The managed `collect` CLI and Operator Panel workflow start the commissioned
x_air teleoperation sidecars. Each CAN endpoint must first pass the configured
motion-free motor-feedback probe; successful sidecar creation then causes
immediate enable/alignment motion. Policy-command transport remains unavailable.

## Command blockers

- The selected x_air release still contains prebuilt Control, Dynamics,
  JointMapper, and CAN libraries; its public wrapper is not complete controller
  source.
- J2 motor, URDF, and control coordinates have not been reconciled on all arms.
- The previous `can2` bus-off event has not been closed out under sustained load.
- No reviewed Runtime adapter implements command-resource disable and release.

The tracked configuration records these gates separately and sets
`command_ready = false`. A future change may enable commands only when every gate
is supported by evidence, corresponding tests, and a reviewed live adapter.
System schema version 4 rejects an implemented transport or `command_ready = true`, so
enabling commands requires a deliberate schema and code review rather than a
single configuration edit.

The collection loader independently requires a commissioned teleoperation path
and the configured camera identities. Both wrist identities and AgentView
remain commissioned. Teleoperation was recommissioned after a complete
motor-power restart restored both right buses and a bounded protected observer
window passed. This does not affect the independent policy-command blockers.
Synthetic sources exist only for pure integration tests and cannot enable the
Runtime.

Collection now requires a valid HKVL serial FT stream before creating a
motion-capable teleoperation owner. The collector records sensor-frame wrench
with host-receipt time alignment; an absent or stale stream stops collection.
These readings are observations only and are not a force-control or safety-stop
input. They have no verified tool-gravity compensation or sensor-to-tool
coordinate transform.

## Operator-authorized automatic diagnostic (2026-10-11)

The operator explicitly waived the complete-controller-source prerequisite for
a bounded experimental right-Follower owner and authorized the corresponding
AGENTS.md exception. `automatic-right` is this separate live diagnostic; it
does not implement production policy transport or change `command_ready`.
It uses the public motor CAN SDK with an exclusive lifecycle lock and no
teleoperation owner running. Only the configured right Follower is opened.
Physical identities, motor types, gains, firmware ranges, joint bounds and
system timeouts come from system configuration. The additional experimental
envelope is tracked in `configs/diagnostics/right_automatic_motion.json`.

The launcher requires stopped, correctly mapped interfaces, zero CAN errors,
and every motor in every motion-free preflight round. It never recovers a bus
or retries enable automatically. The native owner holds a fresh measured pose,
then optionally performs a bounded single-joint sweep or explicit closed
multi-joint waypoint path, always with the gripper reference fixed. It checks feedback age/status, bounds, relative excursion, measured
speed, torque, temperature and persistent tracking error. Its tracking action
is stop, separate from the unchanged teleoperation warning exception.
The parent supervises heartbeat/CAN health; the child handles parent death.
Shutdown sends disable and requires fresh disabled replies from all eight
motors before claiming success. If the child exits without confirmation, only
one sequential disable-only cleanup owner may run after it has been reaped;
no enable or target command is allowed in that cleanup mode. The link is then
lowered, and any unconfirmed disable is reported as requiring intervention.

For legacy single-joint policies, the selected joint is explicit in version-3 native input.
Only J4, J6 or J7 can be selected; gripper and other joints cannot be selected.
Native and offline checks use that same selected index for trajectory bounds
and fixed-other-target checks. New J4/J6 commissioning policies use +/-0.25 and
+/-0.5 degrees respectively with 8-second transitions, unchanged PD gains and
guards. Run the single-cycle policy before its repeated counterpart; a failed
stage stops the sequence without retry. J7-only baseline studies reject J4/J6
captures. Selecting a joint does not establish model-coordinate calibration.

Following the operator's 2026-10-12 request for larger coordinated unloaded
motion, `isolated_right_follower_waypoints_v1` uses `active_motor = "arm"`,
exact named eight-motor `waypoints_deg` offsets, and native V4 input. V3 retains
its existing single-joint restriction. V4 has 3..64 points, first and last all
zero, at least two moving arm joints, and zero gripper offset everywhere.
`amplitude_deg` is the absolute per-axis offset cap for this policy kind.
Each offset also obeys its motor's explicit excursion budget. Every segment
uses a synchronized monotonic quintic blend with zero endpoint velocity and
acceleration. Python and native validation check EACH inter-point displacement
against analytic peak speed/acceleration, including direction reversals.
Native startup checks every waypoint relative to the fresh measured reference
before enable; monotonic interpolation then stays inside those joint bounds.
The reviewer verifies each saved target against the entire scheduled path,
checks measured bounds/excursions and labels holds/repeated cycles by waypoint.
V4 rejects paired/manual handling in both launcher and native core. Existing
single-joint model analyses do not accept this multi-joint controller trajectory.

`right_multijoint_unloaded_single.json` defines the experimental path:
six coordinated postures plus return, 40-second transitions, 5-second holds,
320-second path following the passed five-second hold in `sequence` mode.
Offset ranges J2 [0,10], J3 [-10,10], J4 [0,16], J5/J6/J7 [-10,10] degrees;
J1 and gripper stay fixed. Reference peak speed <=0.9375 deg/s and acceleration
<=0.07217 deg/s^2. The new path cap is 16 degrees; measured excursion budgets
are [3,12,12,18,12,12,12,3] degrees. Gains, tracking, torque,
temperature, CAN/freshness, supervision and disable checks are unchanged.
Joint-bounded motion does NOT establish Cartesian/table/cable clearance or
calibrated model coordinates. Require current unloaded/clear-space context
before commissioning; a fault stops the sequence without retry. No new
hardware motion has been executed by the software extension itself.

First live attempt on 2026-10-12: the hold passed, but the coordinated sweep
stopped about 15.36 seconds into its first segment at J5 reported speed
15.5307 deg/s against the unchanged 15 deg/s hard guard. Adjacent position
feedback changed 0.17486 deg in 10.138 ms; this cannot be dismissed as speed-field
noise alone. Operator reported no visible abnormality/contact. All eight
disabled replies were verified, links lowered with zero CAN errors, no retry.
The path remains NOT commissioned. Only one enabled exceedance was recorded
before disable, which does not establish its duration under continued control.
The paired manual-handling exception does not authorize transient overspeed
during this automatic path. Detailed hashes/diagnosis are in HANDOFF.md.

After that failure, the operator explicitly requested 45 deg/s instead of the
15 deg/s stop threshold. The current multi-joint unloaded policy therefore sets
`maximum_observed_speed_deg_s = [45,45,45,45,45,45,45,15]`: the seven arm joints
use an immediate 45 deg/s hard stop; the fixed gripper retains 15 deg/s. This is
a new experimental automatic-motion setting, NOT the paired manual-handling
exception and not a validated manufacturer safety rating. No persistence filter
was added. Reference speed remains <=0.9375 deg/s with identical waypoints and
timing; all other guards/gains are unchanged. Legacy single-joint and paired
policies are unchanged. The historical failed capture retains its embedded
15 deg/s policy and FAILED status. The changed threshold has not been live
commissioned; the configuration edit itself does not restart hardware.

The operator then explicitly started a new run with the 45 deg/s setting.
On 2026-10-12 at 08:25:51 UTC it ended FAILED: hold passed, but the sweep
triggered persistent J7 tracking error at ~22.65 seconds (0.8 deg for 0.1 s;
last saved error 0.844 deg). No revised speed threshold was exceeded; enabled
speed peak was 23.9257 deg/s on J5. All eight disabled replies and clean CAN
shutdown were verified, no automatic retry. This leaves the full path
uncommissioned; it is not authorization to relax tracking. See HANDOFF.md for
source hashes and the separately labelled offline failure diagnosis.

After the J7 tracking stop the operator explicitly requested raising that
threshold. The current multi-joint policy sets J7 tracking tolerance to 2 deg,
matching the other motors, with the same 0.1-second persistence. This replaces
the experimental 0.8-deg J7 setting only for future runs of this policy.
Speed caps remain [45,45,45,45,45,45,45,15] deg/s; trajectory, gains, bounds,
excursion, effort, temperature, communication and disable checks are unchanged.
Native tracking still latches a failure on sustained error above 2 deg.
The setting was software-verified before the full run recorded below. The earlier
failed recording keeps its original 0.8-deg policy and FAILED status.

On explicit operator instruction the 45 deg/s / 2-deg version completed its
first full sequence at 2026-10-12 08:50:54 UTC. Hold and all 320 seconds of the
path passed preflight, live guards, all-eight verified disable and offline
review. Operator confirmed the entire run had no contact or abnormality.
Maximum tracking error 1.39855 deg; maximum enabled speed 22.24671 deg/s.
All CAN links returned DOWN with zero errors or qdisc drops. This commissions
that bounded acquisition from the recorded reference and scene; it does not
establish arbitrary-start collision clearance, manufacturer safety ratings,
production command readiness, or FT accuracy. Native startup must still check
fresh reference/whole-path bounds and all preflight gates on every later run.
The earlier failed runs retain their original outcomes and thresholds.

`right_multijoint_unloaded_reverse.json` reverses only the commissioned path's
waypoint order for independent unloaded model validation. All timing, amplitude,
gains and guards are identical. It traverses the same offset-space segments in
reverse, starting from fresh feedback; absolute posture may differ after disable.
The usual whole-path bounds, current clear/unloaded scene, motion-free preflight,
five-second hold, independent supervision and verified disable still apply.
Freeze the forward model before collection; evaluate coverage using actual
measured positions, never assume reversal guarantees matching absolute poses.

`right_multijoint_unloaded_balanced.json` concatenates the commissioned forward
and reverse paths into 15 waypoints / 635 seconds under one fresh reference.
The middle zero waypoint is an ordinary five-second hold, with no disable or
re-enable between directions. All per-segment timing, gains, thresholds, offset
envelope and raw capacity (600,000 frames) stay unchanged. The 508,000 nominal
feedback frames plus the ten-second reserve fit that capacity. The usual
five-second commissioning hold and preflight precede this bounded sweep.
This is training acquisition, not an independently validated torque model.

`right_multijoint_unloaded_validation75.json` uses 75% of the commissioned
forward offsets with the same 320-second schedule and unchanged guards/gains.
It was chosen before fitting the balanced data. Six-joint reference spans are
7.5/15/12/15/15/15 degrees, peak reference speed 0.703125 deg/s. It is still a
fresh-reference, whole-path-bound-checked acquisition, not a Cartesian clearance
certificate. Freeze the balanced models before starting this independent run;
evaluate measured coverage without re-centering angles or widening support.

Initial commissioning is a five-second current-position hold, followed only
on success by a small wrist sweep. This PD controller has no model gravity or
friction feedforward; its captures are a different controller context from
the earlier opaque teleoperation controller. Raw-motor coordinates and these
experimental guard limits are not calibrated FT or collision avoidance.

The experimental `--mode hold` owner also supports an explicit normal finish:
create `finish.request` in that capture's output directory containing exactly
`finish` followed by one newline. Only after the native owner reports running
does the supervisor send its hold-completion signal. Completion waits until
the tracked `hold_seconds` minimum, preserves all guards, and verifies disable
before marking success. SIGINT/SIGTERM, faults and failed cleanup remain failed
runs; a finish request cannot reclassify them as successful. Sweeps ignore this
request. The dedicated `right_automatic_operator_hold.json` bounds a stationary
experiment to 600 seconds with full raw retention; it is not indefinite holding.
Use it with `--mode hold`, not `sequence`. A request ends the session and never
causes a new movement or enable. A requested shorter hold must carry matching
request/completion/duration evidence to pass offline review.

After the native owner closes its SDK and emits verified `disabled`, the parent
allows buffered file output within the tracked teleoperation shutdown timeout,
measured once from the first disabled marker. Subsequent output cannot extend
that deadline. Before verified disable, the normal live heartbeat applies.
Finalization timeout, nonzero native exit, operator stop or health failure still
fail the run; the child is reaped before link-down or any sequential cleanup.

For a same-session load experiment, establish a stationary running hold before
requesting manual attachment. Label attachment/removal transitions separately;
accept measurement windows only after the operator confirms hands clear and
the measurements have settled. Keep gripper opening and reference fixed. Do
not infer force from an unlabelled window, fit load response into an unloaded
baseline, or claim that holding a fixed reference guarantees an identical pose.

The experimental `--mode paired` runs three sweeps with one owner and one fresh
reference: baseline, stationary attach wait, middle sweep, stationary remove
wait, final sweep. `right_joint4_paired.json` caps the ENTIRE session at 600
seconds, including both waits and settling. Each sweep has two +/-0.5-degree J4
cycles. Explicit `attach.ready` (`attached\n`) and `remove.ready`
(`removed\n`) files advance only their corresponding waiting phase; premature
or malformed files fail closed. There must be enough time remaining for every
remaining sweep. No phase auto-retries or releases/re-enables the motors.

Following the operator's request after the manual-attachment J5 stop, the
paired policy now has explicit `manual_handling` settings. These are software
validated experimental candidates, NOT commissioned hardware limits. Only
stationary awaiting/settling phases use this policy. J5/J6/J7 have an immediate
45 deg/s ceiling; the other five motors retain 15 deg/s. Above the ordinary
15 deg/s limit, a continuous 0.1-second excursion still stops. Automatic sweeps,
including their scheduled holds, retain immediate 15 deg/s protection. No
position bound, relative excursion, torque, temperature, tracking, freshness,
CAN health, controller gain or disable requirement changes. Target must equal
the initial eight-motor reference whenever manual handling is active.

Acknowledgement enters `settling_loaded` or `settling_empty_after`, never
immediate motion. All eight positions must remain within a 0.05-degree span
and reported speeds within 5 deg/s for two continuous seconds AFTER that
acknowledgement. Renewed motion resets the dwell; 20 seconds without settling
fails and disables. Global cap remains 600 seconds and sufficient time for
remaining sweeps plus worst-case settling must remain at each acknowledgement.
Settling data are excluded from training/load evaluation. Native V3 explicitly
encodes presence/absence and every handling value; V1/V2 input is rejected.
Policies without handling preserve the old strict behavior and phase sequence;
handling policies are rejected for non-paired motion modes. Raw speed-exceedance
counts in offline review still use the ordinary 15 deg/s threshold, including
manual periods, and are not silently erased by this exception. Never relabel
the earlier failed experiment as successful using the new policy.
An all-unloaded commissioning run may explicitly advance both waits without
manual actions, but must record `middle_stage_loaded=false`; phase names and
advance tokens alone are never evidence that an object was attached. Paired
analysis requires hash-bound stage context, fits only initial unloaded data,
and evaluates subsequent stages without refitting. It does not claim a model
was frozen before acquisition, calibrated FT, or runtime eligibility.

The separate `right_joint4_threequarter_paired_long_wait.json` retains the
+/-0.75-degree trajectory and all handling/motion guards but gives manual
attachment/removal a 1200-second total session cap and 1,000,000-frame raw
capacity. It is a prepared duration extension, not yet live commissioned.
Acknowledgements still require enough time for remaining sweeps and worst-case
settling; there is no indefinite hold or automatic restart. Existing 600-second
policies and historical capture policies remain unchanged. A stopped partial
capture may receive a separately labelled exploratory diagnosis, never a passed
paired review or fabricated missing phase.

`review-automatic --recheck-offline-failure` is a pure data recheck after an
analysis bug fix. It requires a zero native exit, verified disable, no cleanup
error, and an original failure explicitly originating in offline review. Every
integrity check still runs. It preserves the original failed report and writes
a separate result bound to that report's hash. It cannot reclassify a live
guard/stop/cleanup failure or trigger hardware operations. Paired analysis has
the same explicit option and records the recheck in its output.

`trajectory.WristSweep` and `TrajectoryRehearsal` are pure offline planning and
fault models. They contain no SDK/transport/lifecycle adapter and do not clear
command readiness. Their explicit experimental speed, acceleration and tracking
budgets are not commissioned motor limits. Rehearsal `RELEASE_REQUIRED` means
no further simulated targets; its caller-supplied disable/release acknowledgement
does not operate or observe physical motors. Real watchdog scheduling, exclusive
command ownership and verified disable/release remain implementation gates.

## Live-work rules

- Never run zeroing, calibration, gravity compensation, MIT motion, or torque
  commands without an explicit request and a clear physical workspace.
- Treat `xarm_teleop_create_unilateral` as a motion operation: it initializes
  and enables motors and calls position alignment before `xarm_teleop_start`.
  It cannot be used for a read-only J2 check.
- The unilateral controller is joint-space teleoperation, not Cartesian IK.
  The reviewed wrapper copies the seven leader joint positions directly to the
  follower reference, and the pinned JointMapper implementation is identity.
  An obstructed or mechanically unreachable follower pose therefore does not
  have an IK rejection boundary; without an outer guard the controller can
  keep commanding the blocked joint.
- The guarded lifecycle must receive every configured motor response ID on
  every round of the tracked preflight window before calling
  `xarm_teleop_create_unilateral`. This probe may query state but never calls
  enable, disable, MIT control, gripper control, zeroing, or alignment APIs.
- Recommission the sidecar one side at a time. Stop any existing teleoperation
  service for that side first, and never let two controller processes own the
  same CAN endpoint concurrently.
- The commissioned migrated-host path is `just sdk-start <left|right>`. It
  renders hardware values from tracked config, permits only disjoint CAN pairs
  to run together, and owns disable/down cleanup for its selected pair.
- `just sdk-start-right-only` is the isolated fallback while the left CAN path
  is unavailable. It opens only configured right endpoints `can0` and `can2`,
  requires both left endpoints to remain `DOWN`, and holds an exclusive
  lifecycle lock so another Runtime side cannot start concurrently. Use only
  `collect-right`, `reset-right`, or `panel-right` for managed right-side
  workflows; the default recipes remain intentionally bimanual.
- De-energize the affected arm before disconnecting or reconnecting its CAN
  cable. A disconnected left CAN cable is not permission to weaken the
  right-only interlock.
- Stop the deployed teleoperation services before any direct CAN investigation.
- A read that enables motors or emits zero-torque frames is not read-only.
- Starting a managed collection creates exactly the configured x_air runtimes
  once. After fresh state resumes, collection explicitly runs `AdjustPosition`
  before the first episode gate. Save and discard stop accepting frames, run
  `AdjustPosition` on the
  active handles, and only then encode or discard the episode;
  `r` or the Panel's Reset input can run it again at the episode-start gate.
  Keep the workspace clear for either motion and use teleoperation to refine
  the intended start pose before pressing Enter.
- Standalone Reset and the Panel Reset workflow start only the guarded selected
  teleoperation lifecycle, require fresh selected state after startup
  alignment, and then disable the selected pair(s). They do not authorize
  policy commands.
- The lifecycle child owns disable/down cleanup. Managed shutdown checks its
  exit status and passively verifies both owned links are DOWN; it does not
  invoke a second legacy controller or disable script. A cleanup failure or
  forced process termination is a failed shutdown, not proof of motor disable,
  and must not be followed by automatic restart.
- A command timeout must disable command resources; retaining a stale publisher
  while accepting a new lease is process-fatal.
- Preview, logging, policy inference, and camera encoding must never block the
  realtime command loop.
- The marked persistent Camera Service is the only physical camera owner.
  MJPEG and raw collection clients read its latest frames; neither may reopen a
  device or advance another consumer's delivery sequence.
- Collection shutdown leaves the read-only Camera Service running. Use
  `just cameras stop` to release its handles. Never start a second camera reader
  while the marked owner is active.
- State publication runs outside the SDK callback. Missing consumers are
  tolerated; malformed, non-finite, wrong-DOF, or stale callback data is fatal.
- Optional passive Follower logging runs inside the SDK-owning sidecar and
  never sends CAN frames. It requires explicit output/capacity arguments;
  default launches leave it disabled. Raw samples are buffered with kernel
  receive timestamps and socket overflow counts, then written after SDK handle
  release. Capture faults stop that diagnostic run. Full retention (default)
  fails when capacity is exhausted. Explicit `--feedback-retention latest`
  instead retains the most recent bounded window and records the number of
  evicted older frames; this is intentional history eviction, never permission
  to ignore socket overflow, stale/missing feedback or CAN faults. It permits
  operator-ended observation without a diagnostic duration limit. A raw capture, observed
  status nibble, or same-pose offline tare is not validated FT or permission for
  force control. Do not run the standalone querying capture beside a sidecar.
- The native sidecar also applies the tracked side-specific seven-joint bounds
  to both leader and follower feedback. Any bound violation is immediately
  fatal. Per the operator's explicit decision, a leader/follower error that
  remains over its per-joint limit past the configured persistence timeout
  identifies the exact side and joint and emits one nonfatal warning per
  excursion; it does not stop the session. These checks use increasing callback
  timestamps and are reset only after a successful `AdjustPosition`.
  Most tracked bounds conservatively intersect the rendered side-specific URDF
  ranges with narrower limit constants found in the pinned release because the
  opaque controller could not be shown to enforce either set. On 2026-10-19 the
  operator explicitly changed left J6's outer feedback stop to +/-50 deg, then
  requested the same change for right J6. Both exceed the pinned URDF's +/-45 deg.
  These are uncommissioned guard expansions,
  not evidence that the joint has additional mechanical travel or that the
  opaque controller will accept it. Do not use either expanded region before
  physical-range/coordinate verification. The left side also needs a clean
  motion-free bus probe after the reported CAN warning/passive event.
- The sidecar caps and verifies every SDK thread at the tracked FIFO priority.
  It snapshots both CAN links before creation and stops on any live error
  counter or cumulative warning, passive, bus-off, bus-error, arbitration-loss,
  or restart increase.
- Before a sidecar reports ready, the outer lifecycle also watches live CAN
  controller state. Before the opaque SDK can enable motors, it uses the
  low-level CAN SDK without `enable_all` to require repeated feedback from all
  eight configured motor IDs on the leader and then the follower. Startup CAN
  faults fail closed without resetting an adapter or retrying. Managed
  bimanual startup holds both processes before the motion-capable SDK
  constructor and releases them only after both sides pass the complete
  motion-free preflight. Once released, CAN faults always stop the session.
- CAN health includes qdisc drops from `tc -s qdisc`; ordinary interface
  counters do not expose the queue drops that caused the original stutter.
- A clean PCAN reset followed by no ACK from all eight configured motor nodes
  is not an adapter-state recovery case. Keep the link down and inspect the
  motor-bus supply/protection, transceiver, and arm harness with power removed;
  do not loop USB resets or motion-free queries.
- The Operator Panel and persistent MJPEG preview bind to the trusted
  robot LAN and have no user authentication or transport encryption. Do not
  expose either endpoint to an untrusted network.

Dataset doctor and v2.1 export read local artifacts only. They must not be given
device paths, and neither operation authorizes deletion or replacement of a
canonical dataset. Export publishes a new derivative and refuses an existing
target.
