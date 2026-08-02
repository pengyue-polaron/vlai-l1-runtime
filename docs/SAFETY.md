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
  once. Save and discard stop accepting frames, run `AdjustPosition` on the
  active handles, and only then encode or discard the episode;
  `r` or the Panel's Reset input can run it again at the episode-start gate.
  Keep the workspace clear for either motion and use teleoperation to refine
  the intended start pose before pressing Enter.
- Standalone Reset and the Panel Reset workflow start only the guarded selected
  teleoperation lifecycle, require fresh selected state after startup
  alignment, and then disable the selected pair(s). They do not authorize
  policy commands.
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
- The native sidecar also applies the tracked side-specific seven-joint bounds
  to both leader and follower feedback. Any bound violation is immediately
  fatal. Per the operator's explicit decision, a leader/follower error that
  remains over its per-joint limit past the configured persistence timeout
  identifies the exact side and joint and emits one nonfatal warning per
  excursion; it does not stop the session. These checks use increasing callback
  timestamps and are reset only after a successful `AdjustPosition`.
  The deployed bounds conservatively intersect the rendered side-specific URDF
  ranges with narrower limit constants found in the pinned release because the
  opaque controller could not be shown to enforce either set.
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
