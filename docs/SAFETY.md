# Safety

The managed `collect` CLI and Operator Panel workflow start the commissioned
x_air teleoperation sidecars and therefore cause immediate enable/alignment
motion. Policy-command transport remains unavailable.

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
System schema version 2 rejects an implemented transport or `command_ready = true`, so
enabling commands requires a deliberate schema and code review rather than a
single configuration edit.

The collection loader independently requires a commissioned teleoperation path
and the configured camera identities. Both wrist identities, AgentView, and
teleoperation are commissioned, so the managed collection action is exposed.
This does not satisfy the independent policy-command blockers. Synthetic
sources exist only for pure integration tests and cannot enable the Runtime.

## Live-work rules

- Never run zeroing, calibration, gravity compensation, MIT motion, or torque
  commands without an explicit request and a clear physical workspace.
- Treat `xarm_teleop_create_unilateral` as a motion operation: it initializes
  and enables motors and calls position alignment before `xarm_teleop_start`.
  It cannot be used for a read-only J2 check.
- Recommission the sidecar one side at a time. Stop any existing teleoperation
  service for that side first, and never let two controller processes own the
  same CAN endpoint concurrently.
- The commissioned migrated-host path is `just sdk-start <left|right>`. It
  renders hardware values from tracked config, permits only disjoint CAN pairs
  to run together, and owns disable/down cleanup for its selected pair.
- Stop the deployed teleoperation services before any direct CAN investigation.
- A read that enables motors or emits zero-torque frames is not read-only.
- The episode-start prompt is an operator gate, not an automatic reset. Use
  teleoperation to place the robot at the intended start pose and keep the
  workspace clear before pressing Enter.
- A command timeout must disable command resources; retaining a stale publisher
  while accepting a new lease is process-fatal.
- Preview, logging, policy inference, and camera encoding must never block the
  realtime command loop.
- State publication runs outside the SDK callback. Missing consumers are
  tolerated; malformed, non-finite, wrong-DOF, or stale callback data is fatal.
- The sidecar caps and verifies every SDK thread at the tracked FIFO priority.
  It snapshots both CAN links before creation and stops on any live error
  counter or cumulative warning, passive, bus-off, bus-error, arbitration-loss,
  or restart increase.
- CAN health includes qdisc drops from `tc -s qdisc`; ordinary interface
  counters do not expose the queue drops that caused the original stutter.
- The Operator Panel currently binds to the trusted robot LAN and has no user
  authentication or transport encryption. Do not expose it to an untrusted
  network.

Dataset doctor and v2.1 export read local artifacts only. They must not be given
device paths, and neither operation authorizes deletion or replacement of a
canonical dataset. Export publishes a new derivative and refuses an existing
target.
