# Safety

The current repository is intentionally incapable of motion.

## Command blockers

- The production unilateral C++ source is not present in the onboard snapshot.
- J2 motor, URDF, and control coordinates have not been reconciled on all arms.
- The previous `can2` bus-off event has not been closed out under sustained load.
- The provisional joint limits have not been commissioned on this physical unit.
- No reviewed Runtime adapter implements command-resource disable and release.

The tracked configuration records these gates separately and sets
`command_ready = false`. A future change may enable commands only when every gate
is supported by evidence, corresponding tests, and a reviewed live adapter.
Schema version 1 rejects an implemented transport or `command_ready = true`, so
enabling commands requires a deliberate schema and code review rather than a
single configuration edit.

The collection loader additionally requires both configured camera identities.
The current config leaves them disabled, so `collection_ready` is false and the
Operator Panel exposes no live collection or camera action. Synthetic sources
exist only for pure integration tests and cannot enable the Runtime.

## Live-work rules

- Never run zeroing, calibration, gravity compensation, MIT motion, or torque
  commands without an explicit request and a clear physical workspace.
- The deployed systemd services and their stop hooks remain the only approved
  live teleoperation path until superseded deliberately.
- Stop the deployed teleoperation services before any direct CAN investigation.
- A read that enables motors or emits zero-torque frames is not read-only.
- A command timeout must disable command resources; retaining a stale publisher
  while accepting a new lease is process-fatal.
- Preview, logging, policy inference, and camera encoding must never block the
  realtime command loop.
- The Operator Panel currently binds to the trusted robot LAN and has no user
  authentication or transport encryption. Do not expose it to an untrusted
  network.

Dataset doctor and v2.1 export read local artifacts only. They must not be given
device paths, and neither operation authorizes deletion or replacement of a
canonical dataset. Export publishes a new derivative and refuses an existing
target.
