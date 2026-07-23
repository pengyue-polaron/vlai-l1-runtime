# Architecture

The intended dependency direction is:

```text
VLAI workflows / embodied-ops adapter
  -> LeRobot Robot and Teleoperator clients
  -> VLAI L1 Runtime transport
  -> command lease and safety gates
  -> realtime C++ device owners
  -> CAN and cameras
```

The pure configuration, public contracts, command-session state machine, and
hardware-independent collection/dataset layer exist today. The teleoperation
observation path is commissioned, while the independent policy-command
transport remains unimplemented.

## Teleoperation observation path

The repository pins `x_air_sdk` by Git submodule revision. One native sidecar
per side calls only its public unilateral C API and is the sole owner of that
leader/follower CAN pair. It intentionally does not use the upstream ROS2 node:
that node opens an additional leader-gripper CAN handle and exposes an
unscoped motion service.

The x_air callback supplies seven arm joints plus one gripper for the leader and
follower in radians. The callback only copies finite values into a bounded
slot. A non-realtime publisher thread emits a 152-byte, little-endian Unix
datagram containing protocol version, side, source sequence, monotonic
timestamp, and both eight-value vectors. A stale callback, wrong DOF, or
non-finite value stops the sidecar process. Missing datagram consumers never
block the control loop.

The Python adapter accepts only exact protocol packets, requires increasing
per-side sequences and bounded left/right skew, then converts radians to the
sixteen named degree-valued Runtime features. Leader positions become the
collection action and follower positions become the observation. No positional
slicing escapes this adapter.

The native boundary also owns process scheduling and CAN health. It caps the
SDK's internal FIFO request at the System-owned priority, verifies every thread,
and polls both CAN controllers through rtnetlink. A live error counter or any
increase in warning, passive, bus-off, bus-error, arbitration-loss, or restart
statistics terminates the session and lets SDK destruction disable the motors.

Teleoperation and policy commands are separate readiness domains. Collection
depends on a commissioned teleoperation observer and commissioned cameras; it
does not depend on a policy-command transport.

## Repository ownership

The tracked System configuration owns the four CAN endpoint identities, common
CAN-FD settings, motor identities, all deployed gain and friction vectors, the
pinned teleoperation release, camera roles, local endpoints, service names, and
readiness evidence.
Loaders reject unknown or missing behavior-affecting keys.

Robot observations and commands use the same sixteen named position features as
LeRobot's bimanual OpenArm convention: eight left features followed by eight
right features, expressed in degrees. Feature order is presentation metadata;
decoding is always by exact name.

The future Runtime will permit concurrent read-only observers and at most one
opaque command lease. A heartbeat extends session liveness only. A successful
command extends command activity. Every command requires fresh, monotonically
sequenced follower feedback. The first command requires a sample produced after
lease acquisition and must hold that measured follower pose. Lease mismatch,
stale input, sequence failure, timeout, or release must detach and disable
command resources before another lease can be granted.

## Cameras

The Runtime repository owns the left-wrist, right-wrist, and optional AgentView
camera identities and their freshness/skew contract. One marked persistent
Camera Service opens each enabled V4L2 stream exactly once and completes a
bounded configuration-owned warmup. It serves exact raw RGB frame sets over a
versioned local Unix socket and read-only MJPEG from the same latest frames.
Collection is a raw-frame client and never reopens a physical camera. The
lower-rate preview encoder cannot advance collection delivery state.
`embodied-ops` receives only normalized health, preview URLs, and explicit
start/stop launches through the L1 presentation provider; it never owns a
camera.

The raw protocol binds every client to a digest of the tracked camera contract,
requires every enabled role in configured order, preserves device identity,
stream epoch, source sequence and monotonic timestamp, and validates exact
`HxWx3 uint8` payload sizes before allocation. A mismatched configuration,
truncated payload, stale sequence, incoherent set, or owner failure is
fail-closed. Preview and raw consumers use independent threads, so a slow
browser cannot block the physical readers or collection.

Each frame identifies its configured device and stream epoch. The bridge owns a
stateful validator for sequence and timestamp continuity. After a deliberate
stream restart, the bridge must declare each restarted role's new epoch before
the validator will accept sequence or timestamp rollback.

Each capture begins with concurrent reads. If independently clocked cameras
land outside the tracked group-skew window, the bridge advances only the
lagging stream or streams until it forms a coherent set or the bounded capture
deadline expires. Validation still rejects any incoherent set that crosses the
bridge boundary.

Both required wrist roles are mapped to their visually verified D405 serials
and enabled. The optional AgentView role is mapped to its verified D455 serial
and enabled for the current dataset contract. The three-camera collection
contract is commissioned. A physical USB disconnect still fails the current
episode closed and requires the operator to restore the affected device before
retrying.

## Collection and datasets

The collection dependency direction is:

```text
x_air state observer + Camera Bridge
  -> CollectionSample
  -> SampleAssembler
  -> DirectLeRobotEpisode
  -> canonical LeRobot v3 dataset
  -> independently generated v2.1 derivative
```

`CollectionSample` holds named follower state, named leader action, and the
enabled timestamped camera frames. `SampleAssembler` is the only place that
combines freshness, skew, continuity, finite-value, and image-shape checks. It
stays free of LeRobot, NumPy, ROS, and device APIs. NumPy materialization happens
only at the dataset writer boundary. The Runtime does not apply joint or gripper
position ranges to observation or teleoperation action values.

For each frame, live collection targets the midpoint of the earliest and latest
camera timestamps and selects the complete left/right robot-state pair closest
to that time. The configured robot/camera skew remains a validation boundary,
not a queue-draining heuristic.

The canonical dataset uses the same 16 degree-valued features at observation
and action boundaries. It is not named after a policy and no model-specific
normalization is stored. The v2.1 exporter always reads the canonical v3
dataset; one derivative never becomes the source of another derivative.

Every saved episode is appended through a hidden sibling dataset snapshot.
Camera frames enter LeRobot through the tracked asynchronous image writer, and
the complete live loop must meet the tracked minimum capture rate. Robot
runtimes stop immediately after capture; encoding and publication run after
motion shutdown while the read-only Camera Service may remain available.
Existing data, video, and image payloads are
hard-linked as immutable inputs; metadata is copied. LeRobot finalization,
provenance generation, and a complete payload doctor run before the staging
directory is atomically installed. A failed append leaves the prior complete
dataset authoritative. Staging or backup leftovers block reuse until they are
inspected.

Managed collection starts or verifies the marked Camera Service, connects one
raw client, and preflights cameras before motor enable. It then starts both
guarded x_air runtimes and waits for paired state. The operator uses
teleoperation to place the robot at the episode start pose and presses Enter;
the workflow rechecks all cameras before the first recorded frame. During
recording, Enter saves, `d` discards, and `q` discards and quits. A tracked
maximum frame count auto-saves, and save/discard advances to the next episode.
Each episode stops both robot sidecars before LeRobot finalization and starts a
new managed motion session only after the operator advances. Collection
shutdown closes only its raw client, so preview remains available. No automatic
reset is claimed while the Runtime has no reviewed fixed-pose command transport.

`embodied-ops` provides the generic episode decision, task registry,
freshness/skew, transaction, normalized camera-health, and Panel contracts.
This repository owns their L1 adapter and all LeRobot-specific dataset code.
That boundary lets a future OpenArm or other robot reuse the workflow protocol
without pretending that its hardware schema is the same as L1.

## Operator Panel

The L1 adapter exposes hardware-free collection validation, canonical dataset
doctor, and v2.1 export workflows, plus the commissioned finite live-collection
workflow. It also exposes explicit start/stop controls and normalized health
for the persistent read-only Camera Service. The live workflow uses the same
tracked readiness gates as the CLI and owns its camera client, teleoperation,
collection, and motion shutdown lifecycle; it does not own the persistent
camera process.
Capture progress uses the `embodied-ops` latest-value protocol instead of
durable terminal lines. The adapter also owns a strict create-only JSON prompt
registry for collection task text. It does not advertise reset, checkpoint,
evaluation, or policy-motion actions while command transport is unimplemented.
