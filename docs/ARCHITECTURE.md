# Architecture

The intended dependency direction is:

```text
embodied-ops workflow and Operator Panel contracts
  -> VLAI workflow adapter and LeRobot dataset composition
  -> LeRobot Robot and Teleoperator clients
  -> VLAI L1 Runtime transport
  -> command lease and safety gates
  -> realtime C++ device owners
  -> CAN and cameras
```

The pure configuration, public contracts, command-session state machine, and
hardware-independent collection/dataset layer exist today. The teleoperation
observation path is commissioned after recovery of `can0` and a bounded test of
the new joint-safety monitor. The independent policy-command transport remains
unimplemented.

`embodied-ops` is the cross-robot operator-workflow layer, not a competing
hardware API. It owns stable CLI presentation, collection input/decision
contracts, reset-policy selection, task registries, timing checks, atomic
artifacts, canonical contract digests, and the versioned Operator Panel catalog
and form builders. VLAI composes those contracts with its extra pre-recording
AdjustPosition action. Joint names, degrees, CAN ownership, camera identities,
LeRobot layout, physical Reset behavior, and every readiness decision remain
L1-specific.

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

The pinned unilateral path is a direct joint-space reference path rather than
Cartesian inverse kinematics: the public wrapper copies leader arm state into
the follower references and the pinned release's JointMapper is identity. The
native boundary therefore independently checks both seven-joint feedback
vectors against side-specific System-owned bounds and measures leader/follower
following error on increasing callback timestamps. A bound violation is
immediately fatal. The configured following-error action is `warn`, so an
over-limit error that persists for the tracked timeout emits one nonfatal event
per excursion with side and joint detail. The gripper remains covered by the
existing finite/stale/CAN checks but is not part of the seven-joint arm envelope.

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
The outer lifecycle independently watches CAN state while the opaque SDK is
still inside its blocking constructor. Before invoking that motion-capable
constructor, it opens each endpoint sequentially through the low-level CAN SDK
without enabling motors. At the tracked rate and duration, every round must
receive the exact eight System-owned motor response IDs; one missing response
blocks alignment. Startup faults close the pair without rebinding or retrying
an adapter, so evidence is preserved and state continuity is never fabricated
across a control fault. Managed bimanual orchestration holds both processes
after their complete motion-free probes and releases the motion-capable SDK
constructors only when both sides are ready.

An isolated single-side launch reuses the same configured lifecycle rather
than defining another hardware mapping. It acquires the lifecycle lock
exclusively, requires both peer CAN interfaces to be administratively `DOWN`
before active-side setup, and continues checking that condition until shutdown.
Ordinary paired side launches take shared ownership, so neither can overlap an
isolated session.

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

The raw transport and MJPEG preview retain each configured source dimension.
An optional per-stream observation crop is part of the same tracked System
camera contract. `SampleAssembler` applies it only after validating the full
raw frame. The commissioned AgentView crop is the centered 480×480 region of
the 640×480 source; both wrist observations remain full-frame.

Each frame identifies its configured device and stream epoch. The bridge owns a
stateful validator for sequence and timestamp continuity. After a deliberate
stream restart, the bridge must declare each restarted role's new epoch before
the validator will accept sequence or timestamp rollback.

Each capture begins with concurrent reads. If independently clocked cameras
land outside the tracked group-skew window, the bridge advances only the
lagging stream or streams until it forms a coherent set or the bounded capture
deadline expires. Validation still rejects any incoherent set that crosses the
bridge boundary.

Both wrist roles are mapped to their visually verified D405 serials and
enabled in the complete physical camera contract. The optional AgentView role
is mapped to its verified D455 serial and enabled. A collection config selects
which enabled roles are persisted without redefining physical identity. A
physical USB disconnect still fails the current
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

`CollectionSample` holds selected named follower state, selected named leader
action, and the explicitly recorded timestamped camera frames. `SampleAssembler` is the only place that
combines freshness, skew, continuity, finite-value, and image-shape checks. It
also applies the tracked observation crop without importing an image library.
It stays free of LeRobot, NumPy, ROS, and device APIs. NumPy materialization
happens only at the dataset writer boundary. The Runtime does not apply joint
or gripper normalization to stored observation or teleoperation action values;
the live sidecar's separate arm envelope is a fail-closed lifecycle guard and
never clamps or rewrites a sample.

For each frame, live collection targets the midpoint of the earliest and latest
recorded-camera timestamps and selects the configured robot state closest
to that time. The configured robot/camera skew remains a validation boundary,
not a queue-draining heuristic.

The bimanual collection contract uses the same 16 degree-valued features at
observation and action boundaries. The right-only contract uses exactly the
eight right-side features and records only Right Wrist plus AgentView. Neither
contract fabricates inactive-side values. No model-specific normalization is
stored. The v2.1 exporter always reads the canonical v3
dataset; one derivative never becomes the source of another derivative.

Every saved episode is appended through a hidden sibling dataset snapshot.
Camera frames enter LeRobot through the tracked asynchronous image writer, and
the complete live loop must meet the tracked minimum capture rate. After each
save/discard capture stops, the active runtime runs `AdjustPosition` before
encoding or discard finalization. It remains enabled at Reset until another
episode begins or the operator quits; the read-only Camera Service may remain available.
Existing data, video, and image payloads are
hard-linked as immutable inputs; metadata is copied. LeRobot finalization,
provenance generation, and a complete payload doctor run before the staging
directory is atomically installed. A failed append leaves the prior complete
dataset authoritative. Staging or backup leftovers block reuse until they are
inspected.

Managed collection starts or verifies the marked Camera Service, connects one
raw client, and preflights recorded camera roles before motor enable. Bimanual
mode starts both guarded x_air runtimes and releases their SDK constructors only
after both sides pass the motion-free CAN preflight. Isolated mode starts only
the selected side with the peer CAN pair locked down. SDK construction performs
`AdjustPosition` before the workflow accepts selected state.
The operator may run the same
routine again with `r`, uses teleoperation to refine the episode start pose,
and presses Enter. The workflow rechecks all cameras before the first recorded
frame. During recording, Enter saves, `d` discards, and `q` discards and quits.
Recording is operator-bounded and has no automatic duration limit.
One collection command owns the selected sidecar(s) across every episode.
Save/discard first resets the selected side(s) on the same active handles;
finalization then runs while the realtime loop remains active at Reset. Only
quit, failure, or interruption stops the sidecars and returns their owned CAN
links to `DOWN`. Collection shutdown closes only its raw client, so preview
remains available. The
standalone reset workflow uses this same startup alignment lifecycle without
opening cameras or a dataset.

`embodied-ops` provides the generic CLI, collection interaction and reset
policy, episode decision, task registry, freshness/skew, transaction,
contract-digest, normalized camera-health, and strict Panel catalog contracts.
The Runtime imports that implementation through thin compatibility and L1
composition modules instead of retaining parallel copies. This repository owns
the L1 adapter and all LeRobot-specific dataset code.
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
durable terminal lines. Catalogs declare the shared schema version and are
built from shared field primitives; collection readiness is expressed by
whether the live workflow is present rather than an L1-only Web schema field.
The adapter also owns a strict create-only JSON prompt registry for collection
task text. Its Reset action is limited to the SDK alignment lifecycle; it does
not advertise checkpoint, evaluation, or policy-motion actions while command
transport is unimplemented.
