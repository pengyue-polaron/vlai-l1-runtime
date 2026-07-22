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
hardware-independent collection/dataset layer exist today. A candidate
teleoperation observation path exists, while the independent policy-command
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
non-finite value stops the candidate process. Missing datagram consumers never
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
CAN-FD settings, motor identities, all deployed gain and friction vectors,
provisional joint limits, the pinned teleoperation release, camera roles, local
endpoints, service names, and readiness evidence.
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
camera identities and their freshness/skew contract. The Camera Bridge opens
each enabled RealSense exactly once and exposes timestamped RGB frames to live
collection. `embodied-ops` receives only normalized health and preview URLs
through an optional presentation provider; it never owns a camera.

Each frame identifies its configured device and stream epoch. The bridge owns a
stateful validator for sequence and timestamp continuity. After a deliberate
stream restart, the bridge must declare each restarted role's new epoch before
the validator will accept sequence or timestamp rollback.

Both required wrist roles are currently uncommissioned and disabled. AgentView
is optional and has no assigned driver yet. Collection is therefore unavailable
by construction, but adding AgentView later does not require mislabeling either
wrist stream.

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

`CollectionSample` holds named follower state, named leader action, and the two
timestamped camera frames. `SampleAssembler` is the only place that combines
freshness, skew, continuity, joint-limit, action-step, and image-shape checks.
It stays free of LeRobot, NumPy, ROS, and device APIs. NumPy materialization
happens only at the dataset writer boundary.

The canonical dataset uses the same 16 degree-valued features at observation
and action boundaries. It is not named after a policy and no model-specific
normalization is stored. The v2.1 exporter always reads the canonical v3
dataset; one derivative never becomes the source of another derivative.

Every saved episode is appended through a hidden sibling dataset snapshot.
Existing data, video, and image payloads are hard-linked as immutable inputs;
metadata is copied. LeRobot finalization, provenance generation, and a complete
payload doctor run before the staging directory is atomically installed. A
failed append leaves the prior complete dataset authoritative. Staging or backup
leftovers block reuse until they are inspected.

`embodied-ops` provides the generic episode decision, freshness/skew,
transaction, and Panel contracts. This repository owns their L1 adapter and all
LeRobot-specific dataset code. That boundary lets a future OpenArm or other
robot reuse the workflow protocol without pretending that its hardware schema
is the same as L1.

## Operator Panel

With the current uncommissioned configuration, the L1 adapter exposes three
hardware-free workflows: collection configuration validation, canonical dataset
doctor, and v2.1 export. It reports the tracked live-collection blockers and
does not advertise camera, reset, or motion actions. Once the teleoperation and
camera gates are commissioned, the same adapter adds its finite live-collection
workflow without changing the reusable Web application.
