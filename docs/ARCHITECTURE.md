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
hardware-independent collection/dataset layer exist today. No arrow below the
Runtime transport has been implemented.

## Repository ownership

The tracked System configuration owns the four CAN endpoint identities, common
CAN-FD settings, motor identities, deployed gains, provisional joint limits,
camera roles, local endpoints, service names, and command-readiness evidence.
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

The Runtime repository owns the AgentView and wrist camera identities and their
freshness/skew contract. A future Camera Bridge will open each physical device
once, expose raw timestamped frames to collection/inference, and independently
produce a lower-rate Web preview. `embodied-ops` receives only normalized health
and preview URLs through its optional camera provider.

Each frame identifies its configured device and stream epoch. The bridge owns a
stateful validator for sequence and timestamp continuity. After a deliberate
stream restart, the bridge must declare each restarted role's new epoch before
the validator will accept sequence or timestamp rollback.

Both camera roles are currently uncommissioned and disabled. Collection is
therefore unavailable by construction.

## Collection and datasets

The collection dependency direction is:

```text
future Runtime observer + Camera Bridge
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

The L1 adapter currently exposes three hardware-free workflows: collection
configuration validation, canonical dataset doctor, and v2.1 export. It reports
the tracked live-collection blockers and does not advertise a runnable collect,
camera, reset, or motion action. When the Runtime observer and Camera Bridge are
commissioned, the adapter may add collect and camera capabilities without any
change to the reusable Web application.
