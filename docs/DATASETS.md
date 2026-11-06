# Datasets

## Canonical v3 dataset

Each tracked collection config owns dataset roots, repository ID prefix, active
teleoperation sides, recorded camera roles, FPS,
minimum effective capture rate, asynchronous image-writer concurrency, sample
freshness, reset points, leading-stillness thresholds, and state/action,
robot/camera, FT/robot, scale/robot and camera-pair skew. It points to
exactly one System config; camera dimensions and physical identities, joint
names, FT and scale serial identities, physical units, and robot identity are not duplicated.

An experiment named `pick_blocks_v1` is stored at:

```text
data/datasets/pick_blocks_v1/
```

Its repository ID is derived as
`pengyue-polaron/vlai-l1-pick_blocks_v1`. The committed
`meta/vlai_l1.json` records the task, schema identity, topology, feature names,
camera roles, FPS, config hashes, and final episode/frame counts.

The tracked System camera contract defines the observation crop for each
stream. AgentView is captured at 640×480 for preview and camera transport, then
center-cropped to `(x=80, y=0, width=480, height=480)` as it enters the canonical
dataset. Both side strips are excluded; the wrist streams remain full-frame.
The same square AgentView pixels therefore appear in the canonical v3 dataset
and every v2.1 derivative.

The canonical features are:

```text
observation.state         float32[16], degrees
action                    float32[16], degrees
observation.ft_wrench     float32[6], sensor-frame Fx/Fy/Fz N, Mx/My/Mz Nm
observation.ft_time_delta_s float32[1], FT host receipt minus robot state time
observation.scale_weight_g float32[1], scale display reading in grams
observation.scale_time_delta_s float32[1], scale host receipt minus robot state time
observation.scale_stable float32[1], 1 when serial status token is ST, otherwise 0
observation.scale_tared  float32[1], 1 when serial mode token is TR, otherwise 0
observation.scale_net    float32[1], 1 when serial mode token is NT, otherwise 0
observation.images.wrist_left   video[height,width,3]
observation.images.wrist_right  video[height,width,3]
observation.images.agent        video[height,width,3]
```

Only enabled roles explicitly selected by the collection config are part of an
experiment's feature contract. The current right-only physical assignment is
Right Wrist D405 and D455 AgentView; both passed the tracked camera-only check
in the current USB3 hub positions. The historical bimanual contract additionally
requires a distinct
Left Wrist camera.

Both vectors use:

```text
left_joint_1.pos ... left_joint_7.pos, left_gripper.pos,
right_joint_1.pos ... right_joint_7.pos, right_gripper.pos
```

`configs/collection/right_only.toml` uses a separate dataset root and repository
prefix. Its canonical features are `float32[8]` right-side state/action plus
the same HKVL FT and scale features, and
`observation.images.wrist_right` and `observation.images.agent`. The Camera
Service owns and previews both selected cameras when connected.
Left Wrist pixels are neither passed to LeRobot nor encoded into this dataset.

The collector owns the configured HKVL USB-RS485 serial port and requires a
valid stream before starting any teleoperation runtime. At each 30 FPS frame,
it selects the closest decoded FT sample by host monotonic receipt time, with
a tracked 50 ms maximum FT/robot skew and the common freshness limit. Missing,
stale, nonfinite, or out-of-order FT data fail the episode. The FT feature is
the sensor's reported frame, without zeroing, tool-weight compensation, or a
transform to the end effector. The time delta is based on host receipt, not a
sensor-supplied timestamp. The canonical dataset contains one FT observation
per video/robot frame; it does not retain the complete high-rate serial stream.

The collector also owns the configured `/dev/vlai-scale` RS232 port. The observed
line format is `ST,NT,+     12 g` at 9600 8N1; after tare we observed
`ST,TR,+      0 g`. It accepts ST/US status, NT/GS/TR mode and signed g/kg
readings, converts kg to grams, and records the observed
status flags. Each robot/video frame uses the most recent scale reading that
preceded its robot timestamp; one reading may serve several 30 FPS frames.
The tracked age limit is 600 ms and scale/robot skew limit is 500 ms. Missing,
malformed, or stale scale data fail collection. Host-receipt time is not a
sensor timestamp. The operator stated that the receiving container is tared
before each episode; software does not perform or verify tare. The saved value
is the scale's displayed weight, not an independently calculated poured mass.
The full scale serial stream is not retained in the canonical dataset.

## Publication contract

Camera frames are persisted through LeRobot's configured asynchronous image
writer. The Runtime measures the complete capture loop, including validation
and frame enqueue, and rejects a run below `minimum_capture_fps`. Before any
frame reaches LeRobot, the shared streaming trimmer builds its reference from
the configured initial action window and waits for consecutive threshold
crossings. It keeps only the configured preroll plus motion-confirmation window
in memory, then publishes that window and every following frame. An episode
with no detected movement remains empty and is discarded. Joint and gripper
thresholds stay in the robot-owned collection config because L1 actions are
truthful degrees. There is no
per-frame action-delta limit; named state and action values must still be
finite, fresh, synchronized, and increasing in sequence. The Runtime does not
apply joint or gripper position ranges to collected values.

Each episode is buffered by LeRobot inside a hidden sibling directory. After
the last accepted frame, save/discard runs `AdjustPosition` on the still-active
selected runtime(s). Saving then performs video encoding, LeRobot finalization,
provenance writing, and a deep
validation of task metadata, episode ranges, Parquet row counts and columns,
and every referenced video. Only then is the complete directory renamed over
the dataset target. Appending requires hard-link support so old large payloads
stay immutable and are not copied on every episode.

Discard clears the episode buffer and removes its staging directory. Any
`.staging-*` or `.backup-*` sibling blocks future use until inspected. The tools
never guess whether a crash leftover is safe to delete.

## Doctor

```bash
vlai-l1 dataset doctor pick_blocks_v1
# Machine-readable output:
vlai-l1 dataset doctor pick_blocks_v1 --json
# Right-only:
vlai-l1 dataset doctor --side right pick_blocks_v1
```

The doctor compares the dataset with the currently tracked collection and
System config hashes. It also rejects symlinks, special files, schema drift,
feature drift, task drift, non-contiguous episode/frame ranges, missing Parquet
columns, incorrect row counts, and missing videos. It requires the `dataset`
extra but never opens robot or camera devices.

## v2.1 export

The Runtime validates its L1 feature/task/provenance contract, then delegates
the shared LeRobot file-graph traversal, episode Parquet rewrite, H.264 slicing,
and v2.1 metadata generation to `embodied-ops[lerobot-dataset]`. L1 derivative
identity and atomic publication remain owned here.

```bash
vlai-l1 dataset export-v21 pick_blocks_v1
# Machine-readable output:
vlai-l1 dataset export-v21 pick_blocks_v1 --json
# Right-only:
vlai-l1 dataset export-v21 --side right pick_blocks_v1
```

This creates `data/derivatives/pick_blocks_v1-v2.1` as a new, atomic output. It
will not overwrite an existing derivative. Each v3 video range is sliced into
an episode-local H.264 file, verified with `ffprobe`, and accompanied by v2.1
JSONL metadata. Frame count and encoded dimensions are both checked against the
canonical feature contract. The derivative retains namespaced source provenance
and records its source repository ID and counts.

The exporter requires `ffmpeg`, `ffprobe`, Python 3.12, and the `dataset`
extra. The canonical v3 dataset remains the sole source of truth.
