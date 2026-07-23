# Datasets

## Canonical v3 dataset

`configs/collection/default.toml` owns dataset roots, repository ID prefix, FPS,
minimum effective capture rate, asynchronous image-writer concurrency, sample
freshness, and state/action, robot/camera and camera-pair skew. It points to
exactly one System config; camera dimensions and roles, joint names, physical
units, and robot identity are not duplicated.

An experiment named `pick_blocks_v1` is stored at:

```text
data/datasets/pick_blocks_v1/
```

Its repository ID is derived as
`pengyue-polaron/vlai-l1-pick_blocks_v1`. The committed
`meta/vlai_l1.json` records the task, schema identity, topology, feature names,
camera roles, FPS, config hashes, and final episode/frame counts.

The canonical features are:

```text
observation.state         float32[16], degrees
action                    float32[16], degrees
observation.images.wrist_left   video[height,width,3]
observation.images.wrist_right  video[height,width,3]
observation.images.agent        video[height,width,3]
```

Only enabled camera roles are part of an experiment's feature contract. The two
wrist roles are required by the platform configuration. AgentView remains an
optional role, but its commissioned D455 is enabled in the current configuration
and is therefore part of every newly collected frame.

Both vectors use:

```text
left_joint_1.pos ... left_joint_7.pos, left_gripper.pos,
right_joint_1.pos ... right_joint_7.pos, right_gripper.pos
```

## Publication contract

Camera frames are persisted through LeRobot's configured asynchronous image
writer. The Runtime measures the complete capture loop, including validation
and frame enqueue, and rejects a run below `minimum_capture_fps`. There is no
per-frame action-delta limit; named state and action values must still be
finite, fresh, synchronized, and increasing in sequence. The Runtime does not
apply joint or gripper position ranges to collected values.

Each episode is buffered by LeRobot inside a hidden sibling directory. The
robot and cameras are stopped immediately after the last accepted frame.
Saving then performs video encoding, LeRobot finalization, provenance writing,
and a deep validation of task metadata, episode ranges, Parquet row counts and
columns, and every referenced video. Only then is the complete directory
renamed over the dataset target. Appending requires hard-link support so old
large payloads stay immutable and are not copied on every episode.

Discard clears the episode buffer and removes its staging directory. Any
`.staging-*` or `.backup-*` sibling blocks future use until inspected. The tools
never guess whether a crash leftover is safe to delete.

## Doctor

```bash
vlai-l1 dataset-doctor \
  --config configs/collection/default.toml \
  --experiment pick_blocks_v1
```

The doctor compares the dataset with the currently tracked collection and
System config hashes. It also rejects symlinks, special files, schema drift,
feature drift, task drift, non-contiguous episode/frame ranges, missing Parquet
columns, incorrect row counts, and missing videos. It requires the `dataset`
extra but never opens robot or camera devices.

## v2.1 export

```bash
vlai-l1 export-v21 \
  --config configs/collection/default.toml \
  --experiment pick_blocks_v1
```

This creates `data/derivatives/pick_blocks_v1-v2.1` as a new, atomic output. It
will not overwrite an existing derivative. Each v3 video range is sliced into
an episode-local H.264 file, verified with `ffprobe`, and accompanied by v2.1
JSONL metadata. The derivative retains namespaced source provenance and records
its source repository ID and counts.

The exporter requires `ffmpeg`, `ffprobe`, Python 3.12, and the `dataset`
extra. The canonical v3 dataset remains the sole source of truth.
