# Datasets

## Format and configuration

Canonical datasets use LeRobot v3.0 under `data/datasets/<experiment>/`.
Collection TOML selects active sides, camera roles, rates, synchronization limits,
trim/reset behavior and output paths. System configuration defines device
identities, joint names, units and image settings. Read these configurations for
current parameter values.

| Feature | Meaning |
|---|---|
| `observation.state` | Follower joint/gripper positions in SDK degrees. |
| `action` | Leader joint/gripper positions in the same order and units. |
| `observation.ft_wrench` | Sensor-frame Fx/Fy/Fz in N and Mx/My/Mz in Nm. |
| `observation.ft_time_delta_s` | FT host receipt time minus robot state time. |
| `observation.scale_weight_g` | Scale display reading converted to grams. |
| `observation.scale_time_delta_s` | Scale host receipt time minus robot state time. |
| `observation.scale_stable/tared/net` | Observed ST/TR/NT flags, not software verification of tare. |
| `observation.images.*` | Selected camera RGB videos. |

Bimanual state/action vectors contain 16 values:
left J1-J7, left gripper, right J1-J7, right gripper.
`configs/collection/right_only.toml` selects eight right-side values and
Right Wrist/AgentView images; it does not fabricate left-side data.
Joint values are stored without model normalization or clipping.
AgentView's configured centered crop is 480x480 from a 640x480 source;
wrist images remain full frame. Exports retain the same image contract.

The current collector requires the configured HKVL FT and RS232 scale streams.
FT is matched by host receipt time; the scale uses the latest preceding reading.
Missing, malformed, stale or unsynchronized inputs fail collection under its
configuration. Host receipt time is not a sensor timestamp.
FT has no implicit tool-gravity compensation or end-effector transform.
Scale readings are displayed weight, not an independently derived poured mass;
software does not perform or verify tare.

## Saving and discarding

Collection trims the leading still portion using configured action thresholds
and retains preroll. No-motion episodes are discarded. Capture rate, names,
finite values, freshness, synchronization and sequence are validated.

Right-only save leaves teleoperation active at its current pose; discard runs
SDK Reset. Bimanual save and discard both Reset. Encoding/finalization then
publish a complete hidden staging dataset by atomic rename.
Existing payloads are immutable hardlinks; appending requires hardlink support.
Metadata records schema, configuration identity and episode/frame counts.

Discard removes its episode staging buffer. Crash-leftover staging/backup paths
need inspection before reuse; they may contain unpublished data.

## Check and export

```bash
just dataset-doctor EXPERIMENT
just dataset-doctor EXPERIMENT --json
just export-v21 EXPERIMENT
just export-v21 EXPERIMENT --json
```

For right-only CLI selection, use
`vlai-l1 dataset doctor --side right EXPERIMENT` or
`vlai-l1 dataset export-v21 --side right EXPERIMENT`.
These operations inspect local artifacts without opening robot/camera devices.

Doctor checks configuration/schema, feature/task consistency, contiguous frame
ranges, Parquet rows and referenced videos. Export creates a new derivative in
`data/derivatives/<experiment>-v2.1`, slices video into episode-local H.264,
and verifies frame counts and dimensions. It refuses an existing output target.
The canonical v3 dataset remains the source. Export requires Python 3.12,
the dataset extra, ffmpeg and ffprobe.
