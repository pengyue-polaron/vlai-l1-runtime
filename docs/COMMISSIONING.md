# x_air commissioning

The pinned x_air build is the commissioned teleoperation runtime. Repeat live
commissioning only after a behavior-affecting runtime, SDK, configuration, or
hardware change, and only after the operator explicitly confirms a clear
workspace. `xarm_teleop_create_unilateral` enables motors and performs position
alignment before `start`, so launching the sidecar must be treated as immediate
motion.

The Runtime submodule points at the first-party `pengyue-polaron/vlai-x-air-sdk`
mirror and pins commit `bf300508e179f652b23f0efaf3b6c9048f1f12e9`. The mirror
preserves the exact reviewed dependency even if the upstream repository moves.

## Hardware-free preparation

```bash
git submodule update --init --recursive
just sdk-verify
just sdk-build

source /opt/ros/humble/setup.bash
source build/xair-install/setup.bash
just sdk-prepare
```

The manifest records the selected revision, SDK version, architecture, library
hashes, generated assets, state protocol, and exact CAN pair for both sides.
`ldd build/xair-sidecar/vlai_l1_xair_sidecar` must contain no `not found`
entries.

## Live sequence

1. Stop both existing teleoperation services and confirm no other controller
   process remains.
2. Verify all four CAN links are down. Bring up only the two links needed by the
   current side at the tracked 1 Mbit/s nominal and 5 Mbit/s data rate.
3. Verify J2 direction without enabling teleoperation. The public teleoperation
   API is not a read-only probe and must not be used for this step.
4. Start the state observer before the sidecar:

   ```bash
   just sdk-observe 20 3
   ```

   `bimanual` waits for a fresh packet from both sidecars, checks the tracked
   left/right timestamp skew, and prints a compact summary plus the final named
   follower observation and leader action in degrees. Use `left` or `right`
   only for an isolated single-side stage.

5. Test the left pair first (`can1` leader to `can3` follower). Keep both arms
   mechanically aligned before creation and stop on unexpected alignment,
   direction, force, sound, vibration, stale state, or CAN errors.
6. Stop, run the deployed disable hook, bring both links down, and inspect bus
   counters before advancing.
7. Repeat for the right pair (`can0` leader to `can2` follower). This stage must
   specifically close the previous `can2` stability concern.
8. Only after both isolated stages pass, run both sidecars together and require
   the `bimanual` observer to complete all requested samples without a timeout,
   sequence failure, or left/right skew failure.

The sidecar command takes no hardware defaults. Populate every argument from
`build/xair-assets/manifest.json` and the tracked config:

```text
vlai_l1_xair_sidecar
  --side <left|right>
  --leader-can <tracked leader interface>
  --follower-can <tracked follower interface>
  --leader-urdf build/xair-assets/v10_leader.urdf
  --follower-urdf build/xair-assets/v10_follower.urdf
  --config-dir build/xair-assets/config
  --state-socket /run/vlai-l1/teleop-state.sock
  --publish-hz 100
  --state-timeout-ms 100
  --rt-priority 20
  --can-health-poll-ms 100
```

The sidecar caps the SDK's internally requested FIFO priority at the tracked
value, then verifies every process thread at that exact policy. It snapshots
both CAN controllers before SDK creation and stops the session on any live
error counter or increase in warning, passive, bus-off, bus-error, arbitration
loss, or restart statistics.

Do not use the upstream `start_xarm_teleop_both.sh`: its current launch
arguments reverse the documented left/right CAN pairs. Do not use the upstream
ROS2 wrapper either; it opens an additional gripper handle on the leader bus.

## Camera and collection stage

The camera streams were identified from live RGB frames: D405 `255323074436` is
the left wrist, D405 `255323074499` is the right wrist, and D455 `251643060089`
is AgentView. All three are enabled in System config; AgentView remains an
optional platform role but is part of the current dataset contract. Verify a
short discard before saving data:

```bash
just collect commissioning "hold position" 30 discard
```

`just collect` owns the whole live session. It preflights the state socket and
all three cameras before starting either robot runtime, starts the left and
right runtimes, and waits for paired state. Use teleoperation to place the robot
at the episode start pose, then press Enter in the terminal or Start recording
in the Panel. Collection rechecks all cameras after confirmation, records, and
always disables both pairs on completion, failure, or interruption. Start it
with no manual `sdk-start`, observer, camera reader, or competing controller
active.

Then record one short saved episode and run `dataset-doctor`. A successful
teleoperation commissioning changes only the teleoperation and hardware
evidence gates; it does not make the separate policy-command transport ready.

## Current live evidence

Commissioning completed on 2026-07-23. The original stutter reproduced with
both the current sidecar and the legacy controller, ruling out the state
callback and SDK wrapper. `tc -s qdisc` then exposed the hidden failure:
`txqueuelen=10` had dropped about 184k packets on each active left bus even
though ordinary CAN statistics reported zero drops. The tracked queue length
of 1000 produced no new qdisc drops and restored smooth motion.

Formal `just sdk-start` tests passed for left `can1 -> can3`, right
`can0 -> can2`, and both sides simultaneously. Every active bus remained
ERROR-ACTIVE with zero live CAN errors; the paired observer completed 3000
samples at 100 Hz. With both 500 Hz controllers and that observer active, the
parallel camera bridge captured 300 synchronized sets from both D405 cameras
and AgentView at 29.997 FPS with 32.88 ms maximum skew. All stop paths disabled
the motors, drained qdisc backlog to zero, and returned can0-can3 to
DOWN/STOPPED.

During a later collection attempt, the shared external USB hub reset as a
whole: both wrist cameras, AgentView, and all four PCAN adapters disconnected
and re-enumerated together. OpenCV then reported `ENODEV` for the stale
`/dev/video4` node. This is a hub, upstream-cable, or hub-power failure, not a
camera-role mapping error. The managed collection preflight now prevents motor
startup when a camera is already unavailable and its cleanup stops the robot
if a device disappears mid-session; repeated resets still require the physical
USB path to be repaired.

A later AgentView-only failure produced repeated USB disconnect and
re-enumeration events for the D455 while its two D405 peers and PCAN adapters
remained present. The D455 runs RGB-only at USB 2 speed with autosuspend
disabled, so that event points to its cable, port, or power path rather than the
camera-role mapping. An episode interrupted by any camera disconnect remains
invalid and is never published.
