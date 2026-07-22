# x_air commissioning

The pinned x_air build is a candidate until the tracked live gates are closed.
Run the live stages only after the operator explicitly confirms a clear
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

1. Stop both existing teleoperation services and confirm no old or candidate
   controller process remains.
2. Verify all four CAN links are down. Bring up only the two links needed by the
   current side at the tracked 1 Mbit/s nominal and 5 Mbit/s data rate.
3. Verify J2 direction without enabling teleoperation. The public teleoperation
   API is not a read-only probe and must not be used for this step.
4. Start the state observer before the candidate:

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

The two D405 streams were identified from live RGB frames: `255323074436` is the
left wrist and `255323074499` is the right wrist. Both are enabled in System
config; AgentView remains optional. Verify a short discard before saving data:

```bash
just collect commissioning "hold position" 30 discard
```

Then record one short saved episode and run `dataset-doctor`. A successful
teleoperation commissioning changes only the teleoperation and hardware
evidence gates; it does not make the separate policy-command transport ready.

## Current live evidence

The 2026-07-22 left-side test first reproduced visible stutter while the public
SDK internally raised its three 500 Hz workers from the requested FIFO 20 to
hard-coded FIFO 50. That run also increased `can1` warning/passive counters.

After adding the process-level FIFO cap, all five candidate threads verified at
FIFO 20. The next left-side run still failed immediately: `can3` reached live
TX/RX error counts 8/84 and its cumulative passive count increased from 1 to 4.
The new CAN guard detected the transition, stopped the SDK session, and SDK
destruction disabled the motors. Repository cleanup then returned all buses to
`DOWN/STOPPED`. This proves the scheduling and automatic-stop paths, but it does
not commission teleoperation; inspect the left-follower power and physical CAN
path before another motion test.
