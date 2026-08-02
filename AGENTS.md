# VLAI L1 Runtime Agent Guide

Read `docs/HANDOFF.md` for the current workspace state and next safe test stage.
Read `docs/SAFETY.md` before changing runtime, lifecycle, command, calibration,
or camera code. On the onboard deployment, the host-level
`/home/sunrise/AGENTS.md` additionally governs live operations.

## Hard constraints

- Treat all CAN publishers and motor handles as live hardware.
- Until tracked readiness gates are resolved, do not add a live command adapter,
  enable path, calibration path, reset path, or systemd integration.
- Static description, validation, and tests must never initialize ROS, CAN,
  cameras, systemd, or `/dev` resources.
- One process will ultimately own each CAN endpoint and each camera. LeRobot
  plugins remain transport clients and never open physical devices.
- Runtime configuration is the single source of physical identity and safety
  values. Do not duplicate behavior-affecting defaults in code or environment
  variables.
- Preserve truthful degrees at the LeRobot boundary. Any model-specific
  representation belongs in an explicit processor.
- Require exact named vectors, finite values, monotonic source timestamps, and
  fail-closed command-session transitions.
- Do not copy opaque binaries, generated build trees, backups, or legacy
  compatibility wrappers into this repository.

## Live CAN recovery

- Treat `ERROR-PASSIVE`, `BUS-OFF`, any nonzero live CAN error counter, or a
  missing configured motor response as a real startup fault. Fail closed; never
  let collection automatically reset, retry, or continue into SDK alignment.
- The guarded launcher already performs a normal interface
  `down -> configure -> up` cycle. Do not repeatedly start collection to clear a
  fault; repeated unacknowledged transmissions can drive the controller back to
  `txerr=128`.
- A PEAK driver unbind/bind may leave PCAN-USB FD controller state intact. Only
  an explicit operator request authorizes a bounded USB-level reset of the one
  affected adapter. Resolve its exact configured `parentdev`, require every CAN
  interface and controller process to be down, and never reset a hub, wildcard,
  or unverified USB target.
- After any driver or USB reset, wait for udev, verify every CAN interface still
  maps to its configured USB parent, and leave all links down. Before any
  motion-capable SDK constructor, run only the tracked motion-free feedback
  probe and require every configured motor ID on every round, `ERROR-ACTIVE`,
  zero live error counters, and no qdisc drops.
- On 2026-08-02, right leader `can0` remained at `txerr=128` after ordinary link
  cycling and driver rebind. A targeted USB reset of configured parent
  `1-3.1:1.0`, followed by fresh CAN-FD configuration, restored 40/40 probe
  rounds for response IDs `0x011` through `0x018` with 320 matching TX/RX frames
  and zero errors. This is evidence for that incident, not permission to make
  recovery automatic or to ignore a future physical no-ACK fault.
- A later 2026-08-02 `can0` incident followed a mechanically constrained
  right-side teleoperation session. A targeted reset cleared the PCAN adapter,
  but the first motion-free state query then received no response from any of
  the eight leader motor IDs and returned the controller to `txerr=128`. Treat
  an all-node no-ACK after a clean adapter reset as a leader-bus power,
  transceiver, harness, or node-side protection fault; repeated USB resets are
  not a software repair.
- The same all-eight-node no-ACK fault reproduced on the first motion-free
  round after the operator reported a restart: the freshly configured PCAN
  baseline was ERROR-ACTIVE with zero live errors, then eight queries produced
  zero responses and `txerr=128`. Do not run another CAN query or SDK startup
  until right-Leader motor-bus power and physical continuity have been checked.
- A later complete robot/motor-power restart re-enumerated every adapter with
  zero counters and restored 40/40 feedback rounds on both `can0` and `can2`.
  A protected 15-second isolated-right window and guarded shutdown then passed
  with zero CAN errors or qdisc drops. This distinguishes motor-power recovery
  from host-only restart; it does not authorize automatic power cycling.

## Teleoperation joint safety

- The pinned unilateral SDK copies leader joint positions directly into
  follower joint references through an identity joint mapper. Do not describe
  this path as Cartesian IK and do not assume that an unreachable or obstructed
  follower target will be rejected inside the opaque controller.
- Keep the side-specific seven-joint bounds, following-error vector, and
  persistence timeout in tracked Runtime configuration. The native sidecar
  must fail closed, identify the side/joint/role, and release the SDK handle on
  an out-of-bounds position or non-monotonic state. Per the operator's explicit
  decision, `following_error_action = "warn"`: a sustained leader/follower
  error emits one warning per excursion but does not stop the Runtime. Do not
  silently extend this exception to hard bounds, CAN health, or state freshness.
- After an obstruction or unexpected force, stop the active lifecycle and
  remove the load before diagnosis. Do not restart alignment, Reset, or
  collection until the motion-free probe passes every motor ID with zero CAN
  errors.

## Change hygiene

- Inspect `git status` and `git diff` first.
- Use `rg` for search and `apply_patch` for edits.
- Keep hardware dependencies lazy and pure validation available on Python 3.10.
- Before handoff run pytest, Ruff, and `git diff --check`; state explicitly that
  checks were hardware-free.
