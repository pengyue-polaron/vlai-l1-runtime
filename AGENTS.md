# VLAI L1 Runtime Agent Guide

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

## Change hygiene

- Inspect `git status` and `git diff` first.
- Use `rg` for search and `apply_patch` for edits.
- Keep hardware dependencies lazy and pure validation available on Python 3.10.
- Before handoff run pytest, Ruff, and `git diff --check`; state explicitly that
  checks were hardware-free.
