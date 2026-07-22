# Source gaps

The active deployment contains `unilateral_control`,
`unilateral_control_ros2`, `bilateral_control`, and `gravity_comp` AArch64
binaries, but the corresponding teleoperation C++ source is absent from both the
active `xarm_teleop` directory and the 2026-07-16 backup. Embedded binary paths
refer to an unavailable `/home/sunrise/x_air_chage` source tree.

The new Runtime must not wrap or redistribute these opaque binaries as its
implementation. Command support remains blocked until:

1. the exact source and build inputs are recovered or a replacement is written;
2. the binary is reproducibly built for the onboard host;
3. J2 signs, offsets, and limits are verified read-only on all four arms;
4. `can2` stability is validated under representative sustained load;
5. startup hold, sequence, freshness, lease, and disable behavior are covered by
   hardware-free tests and staged live commissioning;
6. each physical CAN endpoint has one explicit owner and shutdown order.

The lower-level `xarm_can` and ROS 2 sources in the backup may be audited as
provenance, but they are not silently vendored into this repository.
