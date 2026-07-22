# Source gaps

The active deployment contains `unilateral_control`,
`unilateral_control_ros2`, `bilateral_control`, and `gravity_comp` AArch64
binaries, but the corresponding teleoperation C++ source is absent from both the
active `xarm_teleop` directory and the 2026-07-16 backup. Embedded binary paths
refer to an unavailable `/home/sunrise/x_air_chage` source tree.

The pinned public `x_air_sdk` release supplies the C API wrapper, ROS2 wrapper,
configuration, and robot description. Its Control, Dynamics, JointMapper, and
CAN implementations are still prebuilt shared objects. The public AArch64
library is also a different build from the active January deployment. The
repository root currently has no license file, although the xarm_teleop package
declares Apache-2.0. Treat redistribution rights as unresolved.

The Runtime pins the external checkout rather than copying or republishing its
opaque artifacts. Its candidate sidecar links only to the public C ABI. Policy
command support remains blocked until:

1. an accepted controller release and its build provenance are recorded;
2. J2 signs, offsets, and limits are verified on all four arms;
3. `can2` stability is validated under representative sustained load;
4. startup hold, sequence, freshness, lease, and disable behavior are covered by
   hardware-free tests and staged live commissioning;
5. each physical CAN endpoint has one explicit owner and shutdown order.

The lower-level `xarm_can` and ROS 2 sources in the backup may be audited as
provenance, but they are not silently vendored into this repository.
