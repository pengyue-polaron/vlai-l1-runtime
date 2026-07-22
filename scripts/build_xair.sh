#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "$0")/.." && pwd)
xair_root="$repo_root/external/x-air-sdk"
ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [[ ! -f "$ros_setup" ]]; then
    echo "FAIL ROS 2 setup is missing: $ros_setup" >&2
    exit 1
fi
if [[ ! -f "$xair_root/publish/modules/src/xarm_teleop/include/xarm_teleop_sdk.h" ]]; then
    echo "FAIL initialize the pinned x_air SDK submodule first" >&2
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "$ros_setup"
set -u

export XARM_SDK_ROOT="$xair_root/publish/xarm_can/package"
colcon --log-base "$repo_root/build/xair-log" build \
    --base-paths \
    "$xair_root/publish/modules/src/xarm_description" \
    "$xair_root/publish/modules/src/xarm_teleop" \
    --build-base "$repo_root/build/xair-colcon" \
    --install-base "$repo_root/build/xair-install" \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DXARM_SDK_ROOT="$XARM_SDK_ROOT"

cmake \
    -S "$repo_root/native/xair_sidecar" \
    -B "$repo_root/build/xair-sidecar" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DX_AIR_SDK_ROOT="$xair_root"
cmake --build "$repo_root/build/xair-sidecar" --parallel
ctest --test-dir "$repo_root/build/xair-sidecar" --output-on-failure

echo "PASS $repo_root/build/xair-sidecar/vlai_l1_xair_sidecar"
