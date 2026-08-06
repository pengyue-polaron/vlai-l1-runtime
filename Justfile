set shell := ["bash", "-e", "-o", "pipefail", "-c"]
set quiet

repo := justfile_directory()
vpy := repo + "/.venv/bin/python"
uv := env("UV_BIN", "uv")
system_config := repo + "/configs/system/vlai_l1.toml"

default:
    @just --list

# Development

check:
    {{ vpy }} -m pytest
    {{ vpy }} -m ruff check .
    {{ vpy }} -m ruff format --check .

# x_air SDK preparation and commissioning observation

sdk-build:
    {{ repo }}/scripts/build_xair.sh

sdk-verify:
    {{ vpy }} -m vlai_l1_runtime.cli verify-xair --config {{ system_config }}

sdk-describe side="left":
    {{ vpy }} -m vlai_l1_runtime.cli describe-xair \
        --config {{ system_config }} \
        --side "{{ side }}"

sdk-prepare output="build/xair-assets":
    {{ vpy }} -m vlai_l1_runtime.cli prepare-xair \
        --config {{ system_config }} \
        --output "{{ repo }}/{{ output }}"

sdk-observe samples="3000" timeout="15":
    sudo install -d -o "$(id -un)" -g "$(id -gn)" /run/vlai-l1
    {{ vpy }} -m vlai_l1_runtime.cli observe-xair \
        --config {{ system_config }} \
        --side bimanual \
        --samples "{{ samples }}" \
        --timeout "{{ timeout }}"

sdk-start side="left":
    {{ vpy }} -m vlai_l1_runtime.cli verify-xair --config {{ system_config }}
    sudo {{ vpy }} -m vlai_l1_runtime.cli run-xair \
        --config {{ system_config }} \
        --side "{{ side }}"

sdk-start-right-only:
    {{ vpy }} -m vlai_l1_runtime.cli verify-xair --config {{ system_config }}
    sudo {{ vpy }} -m vlai_l1_runtime.cli run-xair \
        --config {{ system_config }} \
        --side right \
        --isolated-side

sdk-status:
    #!/usr/bin/env bash
    set -euo pipefail
    systemctl is-active xarm-teleop-left.service || true
    systemctl is-active xarm-teleop-right.service || true
    pgrep -af '[v]lai_l1_xair_sidecar|[u]nilateral_control' || true
    for interface in can0 can1 can2 can3; do
        echo "### ${interface}"
        ip -details -statistics link show "${interface}" | sed -n '1,18p'
        tc -s qdisc show dev "${interface}"
    done

# Stop managed x_air runtimes and disable both arm pairs.
stop:
    #!/usr/bin/env bash
    set -euo pipefail
    sudo pkill -INT -f '[v]lai_l1_runtime.cli run-xair' || true
    sudo pkill -INT -f '^{{ repo }}/build/xair-sidecar/vlai_l1_xair_sidecar ' || true
    sleep 2
    sudo /opt/xarm_teleop/disable_unilateral_pair.sh left_arm
    sudo /opt/xarm_teleop/disable_unilateral_pair.sh right_arm

# Cameras and collection

setup-camera:
    {{ uv }} sync --frozen --python 3.12 --extra camera

setup-dataset:
    {{ uv }} sync --frozen --python 3.12 --extra camera --extra dataset

camera-list:
    #!/usr/bin/env bash
    set -euo pipefail
    shopt -s nullglob
    {
        for path in /dev/v4l/by-id/*; do
            device="$(readlink -f "${path}")"
            udevadm info --query=property --name="${device}" \
                | sed -n 's/^ID_SERIAL_SHORT=//p'
        done
    } | sort -u

camera-check samples="30" timeout="0.25":
    {{ repo }}/scripts/camera_service.sh start
    {{ vpy }} -m vlai_l1_runtime.cli camera-check \
        --config {{ system_config }} \
        --samples "{{ samples }}" \
        --timeout "{{ timeout }}"

# Start, stop, inspect, or read logs from the persistent cameras.
cameras action="start":
    {{ repo }}/scripts/camera_service.sh "{{ action }}"

# Check configured CAN and cameras without moving the robot.
hardware *args:
    {{ vpy }} -m vlai_l1_runtime.cli hardware {{ args }}

# Check only the commissioned right-side hardware.
hardware-right *args:
    {{ vpy }} -m vlai_l1_runtime.cli hardware --side right {{ args }}

# Open the bimanual Operator Panel.
panel:
    {{ vpy }} -m vlai_l1_runtime.cli panel

# Open the right-only Operator Panel.
panel-right:
    {{ vpy }} -m vlai_l1_runtime.cli panel --side right

# Move the bimanual robot to its tracked collection reset state.
reset:
    {{ vpy }} -m vlai_l1_runtime.cli reset

# Move only the right side to its tracked collection reset state.
reset-right:
    {{ vpy }} -m vlai_l1_runtime.cli reset --side right

# Reset and collect bimanual episodes into one experiment.
collect experiment task:
    {{ repo }}/scripts/collect.sh \
        --task "{{ task }}" \
        "{{ experiment }}"

# Reset and collect right-only episodes into one experiment.
collect-right experiment task:
    {{ repo }}/scripts/collect.sh \
        --side right \
        --task "{{ task }}" \
        "{{ experiment }}"

# Validate a bimanual canonical dataset.
dataset-doctor experiment *args:
    {{ vpy }} -m vlai_l1_runtime.cli dataset doctor \
        "{{ experiment }}" {{ args }}

# Validate a right-only canonical dataset.
dataset-doctor-right experiment *args:
    {{ vpy }} -m vlai_l1_runtime.cli dataset doctor \
        --side right \
        "{{ experiment }}" {{ args }}

# Export a bimanual canonical dataset to LeRobot v2.1.
export-v21 experiment *args:
    {{ vpy }} -m vlai_l1_runtime.cli dataset export-v21 \
        "{{ experiment }}" {{ args }}

# Export a right-only canonical dataset to LeRobot v2.1.
export-v21-right experiment *args:
    {{ vpy }} -m vlai_l1_runtime.cli dataset export-v21 \
        --side right \
        "{{ experiment }}" {{ args }}

trim-leading-stillness source target:
    {{ vpy }} -m vlai_l1_runtime.cli dataset trim-leading-stillness \
        "{{ source }}" \
        "{{ target }}"

trim-leading-stillness-right source target:
    {{ vpy }} -m vlai_l1_runtime.cli dataset trim-leading-stillness \
        --side right \
        "{{ source }}" \
        "{{ target }}"

plan-leading-stillness source target:
    {{ vpy }} -m vlai_l1_runtime.cli dataset trim-leading-stillness \
        --dry-run \
        "{{ source }}" \
        "{{ target }}"

plan-leading-stillness-right source target:
    {{ vpy }} -m vlai_l1_runtime.cli dataset trim-leading-stillness \
        --side right \
        --dry-run \
        "{{ source }}" \
        "{{ target }}"
